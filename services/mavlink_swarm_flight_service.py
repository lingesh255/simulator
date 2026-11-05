"""GUI-facing: fly an area-coverage plan's per-drone lanes against real
external MAVLink endpoints - one connection, and one full MISSION_ITEM_INT
mission upload + AUTO run, per drone. The multi-drone counterpart of
`services.mavlink_flight_service` (which flies a single shared route).

    [(DroneConfig, [LatLon, ...]), ...]        one lane per drone
        -> one worker thread per drone, each running
           engine.mavlink_mission.upload_and_fly against its own
           `udp:host:port` (same MISSION_COUNT / MISSION_REQUEST_INT /
           MISSION_ITEM_INT / MISSION_ACK / ARM / DO_SET_MODE AUTO /
           MISSION_ITEM_REACHED handshake scripts/mock_sitl.py answers)
        -> each vehicle's own GLOBAL_POSITION_INT / SYS_STATUS / HEARTBEAT,
           merged into one SwarmTelemetryBatch carrying every drone on every
           emit (the map's drone model drops any sysid missing from a batch,
           so a per-drone batch would make the other drones flicker)

Strictly opt-in, same as the single-drone service: the Mission Planner
panel's "Fly via real MAVLink" toggle. With "Use built-in mock vehicle"
checked, `MainWindow` launches one `scripts/mock_sitl.py` per drone first,
each on its own port and seeded with the shared base point.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

from PySide6.QtCore import QObject, QThread, Signal, Slot
from pymavlink.dialects.v20 import ardupilotmega as mav2

from contracts.gui_orchestration import (
    DroneStatus,
    DroneTelemetry,
    FaultType,
    SwarmTelemetryBatch,
)
from engine.mavlink_mission import FlightAborted, upload_and_fly
from engine.peer_bus import DOWN, local_mesh

# How high (metres above the launch point) a formation drone must be before
# the drone behind it may take off.
AIRBORNE_CLEARANCE_M = 5.0


@dataclass
class _VehicleState:
    """One external vehicle's last-known state, updated only from its own
    MAVLink messages - nothing here is simulated."""

    sysid: int
    name: str
    lat: float
    lon: float
    altitude_m: float = 0.0
    heading_deg: float = 0.0
    ground_speed_mps: float = 0.0
    battery_pct: float = 100.0
    armed: bool = False
    gps_ok: bool = True
    custom_mode: int = 0
    started: bool = False
    done: bool = False


def _peer_ids(assignments: list) -> list[str]:
    """One unique peer-bus name per assignment: the drone's own name, unless
    two drones share one (then the sysid is appended to tell them apart)."""
    names = [name for _sysid, name, _pts in assignments]
    if len(set(names)) == len(names):
        return names
    return [f"{name}#{sysid}" for sysid, name, _pts in assignments]


class _SwarmWorker(QObject):
    batch_ready = Signal(object)
    progress = Signal(str)
    finished = Signal()
    failed = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self._abort = False
        self._lock = threading.Lock()
        self._states: dict[int, _VehicleState] = {}
        self._start_time = 0.0
        self._tick = 0

    def request_abort(self) -> None:
        self._abort = True

    @Slot(list, list, list, float, object, list, list)
    def run(
        self,
        assignments: list,
        connection_strings: list,
        altitudes: list,
        launch_stagger_s: float = 0.0,
        launch_point: object = None,
        speeds: list | None = None,
        slots: list | None = None,
    ) -> None:
        """`slots[i]` (formation only) is assignment i's `(lat, lon)` holding
        spot after its climb-out - for the lead, its advance point a few
        metres forward of the source; for a wing, its slot offset from that.
        Defaults to each route's first point.

        `speeds[i]` is assignment i's cruise speed in m/s - that drone's own
        configured speed (its profile's `max_velocity_mps`); `None` leaves
        every vehicle at `engine.mavlink_mission.DEFAULT_CRUISE_SPEED_MPS`.

        `assignments` is `[(sysid, name, [(lat, lon), ...]), ...]`,
        `connection_strings[i]`/`altitudes[i]` are that assignment's pymavlink
        connection and cruise altitude (metres).

        `launch_stagger_s` > 0 makes this a V-formation run: every drone
        lifts off from the one shared `launch_point` `(lat, lon)` - the
        source the operator picked, which is also where each vehicle is
        parked - and climbs out to its own slot (`pts[0]`). Every drone's
        thread runs concurrently - all connect, upload and arm at once - and
        only the takeoffs are sequenced, by the drones messaging each other:
        a drone does not leave the pad until the drone ahead of it has
        announced it is `AIRBORNE_CLEARANCE_M` up (while still climbing to
        its own altitude), plus `launch_stagger_s` seconds of settling time;
        sequencing on that message rather than on a fixed clock is what keeps
        two vehicles off the same launch point. `launch_point` defaults to
        the first assignment's own first point when not given.

        0 (the default, and what an area-coverage mission always passes)
        launches every drone together, straight onto its own route, exactly
        as before this parameter existed.
        """
        speeds = speeds or None
        slots = slots or None
        self._abort = False
        self._tick = 0
        self._start_time = time.monotonic()
        is_formation = launch_stagger_s > 0 and bool(assignments)
        # Where each vehicle actually sits before it flies: its own route's
        # start for an area-coverage lane, but for a formation the one shared
        # launch point every drone takes off from (its slot is airborne, and
        # it only reaches it after climbing out).
        pad = (
            tuple(launch_point) if launch_point is not None
            else (assignments[0][2][0] if assignments else (0.0, 0.0))
        ) if is_formation else None
        with self._lock:
            self._states = {
                sysid: _VehicleState(
                    sysid=sysid, name=name,
                    lat=pad[0] if pad is not None else pts[0][0],
                    lon=pad[1] if pad is not None else pts[0][1],
                )
                for sysid, name, pts in assignments
            }
        self._emit_batch()  # show every drone parked at base straight away

        errors: dict[int, str] = {}

        # A formation's sequencing is peer-to-peer: each drone has its own
        # `PeerBus` endpoint (see engine.peer_bus) and the drones tell each
        # other what they have done instead of sharing a flag between threads.
        #   AIRBORNE announced as soon as a drone's own telemetry shows it
        #            `AIRBORNE_CLEARANCE_M` up - while it is still climbing.
        #            All the drones connect, upload and arm at the same time;
        #            drone i then waits (in `before_auto`, i.e. armed, just
        #            before AUTO starts its takeoff) to hear it from drone
        #            i-1, so the drones leave the shared pad one after
        #            another but overlap in the air; the apex has no one ahead.
        #   IN_SLOT  announced once a drone has climbed out and is holding in
        #            its own slot (the `MAV_CMD_NAV_LOITER_UNLIM` upload
        #            below). Every drone waits to hear it from all the others
        #            before heading for its real route.
        #   READY    announced once a drone is armed with its real route
        #            uploaded. Every drone's upload/arm handshake takes a
        #            different time, so letting each switch to AUTO as soon as
        #            its own finishes sends them off seconds apart (tens of
        #            metres at cruise speed); instead each waits to hear READY
        #            from all the others and they enter AUTO together.
        #   DOWN     announced when a drone's flight ends or fails, so the
        #            rest never wait on it for the timeout.
        # Only `launch_stagger_s > 0` (a V-formation) creates the buses; an
        # area-coverage run (0) never touches them, so every drone there goes
        # straight to its real route exactly as before.
        peer_ids = _peer_ids(assignments)
        buses = (
            local_mesh(
                peer_ids,
                on_heard=lambda me, sender, topic, _data: self.progress.emit(
                    f"{me}: heard '{topic}' from {sender}"
                ),
            )
            if is_formation else {}
        )

        def fly(
            sysid: int, name: str, pts: list, conn: str, altitude_m: float, index: int,
            speed_mps: float | None,
        ) -> None:
            master = None
            bus = buses.get(peer_ids[index])  # None outside a formation
            # Only passed when given, so upload_and_fly's own default applies otherwise.
            speed_kwargs = {} if speed_mps is None else {"cruise_speed_mps": float(speed_mps)}
            try:
                from pymavlink import mavutil

                # Every drone connects, uploads and arms right away,
                # concurrently; only the moment of takeoff is sequenced (see
                # `hold_for_takeoff` below).
                self.progress.emit(f"{name}: connecting to {conn} ...")
                master = mavutil.mavlink_connection(conn)
                with self._lock:
                    self._states[sysid].started = True

                if is_formation:
                    # V-formation: take off from the shared pad, climb to
                    # this slot's cruise altitude, fly out to the slot itself
                    # (`pts[0]` - for the apex that is the pad, so it is a
                    # straight climb) and hold there on a real
                    # MAV_CMD_NAV_LOITER_UNLIM item, not just "fly to a
                    # nearby waypoint". The next drone is released from the pad
                    # as soon as this one is clear of it (AIRBORNE, below),
                    # not when it reaches the slot; the whole formation then
                    # waits for every slot to be filled before any of them
                    # heads toward its real first waypoint. Without that gate,
                    # whichever drone settles first just carries on alone and
                    # the V strings out along the route instead of departing
                    # intact - see services.thread_backend's equivalent gate
                    # for the in-app (non-MAVLink) execution path.
                    slot = tuple(slots[index]) if slots else pts[0]
                    airborne_sent = False

                    def on_climb_message(msg, s: int = sysid, n: str = name) -> None:
                        nonlocal airborne_sent
                        self._on_message(s, msg)
                        if (
                            not airborne_sent
                            and msg.get_type() == "GLOBAL_POSITION_INT"
                            and msg.relative_alt / 1000.0 >= AIRBORNE_CLEARANCE_M
                        ):
                            # The drone's own telemetry says it is up and
                            # clear: tell the drone behind it, armed and
                            # waiting, that it may go - while this one is
                            # still climbing, not once it is in its slot.
                            airborne_sent = True
                            self.progress.emit(
                                f"{n}: airborne ({AIRBORNE_CLEARANCE_M:g}m, clear of the launch point) "
                                f"- telling the other drones."
                            )
                            bus.announce("AIRBORNE")

                    def hold_for_takeoff() -> None:
                        """Armed and uploaded, just before AUTO starts the takeoff.
                        Waits to hear the drone ahead is airborne, then lets
                        `launch_stagger_s` pass. A drone ahead that dropped out
                        (DOWN) releases this one too, so it never waits on a
                        takeoff that cannot happen."""
                        ahead = peer_ids[index - 1]
                        self.progress.emit(f"{name}: armed on the launch point - waiting for {ahead} to lift off ...")
                        bus.wait_for_peers(
                            "AIRBORNE", [ahead], timeout_s=900.0,
                            should_abort=lambda: self._abort,
                            pump=lambda: self._drain(master, sysid),
                        )
                        waited = 0.0
                        while waited < launch_stagger_s:
                            if self._abort:
                                raise FlightAborted(f"{name}: aborted while waiting on the launch point")
                            self._drain(master, sysid)
                            time.sleep(0.2)
                            waited += 0.2
                        self.progress.emit(f"{name}: {ahead} is clear of the launch point - taking off.")

                    upload_and_fly(
                        master, [pad, slot], altitude_m,
                        heartbeat_timeout_s=30.0,
                        on_progress=lambda m, n=name: self.progress.emit(f"{n}: {m}"),
                        should_abort=lambda: self._abort,
                        on_message=on_climb_message,
                        final_command=mav2.MAV_CMD_NAV_LOITER_UNLIM,
                        before_auto=hold_for_takeoff if index > 0 else None,
                        **speed_kwargs,
                    )
                    self.progress.emit(f"{name}: in slot at {altitude_m:g}m - launch point clear.")
                    bus.announce("IN_SLOT", lat=slot[0], lon=slot[1], alt=altitude_m)
                    if len(bus.heard("IN_SLOT")) < len(peer_ids):
                        self.progress.emit(f"{name}: holding - waiting for the rest of the formation to climb ...")
                    # A drone that dropped out meanwhile fails the assembly at
                    # once (down_ok=False). Holding drones still stream
                    # telemetry and nobody reads their socket otherwise, so the
                    # backlog overflows the kernel's UDP buffer (thousands of
                    # dropped datagrams over a long hold - the next mission
                    # upload's reply can be among them) and the map shows the
                    # drone frozen at wherever it last reported; `pump` drains it.
                    bus.wait_for_peers(
                        "IN_SLOT", peer_ids, timeout_s=120.0, down_ok=False,
                        should_abort=lambda: self._abort,
                        pump=lambda: self._drain(master, sysid),
                    )
                    self.progress.emit(f"{name}: formation assembled - proceeding to the real route.")

                def hold_for_release(n: str = name, s: int = sysid) -> None:
                    self.progress.emit(f"{n}: armed and ready - waiting to depart with the formation.")
                    bus.announce("READY")
                    # A drone that dropped out must not hold the rest here
                    # (down_ok), same as before.
                    bus.wait_for_peers(
                        "READY", peer_ids, timeout_s=60.0,
                        should_abort=lambda: self._abort,
                        pump=lambda: self._drain(master, s),
                    )

                upload_and_fly(
                    master, pts, altitude_m,
                    heartbeat_timeout_s=30.0,
                    on_progress=lambda m, n=name: self.progress.emit(f"{n}: {m}"),
                    should_abort=lambda: self._abort,
                    on_message=lambda msg, s=sysid: self._on_message(s, msg),
                    before_auto=hold_for_release if is_formation else None,
                    **speed_kwargs,
                )
            except FlightAborted:
                self.progress.emit(f"{name}: stopped.")
            except (RuntimeError, TimeoutError, OSError) as exc:
                errors[sysid] = f"{name}: {exc}"
                self.progress.emit(f"{name}: FAILED - {exc}")
            finally:
                if master is not None:
                    try:
                        master.close()
                    except OSError:
                        pass
                # Never strand the drone behind this one on the pad waiting
                # for a slot arrival that can no longer happen.
                if bus is not None:
                    bus.announce(DOWN)
                with self._lock:
                    st = self._states.get(sysid)
                    if st is not None:
                        st.done = True
                        st.armed = False
                self._emit_batch()

        # Every thread starts now; a formation's own turn-taking is enforced
        # inside `fly`, which holds each drone on the pad until the one ahead
        # of it announces IN_SLOT (see the peer buses above) rather than on a
        # blind sleep here that could not tell whether the pad was actually clear.
        threads = [
            threading.Thread(target=fly, args=(sysid, name, pts, conn, alt, i, spd), daemon=True)
            for i, ((sysid, name, pts), conn, alt, spd) in enumerate(
                zip(assignments, connection_strings, altitudes, speeds or [None] * len(assignments))
            )
        ]
        try:
            for t in threads:
                t.start()
            for t in threads:
                t.join()
        finally:
            for bus in buses.values():
                bus.close()

        if errors and not self._abort:
            self.failed.emit("; ".join(errors[s] for s in sorted(errors)))
        else:
            self.finished.emit()

    # ---- per-vehicle MAVLink -> merged batch ----

    def _drain(self, master, sysid: int, limit: int = 200) -> None:
        """Read and dispatch whatever `master`'s vehicle has sent since the
        last read, without blocking. For the stretches where no
        `upload_and_fly` is reading that link (a formation hold)."""
        for _ in range(limit):
            message = master.recv_match(blocking=False)
            if message is None:
                return
            self._on_message(sysid, message)

    def _on_message(self, sysid: int, message) -> None:
        kind = message.get_type()
        with self._lock:
            st = self._states.get(sysid)
            if st is None:
                return
            if kind == "GLOBAL_POSITION_INT":
                st.lat = message.lat / 1e7
                st.lon = message.lon / 1e7
                st.altitude_m = message.relative_alt / 1000.0
                st.heading_deg = (message.hdg / 100.0) % 360.0
                st.ground_speed_mps = math.hypot(message.vx, message.vy) / 100.0
            elif kind == "SYS_STATUS":
                if message.battery_remaining >= 0:
                    st.battery_pct = float(message.battery_remaining)
            elif kind == "GPS_RAW_INT":
                st.gps_ok = message.fix_type >= 2
            elif kind == "HEARTBEAT":
                st.armed = bool(message.base_mode & mav2.MAV_MODE_FLAG_SAFETY_ARMED)
                st.custom_mode = message.custom_mode
        if kind == "GLOBAL_POSITION_INT":
            self._emit_batch()  # position is what actually moves the map marker

    def _emit_batch(self) -> None:
        with self._lock:
            self._tick += 1
            tick = self._tick
            timestamp = time.monotonic() - self._start_time
            drones = [
                DroneTelemetry(
                    sysid=st.sysid,
                    status=(
                        DroneStatus.IN_FLIGHT if st.armed
                        else DroneStatus.LANDED if st.done
                        else DroneStatus.STANDBY
                    ),
                    lat=st.lat,
                    lon=st.lon,
                    altitude_m=st.altitude_m,
                    heading_deg=st.heading_deg,
                    ground_speed_mps=st.ground_speed_mps,
                    battery_pct=max(0.0, min(100.0, st.battery_pct)),
                    link_quality_pct=100.0,  # receiving telemetry at all => link up
                    active_faults=[] if st.gps_ok else [FaultType.GPS_LOSS],
                    raw_mavlink=(
                        f"[external MAVLink] {st.name} armed={st.armed} mode={st.custom_mode} "
                        f"pos=({st.lat:.6f},{st.lon:.6f},{st.altitude_m:.1f}) batt={st.battery_pct:.1f}%"
                    ),
                )
                for st in self._states.values()
            ]
        self.batch_ready.emit(
            SwarmTelemetryBatch(tick=tick, timestamp=timestamp, drones=drones)
        )


class MavlinkSwarmFlightService(QObject):
    """GUI-facing handle: call `run_async`, get `batch_ready`/`progress`/
    `finished`/`failed` - the same signal surface as `MavlinkFlightService`."""

    batch_ready = Signal(object)
    progress = Signal(str)
    finished = Signal()
    failed = Signal(str)

    _run_requested = Signal(list, list, list, float, object, list, list)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._thread = QThread(self)
        self._worker = _SwarmWorker()
        self._worker.moveToThread(self._thread)
        self._worker.batch_ready.connect(self.batch_ready)
        self._worker.progress.connect(self.progress)
        self._worker.finished.connect(self.finished)
        self._worker.failed.connect(self.failed)
        self._run_requested.connect(self._worker.run)
        self._thread.start()

    def run_async(
        self,
        assignments: list,
        connection_strings: list,
        altitudes: list,
        launch_stagger_s: float = 0.0,
        launch_point: object = None,
        speeds: list | None = None,
        slots: list | None = None,
    ) -> None:
        """`slots`: formation holding spot per assignment, `(lat, lon)` (see
        `_SwarmWorker.run`). `speeds`: one cruise speed (m/s) per assignment - each drone's own
        configured speed - or None for the default. `assignments`: `[(sysid, name, [(lat, lon), ...]), ...]`.
        `connection_strings`/`altitudes`: one pymavlink connection string and
        cruise altitude (metres) per assignment. `launch_stagger_s` > 0 makes
        it a V-formation: every drone takes off from the shared
        `launch_point` `(lat, lon)` and climbs out to its own slot, one drone
        at a time, each waiting `launch_stagger_s` after the drone ahead of
        it reaches its slot - see `_SwarmWorker.run`."""
        self._run_requested.emit(
            assignments, connection_strings, altitudes, launch_stagger_s, launch_point,
            list(speeds) if speeds is not None else [],
            [list(x) for x in slots] if slots is not None else [],
        )

    def abort(self) -> None:
        self._worker.request_abort()

    def shutdown(self) -> None:
        self._worker.request_abort()
        self._thread.quit()
        self._thread.wait(2000)
