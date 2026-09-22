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

    @Slot(list, list, list, float, object)
    def run(
        self,
        assignments: list,
        connection_strings: list,
        altitudes: list,
        launch_stagger_s: float = 0.0,
        launch_point: object = None,
    ) -> None:
        """`assignments` is `[(sysid, name, [(lat, lon), ...]), ...]`,
        `connection_strings[i]`/`altitudes[i]` are that assignment's pymavlink
        connection and cruise altitude (metres).

        `launch_stagger_s` > 0 makes this a V-formation run: every drone
        lifts off from the one shared `launch_point` `(lat, lon)` - the
        source the operator picked, which is also where each vehicle is
        parked - and climbs out to its own slot (`pts[0]`), strictly one
        drone at a time. A drone does not leave the pad until the drone
        ahead of it has actually reached its slot, plus `launch_stagger_s`
        seconds of settling time; sequencing on the arrival rather than on a
        fixed clock is what keeps two vehicles off the same launch point.
        This mirrors `ThreadSwarmBackend.start_formation_mission`'s in-app
        behaviour (see `services.plan_service.FORMATION_LAUNCH_STAGGER_S`)
        on this real-MAVLink path. `launch_point` defaults to the first
        assignment's own first point when not given.

        0 (the default, and what an area-coverage mission always passes)
        launches every drone together, straight onto its own route, exactly
        as before this parameter existed.
        """
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

        # Formation assembly gate: once a drone has climbed out from the pad
        # and is holding in its own slot (see the `MAV_CMD_NAV_LOITER_UNLIM`
        # upload below), it adds itself here and then polls until every other
        # drone in this run has too - only `launch_stagger_s > 0` (a
        # V-formation) ever populates this; an area-coverage run (0) never
        # touches it, so every drone there goes straight to its real route
        # exactly as before this existed.
        assembly_total = len(assignments)
        climbed: set[int] = set()
        climbed_lock = threading.Lock()
        # One event per formation drone, set the moment it reports settled in
        # its slot. Drone i waits on drone i-1's before it even connects, so
        # the drones leave the shared pad strictly one at a time; index 0's
        # is pre-set because the apex goes first with nothing to wait for.
        in_slot_events = [threading.Event() for _ in assignments]
        if in_slot_events:
            in_slot_events[0].set()

        def wait_for_formation(sysid: int, name: str, timeout_s: float = 120.0) -> None:
            with climbed_lock:
                climbed.add(sysid)
                ready = len(climbed) >= assembly_total
            if ready:
                return
            self.progress.emit(f"{name}: holding - waiting for the rest of the formation to climb ...")
            waited = 0.0
            while True:
                with climbed_lock:
                    if len(climbed) >= assembly_total:
                        return
                if self._abort:
                    raise FlightAborted(f"{name}: aborted while holding for the formation")
                if waited >= timeout_s:
                    raise TimeoutError(
                        f"{name}: timed out after {timeout_s:.0f}s waiting for the rest of the "
                        f"formation to reach cruise altitude"
                    )
                time.sleep(0.2)
                waited += 0.2

        def fly(sysid: int, name: str, pts: list, conn: str, altitude_m: float, index: int) -> None:
            master = None
            try:
                from pymavlink import mavutil

                if is_formation and index > 0:
                    # Hold on the pad until the drone ahead has climbed out
                    # to its slot, then give it `launch_stagger_s` to settle.
                    self.progress.emit(
                        f"{name}: on the launch point - waiting for the drone ahead to reach its slot ..."
                    )
                    while not in_slot_events[index - 1].wait(0.2):
                        if self._abort:
                            raise FlightAborted(f"{name}: aborted while waiting on the launch point")
                    waited = 0.0
                    while waited < launch_stagger_s:
                        if self._abort:
                            raise FlightAborted(f"{name}: aborted while waiting on the launch point")
                        time.sleep(0.2)
                        waited += 0.2

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
                    # nearby waypoint". Reaching the slot releases the next
                    # drone from the pad; the whole formation then waits for
                    # every slot to be filled before any of them heads toward
                    # its real first waypoint. Without that gate, whichever
                    # drone settles first just carries on alone and the V
                    # strings out along the route instead of departing intact
                    # - see services.thread_backend's equivalent gate for the
                    # in-app (non-MAVLink) execution path.
                    slot = pts[0]
                    upload_and_fly(
                        master, [pad, slot], altitude_m,
                        heartbeat_timeout_s=30.0,
                        on_progress=lambda m, n=name: self.progress.emit(f"{n}: {m}"),
                        should_abort=lambda: self._abort,
                        on_message=lambda msg, s=sysid: self._on_message(s, msg),
                        final_command=mav2.MAV_CMD_NAV_LOITER_UNLIM,
                    )
                    self.progress.emit(f"{name}: in slot at {altitude_m:g}m - launch point clear.")
                    in_slot_events[index].set()
                    wait_for_formation(sysid, name)
                    self.progress.emit(f"{name}: formation assembled - proceeding to the real route.")

                upload_and_fly(
                    master, pts, altitude_m,
                    heartbeat_timeout_s=30.0,
                    on_progress=lambda m, n=name: self.progress.emit(f"{n}: {m}"),
                    should_abort=lambda: self._abort,
                    on_message=lambda msg, s=sysid: self._on_message(s, msg),
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
                if is_formation:
                    in_slot_events[index].set()
                with self._lock:
                    st = self._states.get(sysid)
                    if st is not None:
                        st.done = True
                        st.armed = False
                self._emit_batch()

        # Every thread starts now; a formation's own turn-taking is enforced
        # inside `fly`, which holds each drone on the pad until the one ahead
        # of it reports in slot (see `in_slot_events`) rather than on a blind
        # sleep here that could not tell whether the pad was actually clear.
        threads = [
            threading.Thread(target=fly, args=(sysid, name, pts, conn, alt, i), daemon=True)
            for i, ((sysid, name, pts), conn, alt) in enumerate(
                zip(assignments, connection_strings, altitudes)
            )
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        if errors and not self._abort:
            self.failed.emit("; ".join(errors[s] for s in sorted(errors)))
        else:
            self.finished.emit()

    # ---- per-vehicle MAVLink -> merged batch ----

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

    _run_requested = Signal(list, list, list, float, object)

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
    ) -> None:
        """`assignments`: `[(sysid, name, [(lat, lon), ...]), ...]`.
        `connection_strings`/`altitudes`: one pymavlink connection string and
        cruise altitude (metres) per assignment. `launch_stagger_s` > 0 makes
        it a V-formation: every drone takes off from the shared
        `launch_point` `(lat, lon)` and climbs out to its own slot, one drone
        at a time, each waiting `launch_stagger_s` after the drone ahead of
        it reaches its slot - see `_SwarmWorker.run`."""
        self._run_requested.emit(
            assignments, connection_strings, altitudes, launch_stagger_s, launch_point,
        )

    def abort(self) -> None:
        self._worker.request_abort()

    def shutdown(self) -> None:
        self._worker.request_abort()
        self._thread.quit()
        self._thread.wait(2000)
