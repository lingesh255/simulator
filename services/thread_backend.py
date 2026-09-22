"""Drone-thread flight source.

Runs the swarm as independent threads talking to a Ground Control Station over
UDP (`engine.drone_thread`, `engine.drone_link`, `engine.gcs`), and presents the
same signals and methods as `LocalFlightSimulator`, so the main window can use
it without knowing that each drone is now its own node on a network.

Telemetry is pulled on a GUI-thread timer rather than pushed from the drone
threads: Qt widgets may only be touched from the GUI thread, and polling a
queue keeps the crossing explicit.
"""
from __future__ import annotations

from typing import Optional

from PySide6.QtCore import QObject, QTimer, Signal

from contracts.gui_orchestration import (
    DroneConfig,
    DroneStatus,
    DroneTelemetry,
    FaultType,
    InjectFault,
    LatLon,
    SwarmTelemetryBatch,
)
from engine.drone_link import DroneLink
from engine.drone_thread import (
    ARRIVAL_RADIUS_M,
    CRITICAL_BATTERY_PCT,
    LOW_BATTERY_PCT,
    DroneSnapshot,
    HealthStatus,
    MissionState,
    Position,
    haversine_m as engine_haversine,
)
from engine.flight_controller import SimulatedFlightController
from engine.fleet import DroneFleet
from engine.gcs import GroundControlStation
from engine.protocol import CommandName
from engine.terrain import plan_terrain_profile
from services.local_flight import EmergencyLanding, nearest_point, plan_flights

POLL_MS = 100

# How close (metres) a drone's altitude must be to its target for
# `ThreadSwarmBackend._check_formation_assembly` to call it "there" - the
# climb asymptotically approaches the target (see engine.flight_controller's
# clamped climb step), so an exact `==` would never fire.
ALTITUDE_ARRIVAL_M = 0.5

# The node's mission vocabulary, in the terms the dashboard speaks.
_STATUS_MAP = {
    MissionState.IDLE: DroneStatus.STANDBY,
    MissionState.ARMED: DroneStatus.ARMED,
    MissionState.TAKEOFF: DroneStatus.TAKING_OFF,
    MissionState.EN_ROUTE: DroneStatus.IN_FLIGHT,
    MissionState.HOLDING: DroneStatus.IN_FLIGHT,
    MissionState.LANDING: DroneStatus.LANDING,
    MissionState.LANDED: DroneStatus.LANDED,
    MissionState.ABORTED: DroneStatus.FAILSAFE,
    MissionState.FAILED: DroneStatus.FAILSAFE,
}


def _to_latlon(position: Optional[Position]) -> Optional[LatLon]:
    if position is None:
        return None
    return LatLon(lat=position.lat, lon=position.lon)


class _FlightView:
    """What the main window's dialogs read, backed by a live drone thread."""

    def __init__(self, config: DroneConfig, drone, link: DroneLink, home: LatLon):
        self.config = config
        self._drone = drone
        self._link = link
        self._home = home

    @property
    def _snap(self) -> DroneSnapshot:
        return self._drone.snapshot()

    @property
    def position(self) -> LatLon:
        return _to_latlon(self._snap.position)

    @property
    def start(self) -> LatLon:
        return self._home

    @property
    def battery_pct(self) -> float:
        return self._snap.battery

    @property
    def gps_hold(self) -> bool:
        snap = self._snap
        return snap.mission_state is MissionState.HOLDING and not snap.gps_available

    @property
    def is_lost(self) -> bool:
        return self._snap.mission_state is MissionState.FAILED

    @property
    def is_done(self) -> bool:
        return self._snap.mission_state in (
            MissionState.LANDED, MissionState.ABORTED, MissionState.FAILED
        )

    @property
    def cruise_speed_mps(self) -> float:
        return self._drone.cruise_speed_mps

    @property
    def range_m(self) -> float:
        endurance_s = (self.config.battery_capacity_mah / 1000.0) / 20.0 * 3600.0
        return max(0.0, endurance_s) * self.cruise_speed_mps

    def can_reach(self, point: LatLon) -> bool:
        """Endurance estimate on the remaining charge - the same rule the local
        preview applies, evaluated against live node state."""
        snap = self._snap
        remaining_mah = self.config.battery_capacity_mah * max(0.0, snap.battery) / 100.0
        endurance_s = (remaining_mah / 1000.0) / 20.0 * 3600.0
        target = Position(lat=point.lat, lon=point.lon)
        travel_s = engine_haversine(snap.position, target) / max(0.1, self.cruise_speed_mps)
        descent_s = snap.position.alt_m / 2.0
        return travel_s + descent_s <= endurance_s


class ThreadSwarmBackend(QObject):
    """Flight source backed by one thread per drone, commanded over UDP."""

    batch_ready = Signal(object)
    progress = Signal(float, float)
    finished = Signal(float)
    low_battery = Signal(int, float)
    drone_lost = Signal(int)
    gps_lost = Signal(int)
    gps_restored = Signal(int)
    retargeted = Signal(int)
    link_failed = Signal(str)

    def __init__(self, parent=None, time_scale: float = 20.0, use_firmware: bool = True):
        super().__init__(parent)
        # With firmware on, each drone drives a MAVLink flight controller
        # rather than integrating its own motion.
        self.use_firmware = use_firmware
        self._fleet: Optional[DroneFleet] = None
        self._gcs: Optional[GroundControlStation] = None
        self._views: dict[int, _FlightView] = {}
        self._destination: Optional[LatLon] = None
        # A PDDL-planned mission's full route per drone, and the index of the
        # waypoint each is currently headed for. Absent for a plain
        # start->destination mission, which has nothing to chain.
        self._paths: dict[int, list[LatLon]] = {}
        self._leg_index: dict[int, int] = {}
        # One target cruise altitude per waypoint in `_paths[sysid]`, aligned
        # by index - climbs over a leg's terrain by AGL_CLEARANCE_M, or (see
        # engine.terrain.plan_terrain_profile) routes around it if that would
        # need climbing past the ceiling. Same route for every drone in a
        # mission, so it's computed once in start_mission_path.
        self._altitudes: dict[int, list[float]] = {}
        # Staggered-launch queue for `start_formation_mission`: sysid -> the
        # `_elapsed_s` value at (or after) which its first NAVIGATE fires.
        # Only ever holds the *one* drone whose turn it is - the rest wait in
        # `_launch_pending` until the drone ahead of them has actually
        # reached its slot, because every formation drone now lifts off from
        # the same launch point and two of them cannot occupy it at once.
        # Empty outside a staggered-launch mission, so `_fire_due_launches`
        # is a no-op for every other mission kind.
        self._launch_queue: dict[int, float] = {}
        # Formation drones not yet launched, in apex-first launch order; the
        # head is promoted into `_launch_queue` by `_check_formation_assembly`
        # once the drone ahead of it is settled in its slot.
        self._launch_pending: list[int] = []
        # Seconds between one formation drone reaching its slot and the next
        # one lifting off the shared launch point (settling time), and the
        # launch point itself - both set by `start_formation_mission`.
        self._launch_stagger_s: float = 6.0
        self._formation_pad: Optional[LatLon] = None
        # Formation assembly gate: sysid -> `(slot, cruise_altitude_m)` - the
        # spot in the airborne V this drone was sent to take up after lifting
        # off the shared pad, and the altitude it holds there while waiting
        # for every other formation drone to take up its own. Populated by
        # `_fire_due_launches`, cleared by `_check_formation_assembly` the
        # instant every entry is settled in its slot - which is also the
        # instant all of them are sent toward their real first waypoint,
        # together, so the V departs intact instead of stringing out along
        # the route as each drone finishes its own climb independently.
        self._slot_wait: dict[int, tuple[LatLon, float]] = {}
        # Subset of `_slot_wait` already confirmed to be sitting in its slot.
        self._in_slot: set[int] = set()
        # V-formation bookkeeping for the *landing* side of a mission -
        # mirrors the launch side above. `_formation_launch_order` is the
        # apex-first sysid order (set once by `start_formation_mission`,
        # cleared by `stop`); `_land_wait` is every formation sysid that has
        # arrived at (and is holding over) its own final point, waiting for
        # the rest; `_land_queue` is populated - apex first, `_land_stagger_s`
        # apart - only once *all* of them have arrived, and is what actually
        # staggers the real MAV-level landings (see `_check_formation_landing`
        # / `_fire_due_landings`), so the group doesn't all set down in the
        # same small area at the same instant.
        self._formation_launch_order: list[int] = []
        self._land_wait: set[int] = set()
        self._land_queue: dict[int, float] = {}
        self._land_stagger_s: float = 6.0
        self._landing_started = False
        self._time_scale = time_scale
        self._elapsed_s = 0.0
        self._tick = 0
        self._paused = False
        self._flagged_low: set[int] = set()
        self._flagged_gps: set[int] = set()
        self._flagged_lost: set[int] = set()
        self._emergency: set[int] = set()

        self._timer = QTimer(self)
        self._timer.setInterval(POLL_MS)
        self._timer.timeout.connect(self._poll)

    # ---- Surface shared with the other flight sources ----

    def is_running(self) -> bool:
        return self._timer.isActive()

    def is_active(self) -> bool:
        return bool(self._views) and not all(v.is_done for v in self._views.values())

    @property
    def time_scale(self) -> float:
        return self._time_scale

    @property
    def flights(self) -> list:
        return list(self._views.values())

    def start_mission(self, drones: list[DroneConfig], start: LatLon, destination: LatLon) -> bool:
        """The plain, single-leg mission - a two-point path."""
        return self.start_mission_path(drones, [start, destination])

    def start_mission_path(self, drones: list[DroneConfig], waypoints: list[LatLon]) -> bool:
        """Spawn a node per drone, then fly `waypoints` in order over UDP.

        Each leg is commanded as an ordinary NAVIGATE; `_advance_legs` (called
        every poll tick) sends the next one as soon as a drone reaches the
        current target, so intermediate waypoints are passed through rather
        than landed at - only the final one triggers a landing.
        """
        self.stop()
        if not drones or len(waypoints) < 2:
            return False

        self._gcs = GroundControlStation(verbose=False)
        self._gcs.start()
        self._fleet = DroneFleet()
        self._destination = waypoints[-1]
        # Terrain-aware pass over the (possibly already restricted-area-
        # detoured) lateral route: assigns each leg a cruise altitude that
        # clears its highest point by AGL_CLEARANCE_M ("go above that hill"),
        # inserting a lateral bypass instead of any leg that would need
        # climbing past the ceiling ("turn around the hill").
        waypoints, altitudes = plan_terrain_profile(waypoints)
        home = waypoints[0]

        for drone in drones:
            self._spawn_drone(drone, home)

        if not self._views:
            self.stop()
            return False

        self._gcs.register_many(self._views)
        self._gcs.wait_for_heartbeats(self._views, timeout_s=3.0)

        source = [waypoints[0].lat, waypoints[0].lon, 0.0]
        target = [waypoints[1].lat, waypoints[1].lon, altitudes[1]]
        is_final = 1 >= len(waypoints) - 1
        for sysid in self._views:
            self._paths[sysid] = list(waypoints)
            self._altitudes[sysid] = list(altitudes)
            self._leg_index[sysid] = 1
            self._gcs.navigate(sysid, source, target, final=is_final)

        self._elapsed_s = 0.0
        self._tick = 0
        self._paused = False
        for flags in (self._flagged_low, self._flagged_gps, self._flagged_lost, self._emergency):
            flags.clear()
        self._timer.start()
        return True

    def start_mission_paths(self, assignments: list[tuple[DroneConfig, list[LatLon]]]) -> bool:
        """Like `start_mission_path`, but each drone flies its own distinct
        route rather than everyone sharing one - the forest-search mission's
        per-drone lane, where drone1 and drone2 are planned completely
        separate routes that just happen to both start/end at the same base
        point.

        `_advance_legs`/`_poll` already key every piece of chained-route
        state (`_paths`, `_altitudes`, `_leg_index`) by sysid, so nothing
        below this needs to know or care that the routes differ per drone -
        only the setup here (each drone gets its own terrain profile, home
        point and first NAVIGATE) is genuinely different from
        `start_mission_path`.
        """
        self.stop()
        if not assignments:
            return False

        self._gcs = GroundControlStation(verbose=False)
        self._gcs.start()
        self._fleet = DroneFleet()
        # Progress/finished reporting still tracks a single "how much
        # further to go" figure - the shared base every lane ends at, since
        # that's what remains once a drone reaches the end of its own route.
        self._destination = assignments[0][1][-1]

        per_drone: dict[int, tuple[list[LatLon], list[float]]] = {}
        for drone, waypoints in assignments:
            if len(waypoints) < 2:
                continue
            points, altitudes = plan_terrain_profile(waypoints)
            if not self._spawn_drone(drone, points[0]):
                continue
            per_drone[drone.sysid] = (points, altitudes)

        if not self._views:
            self.stop()
            return False

        self._gcs.register_many(self._views)
        self._gcs.wait_for_heartbeats(self._views, timeout_s=3.0)

        for sysid in self._views:
            points, altitudes = per_drone[sysid]
            source = [points[0].lat, points[0].lon, 0.0]
            target = [points[1].lat, points[1].lon, altitudes[1]]
            is_final = 1 >= len(points) - 1
            self._paths[sysid] = list(points)
            self._altitudes[sysid] = list(altitudes)
            self._leg_index[sysid] = 1
            self._gcs.navigate(sysid, source, target, final=is_final)

        self._elapsed_s = 0.0
        self._tick = 0
        self._paused = False
        for flags in (self._flagged_low, self._flagged_gps, self._flagged_lost, self._emergency):
            flags.clear()
        self._timer.start()
        return True

    def start_formation_mission(
        self,
        assignments: list[tuple[DroneConfig, list[LatLon], float]],
        launch_stagger_s: float = 6.0,
        land_stagger_s: Optional[float] = None,
        launch_point: Optional[LatLon] = None,
    ) -> bool:
        """Like `start_mission_paths`, but for a V-formation: every drone
        lifts off from the *same* launch point and then climbs out to its
        own slot in the airborne V, one drone at a time.

        Each drone gets its own fixed cruise altitude (not the
        terrain-derived one every other mission uses - a formation holds a
        set altitude offset per slot, apex above the wings, not "whatever
        clears the ground"). All three are spawned on the shared pad
        (`launch_point`, defaulting to the apex route's first point - the
        source the operator actually picked), and are launched strictly in
        turn, in `assignments` order: the apex takes off, flies to its slot
        and settles there, and only `launch_stagger_s` seconds *after that*
        does the left wing leave the pad, then the right wing the same way.
        Sequencing on the drone ahead actually arriving - rather than on a
        fixed clock - is what keeps two of them from ever occupying the one
        launch point at the same time. Landing is staggered `land_stagger_s`
        apart (defaults to `launch_stagger_s`) - see
        `_check_formation_landing`.

        `assignments` is `(drone, route, altitude_m)`, already in launch
        order (see `services.plan_service.FORMATION_DRONES`/
        `_run_formation`) - also the landing order. Each route still *starts*
        at that drone's own slot (`engine.pddl_problem.formation_wing_routes`
        offsets every wing waypoint, the first included), so the climb-out
        target is simply `route[0]`, and the real route is flown from there
        unchanged. Every drone is spawned immediately, landed, so they all
        appear on the map together; only the NAVIGATE that arms and launches
        each one is sequenced, via `_launch_queue` / `_launch_pending` /
        `_fire_due_launches` (checked every `_poll` tick).
        """
        self.stop()
        if not assignments:
            return False

        self._gcs = GroundControlStation(verbose=False)
        self._gcs.start()
        self._fleet = DroneFleet()
        self._destination = assignments[0][1][-1]

        # The one point on the ground every drone in the formation takes off
        # from. Defaults to the apex route's source; passed explicitly by the
        # caller when the apex itself may have been dropped from
        # `assignments` (e.g. grounded on battery), so the pad stays the
        # operator's picked source rather than sliding to a wing's slot.
        pad = launch_point if launch_point is not None else assignments[0][1][0]
        self._formation_pad = pad

        per_drone: dict[int, tuple[list[LatLon], list[float]]] = {}
        launch_order: list[int] = []
        for drone, waypoints, altitude_m in assignments:
            if len(waypoints) < 2:
                continue
            # Every drone spawns on the shared pad, not on its own slot - the
            # slot is where it climbs out to once airborne.
            if not self._spawn_drone(drone, pad):
                continue
            per_drone[drone.sysid] = (list(waypoints), [float(altitude_m)] * len(waypoints))
            launch_order.append(drone.sysid)

        if not self._views:
            self.stop()
            return False

        self._gcs.register_many(self._views)
        self._gcs.wait_for_heartbeats(self._views, timeout_s=3.0)

        for sysid in launch_order:
            points, altitudes = per_drone[sysid]
            self._paths[sysid] = points
            self._altitudes[sysid] = altitudes
            # `_leg_index[sysid]` is deliberately NOT set here - each drone's
            # first NAVIGATE (sent below/by `_fire_due_launches`) is a
            # climb-out from the shared pad to its own slot, not the real
            # leg 1; `_check_formation_assembly` sets it once the whole
            # formation is in slot and is actually released toward that leg.

        self._formation_launch_order = list(launch_order)
        self._launch_stagger_s = launch_stagger_s
        self._land_stagger_s = launch_stagger_s if land_stagger_s is None else land_stagger_s

        self._elapsed_s = 0.0
        self._tick = 0
        self._paused = False
        for flags in (self._flagged_low, self._flagged_gps, self._flagged_lost, self._emergency):
            flags.clear()

        # Only the apex is queued now, due at t=0, so `_fire_due_launches`
        # sends it off immediately below rather than waiting for the first
        # poll tick. Each following drone is promoted out of
        # `_launch_pending` by `_check_formation_assembly`, once the drone
        # ahead of it has actually reached its slot and the pad is clear.
        self._launch_queue = {launch_order[0]: 0.0}
        self._launch_pending = list(launch_order[1:])
        self._fire_due_launches()

        self._timer.start()
        return True

    def _fire_due_launches(self) -> None:
        """Send the first NAVIGATE for the queued drone whose turn to leave
        the shared pad has come - see `start_formation_mission`. A no-op
        once the queue is empty, so it's safe to call from every `_poll`
        tick regardless of mission kind.

        That first NAVIGATE runs from the shared launch point up to this
        drone's *own slot* in the airborne V (`route[0]`), not to its real
        first waypoint: it climbs to the slot altitude and flies out to the
        slot, then holds there. For the apex the slot is the pad itself, so
        that NAVIGATE is a straight climb with no lateral movement (see
        `engine.flight_controller.SimulatedFlightController._fly_mission` -
        an already-arrived waypoint completes the "mission" on the same tick
        the climb finishes). `_check_formation_assembly` watches for the
        arrival: it both frees the pad for the next drone and, later, sends
        the whole assembled V onward together - never each drone the moment
        it personally settles."""
        if not self._launch_queue or self._gcs is None:
            return
        due = [sysid for sysid, at in self._launch_queue.items() if self._elapsed_s >= at]
        for sysid in due:
            del self._launch_queue[sysid]
            points = self._paths.get(sysid)
            altitudes = self._altitudes.get(sysid)
            if not points or not altitudes:
                continue
            pad = self._formation_pad or points[0]
            slot = points[0]
            altitude = altitudes[0]
            self._gcs.navigate(
                sysid, [pad.lat, pad.lon, 0.0], [slot.lat, slot.lon, altitude], final=False,
            )
            self._slot_wait[sysid] = (slot, altitude)

    def _check_formation_assembly(self, snapshots: dict[int, DroneSnapshot]) -> None:
        """Watch each launched drone climb out from the shared pad to its own
        slot, and do the two things that arrival unblocks - see
        `_fire_due_launches`.

        First, launch sequencing: the moment the drone currently climbing out
        settles in its slot, the pad is clear, so the next drone waiting in
        `_launch_pending` is queued to lift off `_launch_stagger_s` seconds
        later. Sequencing on that arrival rather than on a fixed clock is
        what keeps the drones taking off one at a time - nobody leaves the
        pad while somebody else is still on it or climbing away from it.

        Second, the departure gate: once every formation drone is settled in
        its own slot, all of them are released toward their real first
        waypoint on the same tick, so the V departs intact instead of
        stringing out. A drone that hasn't launched yet
        (`_launch_queue`/`_launch_pending`) or hasn't reached its slot holds
        the whole group. A no-op once `_slot_wait` is empty, so safe to call
        every tick."""
        if not self._slot_wait or self._gcs is None:
            return

        for sysid, (slot, target_alt) in self._slot_wait.items():
            if sysid in self._in_slot:
                continue
            snap = snapshots.get(sysid)
            if snap is None or snap.position.alt_m < target_alt - ALTITUDE_ARRIVAL_M:
                continue  # still climbing
            if engine_haversine(
                snap.position, Position(lat=slot.lat, lon=slot.lon)
            ) > ARRIVAL_RADIUS_M:
                continue  # at altitude, still flying out to its slot
            self._in_slot.add(sysid)
            # Pad is clear - let the next drone go, after a settling gap.
            if self._launch_pending:
                nxt = self._launch_pending.pop(0)
                self._launch_queue[nxt] = self._elapsed_s + self._launch_stagger_s

        if self._launch_queue or self._launch_pending:
            return  # somebody still has to leave the pad
        if any(sysid not in self._in_slot for sysid in self._slot_wait):
            return  # at least one drone is still climbing out to its slot

        for sysid in list(self._slot_wait):
            points = self._paths.get(sysid)
            altitudes = self._altitudes.get(sysid)
            if not points or len(points) < 2 or not altitudes:
                continue
            self._gcs.navigate(
                sysid,
                [points[0].lat, points[0].lon, altitudes[0]],
                [points[1].lat, points[1].lon, altitudes[1]],
                # Never land straight off this NAVIGATE, even if leg 1 is
                # the route's last point: a formation drone always arrives
                # and holds first (`_check_formation_landing` sends the real
                # LAND, staggered, once every drone has arrived) - see the
                # matching override in `_advance_legs`.
                final=False,
            )
            self._leg_index[sysid] = 1
        self._slot_wait.clear()
        self._in_slot.clear()

    def _check_formation_landing(self, snapshots: dict[int, DroneSnapshot]) -> None:
        """The landing mirror of `_fire_due_launches`/`_check_formation_assembly`:
        watch every still-active formation drone for arriving at (and
        holding over - see the `final=False` overrides above) its own final
        point, and once *all* of them have, stagger the real LAND commands
        apex first, `_land_stagger_s` apart, via `_land_queue` /
        `_fire_due_landings`. A no-op once landing has already started, or
        outside a formation mission (`_formation_launch_order` empty)."""
        if not self._formation_launch_order or self._landing_started:
            return
        active = [
            sysid for sysid in self._formation_launch_order
            if sysid in self._views and not self._views[sysid].is_done
        ]
        if not active:
            return

        for sysid in active:
            if sysid in self._land_wait:
                continue
            path = self._paths.get(sysid)
            leg = self._leg_index.get(sysid)
            if not path or leg is None or leg < len(path) - 1:
                continue  # not dispatched toward its final point yet
            snap = snapshots.get(sysid)
            if snap is None:
                continue
            final_point = Position(lat=path[-1].lat, lon=path[-1].lon)
            if engine_haversine(snap.position, final_point) <= ARRIVAL_RADIUS_M:
                self._land_wait.add(sysid)

        if not set(active).issubset(self._land_wait):
            return  # at least one active drone hasn't arrived yet

        self._landing_started = True
        self._land_queue = {
            sysid: self._elapsed_s + i * self._land_stagger_s
            for i, sysid in enumerate(s for s in self._formation_launch_order if s in active)
        }

    def _fire_due_landings(self) -> None:
        """Send the real LAND for every queued formation drone whose
        staggered landing time has arrived - see `_check_formation_landing`.
        A no-op once the queue is empty, so safe to call every `_poll` tick
        regardless of mission kind."""
        if not self._land_queue:
            return
        due = [sysid for sysid, at in self._land_queue.items() if self._elapsed_s >= at]
        for sysid in due:
            del self._land_queue[sysid]
            self._send(sysid, CommandName.LAND)

    def _spawn_drone(self, drone: DroneConfig, home: LatLon) -> bool:
        """Bind a UDP link and spawn one drone-thread node starting at
        `home`, registering it in `self._views`. Shared by
        `start_mission_path` (every drone starts at the same point) and
        `start_mission_paths` (each drone's own route may start somewhere
        slightly different, though in practice a forest-search mission's
        lanes all launch from the same shared base)."""
        try:
            link = DroneLink(sysid=drone.sysid, gcs_address=self._gcs.address, verbose=False)
        except OSError as exc:
            self.link_failed.emit(f"{drone.name}: cannot bind SYSID {drone.sysid} ({exc})")
            return False
        controller = None
        if self.use_firmware:
            controller = SimulatedFlightController(
                sysid=drone.sysid,
                home_lat=home.lat,
                home_lon=home.lon,
                cruise_speed_mps=drone.max_velocity_mps,
                battery_capacity_mah=drone.battery_capacity_mah,
            )
        thread = self._fleet.spawn(
            drone_id=drone.sysid,
            name=drone.name,
            cruise_speed_mps=drone.max_velocity_mps,
            cruise_altitude_m=drone.cruise_altitude_m,
            battery_capacity_mah=drone.battery_capacity_mah,
            start=Position(lat=home.lat, lon=home.lon),
            time_scale=self._time_scale,
            link=link,
            flight_controller=controller,
        )
        self._views[drone.sysid] = _FlightView(drone, thread, link, home)
        return True

    def stop(self) -> None:
        self._timer.stop()
        if self._fleet is not None:
            self._fleet.shutdown()
            self._fleet = None
        if self._gcs is not None:
            self._gcs.stop()
            self._gcs = None
        self._views.clear()
        self._paths.clear()
        self._leg_index.clear()
        self._altitudes.clear()
        self._launch_queue.clear()
        self._launch_pending = []
        self._slot_wait.clear()
        self._in_slot.clear()
        self._formation_pad = None
        self._formation_launch_order = []
        self._land_wait.clear()
        self._land_queue.clear()
        self._landing_started = False

    def pause(self) -> None:
        """Hold every node in place while the controller decides."""
        self._paused = True
        self._timer.stop()
        self._broadcast(CommandName.HOLD)

    def resume(self) -> None:
        self._paused = False
        for sysid, view in self._views.items():
            if not view.is_done and sysid not in self._emergency and not view.gps_hold:
                self._send(sysid, CommandName.RESUME)
        if self._views:
            self._timer.start()

    # ---- Commands ----

    def retarget_all(self, destination: LatLon) -> int:
        active = [s for s, v in self._views.items() if not v.is_done]
        self._destination = destination
        altitude = next(iter(self._views.values())).config.cruise_altitude_m if self._views else 50.0
        for sysid in active:
            # A diversion overrides whatever route was planned - the drone now
            # flies straight to the new point, so there is no "next leg".
            self._paths.pop(sysid, None)
            self._leg_index.pop(sysid, None)
            self._send(
                sysid,
                CommandName.NAVIGATE,
                source=None,
                destination=[destination.lat, destination.lon, altitude],
            )
        if active:
            self.retargeted.emit(len(active))
        return len(active)

    def fall_back(self, sysid: int) -> Optional[EmergencyLanding]:
        view = self._views.get(sysid)
        if view is None or view.is_done:
            return None
        self._emergency.add(sysid)
        self._paths.pop(sysid, None)
        self._leg_index.pop(sysid, None)
        if view.can_reach(view.start):
            self._send(sysid, CommandName.RETURN_TO_LAUNCH)
            return EmergencyLanding(
                target=view.start,
                distance_m=engine_haversine(
                    view._snap.position, Position(lat=view.start.lat, lon=view.start.lon)
                ),
                in_place=False,
            )
        self._send(sysid, CommandName.LAND)
        return EmergencyLanding(target=view.position, distance_m=0.0, in_place=True)

    def emergency_land(self, sysid: int, candidates: list[LatLon]) -> Optional[EmergencyLanding]:
        view = self._views.get(sysid)
        if view is None or view.is_done:
            return None
        self._emergency.add(sysid)
        self._paths.pop(sysid, None)
        self._leg_index.pop(sysid, None)
        reachable = [p for p in candidates if view.can_reach(p)]
        target = nearest_point(view.position, reachable)
        if target is None:
            self._send(sysid, CommandName.LAND)
            return EmergencyLanding(target=view.position, distance_m=0.0, in_place=True)
        altitude = view.config.cruise_altitude_m
        self._send(
            sysid, CommandName.NAVIGATE, destination=[target.lat, target.lon, altitude]
        )
        return EmergencyLanding(
            target=target,
            distance_m=engine_haversine(
                view._snap.position, Position(lat=target.lat, lon=target.lon)
            ),
            in_place=False,
        )

    def inject_fault(self, command: InjectFault) -> list[int]:
        """GPS loss is injected through the command link, as the nodes are
        simulated. Other fault types are not modelled here."""
        if command.fault_type is not FaultType.GPS_LOSS:
            return []
        affected = []
        for sysid in command.sysids:
            if sysid in self._views and not self._views[sysid].is_done:
                self._send(sysid, CommandName.SET_GPS, available=False)
                affected.append(sysid)
        return affected

    def clear_faults(self, sysids: Optional[list[int]] = None) -> list[int]:
        targets = sysids if sysids is not None else list(self._views)
        restored = []
        for sysid in targets:
            if sysid in self._views:
                self._send(sysid, CommandName.SET_GPS, available=True)
                restored.append(sysid)
        return restored

    def preflight_check(self, drones: list[DroneConfig], start: LatLon, destination: LatLon):
        flights = plan_flights(drones, start, destination)
        return [f for f in flights if f.feasible], [f for f in flights if not f.feasible]

    # ---- Internals ----

    def _send(self, sysid: int, command: CommandName, **kwargs) -> None:
        if self._gcs is not None:
            self._gcs.send(sysid, command, **kwargs)

    def _broadcast(self, command: CommandName, **kwargs) -> None:
        for sysid, view in self._views.items():
            if not view.is_done:
                self._send(sysid, command, **kwargs)

    def _poll(self) -> None:
        """Pull the latest snapshot from each node and publish one batch."""
        if self._fleet is None:
            return
        self._tick += 1
        self._elapsed_s += POLL_MS / 1000.0 * self._time_scale
        self._fire_due_launches()
        self._fire_due_landings()

        snapshots = self._fleet.latest()
        if not snapshots:
            return

        self._check_formation_assembly(snapshots)
        self._check_formation_landing(snapshots)

        drones = [self._telemetry_of(s) for s in snapshots.values()]
        self.batch_ready.emit(
            SwarmTelemetryBatch(tick=self._tick, timestamp=self._elapsed_s, drones=drones)
        )

        remaining = 0.0
        for sysid, view in self._views.items():
            if view.is_done or self._destination is None:
                continue
            snap = view._snap
            target = Position(lat=self._destination.lat, lon=self._destination.lon)
            remaining = max(
                remaining,
                engine_haversine(snap.position, target) / max(0.1, view.cruise_speed_mps),
            )
        self.progress.emit(self._elapsed_s, self._elapsed_s + remaining)

        if all(v.is_done for v in self._views.values()):
            self._timer.stop()
            self.finished.emit(self._elapsed_s)
            return

        self._advance_legs(snapshots)
        self._check_exceptions(snapshots)

    def _advance_legs(self, snapshots: dict[int, DroneSnapshot]) -> None:
        """Chain a PDDL-planned route: as soon as a drone reaches the
        waypoint it is currently headed for, send it on to the next one
        instead of letting it land there - only the final waypoint is
        allowed to trigger a landing.
        """
        for sysid, snap in snapshots.items():
            if sysid in self._launch_queue:
                continue  # staggered launch: hasn't been sent its first NAVIGATE yet
            if sysid in self._slot_wait:
                continue  # climbing out to / holding in its slot, waiting for the rest
            path = self._paths.get(sysid)
            if not path:
                continue
            leg = self._leg_index.get(sysid, len(path) - 1)
            if leg >= len(path) - 1:
                continue  # already headed for the final destination
            target = Position(lat=path[leg].lat, lon=path[leg].lon)
            if engine_haversine(snap.position, target) > ARRIVAL_RADIUS_M:
                continue
            leg += 1
            self._leg_index[sysid] = leg
            altitudes = self._altitudes.get(sysid)
            if altitudes and leg < len(altitudes):
                altitude = altitudes[leg]
            else:
                altitude = self._views[sysid].config.cruise_altitude_m if sysid in self._views else 50.0
            is_formation = sysid in self._formation_launch_order
            self._send(
                sysid, CommandName.NAVIGATE,
                destination=[path[leg].lat, path[leg].lon, altitude],
                # A formation drone always arrives and holds, even on its
                # last leg - see the matching override in
                # `_check_formation_assembly` and `_check_formation_landing`.
                final=(leg >= len(path) - 1) and not is_formation,
            )

    def _check_exceptions(self, snapshots: dict[int, DroneSnapshot]) -> None:
        """Surface the first situation needing a decision, and hold the swarm."""
        for sysid, snap in snapshots.items():
            if snap.mission_state is MissionState.FAILED and sysid not in self._flagged_lost:
                self._flagged_lost.add(sysid)
                self.drone_lost.emit(sysid)
                return

            if not snap.gps_available and sysid not in self._flagged_gps:
                self._flagged_gps.add(sysid)
                self._flagged_low.discard(sysid)  # a hold is a new situation
                self.pause()
                self.gps_lost.emit(sysid)
                return

            if snap.gps_available and sysid in self._flagged_gps:
                self._flagged_gps.discard(sysid)
                self.gps_restored.emit(sysid)
                return

            if (
                snap.battery <= LOW_BATTERY_PCT
                and sysid not in self._flagged_low
                and sysid not in self._emergency
                and snap.mission_state not in (MissionState.LANDED, MissionState.FAILED)
            ):
                self._flagged_low.add(sysid)
                self.pause()
                self.low_battery.emit(sysid, snap.battery)
                return

    def _telemetry_of(self, snap: DroneSnapshot) -> DroneTelemetry:
        faults: list[FaultType] = []
        if not snap.gps_available:
            faults.append(FaultType.GPS_LOSS)
        if snap.health_status is HealthStatus.FAILED:
            faults.append(FaultType.BATTERY_FAIL)

        return DroneTelemetry(
            sysid=snap.drone_id,
            status=_STATUS_MAP.get(snap.mission_state, DroneStatus.STANDBY),
            lat=snap.position.lat,
            lon=snap.position.lon,
            altitude_m=snap.position.alt_m,
            heading_deg=snap.velocity.heading_deg,
            ground_speed_mps=snap.velocity.ground_speed_mps,
            roll_deg=0.0,
            pitch_deg=-5.0 if snap.mission_state is MissionState.EN_ROUTE else 0.0,
            battery_pct=max(0.0, min(100.0, snap.battery)),
            link_quality_pct=100.0 if snap.communication_status.value == "ONLINE" else 40.0,
            active_faults=faults,
            raw_mavlink=(
                f"[{snap.drone_id}] {snap.mission_state.value} "
                f"pos=({snap.position.lat:.6f},{snap.position.lon:.6f},{snap.position.alt_m:.1f}) "
                f"batt={snap.battery:.1f}% {snap.health_status.value}"
                + (f" | {snap.note}" if snap.note else "")
            ),
        )
