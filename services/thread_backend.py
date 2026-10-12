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

import math
import time
from dataclasses import dataclass, field
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
from engine.train_keeping import SCHEDULE_UNIT_MPS, RouteTracker, schedule_at
from services.local_flight import EmergencyLanding, nearest_point, plan_flights

POLL_MS = 100

# Train formation keeping (see ThreadSwarmBackend._keep_train_spacing).
TRAIN_CRUISE_FRACTION = 0.8   # train speed, as a fraction of its slowest drone's top speed
TRAIN_MAX_BOOST = 1.3         # fastest a drone may go to catch up, x the train speed
TRAIN_MIN_MPS = 0.1           # "wait here" - creeping rather than hovering dead still
TRAIN_STRAGGLER_SLACK = 1.5   # spacings a drone may lag before the whole train waits for it
TRAIN_PACE_ACCEL = 1.0        # m/s^2 (simulated) - the pace speeds up and slows down gently
# Design speed of timed formation tracks (engine.star_problem).
TRAIN_SCHEDULE_UNIT_MPS = SCHEDULE_UNIT_MPS
# Train/formation drones tick often enough that one tick is at most this
# much simulated time - at the default 20x time scale a 0.1 s tick is ~25 m
# of flight, far too coarse to hold a 10 m gap (or to look like a smooth
# line on the map). At slower time scales the ordinary 0.1 s tick is enough.
TRAIN_TICK_SIM_S = 0.5


def _local_xy(point, origin) -> tuple[float, float]:
    """(east, north) metres of `point` from `origin` - locally flat."""
    north = (point.lat - origin.lat) * 111_320.0
    east = (point.lon - origin.lon) * 111_320.0 * math.cos(math.radians(origin.lat))
    return east, north


@dataclass
class _TrainState:
    """A train formation in flight. Every drone flies the same SHARED path
    (base -> 8 -> base) between its own way off its pad and its own way onto
    its landing pad. Positions are compared in shared-path metres: a
    drone's progress along its own route minus its private lead-in
    (`shared_from_m`), so 0 is the base point on the way out.

    `pace_m` is the GCS's single "where the front of the train should be"
    on the shared path; drone k belongs `slot_m[k]` behind it.

    A formation flying timed tracks instead (the star plan) gives each
    drone a `schedules` timetable; `pace_m` is then a shared clock in
    nominal seconds, advancing `pace_unit_mps` times slower than a pace in
    metres would (see engine.train_keeping)."""

    order: list[int]                         # sysids, front of the train first
    origin: object                           # LatLon the tracks' local metres are measured from
    tracks: dict[int, RouteTracker]          # sysid -> its route and progress on it
    shared_from_m: dict[int, float]          # sysid -> length of its own pad-to-base lead-in
    slot_m: dict[int, float]                 # sysid -> its place behind the front of the train
    spacing_m: float                         # nominal gap between neighbours
    cruise_mps: float                        # the speed the whole train flies
    total_s: float                           # estimated mission length, for progress
    pace_m: float = 0.0
    pace_mps: float = 0.0                    # the pace's speed as last broadcast
    pace_wall_s: Optional[float] = None
    schedules: dict[int, tuple] = field(default_factory=dict)
    pace_unit_mps: float = 1.0               # metres per unit of pace (1 for a pace in metres)

    def end_m(self, sysid: int) -> float:
        """Shared-path metres at which this drone's own route ends."""
        return self.tracks[sysid].cumulative_m[-1] - self.shared_from_m[sysid]

    def target_m(self, sysid: int) -> float:
        """Where the pace says this drone should be right now."""
        schedule = self.schedules.get(sysid)
        if schedule:
            return schedule_at(schedule, self.pace_m)[0]
        return self.pace_m - self.slot_m[sysid]

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
        self._train: Optional[_TrainState] = None
        self._time_scale = time_scale
        self._default_time_scale = time_scale
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

    def start_mission_paths(
        self,
        assignments: list[tuple[DroneConfig, list[LatLon]]],
        altitude_offsets_m: Optional[list[float]] = None,
        train_routes: Optional[list] = None,
        time_scale: Optional[float] = None,
    ) -> bool:
        """Like `start_mission_path`, but each drone flies its own distinct
        route rather than everyone sharing one - the forest-search mission's
        per-drone lane, where drone1 and drone2 are planned completely
        separate routes that just happen to both start/end at the same base
        point.

        `altitude_offsets_m`, if given, is one extra height in metres per
        assignment, added to every leg of that drone's terrain-derived cruise
        profile. It exists for the 3-D grid ("cube") formation, whose layers
        are separated VERTICALLY: two drones in the same row and column of
        different layers fly the *same* lateral track, so without an offset
        `plan_terrain_profile` would put them at the same altitude and they
        would occupy the same points in space. Every other plan separates its
        drones laterally and passes `None`, keeping one shared profile.

        `_advance_legs`/`_poll` already key every piece of chained-route
        state (`_paths`, `_altitudes`, `_leg_index`) by sysid, so nothing
        below this needs to know or care that the routes - or now the
        altitudes - differ per drone; only the setup here (each drone gets
        its own terrain profile, home point and first NAVIGATE) is genuinely
        different from `start_mission_path`.

        `train_routes`, if given, flies the assignments as a TRAIN (the
        figure-8 formation): the assignments are in front-to-back order,
        each assignment's waypoints are its whole route written out, and
        `train_routes` holds the same route per assignment structured as
        `start -> approach -> loop x laps -> depart` (see
        `engine.figure8_problem.TrainRoute`). Then

        - each drone gets its WHOLE route as one NAVIGATE - uploaded to its
          flight controller as a single mission with a DO_JUMP repeating the
          loop `laps` times - so it flies through every waypoint without
          stopping, instead of holding at each one until `_advance_legs`
          sends the next leg;
        - the GCS keeps one shared pace along the path (the slowest drone's
          top speed times TRAIN_CRUISE_FRACTION, leaving headroom to catch
          up) and broadcasts it (`_keep_train_spacing`); every drone holds
          its own place behind it ON BOARD, every tick
          (`engine.train_keeping`), braking for the drone in front, while
          the pace waits for any drone that has fallen behind.

        `time_scale`, if given, runs this mission at that many times real
        time instead of the backend's default (20x) - a drone show watched
        at 20x is over in seconds.
        """
        self.stop()
        if not assignments:
            return False
        if time_scale:
            # This mission only - e.g. a drone show played closer to real
            # time; `stop()` puts the default back.
            self._time_scale = float(time_scale)

        self._gcs = GroundControlStation(verbose=False)
        self._gcs.start()
        self._fleet = DroneFleet()
        # Progress/finished reporting still tracks a single "how much
        # further to go" figure - the shared base every lane ends at, since
        # that's what remains once a drone reaches the end of its own route.
        self._destination = assignments[0][1][-1]

        offsets = list(altitude_offsets_m or [])
        per_drone: dict[int, tuple[list[LatLon], list[float]]] = {}
        for i, (drone, waypoints) in enumerate(assignments):
            if len(waypoints) < 2:
                continue
            points, altitudes = plan_terrain_profile(waypoints)
            lift = offsets[i] if i < len(offsets) else 0.0
            if lift:
                altitudes = [a + lift for a in altitudes]
            tick_s = min(0.1, TRAIN_TICK_SIM_S / self._time_scale) if train_routes else None
            if not self._spawn_drone(drone, points[0], tick_s=tick_s):
                continue
            per_drone[drone.sysid] = (points, altitudes)

        if not self._views:
            self.stop()
            return False

        self._gcs.register_many(self._views)
        self._gcs.wait_for_heartbeats(self._views, timeout_s=3.0)

        if train_routes:
            self._start_train(assignments, per_drone, train_routes)
        else:
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

    def _start_train(
        self,
        assignments: list[tuple[DroneConfig, list[LatLon]]],
        per_drone: dict[int, tuple[list[LatLon], list[float]]],
        train_routes: list,
    ) -> None:
        """Launch a train formation - see `start_mission_paths`' `train_routes`."""
        order = [d.sysid for d, _r in assignments if d.sysid in self._views]
        configs = {d.sysid: d for d, _r in assignments}
        cruise = TRAIN_CRUISE_FRACTION * min(configs[s].max_velocity_mps for s in order)
        origin = per_drone[order[0]][0][0]
        tracks = {
            sysid: RouteTracker([_local_xy(p, origin) for p in per_drone[sysid][0]])
            for sysid in order
        }
        # The nominal gap: how far apart the first two drones start.
        spacing = 10.0
        if len(order) > 1:
            a, b = per_drone[order[0]][0][0], per_drone[order[1]][0][0]
            spacing = max(1.0, engine_haversine(
                Position(lat=a.lat, lon=a.lon), Position(lat=b.lat, lon=b.lon)))

        route_of = {d.sysid: r for (d, _w), r in zip(assignments, train_routes)}
        schedules = {s: tuple(route_of[s].schedule) for s in order if getattr(route_of[s], "schedule", ())}
        if schedules:
            # Timed tracks: the pace is a clock in nominal seconds, each
            # nominal second being TRAIN_SCHEDULE_UNIT_MPS metres of the
            # tracks' own design speed - so the clock runs as fast as this
            # swarm's cruise speed allows.
            unit = TRAIN_SCHEDULE_UNIT_MPS
            end = max(sched[-1][0] for sched in schedules.values())
            self._train = _TrainState(
                order=order, origin=origin, tracks=tracks,
                shared_from_m={s: 0.0 for s in order}, slot_m={s: 0.0 for s in order},
                spacing_m=spacing, cruise_mps=cruise / unit,
                total_s=end / max(1e-3, cruise / unit),
                pace_m=0.0, schedules=schedules, pace_unit_mps=unit,
            )
        else:
            self._train = _TrainState(
                order=order,
                origin=origin,
                tracks=tracks,
                shared_from_m={s: route_of[s].shared_from_m for s in order},
                slot_m={s: route_of[s].slot_m for s in order},
                spacing_m=spacing,
                cruise_mps=cruise,
                total_s=tracks[order[0]].cumulative_m[-1] / max(0.1, cruise),
                # The front of the train starts where the leader's pad is.
                pace_m=-route_of[order[0]].shared_from_m,
            )
        train = self._train
        for sysid in order:
            points, altitudes = per_drone[sysid]
            route = route_of[sysid]
            altitude = max(altitudes)

            def wire(seq) -> list[list[float]]:
                return [[p.lat, p.lon, altitude] for p in seq]

            self._paths[sysid] = list(points)
            self._altitudes[sysid] = list(altitudes)
            # Nothing for _advance_legs to chain: the drone has the whole route.
            self._leg_index[sysid] = len(points) - 1
            landing = route.depart[-1]
            self._gcs.navigate(
                sysid,
                [route.start.lat, route.start.lon, 0.0],
                [landing.lat, landing.lon, altitude],
                final=True,
                approach=wire(route.approach),
                loop=wire(route.loop),
                laps=route.laps,
                depart=wire(route.depart[:-1]),
                train={
                    "shared_from_m": route.shared_from_m,
                    "slot_m": route.slot_m,
                    "spacing_m": spacing,
                    "max_mps": min(configs[sysid].max_velocity_mps, TRAIN_MAX_BOOST * cruise),
                    "min_mps": TRAIN_MIN_MPS,
                    "schedule": [list(k) for k in schedules.get(sysid, ())],
                },
            )
        # A first, standing-still pace, so every drone knows from its very
        # first tick in the air whether it is its turn to go yet.
        self._broadcast_pace({}, 0.0)

    def _keep_train_spacing(self, snapshots: dict[int, DroneSnapshot]) -> None:
        """The ground station's half of train formation keeping.

        It advances ONE shared pace along the path - where the front of the
        train should be - at the train's cruise speed and broadcasts it,
        time-stamped, every poll; each drone then holds its own place behind
        it on board (`DroneThread.keep_train_place`). The neighbours feed
        in through the ground station:

        - FRONT: each drone's pace message carries where the drone in front
          of it is, for its on-board safety brake;
        - BACK: if any drone has fallen more than TRAIN_STRAGGLER_SLACK
          spacings behind its place, the pace slows - to a stop if need be -
          so the whole train waits for it instead of stretching away.

        Positions are measured ALONG THE ROUTE and brought forward to one
        common instant using each snapshot's tick time. The pace only
        starts once every drone has climbed and is ready to move.
        """
        train = self._train
        now = time.monotonic()
        at_m: dict[int, float] = {}
        waiting_for_climb = False
        for sysid in train.order:
            snap = snapshots.get(sysid)
            if snap is None:
                continue
            if snap.mission_state in (MissionState.IDLE, MissionState.ARMED, MissionState.TAKEOFF):
                waiting_for_climb = True
            if snap.mission_state is not MissionState.EN_ROUTE:
                continue
            along = train.tracks[sysid].progress_m(_local_xy(snap.position, train.origin))
            if snap.wall_s:
                along += snap.velocity.ground_speed_mps * max(0.0, now - snap.wall_s) * self._time_scale
            at_m[sysid] = along - train.shared_from_m[sysid]

        # BACK: how far the worst straggler is behind its place - ignoring
        # drones already at the end of their own route (landing on a pad).
        worst_lag = 0.0
        for sysid, here in at_m.items():
            if here >= train.end_m(sysid) - train.spacing_m:
                continue
            worst_lag = max(worst_lag, train.target_m(sysid) - here)
        excess = (worst_lag - TRAIN_STRAGGLER_SLACK * train.spacing_m) / (2.0 * train.spacing_m)
        wanted_mps = train.cruise_mps * min(1.0, max(0.0, 1.0 - excess))
        if waiting_for_climb:
            wanted_mps = 0.0

        # Move the pace on by the speed the drones were TOLD it had since
        # the last poll - they have been flying to that - and only then
        # change its speed, gently: a pace that jumps leaves several drones
        # behind their places at once, all catching up flat out together,
        # which is how two of them reach a merge point at the same moment.
        sim_dt = 0.0 if train.pace_wall_s is None else (now - train.pace_wall_s) * self._time_scale
        train.pace_m += train.pace_mps * sim_dt
        train.pace_wall_s = now
        step = TRAIN_PACE_ACCEL / train.pace_unit_mps * max(sim_dt, POLL_MS / 1000.0 * self._time_scale)
        train.pace_mps = min(train.pace_mps + step, max(train.pace_mps - step, wanted_mps))
        self._broadcast_pace(at_m, train.pace_mps)

    def _broadcast_pace(self, at_m: dict[int, float], pace_mps: float) -> None:
        """Send every drone the pace, plus where the drone in front of it is."""
        train = self._train
        stamp = time.time()
        for i, sysid in enumerate(train.order):
            front = train.order[i - 1] if i > 0 else None
            self._send(
                sysid, CommandName.TRAIN_PACE,
                at_m=train.pace_m, mps=pace_mps, stamp_s=stamp,
                # A drone in front only means something in a line.
                front_at_m=None if train.schedules else at_m.get(front),
            )

    def _spawn_drone(self, drone: DroneConfig, home: LatLon, tick_s: Optional[float] = None) -> bool:
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
            **({"tick_s": tick_s} if tick_s else {}),
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
        self._train = None
        self._time_scale = self._default_time_scale

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

        snapshots = self._fleet.latest()
        if not snapshots:
            return

        drones = [self._telemetry_of(s) for s in snapshots.values()]
        self.batch_ready.emit(
            SwarmTelemetryBatch(tick=self._tick, timestamp=self._elapsed_s, drones=drones)
        )

        remaining = 0.0
        if self._train is not None:
            # A looping route ends where it began, so "distance to the
            # destination" says nothing - use the planned flying time.
            remaining = max(0.0, self._train.total_s - self._elapsed_s)
        for sysid, view in ([] if self._train is not None else self._views.items()):
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
        if self._train is not None and not self._paused:
            self._keep_train_spacing(snapshots)
        self._check_exceptions(snapshots)

    def _advance_legs(self, snapshots: dict[int, DroneSnapshot]) -> None:
        """Chain a PDDL-planned route: as soon as a drone reaches the
        waypoint it is currently headed for, send it on to the next one
        instead of letting it land there - only the final waypoint is
        allowed to trigger a landing.
        """
        for sysid, snap in snapshots.items():
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
            self._send(
                sysid, CommandName.NAVIGATE,
                destination=[path[leg].lat, path[leg].lon, altitude],
                final=(leg >= len(path) - 1),
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
