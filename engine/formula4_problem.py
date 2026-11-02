"""Formula-4: four drones swapping places in flight, each by its own formula.

    picked source + destination
        -> plan_formula4: every drone's 3-D track from ONE closed-form
           formula of a shared clock - no drone ever needs another's position
        -> Swarm-Planning/Plans/Formula-4/problem.pddl (template) -> retarget_formula4_problem
        -> ENHSP plans each drone on its own: take off, every swap it flies,
           arrive at its destination slot, land
        -> every drone flies its own timed track (engine.train_keeping,
           schedule mode) against the one clock

THE FORMATION. Drones 1 and 2 start in the FRONT line, 3 and 4 in the BACK
line, `LINE_GAP_M` behind them. Drones 1 and 3 share the LEFT lane, 2 and 4
the RIGHT lane, `LANE_GAP_M` apart. All the way to the destination, 1 and 3
keep swapping places, and so do 2 and 4: the front drone slows and dips to
50 m while the back drone speeds up, climbs to 60 m and passes over it.

THE FORMULA. In route metres - x along the route from the source, y across
it (left positive), z height - with clock time tau (one unit = one nominal
second, see `SCHEDULE_UNIT_MPS`), for drone i:

    x_i(tau) = V * tau  +  f_i * (D/2) * cos(w * tau)
    y_i(tau) = l_i * ( W/2  +  f_i * (B/2) * sin(w * tau) )
    z_i(tau) = Z0  -  f_i * A * sin(w * tau)

    f_i = +1 front (drones 1, 2), -1 back (drones 3, 4)
    l_i = +1 left lane (drones 1, 3), -1 right lane (drones 2, 4)

V the formation's speed, D the front-back gap, W the lane gap, Z0 = 55 m
the middle height, A = 5 m the height swing (so 50 m .. 60 m), B a small
sideways step apart while passing (extra clearance), w the swap rate.

    speed along the route   x' = V - f * (D/2) * w * sin(w tau)
        first half turn (sin > 0): the FRONT drone flies V(1-k) and dips to
        Z0 - A = 50 m; the BACK drone flies V(1+k) and climbs to 60 m and
        passes. Next half turn the other way round. k = (D/2) w / V < 1, so
        no drone ever stops or flies backwards.

SEPARATION, from the formula itself, at every tau:

    1 & 3 (and 2 & 4): dx = D cos, dy = B sin, dz = 2A sin
                       d^2 = D^2 cos^2 + (B^2 + 4A^2) sin^2
                       >= min(D, sqrt(B^2 + 4A^2))  = min(20, 11.66) m
    1 & 2 (and 3 & 4): dx = dz = 0, dy = W +- B sin  >= W - B = 14 m
    1 & 4 (and 2 & 3): dy = W exactly                = 20 m

So every pair is always >= 11.66 m apart - more than the 10 m asked for.
Nothing in it depends on where another drone actually is: that is why the
drones need no communication with each other, only the same clock.

THE ENDS. w is chosen so the route holds a whole, EVEN number of swaps:
w * T = N * pi with T = L / V. At tau = 0 and tau = T, sin = 0 and cos = 1,
so every drone is at Z0 on its own slot - its own source / destination
point (the formula at those times) - and drones 1, 2 are in front again.
They take off and land vertically on those slots, at least 20 m apart.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from engine.figure8_problem import TrainRoute
from engine.pddl_problem import LatLon, _from_local, _set_latlon, _to_local
from engine.star_problem import _set_scalar_fluent

DRONE_NAMES = ("drone1", "drone2", "drone3", "drone4")
FRONT = {"drone1": 1, "drone2": 1, "drone3": -1, "drone4": -1}     # f_i
LANE = {"drone1": 1, "drone2": -1, "drone3": 1, "drone4": -1}      # l_i

FORMATION_SPEED = 6.0       # V, metres per clock unit (60% of full cruise speed)
LINE_GAP_M = 20.0           # D, front line to back line
LANE_GAP_M = 20.0           # W, left lane to right lane
SIDE_STEP_M = 6.0           # B, how far a passing pair steps apart sideways
MID_ALT_M = 55.0            # Z0
ALT_SWING_M = 5.0           # A: the slow drone at 50 m, the passing one at 60 m
MAX_SPEED_RATIO = 0.5       # k at most: passing drone 1.5 V, slow drone 0.5 V
MIN_SEPARATION_M = 10.0
SAMPLES_PER_SWAP = 10       # track points per swap (half turn of w)
# A drone uploads its whole track to its autopilot within one tick, at most
# ~10 000 mission messages (engine.drone_thread._service_firmware) - so stay below.
MAX_TRACK_POINTS = 9_000


def min_route_m() -> float:
    """Shortest route that holds two swaps (one full turn) at k <= MAX_SPEED_RATIO."""
    w_max = 2.0 * MAX_SPEED_RATIO * FORMATION_SPEED / LINE_GAP_M
    return 2.0 * math.pi * FORMATION_SPEED / w_max


def formula_min_separation_m() -> float:
    """The smallest gap the formula ever allows between any two drones."""
    return min(
        LINE_GAP_M,
        math.hypot(SIDE_STEP_M, 2.0 * ALT_SWING_M),
        LANE_GAP_M - SIDE_STEP_M,
        LANE_GAP_M,
    )


def formula_position(name: str, tau: float, v: float, w: float) -> tuple[float, float, float]:
    """(x along, y across, z height) of drone `name` at clock time `tau` -
    THE formula (see the module doc)."""
    f, l = FRONT[name], LANE[name]
    s, c = math.sin(w * tau), math.cos(w * tau)
    x = v * tau + f * (LINE_GAP_M / 2.0) * c
    y = l * (LANE_GAP_M / 2.0 + f * (SIDE_STEP_M / 2.0) * s)
    z = MID_ALT_M - f * ALT_SWING_M * s
    return x, y, z


@dataclass(frozen=True)
class Formula4Mission:
    route_m: float                     # L, source to destination
    duration: float                    # T, clock units
    omega: float                       # w, radians per clock unit
    swaps: int                         # N, swaps each pair flies (even)
    speed_ratio: float                 # k
    sources: dict[str, LatLon]         # each drone's own take-off slot
    destinations: dict[str, LatLon]    # each drone's own landing slot
    routes: dict[str, TrainRoute]      # each drone's timed 3-D track
    altitudes: dict[str, list[float]]  # z of each track point, aligned with route.expanded()
    min_separation_m: float            # smallest gap between any two drones on the flown tracks
    source: LatLon                     # the picked points: the formation's centre at
    destination: LatLon                # the start and the end


def formula4_distance_error(source: LatLon, destination: LatLon) -> str | None:
    length = math.hypot(*_to_local(destination, source))
    if length < min_route_m():
        return (
            f"The destination is only {length:.0f} m from the source. Formula-4 needs at "
            f"least {min_route_m():.0f} m - room for one full swap and back - so the "
            f"swapping drones never have to fly more than {1 + MAX_SPEED_RATIO:.1f}x the "
            f"formation's speed."
        )
    return None


def plan_formula4(source: LatLon, destination: LatLon) -> Formula4Mission:
    """Every drone's timed track from source to destination - see the
    module doc. Raises ValueError if the route is too short or too long."""
    problem = formula4_distance_error(source, destination)
    if problem:
        raise ValueError(problem)
    ex, ny = _to_local(destination, source)
    length = math.hypot(ex, ny)
    ux, uy = ex / length, ny / length          # along the route
    nx, ny_ = -uy, ux                          # across it, to the left

    v = FORMATION_SPEED
    w_max = 2.0 * MAX_SPEED_RATIO * v / LINE_GAP_M
    # The most swaps that keep k <= MAX_SPEED_RATIO, rounded down to an even
    # number so the drones finish in the line they started in.
    swaps = max(2, 2 * int(length * w_max / (2.0 * math.pi * v)))
    duration = length / v
    w = swaps * math.pi / duration
    k = (LINE_GAP_M / 2.0) * w / v

    def to_map(x: float, y: float) -> LatLon:
        return _from_local((x * ux + y * nx, x * uy + y * ny_), source)

    steps = swaps * SAMPLES_PER_SWAP
    taus = [duration * i / steps for i in range(steps + 1)]
    routes, altitudes, tracks = {}, {}, {}
    sources, destinations = {}, {}
    for name in DRONE_NAMES:
        pts = [formula_position(name, tau, v, w) for tau in taus]
        tracks[name] = pts
        latlons = [to_map(x, y) for x, y, _z in pts]
        sources[name], destinations[name] = latlons[0], latlons[-1]
        # Timetable: at clock tau_i be s_i metres along my own (ground) track.
        s, keys = 0.0, [(0.0, 0.0)]
        for i in range(1, len(pts)):
            s += math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
            keys.append((taus[i], s))
        loop = latlons[1:-1]
        routes[name] = TrainRoute(
            start=latlons[0],
            approach=[],
            loop=loop,
            laps=1,
            depart=[latlons[-1]],
            schedule=tuple(keys),
            # Heights relative to Z0 - the flight layer flies the track at
            # Z0 (the per-drone altitude offset) plus these.
            loop_alt_offsets=tuple(z - MID_ALT_M for _x, _y, z in pts[1:-1]),
        )
        altitudes[name] = [z for _x, _y, z in pts]
        if len(latlons) > MAX_TRACK_POINTS:
            raise ValueError(
                f"The route is {length / 1000:.1f} km - too long for Formula-4: its "
                f"{swaps} swaps need {len(latlons)} track points per drone, more than the "
                f"{MAX_TRACK_POINTS} a drone's autopilot takes. Pick a destination closer "
                f"to the source."
            )

    # The tracks as flown: straight lines between points, every drone
    # passing its i-th point at the same clock time - so check the gaps
    # between those lines, finely, not only at the points.
    closest = math.inf
    names = list(DRONE_NAMES)
    for i in range(len(taus) - 1):
        for frac in (0.0, 0.25, 0.5, 0.75):
            at = {
                n: tuple(a + (b - a) * frac for a, b in zip(tracks[n][i], tracks[n][i + 1]))
                for n in names
            }
            for a in range(4):
                for b in range(a + 1, 4):
                    closest = min(closest, math.dist(at[names[a]], at[names[b]]))
    if closest < MIN_SEPARATION_M:
        raise ValueError(f"Formula-4's tracks come within {closest:.1f} m of each other.")

    return Formula4Mission(
        route_m=length, duration=duration, omega=w, swaps=swaps, speed_ratio=k,
        sources=sources, destinations=destinations, routes=routes,
        altitudes=altitudes, min_separation_m=closest,
        source=source, destination=destination,
    )


# ---- PDDL ---------------------------------------------------------------------


def _slot(kind: str, name: str) -> str:
    return f"{kind}-{name.removeprefix('drone')}"   # src-1, dst-3, ...


def retarget_formula4_problem(problem_text: str, mission: Formula4Mission) -> str:
    """Put the template's eight slots (each drone's own source and
    destination) on the mission's points, and fill in the formula's numbers."""
    result = problem_text
    for name in DRONE_NAMES:
        result = _set_latlon(result, _slot("src", name), mission.sources[name])
        result = _set_latlon(result, _slot("dst", name), mission.destinations[name])
    for fluent, value in (
        ("formation-speed", FORMATION_SPEED),
        ("line-gap", LINE_GAP_M),
        ("lane-gap", LANE_GAP_M),
        ("side-step", SIDE_STEP_M),
        ("mid-altitude", MID_ALT_M),
        ("altitude-swing", ALT_SWING_M),
        ("swap-rate", mission.omega),
        ("route-length", mission.route_m),
        ("swaps-total", mission.swaps),
        ("min-separation", MIN_SEPARATION_M),
    ):
        result = _set_scalar_fluent(result, fluent, value)
    return result


def plan_swaps(steps) -> dict[str, list[str]]:
    """Each drone's own sequence of actions, in plan order."""
    out: dict[str, list[str]] = {name: [] for name in DRONE_NAMES}
    for step in sorted(steps, key=lambda s: s.index):
        if step.args and step.args[0] in out:
            out[step.args[0]].append(step.action)
    return out


def expected_actions(name: str, swaps: int) -> list[str]:
    """What each drone's own plan must be: take off, then alternately slow
    and dip / speed up and climb (front drones slow first), arrive, land."""
    first, second = ("slow-and-dip", "speed-up-and-climb")
    if FRONT[name] < 0:
        first, second = second, first
    return ["take-off", *([first, second] * (swaps // 2)), "arrive", "land"]

