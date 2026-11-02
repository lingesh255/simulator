"""Formation-Circle: four drones on a turning vertical circle - a spiral -
each by its own formula, with no communication between them.

    picked source + destination
        -> plan_circle_formation: every drone's 3-D track from ONE
           closed-form formula of a shared clock
        -> Swarm-Planning/Plans/Formation-Circle/problem.pddl (template)
           -> retarget_circle_problem
        -> ENHSP plans each drone on its own: take off, join the ring, every
           quarter turn it flies, leave the ring, land
        -> every drone flies its own timed track (engine.train_keeping,
           schedule mode) against the one clock

THE FORMATION. Seen from behind (flying into the page), a circle standing
up across the route, radius R = 10 m, centre Z0 = 50 m high:

                 top (60 m)
    left (50 m)              right (50 m)
                bottom (40 m)

Drone 1 starts at the bottom, drone 2 on the left, drone 3 at the top,
drone 4 on the right. While the circle flies to the destination it TURNS:
every quarter turn each drone moves on one place - bottom -> left -> top ->
right -> bottom - so each drone flies a spiral (a helix) round the route.

THE FORMULA. In route metres - x along the route from the source, y across
it (left positive), z height - with clock time tau (one unit = one nominal
second, see `SCHEDULE_UNIT_MPS`), drone i at angle a_i round the circle
(a = 0 bottom, pi/2 left, pi top, 3pi/2 right):

    a_i(tau) = a_i0 + w * (tau - Tj)          (while spiralling)
    x_i(tau) = V * tau
    y_i(tau) = R * sin(a_i(tau))
    z_i(tau) = Z0 - R * cos(a_i(tau))

    a_10 = 0, a_20 = pi/2, a_30 = pi, a_40 = 3pi/2

SEPARATION while spiralling: every drone has the same x, and all four turn
rigidly together, 90 degrees apart - so the gaps never change:
    neighbours (e.g. 1 & 2):  R * sqrt(2) = 14.14 m
    opposite   (e.g. 1 & 3):  2R          = 20 m
Nothing in it depends on where another drone actually is - that is why the
drones need no communication with each other, only the same clock.

TAKE-OFF AND LANDING. Bottom and top are above the same ground point, so
they can't take off there. The pads are a diamond instead: each drone's pad
is E * cos(a_i0) ahead of its place on the circle - bottom drone E = 10 m
ahead, top drone 10 m behind, left/right on their sides - and it climbs
straight up there to its own height (40 / 50 / 60 / 50 m). Over the first
JOIN_M metres the diamond closes into the circle (no turning yet):

    x_i(tau) = V * tau  +  E cos(a_i0) * (1 - b(tau / Tj)),  b(u) = (1 - cos(pi u)) / 2

and over the last JOIN_M it opens out again onto the destination pads. In
those phases bottom-top are >= 2R = 20 m apart (dz alone) and every other
pair >= sqrt(R^2 + R^2) = 14.14 m - the same minimum as the spiral. The
circle makes a whole number of full TURNS, so every drone ends where it
started on the circle and lands on the matching destination pad.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from engine.figure8_problem import TrainRoute
from engine.pddl_problem import LatLon, _from_local, _set_latlon, _to_local
from engine.star_problem import _set_scalar_fluent
from engine.train_keeping import SCHEDULE_UNIT_MPS

DRONE_NAMES = ("drone1", "drone2", "drone3", "drone4")
# Each drone's starting place on the circle - its angle a_i0.
START_SLOT = {"drone1": "bottom", "drone2": "left", "drone3": "top", "drone4": "right"}
SLOT_ANGLE = {"bottom": 0.0, "left": math.pi / 2, "top": math.pi, "right": 3 * math.pi / 2}
SLOT_ORDER = ("bottom", "left", "top", "right")   # the way round a quarter turn moves a drone

FORMATION_SPEED = 6.0      # V, metres along the route per clock unit
RADIUS_M = 10.0            # R
CENTRE_ALT_M = 50.0        # Z0: bottom 40 m, left/right 50 m, top 60 m
PAD_OFFSET_M = 10.0        # E: bottom pad ahead, top pad behind
JOIN_M = 40.0              # route metres to close the diamond into the circle (and open it)
TURN_M = 80.0              # route metres per full turn, at most
MIN_SEPARATION_M = 10.0
SAMPLES_PER_TURN = 16
JOIN_SAMPLES = 12
# A drone uploads its whole track to its autopilot within one tick, at most
# ~10 000 mission messages (engine.drone_thread._service_firmware) - so stay below.
MAX_TRACK_POINTS = 9_000


def slot_altitude_m(slot: str) -> float:
    return CENTRE_ALT_M - RADIUS_M * math.cos(SLOT_ANGLE[slot])


def min_route_m() -> float:
    """Join, one full turn, split."""
    return 2 * JOIN_M + TURN_M


def formula_min_separation_m() -> float:
    """The smallest gap the formula ever allows between any two drones."""
    return min(RADIUS_M * math.sqrt(2.0), 2 * RADIUS_M, math.hypot(PAD_OFFSET_M, RADIUS_M))


def _ease(u: float) -> float:
    """b(u): 0 -> 1 smoothly, starting and ending at rest."""
    return (1.0 - math.cos(math.pi * max(0.0, min(1.0, u)))) / 2.0


def formula_position(
    name: str, tau: float, v: float, w: float, t_join: float, t_spiral: float
) -> tuple[float, float, float]:
    """(x along, y across, z height) of drone `name` at clock time `tau` -
    THE formula (see the module doc)."""
    a0 = SLOT_ANGLE[START_SLOT[name]]
    pad = PAD_OFFSET_M * math.cos(a0)
    spiral_end = t_join + t_spiral
    if tau <= t_join:                                   # diamond -> circle
        a = a0
        x = v * tau + pad * (1.0 - _ease(tau / t_join))
    elif tau <= spiral_end:                             # the spiral
        a = a0 + w * (tau - t_join)
        x = v * tau
    else:                                               # circle -> diamond
        a = a0 + w * t_spiral
        x = v * tau + pad * _ease((tau - spiral_end) / t_join)
    return x, RADIUS_M * math.sin(a), CENTRE_ALT_M - RADIUS_M * math.cos(a)


@dataclass(frozen=True)
class CircleMission:
    route_m: float                     # L, source to destination
    omega: float                       # w, radians per clock unit
    turns: int                         # full turns of the circle
    quarter_turns: int                 # = 4 * turns: each one moves every drone on one place
    sources: dict[str, LatLon]         # each drone's own take-off pad
    destinations: dict[str, LatLon]    # each drone's own landing pad
    start_alt_m: dict[str, float]      # each drone's height on its pad's circle place
    routes: dict[str, TrainRoute]      # each drone's timed 3-D track
    min_separation_m: float            # smallest gap between any two drones on the flown tracks
    max_speed: float                   # fastest any drone flies, metres per clock unit
    source: LatLon
    destination: LatLon


def circle_distance_error(source: LatLon, destination: LatLon) -> str | None:
    length = math.hypot(*_to_local(destination, source))
    if length < min_route_m():
        return (
            f"The destination is only {length:.0f} m from the source. Formation-Circle needs at "
            f"least {min_route_m():.0f} m - {JOIN_M:.0f} m to close into the circle, one full "
            f"turn of {TURN_M:.0f} m, and {JOIN_M:.0f} m to open out again before landing."
        )
    return None


def plan_circle_formation(source: LatLon, destination: LatLon) -> CircleMission:
    """Every drone's timed track from source to destination - see the
    module doc. Raises ValueError if the route is too short or too long."""
    problem = circle_distance_error(source, destination)
    if problem:
        raise ValueError(problem)
    ex, ny = _to_local(destination, source)
    length = math.hypot(ex, ny)
    ux, uy = ex / length, ny / length          # along the route
    nx, ny_ = -uy, ux                          # across it, to the left

    v = FORMATION_SPEED
    t_join = JOIN_M / v
    spiral_m = length - 2 * JOIN_M
    t_spiral = spiral_m / v
    # As many whole turns as fit at no more than one per TURN_M.
    turns = max(1, int(spiral_m / TURN_M))
    w = 2.0 * math.pi * turns / t_spiral

    def to_map(x: float, y: float) -> LatLon:
        return _from_local((x * ux + y * nx, x * uy + y * ny_), source)

    spiral_steps = turns * SAMPLES_PER_TURN
    taus = (
        [t_join * i / JOIN_SAMPLES for i in range(JOIN_SAMPLES)]
        + [t_join + t_spiral * i / spiral_steps for i in range(spiral_steps)]
        + [t_join + t_spiral + t_join * i / JOIN_SAMPLES for i in range(JOIN_SAMPLES + 1)]
    )
    routes, tracks, sources, destinations, start_alt = {}, {}, {}, {}, {}
    max_speed = 0.0
    for name in DRONE_NAMES:
        pts = [formula_position(name, tau, v, w, t_join, t_spiral) for tau in taus]
        tracks[name] = pts
        latlons = [to_map(x, y) for x, y, _z in pts]
        sources[name], destinations[name] = latlons[0], latlons[-1]
        start_alt[name] = pts[0][2]
        s, keys = 0.0, [(0.0, 0.0)]
        for i in range(1, len(pts)):
            s += math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
            keys.append((taus[i], s))
            dt = taus[i] - taus[i - 1]
            if dt > 0:
                max_speed = max(max_speed, math.dist(pts[i], pts[i - 1]) / dt)
        routes[name] = TrainRoute(
            start=latlons[0],
            approach=[],
            loop=latlons[1:-1],
            laps=1,
            depart=[latlons[-1]],
            schedule=tuple(keys),
            # Heights relative to the drone's own take-off height, which the
            # flight layer gives it as its per-drone altitude offset.
            loop_alt_offsets=tuple(z - start_alt[name] for _x, _y, z in pts[1:-1]),
        )
        if len(latlons) > MAX_TRACK_POINTS:
            raise ValueError(
                f"The route is {length / 1000:.1f} km - too long for Formation-Circle: its "
                f"{turns} turns need {len(latlons)} track points per drone, more than the "
                f"{MAX_TRACK_POINTS} a drone's autopilot takes. Pick a destination closer "
                f"to the source."
            )

    # The tracks as flown: straight lines between points, every drone
    # passing its i-th point at the same clock time.
    closest = math.inf
    for i in range(len(taus) - 1):
        for frac in (0.0, 0.25, 0.5, 0.75):
            at = [
                tuple(a + (b - a) * frac for a, b in zip(tracks[n][i], tracks[n][i + 1]))
                for n in DRONE_NAMES
            ]
            for a in range(4):
                for b in range(a + 1, 4):
                    closest = min(closest, math.dist(at[a], at[b]))
    if closest < MIN_SEPARATION_M:
        raise ValueError(f"Formation-Circle's tracks come within {closest:.1f} m of each other.")

    return CircleMission(
        route_m=length, omega=w, turns=turns, quarter_turns=4 * turns,
        sources=sources, destinations=destinations, start_alt_m=start_alt,
        routes=routes, min_separation_m=closest, max_speed=max_speed,
        source=source, destination=destination,
    )


# ---- PDDL ---------------------------------------------------------------------


def _pad(kind: str, name: str) -> str:
    return f"{kind}-{name.removeprefix('drone')}"   # pad-src-1, pad-dst-3, ...


def retarget_circle_problem(problem_text: str, mission: CircleMission) -> str:
    """Put the template's eight pads (each drone's own take-off and landing
    pad) on the mission's points, and fill in the formula's numbers."""
    result = problem_text
    for name in DRONE_NAMES:
        result = _set_latlon(result, _pad("pad-src", name), mission.sources[name])
        result = _set_latlon(result, _pad("pad-dst", name), mission.destinations[name])
    for fluent, value in (
        ("formation-speed", FORMATION_SPEED),
        ("radius", RADIUS_M),
        ("centre-altitude", CENTRE_ALT_M),
        ("pad-offset", PAD_OFFSET_M),
        ("turn-rate", mission.omega),
        ("route-length", mission.route_m),
        ("quarter-turns-total", mission.quarter_turns),
        ("min-separation", MIN_SEPARATION_M),
        # The clock's unit is SCHEDULE_UNIT_MPS metres at the swarm's full
        # cruise speed - no drone may be asked for more.
        ("max-speed", SCHEDULE_UNIT_MPS),
    ):
        result = _set_scalar_fluent(result, fluent, value)
    return result


def plan_actions(steps) -> dict[str, list[tuple[str, ...]]]:
    """Each drone's own actions in plan order, as (action, args after the drone)."""
    out: dict[str, list[tuple[str, ...]]] = {name: [] for name in DRONE_NAMES}
    for step in sorted(steps, key=lambda s: s.index):
        if step.args and step.args[0] in out:
            out[step.args[0]].append((step.action, *step.args[1:]))
    return out


def expected_actions(name: str, quarter_turns: int) -> list[tuple[str, ...]]:
    """Each drone's own plan: take off from its pad, join the circle at its
    starting place, move one place on per quarter turn, leave the circle,
    land on its destination pad."""
    number = name.removeprefix("drone")
    slot = START_SLOT[name]
    actions: list[tuple[str, ...]] = [("take-off", f"pad-src-{number}"), ("join-circle", slot)]
    k = SLOT_ORDER.index(slot)
    for q in range(quarter_turns):
        actions.append(("turn-quarter", SLOT_ORDER[(k + q) % 4], SLOT_ORDER[(k + q + 1) % 4]))
    actions += [("leave-circle", slot, f"pad-dst-{number}"), ("land", f"pad-dst-{number}")]
    return actions
