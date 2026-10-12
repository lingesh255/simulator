"""Star drone show: geometry, timed tracks and PDDL problem editing.

    plans/starformation/problem.pddl (template)
        -> expand_star_drones          (one formation slot per checked drone)
        -> retarget_star_show_problem  (the picked source, the lap count)
        -> ENHSP plans the show's sequence of shapes
        -> plan_star_show: every drone's own timed track

A drone show in one place, round the picked source - no destination:

    take off       from a compact ring of 10 m pads round the source
    form circle    spread out from the pads into a circle
    then, every lap:
        form star      every drone slides to its place on a star
        form circle    ... and back to the circle
        spin circle    the circle turns once round, every drone flying
                       along it
        form star      into the star again
        spin star      the whole star turns once round its centre
        form circle    back to the circle, ready for the next lap
    land           spread back in from the circle onto the pads and land
                   exactly where each drone took off

THE SHAPES keep every drone in the same ORDER round the centre: the
circle's drones are evenly spaced round it, in the same order as their
places round the star, so circle <-> star is every drone sliding to the
matching place without the order ever changing. The star has
`star_points` tips (5 from 10 drones up, fewer for a smaller swarm, so it
still reads as a star); its tips and inner corners are filled first, any
further drones spread evenly along its edges. The circle runs through the
star's tips. Take-off pads are laid out in rings round the source, so every
drone has a clear direction to spread out in. The whole show is sized up
from the smallest it could be until every two drones are at least
`STAR_SPACING_M` apart on the circle and on the star, and at least 90% of
that at every moment of the moves between pads, circle and star
(`_size_formation`). A spin turns the whole shape rigidly - every drone at
the same angular speed round the centre - so the gaps never change.

TIMING. Every drone flies a timetable against one shared clock (see
`engine.train_keeping`): keyframes "at clock time t, be s metres along my
route". Every phase has the same start and end time for every drone - so
each shape forms all at once, each spin turns as one, and they all land
together. Clock units are nominal seconds at `SCHEDULE_UNIT_MPS`; the
ground station runs the clock as fast as the slowest drone allows. One lap
is uploaded once and repeated on board (`TrainRoute.loop` x laps), so even
many laps are one short mission upload.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass

from engine.figure8_problem import TrainRoute
from engine.pddl_problem import (
    LatLon,
    _add_init_facts,
    _clone_named_lines,
    _find_matching_paren,
    _from_local,
    _section_span,
    _set_unary_fact_value,
    _strip_pair_facts,
    _set_latlon,
    _to_local,
)
from engine.train_keeping import SCHEDULE_UNIT_MPS

STAR_SPACING_M = 10.0          # nearest any two drones may be, in any shape
MIN_STAR_DRONES = 6            # a 3-pointed star's tips and inner corners
MAX_STAR_DRONES = 40           # beyond this the star grows to hundreds of metres
MAX_STAR_POINTS = 5
STAR_INNER_RATIO = 0.45        # inner corners' radius, as a fraction of the tips'
CIRCLE_HOLD_S = 6.0            # nominal seconds each circle is held
STAR_HOLD_S = 4.0              # ... and each star
SPREAD_SPEED = 0.6             # pads <-> circle, as a fraction of full speed
MORPH_SPEED = 0.5              # circle <-> star, as a fraction of full speed
SPIN_SPEED = 0.6               # spins: the outermost drones' speed, as a fraction of full speed
SPIN_STEP_DEG = 10.0           # a spin is flown as waypoints this many degrees apart
MAX_SHOW_LAPS = 50
_MIN_FRACTION = 0.9            # moving between shapes, drones keep >= this x spacing

_LEADER = "drone-lead"
_MEMBER_TOKEN = "drone-m1"     # the template's single member, cloned per drone


def star_points(drone_count: int) -> int:
    """How many tips the star has: 5 once there are drones for all ten of a
    five-pointed star's tips and corners, otherwise as many as there are
    drones for (6-7 drones: 3 tips, 8-9: 4)."""
    return MAX_STAR_POINTS if drone_count >= 2 * MAX_STAR_POINTS else max(3, drone_count // 2)


def star_drone_names(drone_count: int) -> list[str]:
    """`["drone-lead", "drone-m1", ...]` - the leader takes the star's
    forward tip, then the others in order round the star."""
    return [_LEADER] + [f"drone-m{k}" for k in range(1, drone_count)]


# ---- Shapes, in the formation's own frame (x forward, y to the left) -------


def _star_slots(drone_count: int, tip_radius: float) -> list[tuple[float, float]]:
    """Every drone's place on the star, in angle order from the forward
    tip, anticlockwise."""
    p = star_points(drone_count)
    inner = tip_radius * STAR_INNER_RATIO
    corners = []
    for j in range(2 * p):
        angle = j * math.pi / p
        r = tip_radius if j % 2 == 0 else inner
        corners.append((r * math.cos(angle), r * math.sin(angle)))
    # Spread the drones beyond the corners as evenly as possible over the
    # 2p edges, each edge's extras evenly along it.
    extra = drone_count - 2 * p
    per_edge = [extra // (2 * p) + (1 if j < extra % (2 * p) else 0) for j in range(2 * p)]
    slots = []
    for j in range(2 * p):
        a, b = corners[j], corners[(j + 1) % (2 * p)]
        slots.append(a)
        for k in range(1, per_edge[j] + 1):
            f = k / (per_edge[j] + 1)
            slots.append((a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f))
    return slots


def _circle_slots(star: list[tuple[float, float]], radius: float) -> list[tuple[float, float]]:
    """Each drone's place on the circle: evenly spaced round it, in the same
    order as the star's places (which `_star_slots` lists by angle, from
    the forward tip)."""
    n = len(star)
    return [(radius * math.cos(2 * math.pi * i / n), radius * math.sin(2 * math.pi * i / n)) for i in range(n)]


def _min_distance(points: list[tuple[float, float]]) -> float:
    best = math.inf
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            best = min(best, math.hypot(points[i][0] - points[j][0], points[i][1] - points[j][1]))
    return best


def _min_distance_moving(a: list, b: list, steps: int = 20) -> float:
    """Closest two drones get while all move in straight lines a[i] -> b[i]
    together, starting and finishing at the same moment."""
    best = math.inf
    for k in range(steps + 1):
        f = k / steps
        best = min(best, _min_distance(
            [(p[0] + (q[0] - p[0]) * f, p[1] + (q[1] - p[1]) * f) for p, q in zip(a, b)]
        ))
    return best


def _pad_offsets(drone_count: int) -> list[tuple[float, float]]:
    """Take-off pads in rings round the source (east, north metres): ring k
    at k spacings out, holding as many pads as fit a spacing apart - so
    every pad has its own direction from the centre to spread out along."""
    pads: list[tuple[float, float]] = []
    k = 1
    while len(pads) < drone_count:
        take = min(int(2 * math.pi * k), drone_count - len(pads))
        twist = 0.5 * (k % 2)
        r = k * STAR_SPACING_M
        pads += [
            (r * math.cos(2 * math.pi * (j + twist) / take), r * math.sin(2 * math.pi * (j + twist) / take))
            for j in range(take)
        ]
        k += 1
    return pads


def _size_formation(drone_count: int, pads_local: list[tuple[float, float]],
                    to_world) -> tuple[float, list, list, list]:
    """Smallest tip radius at which the star and the circle keep every two
    drones `STAR_SPACING_M` apart, and the moves pads -> circle -> star keep
    them at least `_MIN_FRACTION` of that apart. Pads are matched to circle
    places in angle order, at whichever rotation of that matching keeps the
    spread-out widest. Returns (radius, star, circle, pad index per slot)."""
    n = drone_count
    pad_reach = max((math.hypot(*p) for p in pads_local), default=0.0)
    radius = max(STAR_SPACING_M / (2 * math.sin(math.pi / n)), pad_reach + STAR_SPACING_M, 2 * STAR_SPACING_M)
    pads_by_angle = sorted(range(n), key=lambda i: math.atan2(pads_local[i][1], pads_local[i][0]))
    while True:
        star = _star_slots(n, radius)
        circle = _circle_slots(star, radius)
        if (
            _min_distance(star) >= STAR_SPACING_M - 1e-6
            and _min_distance(circle) >= STAR_SPACING_M - 1e-6
            and _min_distance_moving(circle, star) >= _MIN_FRACTION * STAR_SPACING_M
        ):
            circle_world = [to_world(p) for p in circle]
            slots_by_angle = sorted(range(n), key=lambda i: math.atan2(circle_world[i][1], circle_world[i][0]))
            best = None
            for shift in range(n):
                pad_of = [0] * n
                for k, slot in enumerate(slots_by_angle):
                    pad_of[slot] = pads_by_angle[(k + shift) % n]
                spread = _min_distance_moving([pads_local[pad_of[i]] for i in range(n)], circle_world, 12)
                if best is None or spread > best[0]:
                    best = (spread, pad_of)
            if best[0] >= _MIN_FRACTION * STAR_SPACING_M:
                return radius, star, circle, best[1]
        radius += 2.0


# ---- The show -------------------------------------------------------------------


@dataclass(frozen=True)
class StarShow:
    """A star drone show ready to fly."""

    drone_count: int
    laps: int
    tip_radius_m: float
    routes: dict[str, TrainRoute]       # keyed by star_drone_names
    pads: list[LatLon]                  # each drone's take-off and landing pad, same order
    circle: list[LatLon]
    star: list[LatLon]

    def preview_shape(self) -> list[LatLon]:
        """The star, for drawing on the map."""
        return list(self.star)


def star_drone_count_error(drone_count: int) -> str | None:
    """The operator-facing reason this many drones can't fly the show, or None."""
    if drone_count < MIN_STAR_DRONES:
        return (
            f"A star show needs at least {MIN_STAR_DRONES} drones - a 3-pointed "
            f"star's 3 tips and 3 inner corners. {drone_count} "
            f"{'is' if drone_count == 1 else 'are'} checked.\n\n"
            f"6-7 drones make a 3-pointed star, 8-9 a 4-pointed one, and 10 or more "
            f"a 5-pointed star."
        )
    if drone_count > MAX_STAR_DRONES:
        return (
            f"A star show takes at most {MAX_STAR_DRONES} drones - beyond that, "
            f"keeping every drone {STAR_SPACING_M:.0f} m apart makes the star hundreds "
            f"of metres across. {drone_count} are checked."
        )
    return None


def _arc(centre_angle_from: float, radius: float, turns_deg: float) -> list[tuple[float, float]]:
    """Points along a circle of `radius` round the origin, starting just
    after angle `centre_angle_from` (radians) and turning `turns_deg`
    anticlockwise, `SPIN_STEP_DEG` apart - the start itself excluded, the
    end (back where it began, after a full turn) included."""
    steps = max(1, round(turns_deg / SPIN_STEP_DEG))
    return [
        (radius * math.cos(centre_angle_from + math.radians(turns_deg) * k / steps),
         radius * math.sin(centre_angle_from + math.radians(turns_deg) * k / steps))
        for k in range(1, steps + 1)
    ]


def plan_star_show(source: LatLon, drone_count: int, laps: int = 1) -> StarShow:
    """Every drone's timed track for a star show round `source` - see the
    module doc. Raises ValueError for an unflyable drone or lap count."""
    problem = star_drone_count_error(drone_count)
    if problem:
        raise ValueError(problem)
    if not 1 <= laps <= MAX_SHOW_LAPS:
        raise ValueError(f"Laps must be between 1 and {MAX_SHOW_LAPS}, got {laps}.")

    # The star stands with a tip pointing north: formation frame (x forward,
    # y left) -> local (east, north).
    def to_world(p):
        return (-p[1], p[0])

    n = drone_count
    pads = _pad_offsets(n)
    radius, star_f, circle_f, pad_of = _size_formation(n, pads, to_world)
    star = [to_world(p) for p in star_f]
    circle = [to_world(p) for p in circle_f]
    pad_xy = [pads[pad_of[i]] for i in range(n)]
    unit = SCHEDULE_UNIT_MPS

    def dist(a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def polyline_m(points):
        return sum(dist(a, b) for a, b in zip(points, points[1:]))

    # Shared phase lengths (nominal seconds) - the same for every drone.
    t_spread = max(dist(a, b) for a, b in zip(pad_xy, circle)) / (SPREAD_SPEED * unit)
    t_morph = max(dist(a, b) for a, b in zip(circle, star)) / (MORPH_SPEED * unit)
    t_spin = 2 * math.pi * radius / (SPIN_SPEED * unit)   # one full turn, tips at spin speed

    routes: dict[str, TrainRoute] = {}
    for i, name in enumerate(star_drone_names(n)):
        c, st = circle[i], star[i]
        circle_spin = _arc(math.atan2(c[1], c[0]), radius, 360.0)
        star_spin = _arc(math.atan2(st[1], st[0]), math.hypot(*st), 360.0)
        star_spin[-1] = st           # exactly back on its place
        circle_spin[-1] = c
        # One lap, starting and ending on the drone's place on the circle.
        lap = [st, c, *circle_spin, st, *star_spin, c]
        lap_lengths = [
            dist(c, st),                       # form star
            dist(st, c),                       # form circle
            polyline_m([c, *circle_spin]),     # spin circle
            dist(c, st),                       # form star
            polyline_m([st, *star_spin]),      # spin star
            dist(st, c),                       # form circle
        ]
        lap_times = [t_morph, t_morph, t_spin, t_morph, t_spin, t_morph]
        holds = [STAR_HOLD_S, CIRCLE_HOLD_S, CIRCLE_HOLD_S, STAR_HOLD_S, STAR_HOLD_S, CIRCLE_HOLD_S]

        keys = [(0.0, 0.0)]
        t = t_spread
        s_m = dist(pad_xy[i], c)
        keys.append((t, s_m))
        t += CIRCLE_HOLD_S
        keys.append((t, s_m))
        for _lap in range(laps):
            for length, move_t, hold in zip(lap_lengths, lap_times, holds):
                s_m += length
                t += move_t
                keys.append((t, s_m))
                t += hold
                keys.append((t, s_m))
        s_m += dist(c, pad_xy[i])
        t += t_spread
        keys.append((t, s_m))

        routes[name] = TrainRoute(
            start=_from_local(pad_xy[i], source),
            approach=[_from_local(c, source)],
            loop=[_from_local(p, source) for p in lap],
            laps=laps,
            depart=[_from_local(pad_xy[i], source)],
            schedule=tuple(keys),
        )

    return StarShow(
        drone_count=n,
        laps=laps,
        tip_radius_m=radius,
        routes=routes,
        pads=[_from_local(p, source) for p in pad_xy],
        circle=[_from_local(p, source) for p in circle],
        star=[_from_local(p, source) for p in star],
    )


# ---- PDDL problem editing ---------------------------------------------------


def _goal_and_span(text: str) -> tuple[int, int]:
    goal_open, goal_close = _section_span(text, "goal")
    and_match = re.search(r"\(and\b", text[goal_open:goal_close])
    if not and_match:
        return goal_open, goal_close
    and_open = goal_open + and_match.start()
    return and_open, _find_matching_paren(text, and_open)


def expand_star_drones(problem_text: str, drone_count: int) -> str:
    """Grow the template's leader + one member (`drone-m1`) to
    `drone_count` drones: every `:init` fact and goal conjunct mentioning
    `drone-m1` is cloned per extra member, each drone's `slot-angle` set to
    its own place round the formation, and the `ring-neighbor` facts (each
    drone and the two either side of it round the circle) rebuilt - the
    cloned template pair would otherwise wire every member to the leader."""
    if drone_count < MIN_STAR_DRONES:
        raise ValueError(
            f"star formation drone_count must be >= {MIN_STAR_DRONES}, got {drone_count}"
        )
    names = star_drone_names(drone_count)
    extra = names[2:]
    result = problem_text
    if extra:
        _, obj_close = _section_span(result, "objects")
        result = (
            result[:obj_close]
            + "".join(f"\n        {name} - drone" for name in extra)
            + "\n    "
            + result[obj_close:]
        )
        result = _clone_named_lines(result, _section_span(result, "init"), _MEMBER_TOKEN, extra)
        result = _clone_named_lines(result, _goal_and_span(result), _MEMBER_TOKEN, extra)
    star = _star_slots(drone_count, 1.0)
    for name, (x, y) in zip(names, star):
        result = _set_unary_fact_value(result, "slot-angle", name, math.degrees(math.atan2(y, x)) % 360.0)
    result = _strip_pair_facts(result, "ring-neighbor")
    ring = []
    for a, b in zip(names, names[1:] + names[:1]):
        ring.append(f"\n        (ring-neighbor {a} {b})")
        ring.append(f"\n        (ring-neighbor {b} {a})")
    return _add_init_facts(result, "".join(ring))


def _set_scalar_fluent(problem_text: str, fluent: str, value: float) -> str:
    pattern = re.compile(rf"(\(=\s*\({re.escape(fluent)}\)\s*)[-\d.eE]+(\s*\))")
    result, count = pattern.subn(lambda m: f"{m.group(1)}{value:.6f}{m.group(2)}", problem_text, count=1)
    if not count:
        raise ValueError(f"template has no ({fluent}) fact")
    return result


def retarget_star_show_problem(problem_text: str, source: LatLon, laps: int) -> str:
    """Put an (already expanded) star-show problem at the picked source and
    ask for `laps` laps."""
    return _set_scalar_fluent(_set_latlon(problem_text, "source", source), "laps-required", laps)
