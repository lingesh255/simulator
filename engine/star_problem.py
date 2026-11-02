"""Star drone show: geometry, timed tracks and PDDL problem editing.

    Swarm-Planning/Plans/starformation/problem.pddl (template)
        -> expand_star_drones          (one formation slot per checked drone)
        -> retarget_star_show_problem  (source, destination, lap count and
                                        every drone's turn to go and return)
        -> ENHSP plans the drones' turns and the show's sequence of shapes
        -> plan_star_show: every drone's own timed track

A drone show flown from the picked SOURCE to the picked DESTINATION:

    take off       from a grid of 10 m pads in rows at the source
    fly out        ONE BY ONE: each drone leaves its pad, flies out of the
                   pad block to the corridor's source end, along the
                   corridor to just short of the show and on to its own
                   place on the circle round the destination
    then, every lap:
        form star      every drone slides to its place on a star
        form circle    ... and back to the circle
        spin circle    the circle turns once round, every drone flying
                       along it
        form star      into the star again
        spin star      the whole star turns once round its centre
        form circle    back to the circle, ready for the next lap
    fly back       ONE BY ONE, the same way in reverse: each drone leaves
                   its place, flies the corridor back to the source and
                   lands on the pad it took off from

ONE BY ONE. Every drone flies the same corridor between the pad block's
front (the side facing the destination) and the circle's near side, one
`TRANSIT_GAP_M` behind the one before. The front pads leave first - so no
drone flies over a pad still waiting - and the circle fills from its FAR
side: a drone flying in to its place only ever crosses places still empty.
The way back is exactly the way out played backwards: near side of the
circle first, landing on the back pads first.

THE SHAPES keep every drone in the same ORDER round the centre: the
circle's drones are evenly spaced round it, in the same order as their
places round the star, so circle <-> star is every drone sliding to the
matching place without the order ever changing. The star has
`star_points` tips (5 from 10 drones up, fewer for a smaller swarm, so it
still reads as a star); its tips and inner corners are filled first, any
further drones spread evenly along its edges. The circle runs through the
star's tips. The whole show is sized up from the smallest it could be until
every two drones are at least `STAR_SPACING_M` apart on the circle and on
the star, and at least 90% of that at every moment of the moves between
circle and star (`_size_formation`). A spin turns the whole shape rigidly -
every drone at the same angular speed round the centre - so the gaps never
change.

TIMING. Every drone flies a timetable against one shared clock (see
`engine.train_keeping`): keyframes "at clock time t, be s metres along my
route". Flying out and back each drone has its own times, one gap after
the one before; every show phase has the same start and end time for
every drone - so each shape forms all at once and each spin turns as
one. Clock units are nominal seconds at `SCHEDULE_UNIT_MPS`; the
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
TRANSIT_SPEED = 0.8            # source <-> destination, as a fraction of full speed
TRANSIT_GAP_M = 3 * STAR_SPACING_M  # one drone behind the next, flying out and back
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


def grid_pad_offsets(
    drone_count: int, toward: tuple[float, float] = (0.0, 1.0), spacing: float = STAR_SPACING_M
) -> list[tuple[float, float]]:
    """Take-off pads as a ground crew lays them out: a roughly square grid
    of rows, `spacing` apart, centred on the source (east, north metres),
    its rows square to `toward` (a unit vector, the way the drones head
    off). The column count is odd, so a column of pads runs straight down
    the centre line. Rows fill from the front (the `toward` side), each from
    the centre outwards, so a partly filled back row sits centred."""
    cols = max(1, math.ceil(math.sqrt(drone_count)))
    cols += 1 - cols % 2
    rows = math.ceil(drone_count / cols)
    left = (-toward[1], toward[0])
    centre = (cols - 1) / 2.0
    pads: list[tuple[float, float]] = []
    for row in range(rows):
        along = ((rows - 1) / 2.0 - row) * spacing
        for c in sorted((c - centre for c in range(cols)), key=lambda c: (abs(c), c)):
            across = c * spacing
            pads.append((toward[0] * along + left[0] * across, toward[1] * along + left[1] * across))
    return pads[:drone_count]


def grid_pad_reach(drone_count: int, spacing: float = STAR_SPACING_M) -> float:
    """How far the furthest pad of `grid_pad_offsets` is from the source -
    the same whichever way the grid faces."""
    return max(math.hypot(*p) for p in grid_pad_offsets(drone_count, spacing=spacing))


def _size_formation(drone_count: int) -> tuple[float, list, list]:
    """Smallest tip radius at which the star and the circle keep every two
    drones `STAR_SPACING_M` apart, and the moves circle <-> star keep them
    at least `_MIN_FRACTION` of that apart. Returns (radius, star, circle)."""
    n = drone_count
    radius = max(STAR_SPACING_M / (2 * math.sin(math.pi / n)), 2 * STAR_SPACING_M)
    while True:
        star = _star_slots(n, radius)
        circle = _circle_slots(star, radius)
        if (
            _min_distance(star) >= STAR_SPACING_M - 1e-6
            and _min_distance(circle) >= STAR_SPACING_M - 1e-6
            and _min_distance_moving(circle, star) >= _MIN_FRACTION * STAR_SPACING_M
        ):
            return radius, star, circle
        radius += 2.0


def _pad_reach(drone_count: int) -> float:
    return grid_pad_reach(drone_count)


def star_show_min_distance_m(drone_count: int) -> float:
    """How far the destination must be from the source for this many
    drones: the pad block and the show's circle clear of each other, with
    the corridor between them at least a gap long."""
    radius = _size_formation(drone_count)[0]
    return _pad_reach(drone_count) + radius + 2 * STAR_SPACING_M + TRANSIT_GAP_M


def star_show_distance_error(source: LatLon, destination: LatLon, drone_count: int) -> str | None:
    """The operator-facing reason the destination is too close to the
    source for this many drones, or None."""
    span = math.hypot(*_to_local(destination, source))
    needed = star_show_min_distance_m(drone_count)
    if span >= needed:
        return None
    return (
        f"The destination is {span:.0f} m from the source - a star show with "
        f"{drone_count} drones needs it at least {math.ceil(needed)} m away, so the "
        f"take-off pads and the show's circle are well clear of each other."
    )


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
    arrival_order: list[str]            # drone names in the order they fly out
    return_order: list[str]             # ... and the order they fly back

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


def plan_star_show(source: LatLon, destination: LatLon, drone_count: int, laps: int = 1) -> StarShow:
    """Every drone's timed track for a star show at `destination`, flown
    out from and back to pads at `source` - see the module doc. Raises
    ValueError for an unflyable drone or lap count, or a destination too
    close to the source."""
    problem = star_drone_count_error(drone_count)
    if problem:
        raise ValueError(problem)
    if not 1 <= laps <= MAX_SHOW_LAPS:
        raise ValueError(f"Laps must be between 1 and {MAX_SHOW_LAPS}, got {laps}.")

    # Everything in local (east, north) metres from the source. The star
    # stands with a tip pointing north: formation frame (x forward, y left)
    # -> local, round the destination.
    problem = star_show_distance_error(source, destination, drone_count)
    if problem:
        raise ValueError(problem)
    n = drone_count
    centre = _to_local(destination, source)
    span = math.hypot(*centre)

    def to_world(p):
        return (centre[0] - p[1], centre[1] + p[0])

    radius, star_f, circle_f = _size_formation(n)
    star = [to_world(p) for p in star_f]
    circle = [to_world(p) for p in circle_f]
    unit = SCHEDULE_UNIT_MPS

    def dist(a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def polyline_m(points):
        return sum(dist(a, b) for a, b in zip(points, points[1:]))

    # The corridor: from just ahead of the pad block to just short of the
    # circle, on the line source -> destination.
    ux, uy = centre[0] / span, centre[1] / span
    pads = grid_pad_offsets(n, (ux, uy))

    def ahead(p):
        return p[0] * ux + p[1] * uy

    def across(p):
        return p[1] * ux - p[0] * uy

    corridor_start = (ux * (_pad_reach(n) + 2 * STAR_SPACING_M), uy * (_pad_reach(n) + 2 * STAR_SPACING_M))
    corridor_end = (centre[0] - ux * (radius + STAR_SPACING_M), centre[1] - uy * (radius + STAR_SPACING_M))

    # Out: the pads nearest the corridor first - so every pad still waiting
    # is further from the corridor than the drone leaving, and never on its
    # way - into the circle's far side first. The k-th drone to leave takes
    # the k-th place to fill.
    pads_out = sorted(range(n), key=lambda j: (dist(pads[j], corridor_start), across(pads[j])))
    slots_in = sorted(range(n), key=lambda i: (-ahead((circle[i][0] - centre[0], circle[i][1] - centre[1])),
                                               across((circle[i][0] - centre[0], circle[i][1] - centre[1]))))
    pad_of = [0] * n
    turn_of = [0] * n
    for k, (slot, pad) in enumerate(zip(slots_in, pads_out)):
        pad_of[slot] = pad
        turn_of[slot] = k
    pad_xy = [pads[pad_of[i]] for i in range(n)]

    v = TRANSIT_SPEED * unit
    gap_t = TRANSIT_GAP_M / v
    to_corridor = [dist(pad_xy[i], corridor_start) for i in range(n)]
    corridor_m = dist(corridor_start, corridor_end)
    to_slot = [dist(corridor_end, circle[i]) for i in range(n)]
    # Drone k (in turn) enters the corridor at k gaps after the first.
    enter_0 = max(to_corridor) / v
    show_start = max(enter_0 + turn_of[i] * gap_t + (corridor_m + to_slot[i]) / v for i in range(n)) + CIRCLE_HOLD_S

    # Shared show phase lengths (nominal seconds) - the same for every drone.
    t_morph = max(dist(a, b) for a, b in zip(circle, star)) / (MORPH_SPEED * unit)
    t_spin = 2 * math.pi * radius / (SPIN_SPEED * unit)   # one full turn, tips at spin speed
    lap_times = [t_morph, t_morph, t_spin, t_morph, t_spin, t_morph]
    holds = [STAR_HOLD_S, CIRCLE_HOLD_S, CIRCLE_HOLD_S, STAR_HOLD_S, STAR_HOLD_S, CIRCLE_HOLD_S]
    show_end = show_start + laps * (sum(lap_times) + sum(holds))
    # Back: the last in leaves first, drone k (in its return turn) entering
    # the corridor k gaps after the first.
    leave_0 = show_end + max(to_slot) / v

    routes: dict[str, TrainRoute] = {}
    names = star_drone_names(n)
    for i, name in enumerate(names):
        c, st = circle[i], star[i]
        circle_spin = _arc(math.atan2(c[1] - centre[1], c[0] - centre[0]), radius, 360.0)
        star_spin = _arc(math.atan2(st[1] - centre[1], st[0] - centre[0]), dist(centre, st), 360.0)
        circle_spin = [(centre[0] + x, centre[1] + y) for x, y in circle_spin]
        star_spin = [(centre[0] + x, centre[1] + y) for x, y in star_spin]
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

        # Out, in its turn: wait over the pad, then pad -> corridor -> place.
        enter = enter_0 + turn_of[i] * gap_t
        keys = [(0.0, 0.0)]
        s_m = 0.0
        for t, length in (
            (enter - to_corridor[i] / v, 0.0),
            (enter, to_corridor[i]),
            (enter + corridor_m / v, corridor_m),
            (enter + (corridor_m + to_slot[i]) / v, to_slot[i]),
        ):
            s_m += length
            if t > keys[-1][0]:
                keys.append((t, s_m))
        t = show_start
        keys.append((t, s_m))
        for _lap in range(laps):
            for length, move_t, hold in zip(lap_lengths, lap_times, holds):
                s_m += length
                t += move_t
                keys.append((t, s_m))
                t += hold
                keys.append((t, s_m))
        # Back, in its turn: wait in place, then place -> corridor -> pad.
        leave = leave_0 + (n - 1 - turn_of[i]) * gap_t
        for t, length in (
            (leave - to_slot[i] / v, 0.0),
            (leave, to_slot[i]),
            (leave + corridor_m / v, corridor_m),
            (leave + (corridor_m + to_corridor[i]) / v, to_corridor[i]),
        ):
            s_m += length
            if t > keys[-1][0]:
                keys.append((t, s_m))

        routes[name] = TrainRoute(
            start=_from_local(pad_xy[i], source),
            approach=[_from_local(p, source) for p in (corridor_start, corridor_end, c)],
            loop=[_from_local(p, source) for p in lap],
            laps=laps,
            depart=[_from_local(p, source) for p in (corridor_end, corridor_start, pad_xy[i])],
            schedule=tuple(keys),
        )

    arrival_order = [names[slot] for slot in slots_in]
    return StarShow(
        drone_count=n,
        laps=laps,
        tip_radius_m=radius,
        routes=routes,
        pads=[_from_local(p, source) for p in pad_xy],
        circle=[_from_local(p, source) for p in circle],
        star=[_from_local(p, source) for p in star],
        arrival_order=arrival_order,
        return_order=arrival_order[::-1],
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


def retarget_star_show_problem(problem_text: str, source: LatLon, destination: LatLon,
                               laps: int, show: StarShow) -> str:
    """Put an (already expanded) star-show problem at the picked source and
    destination, ask for `laps` laps, and give every drone its turn to fly
    out and back in `show`'s order."""
    result = _set_latlon(_set_latlon(problem_text, "source", source), "destination", destination)
    # The template's (and its cloned) turns only describe the smallest
    # case - rebuild both chains: first drone's turn, then who follows whom.
    facts = []
    for turn, after, order in (("out-turn", "out-after", show.arrival_order),
                               ("back-turn", "back-after", show.return_order)):
        result = re.sub(rf"[ \t]*\({turn}\s+\S+\)\n?", "", _strip_pair_facts(result, after))
        facts.append(f"\n        ({turn} {order[0]})")
        facts += [f"\n        ({after} {a} {b})" for a, b in zip(order, order[1:])]
    result = _add_init_facts(result, "".join(facts))
    return _set_scalar_fluent(result, "laps-required", laps)
