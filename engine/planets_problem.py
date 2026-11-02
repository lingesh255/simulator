"""Planets (solar-system) drone show: orbits, timed tracks and PDDL editing.

    Swarm-Planning/Plans/planets/problem.pddl (template)
        -> expand_planet_drones          (one orbit per checked drone)
        -> retarget_planets_problem      (source, destination, lap count,
                                          orbit speeds and every drone's
                                          turn to go and return)
        -> ENHSP plans the show: fly out one by one, enter orbits, fly the
           laps, end the show, fly back one by one, land
        -> plan_planets_show: every drone's own timed track

A show flown from the picked SOURCE (the take-off pads) to the picked
DESTINATION (the sun). Drone k (0 = innermost, "mercury") flies a circular
orbit round the destination of radius `FIRST_ORBIT_M + ORBIT_SPACING_M * k`:
20 m, 30 m, ... up to 8 drones (Neptune at 90 m) - drones on neighbouring
orbits 10 m apart. Neighbouring orbits turn in OPPOSITE directions - the
innermost anticlockwise, the next clockwise, and so on.

    take off       from a grid of pads in rows at the source, 10 m apart,
                   facing the destination
    fly out        ONE BY ONE, each from its pad to a gate on the line just
                   ahead of the pad block, then single file along the line
                   source -> destination, `TRANSIT_GAP_M` apart: the
                   innermost planet (on the pad nearest the gate) first.
                   Each drone stops on its own
                   orbit where the line crosses it - the orbit's START
                   point, on the source side of the sun - so a drone only
                   ever flies past start points still empty
    enter orbits   every drone in place, lined up on that line
    laps           RELATIVE MOTION: every drone turns round the sun at the
                   SAME angular speed, so the anticlockwise drones stay in
                   one straight line out from the sun and the clockwise
                   drones in another - two rigid spokes turning opposite
                   ways. A drone's speed is in proportion to its orbit's
                   radius: the innermost (nearest the sun) slowest, the
                   outermost fastest, at `ORBIT_SPEED` of full speed. A lap
                   is one turn - of every drone - so every lap ends with
                   the two spokes lined up again on the start line
    fly back       ONE BY ONE, the fly-out played backwards: the outermost
                   (nearest the source) first, single file back along the
                   line and through the gate, each landing on the pad it
                   took off from

Orbits are `ORBIT_SPACING_M` (10 m) apart, so the two spokes pass each
other with drones on different orbits, 10 m apart; flying out and back the
drones stay in single file on one line, and between the gate and the pads
never fly over a pad that still has a drone on it.

TIMING. As for the star show (`engine.star_problem`): every drone flies a
timetable against one shared clock (`engine.train_keeping`) - keyframes "at
clock time t, be s metres along my route". Flying out and back each drone
has its own times, one gap after the one before; the laps start and end at
the same clock time for every drone, every drone covering the same
fraction of its laps at every moment - so at the same angular speed. Every
move speeds up smoothly from a standstill and slows smoothly back to one
over `RAMP_S` (`_eased_keys`), rather than jumping to full speed and
stopping dead. One lap is uploaded once and repeated on board
(`TrainRoute.loop` x laps).
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
    _from_local,
    _section_span,
    _set_latlon,
    _set_unary_fact_value,
    _strip_pair_facts,
    _to_local,
)
from engine.star_problem import _goal_and_span, _set_scalar_fluent, grid_pad_offsets, grid_pad_reach
from engine.train_keeping import SCHEDULE_UNIT_MPS

PLANET_NAMES = ("mercury", "venus", "earth", "mars", "jupiter", "saturn", "uranus", "neptune")
FIRST_ORBIT_M = 20.0           # the innermost orbit's radius
ORBIT_SPACING_M = 10.0         # radius step from one orbit to the next - neighbours 10 m apart
PAD_CLEARANCE_M = 50.0         # the source at least this far outside the outermost orbit
PAD_SPACING_M = 10.0           # take-off pads, in a grid of rows at the source
MIN_PLANET_DRONES = 2          # two orbits, to turn opposite ways
MAX_PLANET_DRONES = len(PLANET_NAMES)
MAX_PLANET_LAPS = 50
ORBIT_SPEED = 0.6              # the outermost drone's speed, as a fraction of full speed
TRANSIT_SPEED = 0.8            # source <-> destination, as a fraction of full speed
TRANSIT_GAP_M = 30.0           # one drone behind the next, flying out and back
ORBIT_HOLD_S = 4.0             # nominal seconds held lined up before and after the laps
RAMP_S = 5.0                   # nominal seconds to speed up from / slow down to a stop
RAMP_STEPS = 8                 # keyframes along each speed-up or slow-down
ARC_STEP_DEG = 5.0             # an orbit is flown as waypoints at most this many degrees apart
ARC_STEP_M = 4.0               # ... and at most this many metres apart
# An orbit is flown as straight legs between waypoints ON its circle, so
# mid-leg a drone is up to ~2 cm inside it (4 m legs, 90 m radius): orbits
# are this much further apart than ORBIT_SPACING_M, to keep neighbours at
# least ORBIT_SPACING_M apart all the way round.
_CHORD_MARGIN_M = 0.05

_LEADER = "drone-mercury"
_MEMBER_TOKEN = "drone-venus"  # the template's second planet, cloned per drone


def planet_drone_names(drone_count: int) -> list[str]:
    """`["drone-mercury", "drone-venus", ...]`, innermost orbit first."""
    return [f"drone-{name}" for name in PLANET_NAMES[:drone_count]]


def orbit_radius_m(index: int) -> float:
    """Radius of orbit `index` (0 = innermost): 20 m, then 10 m further out
    per orbit - so drones on neighbouring orbits are 10 m apart."""
    return FIRST_ORBIT_M + (ORBIT_SPACING_M + _CHORD_MARGIN_M) * index


def orbit_clockwise(index: int) -> bool:
    """Which way orbit `index` turns: innermost anticlockwise, then
    alternating outwards, so neighbouring orbits always turn opposite ways."""
    return index % 2 == 1


def orbit_speed_fraction(index: int, drone_count: int) -> float:
    """Orbit `index`'s speed as a fraction of full speed: every drone turns
    at the same angular speed, so in proportion to its radius."""
    return ORBIT_SPEED * orbit_radius_m(index) / orbit_radius_m(drone_count - 1)


def planets_drone_count_error(drone_count: int) -> str | None:
    """The operator-facing reason this many drones can't fly the show, or None."""
    if drone_count < MIN_PLANET_DRONES:
        return (
            f"The planets show needs at least {MIN_PLANET_DRONES} drones - two "
            f"neighbouring orbits turning opposite ways. {drone_count} "
            f"{'is' if drone_count == 1 else 'are'} checked."
        )
    if drone_count > MAX_PLANET_DRONES:
        return (
            f"The planets show takes at most {MAX_PLANET_DRONES} drones - one per "
            f"planet of the solar system, orbits {ORBIT_SPACING_M:.0f} m apart "
            f"(the outermost {orbit_radius_m(MAX_PLANET_DRONES - 1):.0f} m out). "
            f"{drone_count} are checked."
        )
    return None


def planets_min_distance_m(drone_count: int) -> float:
    """How far the destination (the sun) must be from the source for this
    many drones: the whole pad grid clear outside the outermost orbit."""
    return orbit_radius_m(drone_count - 1) + PAD_CLEARANCE_M + grid_pad_reach(drone_count, PAD_SPACING_M)


def planets_distance_error(source: LatLon, destination: LatLon, drone_count: int) -> str | None:
    """The operator-facing reason the destination is too close to the
    source for this many drones, or None."""
    span = math.hypot(*_to_local(destination, source))
    needed = planets_min_distance_m(drone_count)
    if span >= needed:
        return None
    return (
        f"The destination (the sun) is {span:.0f} m from the source - with "
        f"{drone_count} drones the outermost orbit is "
        f"{orbit_radius_m(drone_count - 1):.0f} m round the sun, so the destination "
        f"must be at least {math.ceil(needed)} m away to keep the take-off pads clear "
        f"outside it."
    )


# ---- The show -------------------------------------------------------------------


@dataclass(frozen=True)
class PlanetsShow:
    """A planets show ready to fly."""

    drone_count: int
    laps: int
    routes: dict[str, TrainRoute]       # keyed by planet_drone_names
    pads: list[LatLon]                  # each drone's take-off and landing pad, same order
    radii_m: list[float]
    clockwise: list[bool]
    speed_fractions: list[float]        # each drone's orbit speed, as a fraction of full speed
    orbits: list[list[LatLon]]          # each orbit as a closed ring, for drawing
    arrival_order: list[str]            # drone names in the order they fly out
    return_order: list[str]             # ... and the order they fly back

    def preview_shape(self) -> list[LatLon]:
        """The outermost orbit, for drawing on the map."""
        return list(self.orbits[-1])


def _orbit_lap(radius: float, clockwise: bool, start_angle: float) -> list[tuple[float, float]]:
    """One turn of an orbit round the origin, in local (east, north)
    metres, from `start_angle` (radians): the start excluded and the end -
    back on the start point - included."""
    steps = max(math.ceil(360.0 / ARC_STEP_DEG), math.ceil(2 * math.pi * radius / ARC_STEP_M))
    sign = -1.0 if clockwise else 1.0
    points = [
        (radius * math.cos(start_angle + sign * 2 * math.pi * k / steps),
         radius * math.sin(start_angle + sign * 2 * math.pi * k / steps))
        for k in range(1, steps + 1)
    ]
    points[-1] = (radius * math.cos(start_angle), radius * math.sin(start_angle))
    return points


def _eased_keys(t0: float, s0: float, t1: float, s1: float,
                ease_in: bool = True, ease_out: bool = True) -> list[tuple[float, float]]:
    """Timetable keyframes after (t0, s0) up to (t1, s1) for one move that
    starts and/or ends at a standstill: the speed rises smoothly (half a
    cosine) from 0 over `RAMP_S`, holds steady, and falls smoothly back to
    0 - instead of jumping straight to full speed and stopping dead. The
    move still takes exactly t1 - t0, and every drone given the same times
    covers the same FRACTION of its own move at every moment."""
    if s1 <= s0:
        return [(t1, s1)]
    span_t = t1 - t0
    r_in = min(RAMP_S, span_t / 3) if ease_in else 0.0
    r_out = min(RAMP_S, span_t / 3) if ease_out else 0.0
    v = (s1 - s0) / (span_t - (r_in + r_out) / 2)    # steady speed, ramps covering half
    keys = []
    for i in range(1, RAMP_STEPS + 1) if r_in else ():
        u = r_in * i / RAMP_STEPS
        keys.append((t0 + u, s0 + v * (u / 2 - r_in / (2 * math.pi) * math.sin(math.pi * u / r_in))))
    slow_from_t, slow_from_s = t1 - r_out, s1 - v * r_out / 2
    if slow_from_t > (keys[-1][0] if keys else t0):
        keys.append((slow_from_t, slow_from_s))
    for i in range(1, RAMP_STEPS + 1) if r_out else ():
        u = r_out * i / RAMP_STEPS
        keys.append((slow_from_t + u, slow_from_s + v * (u / 2 + r_out / (2 * math.pi) * math.sin(math.pi * u / r_out))))
    keys[-1] = (t1, s1)
    return keys


def plan_planets_show(source: LatLon, destination: LatLon, drone_count: int, laps: int = 1) -> PlanetsShow:
    """Every drone's timed track for a planets show round `destination`,
    flown out from and back to pads at `source` - see the module doc.
    Raises ValueError for an unflyable drone or lap count, or a
    destination too close to the source."""
    problem = planets_drone_count_error(drone_count)
    if problem:
        raise ValueError(problem)
    if not 1 <= laps <= MAX_PLANET_LAPS:
        raise ValueError(f"Laps must be between 1 and {MAX_PLANET_LAPS}, got {laps}.")
    problem = planets_distance_error(source, destination, drone_count)
    if problem:
        raise ValueError(problem)

    # Everything in local (east, north) metres from the sun.
    n = drone_count
    unit = SCHEDULE_UNIT_MPS
    sx, sy = _to_local(source, destination)
    span = math.hypot(sx, sy)
    bx, by = sx / span, sy / span                  # from the sun towards the source
    start_angle = math.atan2(by, bx)
    radii = [orbit_radius_m(k) for k in range(n)]
    clockwise = [orbit_clockwise(k) for k in range(n)]
    # The pads: a grid of rows at the source, facing the sun. Every drone
    # flies out through one GATE on the line, just ahead of the pad block;
    # drone k (innermost first) takes the k-th pad nearest the gate, so any
    # pad on its way there is nearer still and already empty - and coming
    # back, outermost first, it only crosses pads still to be taken. Its
    # place to start orbiting: where its orbit crosses the line.
    source_xy = (sx, sy)
    grid = grid_pad_offsets(n, (-bx, -by), PAD_SPACING_M)
    gate_back = span - grid_pad_reach(n, PAD_SPACING_M) - PAD_SPACING_M
    gate = (bx * gate_back, by * gate_back)
    grid = [(source_xy[0] + p[0], source_xy[1] + p[1]) for p in grid]
    pads = sorted(grid, key=lambda p: (math.hypot(p[0] - gate[0], p[1] - gate[1]),
                                       p[1] * bx - p[0] * by))
    entries = [(bx * r, by * r) for r in radii]
    laps_xy = [_orbit_lap(r, cw, start_angle) for r, cw in zip(radii, clockwise)]

    def polyline_m(points):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))

    lap_lengths = [polyline_m([entry, *lap]) for entry, lap in zip(entries, laps_xy)]
    transit = [polyline_m([p, gate, e]) for p, e in zip(pads, entries)]

    v = TRANSIT_SPEED * unit
    gap_t = TRANSIT_GAP_M / v
    # Every move starts and ends at a standstill, eased (`_eased_keys`): it
    # takes RAMP_S longer than at a steady speed, the ramps flying half as
    # far as steady flight would in their time.
    transit_t = [d / v + RAMP_S for d in transit]
    # Out: innermost (front pad) first, one gap apart; back: outermost first.
    show_start = max(k * gap_t + transit_t[k] for k in range(n)) + ORBIT_HOLD_S
    # Every drone one turn per lap in the same time: the outermost at
    # ORBIT_SPEED, the others slower, in proportion to their radius. All
    # the laps are one eased move, so every drone covers the same fraction
    # of its laps at every moment - the spokes stay straight while they
    # speed up and slow down too.
    t_laps = laps * lap_lengths[-1] / (ORBIT_SPEED * unit) + RAMP_S
    show_end = show_start + t_laps + ORBIT_HOLD_S

    routes: dict[str, TrainRoute] = {}
    names = planet_drone_names(n)
    for k, name in enumerate(names):
        keys = [(0.0, 0.0)]
        out = k * gap_t
        if out > 0:
            keys.append((out, 0.0))            # over its pad until its turn
        s_m = transit[k]
        keys += _eased_keys(out, 0.0, out + transit_t[k], s_m)
        keys.append((show_start, s_m))
        keys += _eased_keys(show_start, s_m, show_start + t_laps, s_m + laps * lap_lengths[k])
        s_m += laps * lap_lengths[k]
        leave = show_end + (n - 1 - k) * gap_t
        keys.append((leave, s_m))
        keys += _eased_keys(leave, s_m, leave + transit_t[k], s_m + transit[k])

        routes[name] = TrainRoute(
            start=_from_local(pads[k], destination),
            approach=[_from_local(p, destination) for p in (gate, entries[k])],
            loop=[_from_local(p, destination) for p in laps_xy[k]],
            laps=laps,
            depart=[_from_local(p, destination) for p in (gate, pads[k])],
            schedule=tuple(keys),
        )

    return PlanetsShow(
        drone_count=n,
        laps=laps,
        routes=routes,
        pads=[_from_local(p, destination) for p in pads],
        radii_m=radii,
        clockwise=clockwise,
        speed_fractions=[orbit_speed_fraction(k, n) for k in range(n)],
        orbits=[[_from_local(entries[k], destination)]
                + [_from_local(p, destination) for p in _orbit_lap(r, False, start_angle)]
                for k, r in enumerate(radii)],
        arrival_order=list(names),
        return_order=names[::-1],
    )


# ---- PDDL problem editing ---------------------------------------------------


def _strip_unary_facts(problem_text: str, predicate: str) -> str:
    """Remove every `(predicate A)` fact and its line."""
    return re.sub(rf"[ \t]*\({re.escape(predicate)}\s+[^\s()]+\)\n?", "", problem_text)


def expand_planet_drones(problem_text: str, drone_count: int) -> str:
    """Grow the template's two planets (`drone-mercury`, `drone-venus`) to
    `drone_count` drones: every `:init` fact and goal conjunct mentioning
    `drone-venus` is cloned per extra planet, each drone's `orbit-radius`
    set to its own orbit, and the orbit directions and `orbit-neighbor`
    facts rebuilt - the clones would otherwise all turn venus's way and
    neighbour mercury."""
    problem = planets_drone_count_error(drone_count)
    if problem:
        raise ValueError(problem)
    names = planet_drone_names(drone_count)
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
    for k, name in enumerate(names):
        result = _set_unary_fact_value(result, "orbit-radius", name, orbit_radius_m(k))
    result = _strip_unary_facts(result, "orbits-clockwise")
    result = _strip_unary_facts(result, "orbits-anticlockwise")
    result = _strip_pair_facts(result, "orbit-neighbor")
    facts = [
        f"\n        ({'orbits-clockwise' if orbit_clockwise(k) else 'orbits-anticlockwise'} {name})"
        for k, name in enumerate(names)
    ]
    for inner, outer in zip(names, names[1:]):
        facts.append(f"\n        (orbit-neighbor {inner} {outer})")
        facts.append(f"\n        (orbit-neighbor {outer} {inner})")
    return _add_init_facts(result, "".join(facts))


def retarget_planets_problem(problem_text: str, source: LatLon, destination: LatLon,
                             laps: int, show: PlanetsShow) -> str:
    """Put an (already expanded) planets problem at the picked source and
    destination, ask for `laps` laps, set every drone's orbit speed, and
    give every drone its turn to fly out and back in `show`'s order."""
    result = _set_latlon(_set_latlon(problem_text, "source", source), "destination", destination)
    for name, fraction in zip(planet_drone_names(show.drone_count), show.speed_fractions):
        result = _set_unary_fact_value(result, "orbit-speed", name, fraction * SCHEDULE_UNIT_MPS)
    # The template's (and its cloned) turns only describe the smallest
    # case - rebuild both chains: first drone's turn, then who follows whom.
    facts = []
    for turn, after, order in (("out-turn", "out-after", show.arrival_order),
                               ("back-turn", "back-after", show.return_order)):
        result = _strip_unary_facts(_strip_pair_facts(result, after), turn)
        facts.append(f"\n        ({turn} {order[0]})")
        facts += [f"\n        ({after} {a} {b})" for a, b in zip(order, order[1:])]
    result = _add_init_facts(result, "".join(facts))
    return _set_scalar_fluent(result, "laps-required", laps)
