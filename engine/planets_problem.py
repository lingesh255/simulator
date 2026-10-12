"""Planets (solar-system) drone show: orbits, timed tracks and PDDL editing.

    plans/planets/problem.pddl (template)
        -> expand_planet_drones          (one orbit per checked drone)
        -> retarget_planets_problem      (the picked source, the lap count)
        -> ENHSP plans the show: enter orbits, fly the laps, land
        -> plan_planets_show: every drone's own timed track

A show round the picked source - the sun - with no destination. Drone k
(0 = innermost, "mercury") flies a circular orbit of radius
`ORBIT_SPACING_M * (k + 1)`: 50 m, 100 m, ... up to 8 drones (Neptune at
400 m). Neighbouring orbits turn in OPPOSITE directions - the innermost
anticlockwise, the next clockwise, and so on - so no two neighbours ever
turn the same way.

    take off       from pads in a line due north of the source, 10 m apart
    enter orbits   every drone flies straight out along that line to the
                   north point of its own orbit (all together, so the
                   order along the line never changes)
    laps           a lap is one turn of the OUTERMOST drone. Every drone
                   flies at about the same speed, so an inner drone goes
                   round its smaller orbit several times in that lap -
                   round(outer radius / its radius) whole turns, which
                   brings every drone back to its north point at the very
                   end of each lap: the planets line up again together
    land           every drone flies straight back in along the line onto
                   its pad and lands, all at the same moment

Orbits are a whole `ORBIT_SPACING_M` apart, and the moves out and in keep
the drones in line one behind another, never closer than the 10 m pads.

TIMING. As for the star show (`engine.star_problem`): every drone flies a
timetable against one shared clock (`engine.train_keeping`) - keyframes "at
clock time t, be s metres along my route" - with every phase starting and
ending at the same clock time for every drone. A lap's length is set so the
drone with the longest lap (usually the outermost) flies it at
`ORBIT_SPEED` of full speed; the others fly theirs in the same time. One
lap is uploaded once and repeated on board (`TrainRoute.loop` x laps).
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
)
from engine.star_problem import _goal_and_span, _set_scalar_fluent
from engine.train_keeping import SCHEDULE_UNIT_MPS

PLANET_NAMES = ("mercury", "venus", "earth", "mars", "jupiter", "saturn", "uranus", "neptune")
ORBIT_SPACING_M = 50.0         # radius step from one orbit to the next
PAD_SPACING_M = 10.0           # take-off pads, in a line north of the source
MIN_PLANET_DRONES = 2          # two orbits, to turn opposite ways
MAX_PLANET_DRONES = len(PLANET_NAMES)
MAX_PLANET_LAPS = 50
ORBIT_SPEED = 0.6              # the longest lap's speed, as a fraction of full speed
SPREAD_SPEED = 0.6             # pads <-> orbits, as a fraction of full speed
ORBIT_HOLD_S = 4.0             # nominal seconds held lined up before and after the laps
ARC_STEP_DEG = 10.0            # an orbit is flown as waypoints at most this many degrees apart
ARC_STEP_M = 20.0              # ... and at most this many metres apart

_LEADER = "drone-mercury"
_MEMBER_TOKEN = "drone-venus"  # the template's second planet, cloned per drone
_START_ANGLE = math.pi / 2     # every orbit starts and ends due north of the source


def planet_drone_names(drone_count: int) -> list[str]:
    """`["drone-mercury", "drone-venus", ...]`, innermost orbit first."""
    return [f"drone-{name}" for name in PLANET_NAMES[:drone_count]]


def orbit_radius_m(index: int) -> float:
    """Radius of orbit `index` (0 = innermost)."""
    return ORBIT_SPACING_M * (index + 1)


def orbit_clockwise(index: int) -> bool:
    """Which way orbit `index` turns: innermost anticlockwise, then
    alternating outwards, so neighbouring orbits always turn opposite ways."""
    return index % 2 == 1


def orbit_turns_per_lap(index: int, drone_count: int) -> int:
    """Whole turns orbit `index` makes in one lap (one turn of the
    outermost orbit), flying at about the outermost drone's speed."""
    outer = orbit_radius_m(drone_count - 1)
    return max(1, round(outer / orbit_radius_m(index)))


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
    turns_per_lap: list[int]
    orbits: list[list[LatLon]]          # each orbit as a closed ring, for drawing

    def preview_shape(self) -> list[LatLon]:
        """The outermost orbit, for drawing on the map."""
        return list(self.orbits[-1])


def _orbit_lap(radius: float, clockwise: bool, turns: int) -> list[tuple[float, float]]:
    """One lap of an orbit in local (east, north) metres: `turns` whole
    turns from the north point, the start excluded and the end - back on
    the north point - included."""
    per_turn = max(math.ceil(360.0 / ARC_STEP_DEG), math.ceil(2 * math.pi * radius / ARC_STEP_M))
    steps = per_turn * turns
    sign = -1.0 if clockwise else 1.0
    points = [
        (radius * math.cos(_START_ANGLE + sign * 2 * math.pi * k / per_turn),
         radius * math.sin(_START_ANGLE + sign * 2 * math.pi * k / per_turn))
        for k in range(1, steps + 1)
    ]
    points[-1] = (0.0, radius)          # exactly back on the north point
    return points


def plan_planets_show(source: LatLon, drone_count: int, laps: int = 1) -> PlanetsShow:
    """Every drone's timed track for a planets show round `source` - see
    the module doc. Raises ValueError for an unflyable drone or lap count."""
    problem = planets_drone_count_error(drone_count)
    if problem:
        raise ValueError(problem)
    if not 1 <= laps <= MAX_PLANET_LAPS:
        raise ValueError(f"Laps must be between 1 and {MAX_PLANET_LAPS}, got {laps}.")

    n = drone_count
    unit = SCHEDULE_UNIT_MPS
    radii = [orbit_radius_m(k) for k in range(n)]
    clockwise = [orbit_clockwise(k) for k in range(n)]
    turns = [orbit_turns_per_lap(k, n) for k in range(n)]
    pads = [(0.0, PAD_SPACING_M * (k + 1)) for k in range(n)]
    entries = [(0.0, r) for r in radii]
    laps_xy = [_orbit_lap(r, cw, t) for r, cw, t in zip(radii, clockwise, turns)]

    def polyline_m(points):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))

    lap_lengths = [polyline_m([entry, *lap]) for entry, lap in zip(entries, laps_xy)]

    # Shared phase lengths (nominal seconds) - the same for every drone.
    t_spread = max(e[1] - p[1] for p, e in zip(pads, entries)) / (SPREAD_SPEED * unit)
    t_lap = max(lap_lengths) / (ORBIT_SPEED * unit)

    routes: dict[str, TrainRoute] = {}
    for k, name in enumerate(planet_drone_names(n)):
        out_m = entries[k][1] - pads[k][1]
        keys = [(0.0, 0.0)]
        t, s_m = t_spread, out_m
        keys.append((t, s_m))
        t += ORBIT_HOLD_S
        keys.append((t, s_m))
        for _lap in range(laps):
            t += t_lap
            s_m += lap_lengths[k]
            keys.append((t, s_m))
        t += ORBIT_HOLD_S
        keys.append((t, s_m))
        t += t_spread
        s_m += out_m
        keys.append((t, s_m))

        routes[name] = TrainRoute(
            start=_from_local(pads[k], source),
            approach=[_from_local(entries[k], source)],
            loop=[_from_local(p, source) for p in laps_xy[k]],
            laps=laps,
            depart=[_from_local(pads[k], source)],
            schedule=tuple(keys),
        )

    return PlanetsShow(
        drone_count=n,
        laps=laps,
        routes=routes,
        pads=[_from_local(p, source) for p in pads],
        radii_m=radii,
        clockwise=clockwise,
        turns_per_lap=turns,
        orbits=[[_from_local(entries[k], source)] + [_from_local(p, source) for p in _orbit_lap(r, False, 1)]
                for k, r in enumerate(radii)],
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


def retarget_planets_problem(problem_text: str, source: LatLon, laps: int) -> str:
    """Put an (already expanded) planets problem at the picked source and
    ask for `laps` laps."""
    return _set_scalar_fluent(_set_latlon(problem_text, "source", source), "laps-required", laps)
