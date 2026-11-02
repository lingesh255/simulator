"""Figure-8 formation: geometry, capacity check and PDDL problem editing.

    Swarm-Planning/Plans/figure8formation/problem.pddl (template)
        -> expand_figure8_drones   (one train slot per checked drone)
        -> retarget_figure8_problem (the operator's 4 clicked points)
        -> ENHSP plans the leader's trip: base -> 8's middle -> laps -> base
        -> figure8_train_routes: every drone's own dense route

The operator clicks four points - the BASE the drones fly from and back to,
then the LEFT end of the 8, the MIDDLE point where its two lobes cross, and
the RIGHT end - and the mission is:

    take off from the base -> fly to the middle point -> fly the 8 `laps`
    times -> leave from the middle point -> fly back to the base -> land

THE 8 (`figure8_geometry`). Each lobe is half of a lemniscate of Gerono,
scaled so its far end lands on the clicked end point:

    p(t) = C + a sin(t) u + a sin(t) cos(t) n

with `u` the unit vector from the crossing C toward that lobe's end, `a`
the C->end distance, and `n` perpendicular to `u`. The two lobes meet at C
at right angles (the X in the middle of the 8), each lobe is as long as it
is wide, and the lobes may differ in size if the middle isn't clicked half
way. The same shape can be flown four ways (either lobe first, either way
round); `plan_figure8_mission` picks the one that suits the base.

THE SPREAD. The drones are spread EVENLY round the whole 8: the lap is cut
into `slot_count` equal slots (`slot_spacing_m` = lap length / slots) and
every drone holds its own slot, so while they are all on the 8 they fill it
at equal distances instead of following one another in a bunch. They still
talk only to the drone directly in front of and behind them, and between
leaving its pad and reaching its landing pad every drone flies the same
SHARED path - base -> 8 -> base - one slot behind the drone in front. Each
drone flies `laps + 1` laps: the extra lap is the time it takes the drones
to fill the 8 one after another and to leave it again, so the whole 8 is
held fully spread for at least the `laps` the operator asked for.

The slot count is the drone count when that is safe, else one or more
slots more (`figure8_slot_count`). The 8 crosses itself in the middle, so
two drones whose slots are a lobe-length apart along the path would reach
the crossing together - e.g. any even number of drones on an 8 with equal
lobes. The slot spacing is chosen so that at the crossing any two drones
are at least `FIGURE8_CROSSING_CLEARANCE_M` apart along the path, which on
two branches crossing at right angles is `FIGURE8_SPACING_M` (10 m) apart
in a straight line. When an extra slot is needed it is left empty, as one
larger gap between the last drone and the leader.

The way out to the 8 and back:

    pads         a compact block of landing pads at the base, 10 m apart,
                 just behind the base point (on the side away from the 8)
    takeoff      each drone climbs over its pad, then slides into line at
                 the base point when its turn comes - nearest pad first
    outbound     the shortest path from the base that meets the 8 at its
                 middle point tangentially, turning no tighter than
                 `FIGURE8_TURN_RADIUS_M` (a Dubins path: arcs and straight
                 lines) - so the train flows onto the 8 without a sharp
                 corner, which would bunch the drones up
    laps         the 8, `laps` times, starting and ending at the middle
    inbound      the same kind of path leaving the middle tangentially, back
                 to the base
    landing      at the base the train peels off onto the pads: the leader
                 to the farthest pad, every later drone to a nearer one - so
                 no drone ever flies over a pad that is already taken

Entering and leaving the 8 tangentially, then turning back to the base,
means one of the base legs generally crosses one loop of the 8 while other
drones are still flying it.

CAPACITY. Every pair of drones is a whole number of slots apart along the
shared path, always. So `plan_figure8_mission` checks the whole path
directly (`_closest_conflict`): for every point of it and every pair
distance j * slot_spacing_m that some two drones have, the two points must
be at least `FIGURE8_SPACING_M` apart in space - the crossing, the base legs
cutting across a lobe, the slide off the pads, all of it. The crossing rule
above (the same inequality is in domain.pddl) is what makes the 8 itself
pass; `figure8_max_drones` is the most drones it allows.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass

from engine.pddl_problem import (
    LatLon,
    _add_init_facts,
    _clone_named_lines,
    _find_matching_paren,
    _from_local,
    _section_span,
    _set_fact_value,
    _set_latlon,
    _set_unary_fact_value,
    _strip_pair_facts,
    _to_local,
)

# Closest any two drones may ever come, in a straight line - and the
# spacing of the landing pads.
FIGURE8_SPACING_M = 10.0
# Gap along the path two drones must have when they reach the middle
# crossing on its two branches. Drones approaching a right-angle crossing
# whose arc positions differ by d are never closer than d / sqrt(2) - so
# this keeps them at least FIGURE8_SPACING_M apart.
FIGURE8_CROSSING_CLEARANCE_M = FIGURE8_SPACING_M * math.sqrt(2.0)
# Smallest slot spacing that can keep the crossing clear at all (the gap at
# the crossing is never more than half a slot).
MIN_FIGURE8_SLOT_M = 2.0 * FIGURE8_CROSSING_CLEARANCE_M
# Smallest crossing->end distance a lobe may have: anything tighter is not
# a flyable loop (and not two separate clicks, really).
MIN_FIGURE8_LOBE_M = 2.0 * FIGURE8_SPACING_M
# Smallest base->middle distance.
MIN_FIGURE8_BASE_M = 3.0 * FIGURE8_SPACING_M
# Tightest turn on the base legs.
FIGURE8_TURN_RADIUS_M = 2.5 * FIGURE8_SPACING_M
# How finely the flown path follows the true curve: one waypoint every this
# many metres, so the drones trace the 8 itself rather than a polygon.
FIGURE8_SAMPLE_M = 2.0
# Speed the formation flies at - an 8 is about shape, not getting anywhere.
FIGURE8_CRUISE_MPS = 5.0
# Vertical separation at the middle crossing: every drone flies the SECOND
# pass through the crossing (the branch from the first lobe into the second)
# this much lower, so the two streams crossing there are apart in height as
# well as along the path. Full depth within FIGURE8_DIP_FLAT_M of the
# crossing, easing back to cruise height over FIGURE8_DIP_RAMP_M either side.
FIGURE8_CROSSING_DIP_M = 10.0
FIGURE8_DIP_FLAT_M = 25.0
FIGURE8_DIP_RAMP_M = 40.0
MIN_FIGURE8_DRONES = 2  # a leader plus at least one follower to talk to
FIGURE8_LAPS = 1  # default times round the 8
MAX_FIGURE8_LAPS = 50
# The PDDL battery is abstract (100 units) and the real per-drone battery
# check runs on each drone's own profile before flight (see
# MainWindow._start_area_coverage_mission), so the whole mission - however
# many laps - is budgeted this many units, keeping ENHSP from refusing a long
# mission over a made-up energy rate.
_PDDL_MISSION_ENERGY = 40.0

_LEADER = "drone-lead"
_FOLLOWER_TOKEN = "drone-f1"  # the template's single follower, cloned per drone

# Location objects in problem.pddl.
BASE_LOCATION = "base"
# The 8's key waypoints in flying order, each at curve parameter `t`:
# into the first lobe from the middle, round its far end, back through the
# middle, round the second lobe and back to the middle. Named by flying
# order, not left/right, because which lobe comes first depends on where
# the base is (see plan_figure8_mission).
FIGURE8_KEY_POINTS: tuple[tuple[str, float], ...] = (
    ("crossing-in", 0.0),
    ("lobe1-a", 0.25 * math.pi),
    ("lobe1-end", 0.5 * math.pi),
    ("lobe1-b", 0.75 * math.pi),
    ("crossing-mid", math.pi),
    ("lobe2-a", 1.25 * math.pi),
    ("lobe2-end", 1.5 * math.pi),
    ("lobe2-b", 1.75 * math.pi),
    ("crossing-out", 2.0 * math.pi),
)

_SAMPLES_PER_LOBE = 2000  # dense sampling used only to measure arc length
_DUBINS_SAMPLE_M = 0.5    # dense sampling of the base legs


class Figure8ShapeError(ValueError):
    """The clicked points don't describe a flyable mission at all (as
    opposed to "not with this many drones") - the operator should click
    again."""


# ---- The 8 itself -----------------------------------------------------------


@dataclass(frozen=True)
class Figure8Geometry:
    """A figure-8 through the operator's three clicks, in flying order."""

    left: LatLon
    crossing: LatLon
    right: LatLon
    # One lap of the 8 at equal arc steps of `step_m`, starting (index 0) at
    # the crossing. Does NOT repeat the start point at the end - it is a ring.
    ring: list[LatLon]
    step_m: float
    right_lobe_m: float
    left_lobe_m: float
    # (name, point, arc metres from the start) for every FIGURE8_KEY_POINTS
    # entry, in the same order.
    key_points: list[tuple[str, LatLon, float]]
    # Unit (east, north) direction the lap leaves the crossing in - and,
    # since the 8 is a smooth closed curve, also the direction it arrives
    # back in.
    entry_direction: tuple[float, float]
    # The ring densely, in local metres around the crossing (for the path
    # checks) - one lap, closed.
    dense_xy: list[tuple[float, float]]

    @property
    def loop_length_m(self) -> float:
        return self.right_lobe_m + self.left_lobe_m

    @property
    def max_drones(self) -> int:
        return figure8_max_drones(self.loop_length_m, self.right_lobe_m)


def figure8_crossing_gap(loop_m: float, lobe_m: float, slots: int) -> float:
    """Smallest gap along the path between two of `slots` equally spaced
    slots when one of them is at the middle crossing: the crossing is passed
    at 0 and again one lobe (`lobe_m`) later, so two slots meet there when
    a whole number of slots is close to a lobe length. Either lobe gives the
    same answer."""
    slot_m = loop_m / slots
    r = lobe_m % slot_m
    return min(r, slot_m - r)


def _slot_counts(loop_m: float, lobe_m: float, drone_count: int):
    """Slot counts that keep the crossing clear for `drone_count` drones,
    fewest first (fewest = the most even spread)."""
    slots = max(drone_count, MIN_FIGURE8_DRONES)
    while loop_m / slots >= MIN_FIGURE8_SLOT_M:
        if figure8_crossing_gap(loop_m, lobe_m, slots) >= FIGURE8_CROSSING_CLEARANCE_M - 1e-9:
            yield slots
        slots += 1


def figure8_slot_count(loop_m: float, lobe_m: float, drone_count: int) -> int | None:
    """Fewest slots, at least `drone_count`, that keep the crossing clear -
    None if the 8 is too small for that many drones."""
    return next(_slot_counts(loop_m, lobe_m, drone_count), None)


def figure8_max_drones(loop_m: float, lobe_m: float) -> int:
    """Most drones an 8 this size can hold spread evenly, one per slot, with
    the crossing clear - see the module doc. 0 if not even two fit."""
    best = 0
    slots = MIN_FIGURE8_DRONES
    while loop_m / slots >= MIN_FIGURE8_SLOT_M:
        if figure8_crossing_gap(loop_m, lobe_m, slots) >= FIGURE8_CROSSING_CLEARANCE_M - 1e-9:
            best = slots
        slots += 1
    return best


def _curve(t: float, frames) -> tuple[float, float]:
    """Point at parameter `t` (0..2pi) of the 8, in local metres around the
    crossing. `frames` is ((a_R, u_R, n_R), (a_L, u_L, n_L))."""
    (a, u, n) = frames[0] if math.sin(t) >= 0.0 and t <= math.pi else frames[1]
    s, c = math.sin(t), math.cos(t)
    along = a * abs(s)
    across = a * s * c
    return (along * u[0] + across * n[0], along * u[1] + across * n[1])


def _arc_lengths(pts: list[tuple[float, float]]) -> list[float]:
    cum = [0.0]
    for p, q in zip(pts, pts[1:]):
        cum.append(cum[-1] + math.hypot(q[0] - p[0], q[1] - p[1]))
    return cum


def _resample(pts: list[tuple[float, float]], cum: list[float], step: float) -> list[tuple[float, float]]:
    """Points every `step` metres along a dense polyline, first point
    included, last point included only if it falls on a step."""
    out: list[tuple[float, float]] = []
    j = 0
    s = 0.0
    while s <= cum[-1] + 1e-6:
        while j + 1 < len(cum) - 1 and cum[j + 1] < s:
            j += 1
        span = cum[j + 1] - cum[j]
        f = (s - cum[j]) / span if span > 0 else 0.0
        f = min(1.0, max(0.0, f))
        out.append((pts[j][0] + (pts[j + 1][0] - pts[j][0]) * f, pts[j][1] + (pts[j + 1][1] - pts[j][1]) * f))
        s += step
    return out


def figure8_geometry(
    left: LatLon,
    crossing: LatLon,
    right: LatLon,
    *,
    sample_m: float = FIGURE8_SAMPLE_M,
    mirror: bool = False,
    reverse: bool = False,
) -> Figure8Geometry:
    """Build the 8 through the three clicked points - see the module doc.

    `mirror` swaps which side of the left-right axis the lap leaves the
    middle on; `reverse` flies the whole lap the other way round (left lobe
    first). The shape is the same either way.

    Raises `Figure8ShapeError` if either lobe would be shorter than
    `MIN_FIGURE8_LOBE_M` (an end clicked too close to the middle), or if the
    two ends are not on opposite sides of the middle.
    """
    right_local = _to_local(right, crossing)
    left_local = _to_local(left, crossing)
    a_r = math.hypot(*right_local)
    a_l = math.hypot(*left_local)
    if min(a_r, a_l) < MIN_FIGURE8_LOBE_M:
        raise Figure8ShapeError(
            "The figure-8 is too small: each end point must be at least "
            f"{MIN_FIGURE8_LOBE_M:.0f} m from the middle crossing point "
            f"(left end is {a_l:.1f} m away, right end {a_r:.1f} m)."
        )
    u_r = (right_local[0] / a_r, right_local[1] / a_r)
    u_l = (left_local[0] / a_l, left_local[1] / a_l)
    if u_r[0] * u_l[0] + u_r[1] * u_l[1] > 0.0:
        raise Figure8ShapeError(
            "The left and right end points must be on opposite sides of the "
            "middle crossing point (click the base, the left end, the middle, "
            "then the right end)."
        )
    # Both lobes take the same side at the crossing, so the curve passes
    # straight through C (an X), instead of doubling back on itself.
    side = -1.0 if mirror else 1.0
    n_r = (-u_r[1] * side, u_r[0] * side)
    n_l = (u_l[1] * side, -u_l[0] * side)
    frames = ((a_r, u_r, n_r), (a_l, u_l, n_l))

    # Dense polyline over the full lap in FLYING order, with arc length.
    total_samples = 2 * _SAMPLES_PER_LOBE
    ts = [2.0 * math.pi * i / total_samples for i in range(total_samples + 1)]
    if reverse:
        ts = [2.0 * math.pi - t for t in ts]
    pts = [_curve(t, frames) for t in ts]
    cum = _arc_lengths(pts)
    first_lobe_m = cum[_SAMPLES_PER_LOBE]
    loop_m = cum[-1]
    right_lobe_m, left_lobe_m = (loop_m - first_lobe_m, first_lobe_m) if reverse else (first_lobe_m, loop_m - first_lobe_m)

    # Equal steps as close to `sample_m` as the loop length allows, so the
    # ring closes exactly.
    n_steps = max(8, round(loop_m / sample_m))
    step_m = loop_m / n_steps
    ring_xy = _resample(pts, cum, step_m)[:n_steps]
    ring = [_from_local(p, crossing) for p in ring_xy]

    key_points: list[tuple[str, LatLon, float]] = []
    for name, t in FIGURE8_KEY_POINTS:
        index = round(t / (2.0 * math.pi) * total_samples)
        key_points.append((name, _from_local(pts[index], crossing), cum[index]))

    dx, dy = pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]
    norm = math.hypot(dx, dy)
    return Figure8Geometry(
        left=left,
        crossing=crossing,
        right=right,
        ring=ring,
        step_m=step_m,
        right_lobe_m=right_lobe_m,
        left_lobe_m=left_lobe_m,
        key_points=key_points,
        entry_direction=(dx / norm, dy / norm),
        dense_xy=pts,
    )


def figure8_capacity_error(geometry: Figure8Geometry, drone_count: int) -> str | None:
    """The crossing-capacity part of the check (see module doc): a message
    for the operator if `drone_count` drones can't fly this 8, else None."""
    if drone_count < MIN_FIGURE8_DRONES:
        return (
            f"A figure-8 formation needs at least {MIN_FIGURE8_DRONES} drones "
            f"(a leader and a follower to keep spacing with). "
            f"{drone_count} {'is' if drone_count == 1 else 'are'} checked."
        )
    if figure8_slot_count(geometry.loop_length_m, geometry.right_lobe_m, drone_count) is not None:
        return None
    fit = geometry.max_drones
    return (
        f"This operation can't be done with {drone_count} drones on this figure-8.\n\n"
        f"The drones are spread evenly round the 8, which is "
        f"{geometry.loop_length_m:.0f} m around - {geometry.loop_length_m / drone_count:.0f} m "
        f"each for {drone_count} drones. To keep every two drones at least "
        f"{FIGURE8_SPACING_M:.0f} m apart where the 8 crosses itself, each drone "
        f"needs at least {MIN_FIGURE8_SLOT_M:.0f} m of the 8.\n\n"
        + (
            f"This 8 fits at most {fit} drones. " if fit >= MIN_FIGURE8_DRONES
            else "This 8 can't fit even two drones. "
        )
        + "Uncheck drones, or click the left and right end points further from the middle."
    )


# ---- The whole mission: base -> 8 -> base ------------------------------------


def _mod2pi(angle: float) -> float:
    return angle % (2.0 * math.pi)


def _dubins_words(alpha: float, beta: float, d: float) -> list[tuple[str, tuple[float, float, float]]]:
    """Candidate Dubins paths (word, (t, p, q) segment lengths in units of
    the turn radius) between normalised poses - the standard closed forms.
    Each candidate is checked by `_dubins` against the real end pose."""
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)
    words = []
    p2 = 2 + d * d - 2 * cab + 2 * d * (sa - sb)
    if p2 >= 0:
        tmp = math.atan2(cb - ca, d + sa - sb)
        words.append(("LSL", (_mod2pi(-alpha + tmp), math.sqrt(p2), _mod2pi(beta - tmp))))
    p2 = 2 + d * d - 2 * cab + 2 * d * (sb - sa)
    if p2 >= 0:
        tmp = math.atan2(ca - cb, d - sa + sb)
        words.append(("RSR", (_mod2pi(alpha - tmp), math.sqrt(p2), _mod2pi(-beta + tmp))))
    p2 = -2 + d * d + 2 * cab + 2 * d * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        words.append(("LSR", (_mod2pi(-alpha + tmp), p, _mod2pi(-beta + tmp))))
    p2 = d * d - 2 + 2 * cab - 2 * d * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        words.append(("RSL", (_mod2pi(alpha - tmp), p, _mod2pi(beta - tmp))))
    tmp = (6.0 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8.0
    if abs(tmp) <= 1:
        p = _mod2pi(2 * math.pi - math.acos(tmp))
        t = _mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
        words.append(("RLR", (t, p, _mod2pi(alpha - beta - t + p))))
    tmp = (6.0 - d * d + 2 * cab + 2 * d * (sb - sa)) / 8.0
    if abs(tmp) <= 1:
        p = _mod2pi(2 * math.pi - math.acos(tmp))
        t = _mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
        words.append(("LRL", (t, p, _mod2pi(beta - alpha - t + p))))
    return words


def _dubins(p0, heading0: float, p1, heading1: float, radius: float) -> list[tuple[float, float]]:
    """Shortest path from point `p0` facing `heading0` to point `p1`
    facing `heading1` (headings as math angles, radians) that never turns
    tighter than `radius` - densely sampled, both ends included. Every
    closed-form candidate is integrated and kept only if it really ends at
    `p1` facing `heading1`."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    d = math.hypot(dx, dy) / radius
    theta = _mod2pi(math.atan2(dy, dx))
    alpha, beta = _mod2pi(heading0 - theta), _mod2pi(heading1 - theta)

    best = None
    for word, lengths in _dubins_words(alpha, beta, d):
        pts = [p0]
        x, y, h = p0[0], p0[1], heading0
        for kind, seg in zip(word, lengths):
            length = seg * radius
            n = max(1, int(math.ceil(length / _DUBINS_SAMPLE_M)))
            ds = length / n
            curvature = {"S": 0.0, "L": 1.0 / radius, "R": -1.0 / radius}[kind]
            for _ in range(n):
                if curvature == 0.0:
                    x += ds * math.cos(h)
                    y += ds * math.sin(h)
                else:
                    turn = curvature * ds
                    x += (math.sin(h + turn) - math.sin(h)) / curvature
                    y += (math.cos(h) - math.cos(h + turn)) / curvature
                    h += turn
                pts.append((x, y))
        end_error = math.hypot(x - p1[0], y - p1[1])
        heading_error = abs((h - heading1 + math.pi) % (2 * math.pi) - math.pi)
        total = sum(lengths)
        if end_error < 0.01 * radius and heading_error < 1e-3 and (best is None or total < best[0]):
            best = (total, pts)
    if best is None:
        raise Figure8ShapeError("Could not find a flyable path between the base and the 8.")
    pts = best[1]
    pts[-1] = p1
    return pts


@dataclass(frozen=True)
class TrainRoute:
    """One drone's whole mission, structured so the flight layer can repeat
    the lap on board instead of being sent every lap written out:

        start -> approach... -> loop... (x laps) -> depart...

    `start` is the drone's takeoff pad and `depart[-1]` its landing pad.

    For keeping the train together: the first `shared_from_m` metres of the
    route are the drone's own way from its pad to the base, after which it
    is on the path every drone shares; it belongs `slot_m` metres behind the
    front of the train along that shared path."""

    start: LatLon
    approach: list[LatLon]
    loop: list[LatLon]
    laps: int
    depart: list[LatLon]
    shared_from_m: float = 0.0
    slot_m: float = 0.0
    # Instead of a place in a line: a timetable of (pace time, metres along
    # this route) keyframes - see engine.train_keeping. Used by formations
    # that change shape (the star plan).
    schedule: tuple[tuple[float, float], ...] = ()
    # Fastest the formation should cruise (m/s), below the drones' own top
    # speed - 0 for no limit.
    cruise_mps: float = 0.0
    # Height change (metres, negative = lower) at each `loop` point, same
    # length as `loop` - empty for a loop flown level.
    loop_alt_offsets: tuple[float, ...] = ()

    def expanded(self) -> list[LatLon]:
        return [self.start, *self.approach, *(self.loop * self.laps), *self.depart]


@dataclass(frozen=True)
class Figure8Mission:
    """A figure-8 mission ready to fly: the 8, the base, and the base legs."""

    geometry: Figure8Geometry
    base: LatLon
    drone_count: int
    laps: int                  # laps flown with the whole 8 filled, as asked
    slot_count: int            # equal slots round the 8 (>= drone_count)
    # Pads in landing order: drone k (0 = leader) lands on landing_pads[k]
    # and takes off from takeoff_pads[k] (the same pads, reversed).
    landing_pads: list[LatLon]
    takeoff_pads: list[LatLon]
    pad_rows: int
    pad_cols: int
    outbound: list[LatLon]     # base -> middle of the 8 (base excluded, middle included)
    inbound: list[LatLon]      # middle -> base (middle excluded, base included)
    outbound_m: float
    inbound_m: float

    @property
    def slot_spacing_m(self) -> float:
        """Distance along the path between neighbouring drones."""
        return self.geometry.loop_length_m / self.slot_count

    @property
    def flown_laps(self) -> int:
        """Laps every drone flies: one more than asked, to fill and empty the 8."""
        return self.laps + 1

    def preview_path(self) -> list[LatLon]:
        """Base -> 8 -> base, for drawing on the map."""
        return [self.base, *self.outbound, *self.geometry.ring[1:], self.geometry.crossing, *self.inbound]


def _closest_conflict(
    path: list[tuple[float, float]], stride: int, drone_count: int
) -> float | None:
    """Worst separation shortfall (metres) on `path` - a polyline sampled
    so that `stride` samples are exactly one slot - between two points that
    two of the drones occupy at the same moment (a whole number of slots
    apart, up to the train's length); None if every such pair stays at
    least `FIGURE8_SPACING_M` apart."""
    worst = None
    for j in range(1, drone_count):
        offset = j * stride
        for i in range(len(path) - offset):
            (xi, yi), (xj, yj) = path[i], path[i + offset]
            short = FIGURE8_SPACING_M - math.hypot(xj - xi, yj - yi)
            if short > 1e-6 and (worst is None or short > worst):
                worst = short
    return worst


def _segments_cross(p1, p2, p3, p4) -> bool:
    """Whether segments p1-p2 and p3-p4 properly intersect."""
    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    d1, d2 = orient(p3, p4, p1), orient(p3, p4, p2)
    d3, d4 = orient(p1, p2, p3), orient(p1, p2, p4)
    return d1 * d2 < 0 and d3 * d4 < 0


def _base_legs_cross_pattern(mission: "Figure8Mission") -> bool:
    """Whether the outbound or inbound leg cuts across the 8 anywhere other
    than where it joins it at the middle. Safe either way (the conflict
    check is what guarantees separation), but a leg that swings through a
    loop to come in from the far side is not the trip anyone expects."""
    crossing = mission.geometry.crossing
    near = 1.5 * FIGURE8_SPACING_M
    ring = [_to_local(p, crossing) for p in mission.geometry.ring]
    ring.append(ring[0])
    for leg in ([mission.base, *mission.outbound], [crossing, *mission.inbound]):
        xy = [_to_local(p, crossing) for p in leg]
        for a, b in zip(xy, xy[1:]):
            if min(math.hypot(*a), math.hypot(*b)) < near:
                continue
            for c, d in zip(ring, ring[1:]):
                if _segments_cross(a, b, c, d):
                    return True
    return False


def _pad_offsets(drone_count: int) -> tuple[list[tuple[int, float]], int, int]:
    """(row, column) of every pad in LANDING order, plus rows and columns.

    The block is roughly square, rows counted from the base away from the
    8 (row 0 level with the base point), columns as signed offsets from the
    base's centre line. The column count is always odd, so the centre line
    runs THROUGH a column of pads: a drone sliding along it is a full
    spacing from the pads either side, never half of one. The nearest rows
    are filled first, centre outwards.

    Landing order: the farthest row first; within a row the left side from
    the outside in, then the right side from the outside in, then the
    centre. A drone heading for its pad only ever passes over pads that are
    taken LATER - nearer rows, and inner pads of its own row - so it never
    flies over a drone already on the ground. Takeoff is the same order
    reversed, for the same reason."""
    cols = max(1, math.ceil(math.sqrt(drone_count)))
    cols += 1 - cols % 2
    rows = math.ceil(drone_count / cols)
    centre = (cols - 1) / 2.0
    pads: list[tuple[int, float]] = []
    for row in range(rows):
        for c in sorted((c - centre for c in range(cols)), key=lambda c: (abs(c), c)):
            pads.append((row, c))
    pads = pads[:drone_count]

    def landing_key(pad: tuple[int, float]):
        row, c = pad
        side = 0 if c < 0 else (1 if c > 0 else 2)
        return (-row, side, -abs(c))

    return sorted(pads, key=landing_key), rows, cols


def _line(a, b, step: float) -> list[tuple[float, float]]:
    """Points from `a` (excluded) to `b` (included), no more than `step` apart."""
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, math.ceil(length / step - 1e-9)) if length > 1e-6 else 0
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(1, n + 1)]


def _build_mission(
    left: LatLon, crossing: LatLon, right: LatLon, base: LatLon,
    drone_count: int, laps: int, slot_count: int, *, mirror: bool, reverse: bool,
) -> tuple[Figure8Mission, list[tuple[float, float]], int]:
    """One orientation's mission, plus its whole path (densely, local
    metres) for the conflict check and how many samples of it make a slot."""
    geometry = figure8_geometry(left, crossing, right, mirror=mirror, reverse=reverse)
    step = FIGURE8_SAMPLE_M     # waypoint spacing of the base legs
    pad = FIGURE8_SPACING_M     # landing pad spacing
    b = _to_local(base, crossing)
    dist = math.hypot(*b)
    toward = (-b[0] / dist, -b[1] / dist)          # base -> middle
    t = geometry.entry_direction
    toward_heading = math.atan2(toward[1], toward[0])
    entry_heading = math.atan2(t[1], t[0])

    # Leave the base facing the middle (straight on from the slot line) and
    # join the 8 along its own direction; leave it the same way and come
    # back to the base facing away from the 8 (straight on into the slots).
    out_dense = _dubins(b, toward_heading, (0.0, 0.0), entry_heading, FIGURE8_TURN_RADIUS_M)
    in_dense = _dubins((0.0, 0.0), entry_heading, b, toward_heading + math.pi, FIGURE8_TURN_RADIUS_M)
    out_cum, in_cum = _arc_lengths(out_dense), _arc_lengths(in_dense)
    out_step = out_cum[-1] / max(1, round(out_cum[-1] / step))
    in_step = in_cum[-1] / max(1, round(in_cum[-1] / step))
    outbound_xy = _resample(out_dense, out_cum, out_step)[1:]
    inbound_xy = _resample(in_dense, in_cum, in_step)[1:]
    # Guard the end points exactly (resampling can stop a hair short).
    outbound_xy[-1] = (0.0, 0.0)
    inbound_xy[-1] = b

    away = (-toward[0], -toward[1])
    side = (-away[1], away[0])                      # left, facing away from the 8
    order, rows, cols = _pad_offsets(drone_count)
    pads_xy = [
        (b[0] + away[0] * r * pad + side[0] * c * pad, b[1] + away[1] * r * pad + side[1] * c * pad)
        for r, c in order
    ]

    mission = Figure8Mission(
        geometry=geometry,
        base=base,
        drone_count=drone_count,
        laps=laps,
        slot_count=slot_count,
        landing_pads=[_from_local(p, crossing) for p in pads_xy],
        takeoff_pads=[_from_local(p, crossing) for p in reversed(pads_xy)],
        pad_rows=rows,
        pad_cols=cols,
        outbound=[_from_local(p, crossing) for p in outbound_xy],
        inbound=[_from_local(p, crossing) for p in inbound_xy],
        outbound_m=out_cum[-1],
        inbound_m=in_cum[-1],
    )

    # The whole path the drones slide along together: the pads' centre line
    # in, the outbound leg, the laps (the train is shorter than a lap, so
    # three cover every way two drones can be spread over them), the inbound
    # leg, and the centre line out - sampled a whole number of times per slot.
    stride = max(1, math.ceil(mission.slot_spacing_m / 1.0))
    h = mission.slot_spacing_m / stride
    depth = (b[0] + away[0] * (rows - 1) * pad, b[1] + away[1] * (rows - 1) * pad)
    pieces = [
        [depth, b],
        out_dense,
        *([geometry.dense_xy] * min(mission.flown_laps, 3)),
        in_dense,
        [b, depth],
    ]
    dense: list[tuple[float, float]] = []
    for piece in pieces:
        dense.extend(piece if not dense else piece[1:])
    path = _resample(dense, _arc_lengths(dense), h)
    # The pads themselves must sit clear of the 8.
    ring = geometry.dense_xy[::10]
    pads_clear = all(
        math.hypot(px - rx, py - ry) >= 2 * pad for px, py in pads_xy for rx, ry in ring
    )
    if not pads_clear:
        path = [path[0]] * (stride + 1) + path      # forces a conflict below
    return mission, path, stride


def plan_figure8_mission(
    base: LatLon,
    left: LatLon,
    crossing: LatLon,
    right: LatLon,
    drone_count: int,
    laps: int = FIGURE8_LAPS,
) -> Figure8Mission:
    """Validate the four clicks and the drone count, and build the mission.

    Raises `Figure8ShapeError` for clicks that can't make a mission at all
    (8 too small, ends on the same side, base too close to the middle), and
    a plain `ValueError` - with the operator-facing message, including the
    most drones this layout DOES fit - when it can't be flown with
    `drone_count` drones.

    Of the four ways round the 8, the preferred one has base legs that
    don't cut across the 8 if there is one, then the shortest base legs;
    the others are tried in order if that one has a conflict.
    """
    if not 1 <= laps <= MAX_FIGURE8_LAPS:
        raise ValueError(f"Laps must be between 1 and {MAX_FIGURE8_LAPS}, got {laps}.")
    geometry = figure8_geometry(left, crossing, right)
    b = _to_local(base, crossing)
    base_m = math.hypot(*b)
    if base_m < MIN_FIGURE8_BASE_M:
        raise Figure8ShapeError(
            f"The base is only {base_m:.0f} m from the middle of the 8 - put it at "
            f"least {MIN_FIGURE8_BASE_M:.0f} m away so the drones can turn onto the "
            f"8 smoothly."
        )
    capacity = figure8_capacity_error(geometry, drone_count)
    if capacity:
        raise ValueError(capacity)

    slot_choices = list(_slot_counts(geometry.loop_length_m, geometry.right_lobe_m, drone_count))

    options = []
    for mirror in (False, True):
        for reverse in (False, True):
            mission, _path, _stride = _build_mission(
                left, crossing, right, base, drone_count, laps, slot_choices[0],
                mirror=mirror, reverse=reverse,
            )
            # Rounded so near-ties between mirror-image options don't hinge
            # on floating-point noise.
            legs_m = round(mission.outbound_m + mission.inbound_m, 1)
            options.append((_base_legs_cross_pattern(mission), legs_m, mirror, reverse))
    options.sort()
    options = [(mirror, reverse) for _crosses, _legs_m, mirror, reverse in options]

    def _fits(n: int, slots: int, mirror: bool, reverse: bool):
        mission, path, stride = _build_mission(
            left, crossing, right, base, n, laps, slots, mirror=mirror, reverse=reverse
        )
        return mission, _closest_conflict(path, stride, n) is None

    # The most even spread first (fewest slots), then the nicest way round.
    for slots in slot_choices:
        for mirror, reverse in options:
            mission, ok = _fits(drone_count, slots, mirror, reverse)
            if ok:
                return mission

    # Doesn't fit any way: find the most drones that would, for the message.
    best = 0
    for n in range(drone_count - 1, MIN_FIGURE8_DRONES - 1, -1):
        if any(
            _fits(n, slots, mirror, reverse)[1]
            for slots in _slot_counts(geometry.loop_length_m, geometry.right_lobe_m, n)
            for mirror, reverse in options
        ):
            best = n
            break
    fit_text = (
        f"This layout fits at most {best} drone{'s' if best != 1 else ''}."
        if best >= MIN_FIGURE8_DRONES
        else "This layout can't fit even two drones."
    )
    raise ValueError(
        f"This operation can't be done with {drone_count} drones on this layout.\n\n"
        f"Spread evenly round the 8 the drones are about "
        f"{geometry.loop_length_m / slot_choices[0]:.0f} m apart, and somewhere on the "
        f"way from the base to the 8 and back the path passes close enough to a part "
        f"of the 8 other drones are still flying that two of them would come closer "
        f"than {FIGURE8_SPACING_M:.0f} m.\n\n"
        f"{fit_text} Uncheck drones, move the base further away (straight above "
        f"or below the middle of the 8 works best - the landing pads behind the "
        f"base need room too), or make the 8 bigger."
    )


def figure8_drone_names(drone_count: int) -> list[str]:
    """`["drone-lead", "drone-f1", "drone-f2", ...]` - front of the train to
    the back, the same naming `expand_figure8_drones` writes into the
    problem file."""
    return [_LEADER] + [f"drone-f{k}" for k in range(1, drone_count)]


def figure8_loop_alt_offsets(geometry: Figure8Geometry) -> tuple[float, ...]:
    """Height offset of every point of one lap (`ring[1:]` then `ring[0]`,
    as flown): 0 everywhere except around the second pass through the
    crossing, which dips `FIGURE8_CROSSING_DIP_M` - see that constant. The
    dip is kept inside its own lobes' first and last quarters, so a small 8
    gets a shorter ramp rather than a dip that reaches the other pass."""
    mid_arc = next(arc for name, _p, arc in geometry.key_points if name == "crossing-mid")
    reach = min(
        FIGURE8_DIP_FLAT_M + FIGURE8_DIP_RAMP_M,
        0.25 * min(geometry.right_lobe_m, geometry.left_lobe_m),
    )
    flat = reach * FIGURE8_DIP_FLAT_M / (FIGURE8_DIP_FLAT_M + FIGURE8_DIP_RAMP_M)
    n = len(geometry.ring)
    offsets = []
    for k in range(1, n + 1):
        off = abs(k * geometry.step_m - mid_arc)
        if off <= flat:
            weight = 1.0
        elif off < reach:
            weight = 0.5 * (1.0 + math.cos(math.pi * (off - flat) / (reach - flat)))
        else:
            weight = 0.0
        offsets.append(-FIGURE8_CROSSING_DIP_M * weight)
    return tuple(offsets)


def figure8_train_routes(mission: Figure8Mission) -> dict[str, TrainRoute]:
    """Each drone's route, keyed by `figure8_drone_names`.

    Drone k (0 = leader) takes off from `takeoff_pads[k]`, slides across to
    the pads' centre line and along it to the base, flies the shared path -
    out to the 8, the laps, back to the base - then along the centre line
    and across to land on `landing_pads[k]`. See `_pad_offsets` for why
    those orders keep everyone clear of occupied pads."""
    n = mission.drone_count
    g = mission.geometry
    crossing = g.crossing
    step = FIGURE8_SAMPLE_M
    b = _to_local(mission.base, crossing)
    loop = [*g.ring[1:], g.ring[0]]          # one lap, middle to middle
    dips = figure8_loop_alt_offsets(g)
    away = None
    if mission.outbound:
        dist = math.hypot(*b)
        away = (b[0] / dist, b[1] / dist)

    def centre_point(pad_xy):
        """Where a pad's row meets the centre line."""
        along = (pad_xy[0] - b[0]) * away[0] + (pad_xy[1] - b[1]) * away[1]
        return (b[0] + away[0] * along, b[1] + away[1] * along)

    routes: dict[str, TrainRoute] = {}
    for k, name in enumerate(figure8_drone_names(n)):
        up = _to_local(mission.takeoff_pads[k], crossing)
        corner = centre_point(up)
        prefix = _line(up, corner, step) + _line(corner, b, step)
        down = _to_local(mission.landing_pads[k], crossing)
        corner = centre_point(down)
        suffix = _line(b, corner, step) + _line(corner, down, step)
        prefix_m = sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip([up, *prefix], prefix))
        approach = [_from_local(p, crossing) for p in prefix] + mission.outbound
        depart = mission.inbound + [_from_local(p, crossing) for p in suffix]
        routes[name] = TrainRoute(
            start=mission.takeoff_pads[k],
            approach=approach,
            loop=loop,
            laps=mission.flown_laps,
            depart=depart,
            shared_from_m=prefix_m,
            slot_m=k * mission.slot_spacing_m,
            cruise_mps=FIGURE8_CRUISE_MPS,
            loop_alt_offsets=dips,
        )
    return routes


# ---- PDDL problem editing ---------------------------------------------------


def _goal_and_span(text: str) -> tuple[int, int]:
    goal_open, goal_close = _section_span(text, "goal")
    and_match = re.search(r"\(and\b", text[goal_open:goal_close])
    if not and_match:
        return goal_open, goal_close
    and_open = goal_open + and_match.start()
    return and_open, _find_matching_paren(text, and_open)


def expand_figure8_drones(problem_text: str, drone_count: int) -> str:
    """Grow the template's leader + one follower (`drone-f1`) to a train of
    `drone_count`: every `:init` fact and goal conjunct mentioning
    `drone-f1` is cloned for `drone-f2` .. `drone-f{drone_count - 1}`, each
    follower's `slot-arc-offset` set to its own place in the train, and the
    `train-neighbor` chain (front <-> back, one symmetric pair per link)
    rebuilt for the real length - the cloned template pair would otherwise
    wire every follower straight to the leader, the same trap
    `expand_formation_drones` avoids for a V's wings.

    `drone_count` must be >= `MIN_FIGURE8_DRONES` (`ValueError` otherwise).
    """
    if drone_count < MIN_FIGURE8_DRONES:
        raise ValueError(
            f"figure-8 formation drone_count must be >= {MIN_FIGURE8_DRONES}, got {drone_count}"
        )
    names = figure8_drone_names(drone_count)
    extra = names[2:]  # beyond the template's own leader + drone-f1

    result = problem_text
    if extra:
        _, obj_close = _section_span(result, "objects")
        result = (
            result[:obj_close]
            + "".join(f"\n        {name} - drone" for name in extra)
            + "\n    "
            + result[obj_close:]
        )
        result = _clone_named_lines(result, _section_span(result, "init"), _FOLLOWER_TOKEN, extra)
        result = _clone_named_lines(result, _goal_and_span(result), _FOLLOWER_TOKEN, extra)

    for k, name in enumerate(names):
        # Nominal spacing here; retarget_figure8_problem rewrites it with
        # the real ring step once the 8's size is known.
        result = _set_unary_fact_value(result, "slot-arc-offset", name, -k * FIGURE8_SPACING_M)

    result = _strip_pair_facts(result, "train-neighbor")
    chain: list[str] = []
    for front, back in zip(names, names[1:]):
        chain.append(f"\n        (train-neighbor {front} {back})")
        chain.append(f"\n        (train-neighbor {back} {front})")
    return _add_init_facts(result, "".join(chain))


def _set_scalar_fluent(problem_text: str, fluent: str, value: float) -> str:
    """`(= (fluent) v)` -> `(= (fluent) value)` for a zero-argument fluent."""
    pattern = re.compile(rf"(\(=\s*\({re.escape(fluent)}\)\s*)[-\d.eE]+(\s*\))")
    result, count = pattern.subn(lambda m: f"{m.group(1)}{value:.6f}{m.group(2)}", problem_text, count=1)
    if not count:
        raise ValueError(f"template has no ({fluent}) fact")
    return result


def figure8_corridor(mission: Figure8Mission) -> list[str]:
    """The leader's location sequence the plan must fly (after the base):
    out to the middle, every lap it flies (`flown_laps`) as key waypoints,
    back to the base."""
    lap = [name for name, _point, _arc in mission.geometry.key_points]
    return lap[:1] + lap[1:] * mission.flown_laps + [BASE_LOCATION]


def retarget_figure8_problem(problem_text: str, mission: Figure8Mission) -> str:
    """Move the base and every key waypoint of an (already expanded)
    figure-8 problem onto `mission`, recompute each leg's along-the-path
    `distance` and `energy-required`, and write the spread-geometry fluents
    the capacity precondition in domain.pddl reads (`drone-spacing`,
    `crossing-gap`, `crossing-clearance`) plus `laps-required` (every lap
    the leader flies). Energy is spread so the whole mission costs
    `_PDDL_MISSION_ENERGY` units, see that constant.
    """
    geometry, laps, drone_count = mission.geometry, mission.flown_laps, mission.drone_count
    first, last = geometry.key_points[0][0], geometry.key_points[-1][0]
    total_m = mission.outbound_m + laps * geometry.loop_length_m + mission.inbound_m
    rate = _PDDL_MISSION_ENERGY / total_m

    result = _set_latlon(problem_text, BASE_LOCATION, mission.base)
    for name, point, _arc in geometry.key_points:
        result = _set_latlon(result, name, point)

    legs = [(a, b, arc_b - arc_a) for (a, _pa, arc_a), (b, _pb, arc_b)
            in zip(geometry.key_points, geometry.key_points[1:])]
    legs += [(BASE_LOCATION, first, mission.outbound_m), (last, BASE_LOCATION, mission.inbound_m)]
    for a, b, leg_m in legs:
        result = _set_fact_value(result, "distance", a, b, leg_m)
        result = _set_fact_value(result, "energy-required", a, b, leg_m * rate)

    result = _set_scalar_fluent(result, "drone-spacing", mission.slot_spacing_m)
    result = _set_scalar_fluent(
        result, "crossing-gap",
        figure8_crossing_gap(geometry.loop_length_m, geometry.right_lobe_m, mission.slot_count),
    )
    result = _set_scalar_fluent(result, "crossing-clearance", FIGURE8_CROSSING_CLEARANCE_M)
    result = _set_scalar_fluent(result, "laps-required", laps)
    leg_count = laps * (len(geometry.key_points) - 1) + 2
    for k, name in enumerate(figure8_drone_names(drone_count)):
        result = _set_unary_fact_value(result, "slot-arc-offset", name, -k * mission.slot_spacing_m)
        result = _set_unary_fact_value(
            result, "cruise-leg-energy", name, _PDDL_MISSION_ENERGY / leg_count
        )
    return result
