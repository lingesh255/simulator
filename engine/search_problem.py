"""PDDL problem-file editing for the forest-search mission.

    plans/Search/SearchProblem.pddl (template)
        -> substitute the picked base point + forest polygon
        -> ENHSP (engine.pddl_planner)
        -> one full-coverage lawnmower route per drone

Unlike `engine.pddl_problem`'s point-to-point retargeting (which relocates an
existing waypoint graph by rotating/scaling/translating it onto a new
source/destination), this *generates* the waypoint graph from scratch every
time: how many waypoints a mission needs depends on the marked area's actual
size relative to each drone's coverage width, so the template itself carries
no waypoints at all (see the template's own docstring-comment) - just
`base` and the two `forest-area` objects each drone is assigned to. Every
waypoint object, and the `connected`/`safe-route`/`distance`/
`energy-required` facts wiring them up, is injected fresh via
`engine.pddl_problem`'s text-injection helpers (the same ones
`retarget_problem` uses to splice a restricted-area detour into the travell
plan).

Coverage geometry - real full-coverage lawnmower sweeps of the actual
marked shape, not a rectangle around it:

1. The marked polygon (in click order - "dynamic dimensions", not
   necessarily a rectangle) is reduced to its convex hull, then fitted with
   its true minimum-area bounding rectangle (rotating calipers - trying
   every hull edge as a candidate orientation and keeping the smallest-area
   fit), not just guessed from its longest edge. This only sets the sweep
   axis/orientation - a skewed or trapezoidal area still gets a sensible
   length/width split this way, instead of the two lanes crossing each
   other.
2. The polygon is split into two lanes by *area*, not just width: the
   split line (perpendicular to the width axis) is placed wherever bisects
   the polygon's actual area in half, found by binary search over
   Sutherland-Hodgman half-plane clipping (so a lopsided shape - a triangle
   or trapezoid - still gives each drone a fair, equal-area share, not just
   an equal-width band that happens to cover very different amounts of
   ground). The two lanes are still geometrically disjoint by construction,
   so drone1 and drone2 can never be credited with covering, or fly
   through, the same ground.
3. Each lane is swept back and forth (a boustrophedon) with passes spaced
   `coverage_width_m` apart (that drone's real sensor/coverage swath, from
   the domain's `coverage-width` fact). Critically, every individual pass
   is clipped to the polygon's *actual* boundary at that pass's position
   (a convex-polygon/horizontal-line intersection), not stretched across
   the full bounding rectangle - a pass near a tapering corner is exactly as
   long as the real ground there, not padded out over empty space outside
   the marked area.
"""
from __future__ import annotations

import math
import re
from typing import Optional

from engine.pddl_problem import (
    LatLon,
    _add_init_facts,
    _add_locations,
    _convex_hull,
    _from_local,
    _set_latlon,
    _to_local,
    haversine_m,
)

DEFAULT_ENERGY_RATE = 0.0125  # energy units per metre - matches engine.pddl_problem's default
_COVERAGE_WIDTH_FACT = re.compile(r"\(=\s*\(coverage-width\s+\S+\)\s*([-\d.eE]+)\s*\)")


def _read_coverage_width_m(problem_text: str, default: float = 50.0) -> float:
    """The `coverage-width` a drone's own problem-file fact declares, so the
    sweep spacing below always matches whatever the PDDL claims rather than
    needing to be kept in sync with it by hand."""
    match = _COVERAGE_WIDTH_FACT.search(problem_text)
    return float(match.group(1)) if match else default


MIN_TOTAL_WIDTH_M = 40.0  # floor on the *combined* two-lane width, so even a very narrow marked
                          # area keeps both lanes comfortably past the domain's 10 m minimum
                          # separation (each lane ends up >= 20 m from the shared centreline)

# Headroom the abstract PDDL battery pool is given over the raw energy a
# generated route actually costs (see retarget_search_problem below). This
# is *not* a real battery model - the mission's real physical feasibility is
# already screened in mAh terms by services.local_flight.
# plan_route_flights_by_drone - it exists only so a large-but-genuinely-
# flyable search area doesn't come back "unsolvable" purely because the
# fixed default battery=100 the template used to hardcode ran out partway
# through a long enough sweep, well before any real aircraft's battery
# would.
BATTERY_SAFETY_MARGIN = 1.6
DEFAULT_MAX_BATTERY = 100.0
RETURN_RESERVE_FRACTION = 0.30  # kept the same proportion as the original template's 30/100


def _min_area_rect(
    hull: list[tuple[float, float]]
) -> tuple[tuple[float, float], tuple[float, float], float, float, float, float]:
    """The hull's true minimum-area bounding rectangle: `(length_dir,
    width_dir, u_min, u_max, v_min, v_max)`, with `length_dir` always the
    longer of the two sides (the sweep axis) and `width_dir` the shorter
    (the axis the two drones' lanes are split along).

    Rotating calipers: the minimum-area rectangle bounding a convex polygon
    always has one side flush with one of its edges, so trying each edge's
    direction as a candidate orientation and keeping the smallest-area
    result is exact - unlike guessing the orientation from the single
    longest edge, which a trapezoid or otherwise skewed quadrilateral can
    easily fool into a bad (crossing-lanes) split.
    """
    n = len(hull)
    best: tuple[float, tuple[float, float], tuple[float, float], float, float, float, float] | None = None
    for i in range(n):
        ax, ay = hull[i]
        bx, by = hull[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        if length < 1e-9:
            continue
        u_dir = (dx / length, dy / length)
        v_dir = (-u_dir[1], u_dir[0])
        us = [p[0] * u_dir[0] + p[1] * u_dir[1] for p in hull]
        vs = [p[0] * v_dir[0] + p[1] * v_dir[1] for p in hull]
        u_min, u_max, v_min, v_max = min(us), max(us), min(vs), max(vs)
        area = (u_max - u_min) * (v_max - v_min)
        if best is None or area < best[0]:
            best = (area, u_dir, v_dir, u_min, u_max, v_min, v_max)

    assert best is not None
    _, length_dir, width_dir, u_min, u_max, v_min, v_max = best
    if (u_max - u_min) < (v_max - v_min):
        length_dir, width_dir = width_dir, length_dir
        u_min, u_max, v_min, v_max = v_min, v_max, u_min, u_max
    return length_dir, width_dir, u_min, u_max, v_min, v_max


def _sweep_passes(v_lo: float, v_hi: float, coverage_width_m: float) -> list[float]:
    """v-coordinates of each sweep pass covering `[v_lo, v_hi]`: enough
    passes, evenly spaced, that each one's strip is no wider than
    `coverage_width_m` - tiling the span exactly, with neither a gap
    between passes nor a wider-than-real-coverage strip."""
    span = max(v_hi - v_lo, 1e-6)
    count = max(1, math.ceil(span / max(coverage_width_m, 1e-6)))
    step = span / count
    return [v_lo + step * (i + 0.5) for i in range(count)]


def _set_unary_fact(problem_text: str, predicate: str, obj: str, value: float) -> str:
    """Like `engine.pddl_problem._set_fact_value`, but for a one-argument
    fact such as `(= (battery drone1) ...)` rather than a `(from, to)` one."""
    pattern = re.compile(
        rf"(\(=\s*\({re.escape(predicate)}\s+{re.escape(obj)}\)\s*)[-\d.eE]+(\s*\))"
    )
    return pattern.sub(lambda m: f"{m.group(1)}{value:.2f}{m.group(2)}", problem_text, count=1)


def _polygon_area(polygon: list[tuple[float, float]]) -> float:
    """Shoelace formula. 0 for a degenerate (< 3 point) polygon."""
    n = len(polygon)
    if n < 3:
        return 0.0
    total = 0.0
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        total += x1 * y2 - x2 * y1
    return abs(total) / 2.0


def _clip_below(polygon: list[tuple[float, float]], v_threshold: float) -> list[tuple[float, float]]:
    """Sutherland-Hodgman: `polygon` intersected with the half-plane
    `v <= v_threshold` (points are `(u, v)`)."""
    n = len(polygon)
    if n < 3:
        return []
    result: list[tuple[float, float]] = []
    for i in range(n):
        cur = polygon[i]
        nxt = polygon[(i + 1) % n]
        cur_in = cur[1] <= v_threshold
        nxt_in = nxt[1] <= v_threshold
        if cur_in:
            result.append(cur)
        if cur_in != nxt_in:
            dv = nxt[1] - cur[1]
            t = (v_threshold - cur[1]) / dv if abs(dv) > 1e-12 else 0.0
            result.append((cur[0] + t * (nxt[0] - cur[0]), v_threshold))
    return result


def _equal_area_split(hull_uv: list[tuple[float, float]], v_min: float, v_max: float) -> float:
    """The v-coordinate that bisects `hull_uv`'s area in half - binary
    search over `_clip_below`, whose area is monotonically non-decreasing in
    `v_threshold` for a convex polygon, so this always converges. Gives each
    drone an equal share of the marked area's actual ground, not just an
    equal-width band that a lopsided shape could split very unevenly."""
    target = _polygon_area(hull_uv) / 2.0
    lo, hi = v_min, v_max
    for _ in range(40):  # far more than enough precision at metre scale
        mid = (lo + hi) / 2.0
        if _polygon_area(_clip_below(hull_uv, mid)) < target:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def _u_range_at_v(hull_uv: list[tuple[float, float]], v: float) -> Optional[tuple[float, float]]:
    """The polygon's `[u_lo, u_hi]` where the horizontal line `v = const`
    actually crosses it, or `None` if that line misses the polygon
    entirely - what a single sweep pass at this `v` should really span,
    instead of the full bounding rectangle's width."""
    n = len(hull_uv)
    us: list[float] = []
    for i in range(n):
        a, b = hull_uv[i], hull_uv[(i + 1) % n]
        a_v, b_v = a[1], b[1]
        if (a_v - v) * (b_v - v) > 1e-9:
            continue  # both endpoints strictly on the same side - no crossing
        if abs(b_v - a_v) < 1e-9:
            if abs(a_v - v) < 1e-9:
                us.append(a[0])
                us.append(b[0])
            continue
        t = (v - a_v) / (b_v - a_v)
        us.append(a[0] + t * (b[0] - a[0]))
    if not us:
        return None
    return min(us), max(us)


def plan_search_lanes(
    base: LatLon, corners: list[LatLon], coverage_width_m: float = 50.0
) -> dict[str, list[LatLon]]:
    """Work out each drone's full-coverage sweep route for a search area
    marked by `corners` (>= 3 points, click order), launched from `base`.

    Returns `{"drone1": [base, wp, wp, ..., base], "drone2": [...]}` - the
    two routes are geometrically disjoint (an equal-area split of the
    marked area, not just an equal-width one - see the module docstring)
    and each individually a real lawnmower sweep of its own lane, clipped to
    the marked area's actual shape at every pass rather than padded out
    over the bounding rectangle around it.
    """
    if len(corners) < 3:
        raise ValueError("a forest search area needs at least 3 corners")

    hull = _convex_hull([_to_local(p, base) for p in corners])
    if len(hull) < 3:
        raise ValueError("the marked forest area is degenerate (all corners collinear)")

    length_dir, width_dir, u_min, u_max, v_min, v_max = _min_area_rect(hull)
    hull_uv = [(p[0] * length_dir[0] + p[1] * length_dir[1], p[0] * width_dir[0] + p[1] * width_dir[1]) for p in hull]

    base_u = 0.0  # base is the local origin, (0, 0) - its projection onto any axis through the origin is 0
    near_is_min = abs(base_u - u_min) <= abs(base_u - u_max)

    # A very thin marked area is padded to a minimum combined width so the
    # two lanes stay comfortably past the domain's minimum separation (see
    # MIN_TOTAL_WIDTH_M) - the polygon itself is unchanged, so per-pass
    # clipping (`_u_range_at_v`) will simply come back empty for any padded
    # sliver beyond the real shape, and that pass is skipped (below).
    if v_max - v_min < MIN_TOTAL_WIDTH_M:
        pad_mid = (v_min + v_max) / 2.0
        v_min, v_max = pad_mid - MIN_TOTAL_WIDTH_M / 2.0, pad_mid + MIN_TOTAL_WIDTH_M / 2.0

    v_split = _equal_area_split(hull_uv, v_min, v_max)
    # Keep each lane at least a sliver of the true span, even for a
    # pathological shape (e.g. a triangle whose apex sits right at one
    # edge) where the equal-area point would otherwise land on the
    # boundary itself.
    v_split = min(max(v_split, v_min + 1.0), v_max - 1.0)

    def to_world(u: float, v: float) -> LatLon:
        x = u * length_dir[0] + v * width_dir[0]
        y = u * length_dir[1] + v * width_dir[1]
        return _from_local((x, y), base)

    routes: dict[str, list[LatLon]] = {}
    for drone, (v_lo, v_hi) in (("drone1", (v_min, v_split)), ("drone2", (v_split, v_max))):
        turns: list[tuple[float, float]] = []
        for i, v in enumerate(_sweep_passes(v_lo, v_hi, coverage_width_m)):
            span = _u_range_at_v(hull_uv, v)
            if span is None:
                continue  # this pass's line falls outside the marked area entirely (only
                          # possible right at a padded-for-minimum-width extreme) - nothing
                          # real to sweep here
            lo_u, hi_u = span
            near_u, far_u = (lo_u, hi_u) if near_is_min else (hi_u, lo_u)
            u_a, u_b = (near_u, far_u) if i % 2 == 0 else (far_u, near_u)
            turns.append((u_a, v))
            turns.append((u_b, v))
        routes[drone] = [base] + [to_world(u, v) for u, v in turns] + [base]

    return routes


def retarget_search_problem(
    problem_text: str,
    base: LatLon,
    corners: list[LatLon],
    coverage_width_m: Optional[float] = None,
) -> str:
    """Build a concrete `SearchProblem.pddl` for a forest area marked by
    `corners` and launched from `base`: every waypoint object and the facts
    wiring it up are generated fresh (see the module docstring) and spliced
    into the template's `:objects`/`:init` sections - the template itself
    declares none of this, since how many waypoints exist depends on the
    marked area's size.

    `coverage_width_m` defaults to whatever `problem_text` itself declares
    via `(= (coverage-width drone1) ...)`, so the generated sweep spacing
    always matches the PDDL's own claim about the drones' coverage swath.
    """
    if coverage_width_m is None:
        coverage_width_m = _read_coverage_width_m(problem_text)
    routes = plan_search_lanes(base, corners, coverage_width_m)

    names: dict[str, list[str]] = {}
    for drone, points in routes.items():
        interior = points[1:-1]  # drop the leading/trailing `base`
        names[drone] = [f"{drone}-wp{i + 1}" for i in range(len(interior))]

    result = _add_locations(problem_text, [n for drone_names in names.values() for n in drone_names])

    facts: list[str] = []
    for drone, points in routes.items():
        drone_names = names[drone]
        chain_names = ["base"] + drone_names + ["base"]
        chain_points = points  # already [base, wp..., base]

        for name, point in zip(drone_names, points[1:-1]):
            facts.append(f"\n        (= (latitude {name}) {point.lat:.7f})")
            facts.append(f"\n        (= (longitude {name}) {point.lon:.7f})")

        for a_name, a_pt, b_name, b_pt in zip(
            chain_names, chain_points, chain_names[1:], chain_points[1:]
        ):
            distance_m = haversine_m(a_pt, b_pt)
            energy = distance_m * DEFAULT_ENERGY_RATE
            facts.append(f"\n        (connected {a_name} {b_name})")
            facts.append(f"\n        (safe-route {a_name} {b_name})")
            facts.append(f"\n        (= (distance {a_name} {b_name}) {distance_m:.2f})")
            facts.append(f"\n        (= (energy-required {a_name} {b_name}) {energy:.2f})")

        lane_area = "forest-lane1" if drone == "drone1" else "forest-lane2"
        facts.append(f"\n        (area-location {lane_area} {drone_names[0]})")

    # Cross-drone corresponding-index distances, both directions, for
    # check-drone-separation (see domain.pddl). The equal-*area* split
    # (see plan_search_lanes) means the two lanes' pass counts can differ
    # for a lopsided shape, so index i only lines up between the two lanes
    # up to whichever one is shorter.
    n1, n2 = len(names["drone1"]), len(names["drone2"])
    common = min(n1, n2)
    for i in range(common):
        a_name, b_name = names["drone1"][i], names["drone2"][i]
        a_pt, b_pt = routes["drone1"][i + 1], routes["drone2"][i + 1]
        distance_m = haversine_m(a_pt, b_pt)
        facts.append(f"\n        (= (distance {a_name} {b_name}) {distance_m:.2f})")
        facts.append(f"\n        (= (distance {b_name} {a_name}) {distance_m:.2f})")

    # The longer lane's waypoints beyond `common` have no same-index
    # counterpart on the other (by-then-finished) lane to check separation
    # against - pair them against `base` instead. By the time the longer
    # lane is on these trailing passes, the shorter lane's drone has
    # necessarily already reached its own last waypoint and is free to have
    # returned to base (nothing in domain.pddl ties one drone's progress to
    # the other's), so this is a real, always-available stand-in position
    # rather than a fiction - and being outside the marked area, base is
    # always a safe distance from any interior waypoint.
    if n1 != n2:
        longer, other = ("drone1", "drone2") if n1 > n2 else ("drone2", "drone1")
        for i in range(common, len(names[longer])):
            name = names[longer][i]
            point = routes[longer][i + 1]
            distance_m = haversine_m(point, base)
            facts.append(f"\n        (= (distance {name} base) {distance_m:.2f})")
            facts.append(f"\n        (= (distance base {name}) {distance_m:.2f})")

    # forest-width/length (informational only - no action reads them, see
    # domain.pddl) from the same bounding-rectangle geometry.
    hull = _convex_hull([_to_local(p, base) for p in corners])
    _, _, u_min, u_max, v_min, v_max = _min_area_rect(hull)
    lane_width_m = max(v_max - v_min, MIN_TOTAL_WIDTH_M) / 2.0
    lane_length_m = abs(u_max - u_min)
    facts.append(f"\n        (= (forest-width forest-lane1) {lane_width_m:.2f})")
    facts.append(f"\n        (= (forest-length forest-lane1) {lane_length_m:.2f})")
    facts.append(f"\n        (= (forest-width forest-lane2) {lane_width_m:.2f})")
    facts.append(f"\n        (= (forest-length forest-lane2) {lane_length_m:.2f})")

    result = _add_init_facts(result, "".join(facts))
    result = _set_latlon(result, "base", base)

    # Scale the abstract battery pool to whatever this specific route
    # actually costs (see BATTERY_SAFETY_MARGIN above) - a fixed 100 was
    # fine for the old fixed 3-waypoint-per-drone template, but a real
    # lawnmower sweep of a large area can easily need several times that
    # much energy well before any real aircraft's battery would be a
    # concern, and the template's default battery/max-battery would make
    # ENHSP correctly, but misleadingly, report the mission unsolvable.
    route_energy = {
        drone: sum(haversine_m(a, b) for a, b in zip(points, points[1:])) * DEFAULT_ENERGY_RATE
        for drone, points in routes.items()
    }
    max_battery = max(DEFAULT_MAX_BATTERY, max(route_energy.values()) * BATTERY_SAFETY_MARGIN)
    return_reserve = max_battery * RETURN_RESERVE_FRACTION
    for drone in routes:
        result = _set_unary_fact(result, "battery", drone, max_battery)
        result = _set_unary_fact(result, "max-battery", drone, max_battery)
        result = _set_unary_fact(result, "minimum-return-battery", drone, return_reserve)

    return result
