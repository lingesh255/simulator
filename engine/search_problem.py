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
    _find_matching_paren,
    _from_local,
    _section_span,
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


MIN_LANE_WIDTH_M = 20.0  # floor on each lane's width: even a very narrow marked area is
                         # padded so its combined width is >= MIN_LANE_WIDTH_M * (lane count),
                         # keeping adjacent lanes' sweeps comfortably past the domain's 10 m
                         # minimum drone separation

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

# How many sweep turn-points per lane are handed to ENHSP as PDDL waypoint
# objects. The drones still FLY the full lawnmower `plan_search_lanes`
# produces - this only coarsens what the planner reasons over, so a
# multi-drone search (each extra drone adds a separation-coupled lane) still
# solves in seconds instead of blowing past any sane planning budget. The
# generated plan keeps its shape - go to the lane, search it, return - just
# with fewer intermediate `move-search` legs. `<= 0` disables coarsening.
PDDL_WAYPOINTS_PER_LANE = 5


def _coarsen_route(points: list[LatLon], max_interior: int) -> list[LatLon]:
    """`points` is `[base, wp, wp, ..., base]`; keep both `base` endpoints
    and at most `max_interior` of the interior turn-points, evenly spaced
    (always including the first and last interior point). `max_interior <= 0`
    or a route already short enough is returned unchanged."""
    interior = points[1:-1]
    if max_interior <= 0 or len(interior) <= max_interior:
        return list(points)
    if max_interior == 1:
        picks = [interior[0]]
    else:
        step = (len(interior) - 1) / (max_interior - 1)
        picks = [interior[round(i * step)] for i in range(max_interior)]
    return [points[0], *picks, points[-1]]


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


def _equal_area_splits(
    hull_uv: list[tuple[float, float]], v_min: float, v_max: float, n_lanes: int
) -> list[float]:
    """The `n_lanes + 1` lane boundaries `[v_min, s1, ..., s_{n-1}, v_max]`
    that carve `hull_uv` into `n_lanes` equal-*area* strips along the v axis.

    Each interior boundary `s_i` is the v-threshold below which `hull_uv`
    holds `i / n_lanes` of its total area, found by binary search over
    `_clip_below` (whose clipped area is monotonically non-decreasing in the
    threshold for a convex polygon, so each search converges). Equal area,
    not equal width, so a lopsided shape still gives every drone a fair share
    of the actual ground rather than an equal-width band that could cover
    very different amounts of it. The boundaries are nudged apart if a
    degenerate shape collapses two of them onto each other, so every lane
    keeps at least a sliver of span."""
    total = _polygon_area(hull_uv)
    interior: list[float] = []
    for i in range(1, n_lanes):
        target = total * i / n_lanes
        lo, hi = v_min, v_max
        for _ in range(40):  # far more than enough precision at metre scale
            mid = (lo + hi) / 2.0
            if _polygon_area(_clip_below(hull_uv, mid)) < target:
                lo = mid
            else:
                hi = mid
        interior.append((lo + hi) / 2.0)

    eps = min(1.0, (v_max - v_min) / (4.0 * n_lanes))
    lower = v_min + eps
    for i, s in enumerate(interior):
        s = max(s, lower)
        s = min(s, v_max - eps * (n_lanes - 1 - i))
        interior[i] = s
        lower = s + eps
    return [v_min, *interior, v_max]


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
    base: LatLon,
    corners: list[LatLon],
    coverage_width_m: float = 50.0,
    n_lanes: int = 2,
) -> dict[str, list[LatLon]]:
    """Work out each drone's full-coverage sweep route for a search area
    marked by `corners` (>= 3 points, click order), launched from `base`,
    split into `n_lanes` lanes - one per drone.

    Returns `{"drone1": [base, wp, wp, ..., base], "drone2": [...], ...}`
    with one entry per lane. The lanes are geometrically disjoint (an
    equal-*area* split of the marked area, not just an equal-width one - see
    the module docstring), so no two drones are ever credited with covering,
    or fly through, the same ground; each route is individually a real
    lawnmower sweep of its own lane, clipped to the marked area's actual
    shape at every pass rather than padded out over the bounding rectangle
    around it.
    """
    if len(corners) < 3:
        raise ValueError("a forest search area needs at least 3 corners")
    n_lanes = max(1, n_lanes)

    hull = _convex_hull([_to_local(p, base) for p in corners])
    if len(hull) < 3:
        raise ValueError("the marked forest area is degenerate (all corners collinear)")

    length_dir, width_dir, u_min, u_max, v_min, v_max = _min_area_rect(hull)
    hull_uv = [(p[0] * length_dir[0] + p[1] * length_dir[1], p[0] * width_dir[0] + p[1] * width_dir[1]) for p in hull]

    base_u = 0.0  # base is the local origin, (0, 0) - its projection onto any axis through the origin is 0
    near_is_min = abs(base_u - u_min) <= abs(base_u - u_max)

    # A thin marked area is padded to a minimum combined width - MIN_LANE_WIDTH_M
    # per lane - so adjacent lanes' sweeps stay comfortably past the domain's
    # minimum drone separation. The polygon itself is unchanged, so per-pass
    # clipping (`_u_range_at_v`) simply comes back empty for any padded sliver
    # beyond the real shape, and that pass is skipped (below).
    min_total_width = MIN_LANE_WIDTH_M * n_lanes
    if v_max - v_min < min_total_width:
        pad_mid = (v_min + v_max) / 2.0
        v_min, v_max = pad_mid - min_total_width / 2.0, pad_mid + min_total_width / 2.0

    bounds = _equal_area_splits(hull_uv, v_min, v_max, n_lanes)

    def to_world(u: float, v: float) -> LatLon:
        x = u * length_dir[0] + v * width_dir[0]
        y = u * length_dir[1] + v * width_dir[1]
        return _from_local((x, y), base)

    routes: dict[str, list[LatLon]] = {}
    for lane in range(n_lanes):
        v_lo, v_hi = bounds[lane], bounds[lane + 1]
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
        routes[f"drone{lane + 1}"] = [base] + [to_world(u, v) for u, v in turns] + [base]

    return routes


def retarget_search_problem(
    problem_text: str,
    base: LatLon,
    corners: list[LatLon],
    coverage_width_m: Optional[float] = None,
    n_lanes: Optional[int] = None,
    pddl_waypoints_per_lane: int = PDDL_WAYPOINTS_PER_LANE,
) -> tuple[str, dict[str, list[LatLon]]]:
    """Build a concrete `SearchProblem.pddl` for a forest area marked by
    `corners` and launched from `base`, and return it together with the full
    lawnmower route each drone should actually fly.

    Returns `(concrete_text, fine_routes)` where `fine_routes` maps each
    drone object ("drone1".."droneN") to its complete
    `[base, wp, wp, ..., base]` sweep. The PDDL waypoint objects spliced into
    `concrete_text` are a *coarsened* skeleton of those routes (at most
    `pddl_waypoints_per_lane` turn-points per lane - see
    `PDDL_WAYPOINTS_PER_LANE`), so ENHSP has a short, tractable plan to find
    even with several separation-coupled lanes, while execution still flies
    the fine routes. `pddl_waypoints_per_lane <= 0` keeps the full sweep in
    the PDDL. The abstract battery pool is scaled from the *fine* route
    length regardless, so feasibility stays honest.

    `n_lanes` is how many drones (== lanes) the search is split across; it
    defaults to however many `(assigned droneN ...)` facts `problem_text`
    already carries (2 for the bare template, more once
    `expand_search_drones` has widened it), so a caller that has already
    expanded the template does not have to say the number twice.

    `coverage_width_m` defaults to whatever `problem_text` itself declares
    via `(= (coverage-width drone1) ...)`, so the generated sweep spacing
    always matches the PDDL's own claim about the drones' coverage swath.
    """
    if coverage_width_m is None:
        coverage_width_m = _read_coverage_width_m(problem_text)
    if n_lanes is None:
        n_lanes = max(2, len(re.findall(r"\(assigned\s+drone\d+\s", problem_text)))
    fine_routes = plan_search_lanes(base, corners, coverage_width_m, n_lanes)
    routes = {
        drone: _coarsen_route(points, pddl_waypoints_per_lane)
        for drone, points in fine_routes.items()
    }
    drone_ids = list(routes)  # ["drone1", "drone2", ...] in lane order

    names: dict[str, list[str]] = {}
    for drone, points in routes.items():
        interior = points[1:-1]  # drop the leading/trailing `base`
        names[drone] = [f"{drone}-wp{i + 1}" for i in range(len(interior))]

    result = _add_locations(problem_text, [n for drone_names in names.values() for n in drone_names])

    facts: list[str] = []
    for lane_index, (drone, points) in enumerate(routes.items(), start=1):
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

        if drone_names:
            facts.append(f"\n        (area-location forest-lane{lane_index} {drone_names[0]})")

    # Cross-drone corresponding-index distances, both directions, for
    # check-drone-separation (see domain.pddl). Adjacent lanes are enough:
    # each drone shares a check with its neighbour, so a chain of
    # neighbour-checks makes every drone `separation-ok` before each hop. The
    # equal-*area* split (see plan_search_lanes) means neighbouring lanes'
    # pass counts can differ for a lopsided shape, so index i only lines up
    # up to whichever of the pair is shorter; a longer lane's trailing
    # waypoints are paired against `base` instead (by then the shorter lane's
    # drone has necessarily reached its last waypoint and is free to have
    # returned to base - nothing ties one drone's progress to another's - and
    # base, being outside the marked area, is always a safe distance away).
    for a, b in zip(drone_ids, drone_ids[1:]):
        na, nb = len(names[a]), len(names[b])
        common = min(na, nb)
        for i in range(common):
            a_name, b_name = names[a][i], names[b][i]
            distance_m = haversine_m(routes[a][i + 1], routes[b][i + 1])
            facts.append(f"\n        (= (distance {a_name} {b_name}) {distance_m:.2f})")
            facts.append(f"\n        (= (distance {b_name} {a_name}) {distance_m:.2f})")
        if na != nb:
            longer = a if na > nb else b
            for i in range(common, len(names[longer])):
                name = names[longer][i]
                distance_m = haversine_m(routes[longer][i + 1], base)
                facts.append(f"\n        (= (distance {name} base) {distance_m:.2f})")
                facts.append(f"\n        (= (distance base {name}) {distance_m:.2f})")

    # forest-width/length (informational only - no action reads them, see
    # domain.pddl) from the same bounding-rectangle geometry.
    hull = _convex_hull([_to_local(p, base) for p in corners])
    _, _, u_min, u_max, v_min, v_max = _min_area_rect(hull)
    lane_width_m = max(v_max - v_min, MIN_LANE_WIDTH_M * n_lanes) / n_lanes
    lane_length_m = abs(u_max - u_min)
    for lane_index in range(1, n_lanes + 1):
        facts.append(f"\n        (= (forest-width forest-lane{lane_index}) {lane_width_m:.2f})")
        facts.append(f"\n        (= (forest-length forest-lane{lane_index}) {lane_length_m:.2f})")

    result = _add_init_facts(result, "".join(facts))
    result = _set_latlon(result, "base", base)

    # Scale the abstract battery pool to whatever the *fine* route actually
    # costs (see BATTERY_SAFETY_MARGIN above) - a fixed 100 was fine for the
    # old fixed 3-waypoint-per-drone template, but a real lawnmower sweep of
    # a large area can easily need several times that much energy well before
    # any real aircraft's battery would be a concern, and the template's
    # default battery/max-battery would make ENHSP correctly, but
    # misleadingly, report the mission unsolvable. Uses `fine_routes`, not
    # the coarsened PDDL skeleton, so the check reflects the whole sweep.
    route_energy = {
        drone: sum(haversine_m(a, b) for a, b in zip(points, points[1:])) * DEFAULT_ENERGY_RATE
        for drone, points in fine_routes.items()
    }
    max_battery = max(DEFAULT_MAX_BATTERY, max(route_energy.values()) * BATTERY_SAFETY_MARGIN)
    return_reserve = max_battery * RETURN_RESERVE_FRACTION
    for drone in fine_routes:
        result = _set_unary_fact(result, "battery", drone, max_battery)
        result = _set_unary_fact(result, "max-battery", drone, max_battery)
        result = _set_unary_fact(result, "minimum-return-battery", drone, return_reserve)

    return result, fine_routes


# ---- Swarm expansion: the 2-drone template -> N drones / N lanes ----------

_SEPARATION_LINE = re.compile(r"minimum-drone-separation")


def _clone_search_lines(text: str, span: tuple[int, int], extra: range, *, skip=None) -> str:
    """Duplicate every line inside ``text[span[0]:span[1]]`` that mentions
    ``drone2`` or ``forest-lane2`` - once per index in ``extra``, swapping
    ``drone2``/``forest-lane2`` for ``droneN``/``forest-laneN`` - inserted
    just before the span's end. Lines matching ``skip`` are left alone."""
    _open, close_pos = span
    clones: list[str] = []
    for line in text[_open:close_pos].splitlines():
        if skip is not None and skip.search(line):
            continue
        if re.search(r"\bdrone2\b", line) or re.search(r"\bforest-lane2\b", line):
            for k in extra:
                new_line = re.sub(r"\bdrone2\b", f"drone{k}", line)
                new_line = re.sub(r"\bforest-lane2\b", f"forest-lane{k}", new_line)
                clones.append(new_line)
    if not clones:
        return text
    return f"{text[:close_pos].rstrip()}\n" + "\n".join(clones) + f"\n    {text[close_pos:]}"


def expand_search_drones(problem_text: str, count: int) -> str:
    """Grow the 2-drone ``forest-drone-search`` template to ``count`` drones
    and ``count`` lanes (``drone1``..``drone{count}`` /
    ``forest-lane1``..``forest-lane{count}``), so a search can run with any
    number of checked drones >= 2, each sweeping its own equal-area lane.

    ``count <= 2`` returns the text unchanged. For a larger swarm every
    ``drone2`` / ``forest-lane2`` line in ``(:init ...)`` and the goal is
    cloned for each extra drone, the ``(:objects ...)`` declarations are
    widened, the ``minimum-drone-separation`` facts are regenerated for each
    adjacent lane pair (both directions), and the goal gains an
    ``(area-covered forest-laneN)`` conjunct per lane - the latter because
    ``complete-forest-search`` only needs *two* covered areas, so with three
    or more lanes the goal itself has to demand every lane be swept.
    """
    if count <= 2:
        return problem_text

    drones = [f"drone{i}" for i in range(1, count + 1)]
    lanes = [f"forest-lane{i}" for i in range(1, count + 1)]
    extra = range(3, count + 1)

    sep_match = re.search(
        r"\(=\s*\(minimum-drone-separation\s+\S+\s+\S+\)\s*([-\d.eE]+)\s*\)", problem_text
    )
    sep_value = sep_match.group(1) if sep_match else "10"

    # 1. widen the (:objects ...) declarations
    obj_open, obj_close = _section_span(problem_text, "objects")
    block = problem_text[obj_open:obj_close]
    block = re.sub(
        r"\bdrone2\b(\s*-\s*drone\b)",
        lambda m: " ".join(["drone2", *(f"drone{i}" for i in extra)]) + m.group(1),
        block,
        count=1,
    )
    block = re.sub(
        r"\bforest-lane2\b(\s*-\s*forest-area\b)",
        lambda m: " ".join(["forest-lane2", *(f"forest-lane{i}" for i in extra)]) + m.group(1),
        block,
        count=1,
    )
    problem_text = problem_text[:obj_open] + block + problem_text[obj_close:]

    # 2. clone every drone2 / forest-lane2 (:init ...) line for the extra
    #    drones (all but the pairwise separation facts, rebuilt next)
    problem_text = _clone_search_lines(
        problem_text, _section_span(problem_text, "init"), extra, skip=_SEPARATION_LINE
    )

    # 3. rebuild minimum-drone-separation for adjacent lane pairs only.
    #    Each drone only ever needs a separation check against its lane
    #    neighbour (a chain of neighbour-checks makes every drone
    #    `separation-ok`), and restricting the fact to those pairs keeps
    #    ENHSP from grounding a `check-drone-separation` for every unrelated
    #    pair - which, for 4+ drones, bloats the search badly.
    problem_text = re.sub(
        r"[ \t]*\(=\s*\(minimum-drone-separation\b[^\n]*\)[ \t]*\n?", "", problem_text
    )
    _init_open, init_close = _section_span(problem_text, "init")
    sep_facts = "".join(
        f"\n        (= (minimum-drone-separation {a} {b}) {sep_value})"
        f"\n        (= (minimum-drone-separation {b} {a}) {sep_value})"
        for a, b in zip(drones, drones[1:])
    )
    problem_text = (
        f"{problem_text[:init_close].rstrip()}{sep_facts}\n    {problem_text[init_close:]}"
    )

    # 4. goal: clone the drone2 conjuncts, then force every lane covered
    goal_open, goal_close = _section_span(problem_text, "goal")
    and_match = re.search(r"\(and\b", problem_text[goal_open:goal_close])
    if and_match:
        and_open = goal_open + and_match.start()
        span = (and_open, _find_matching_paren(problem_text, and_open))
    else:
        span = (goal_open, goal_close)
    problem_text = _clone_search_lines(problem_text, span, extra)

    goal_open, goal_close = _section_span(problem_text, "goal")
    and_match = re.search(r"\(and\b", problem_text[goal_open:goal_close])
    close_at = (
        _find_matching_paren(problem_text, goal_open + and_match.start())
        if and_match
        else goal_close
    )
    covered = "".join(f"\n            (area-covered {lane})" for lane in lanes)
    problem_text = (
        f"{problem_text[:close_at].rstrip()}{covered}\n    {problem_text[close_at:]}"
    )

    return problem_text
