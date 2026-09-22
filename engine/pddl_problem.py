"""PDDL problem-file editing for mission planning.

A plan folder's `problem.pddl` is a *template*: a runnable problem in its own
right (fixed source/destination), plus a waypoint graph (locations, safe and
restricted routes, no-fly zones) built around that specific source and
destination. Picking a new start/destination on the map cannot just move
those two points and leave the waypoints where they were - a waypoint graph
built for a mission near Chennai is not a sensible detour for a mission near
Warangal, and "shortest path via a waypoint on the other side of the planet"
is correctly unsolvable.

Instead the *whole* graph is relocated: every other location is carried along
by the same rotate+scale+translate that maps the template's original
source->destination segment onto the newly picked one, so the detour's shape
(how far each waypoint sits off the direct line, in what direction) is
preserved wherever the mission is retargeted. lat/lon is treated as locally
flat (an equirectangular projection centred on the source of each frame),
which is accurate at the city/regional scale these detours operate at.

Distance/energy-required is recomputed for every edge from the moved
coordinates, at that edge's own original energy-per-metre rate (a plan author
can and does make some edges pricier than others - e.g. this template's
direct, restricted route costs about twice as much per metre as its safe
detour - so a single global rate would misprice the detour after a move).

This works entirely by text substitution over the flat `(= (attr obj) value)`
/ `(connected A B)` facts already in the file - no PDDL parser, and nothing
here is specific to any one plan's location names, so it works for any future
plan folder that shares the same `latitude`/`longitude`/`distance`/
`energy-required`/`connected` vocabulary (guaranteed by sharing `domain.pddl`).
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass

EARTH_RADIUS_M = 6_371_000.0
METRES_PER_DEG_LAT = 111_320.0
DEFAULT_ENERGY_RATE = 0.0125  # energy units per metre, if an edge has no reference cost
NO_FLY_BUFFER_M = 10.0  # clearance a detour keeps from a restricted area's true boundary

_LATLON_FACT = re.compile(
    r"\(=\s*\((latitude|longitude)\s+(\S+)\)\s*([-\d.eE]+)\s*\)"
)
_CONNECTED_FACT = re.compile(r"\(connected\s+(\S+)\s+(\S+)\)")


@dataclass(frozen=True)
class LatLon:
    lat: float
    lon: float


def haversine_m(a: LatLon, b: LatLon) -> float:
    lat1, lat2 = math.radians(a.lat), math.radians(b.lat)
    dlat = lat2 - lat1
    dlon = math.radians(b.lon - a.lon)
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


def read_locations(problem_text: str) -> dict[str, LatLon]:
    """Every `(= (latitude X) v)` / `(= (longitude X) v)` fact, merged by X."""
    lats: dict[str, float] = {}
    lons: dict[str, float] = {}
    for kind, name, value in _LATLON_FACT.findall(problem_text):
        target = lats if kind == "latitude" else lons
        target[name] = float(value)
    return {
        name: LatLon(lat=lats[name], lon=lons[name])
        for name in lats.keys() & lons.keys()
    }


def _read_edges(problem_text: str) -> list[tuple[str, str]]:
    return _CONNECTED_FACT.findall(problem_text)


def _fact_value(problem_text: str, predicate: str, a: str, b: str) -> float | None:
    pattern = re.compile(
        rf"\(=\s*\({re.escape(predicate)}\s+{re.escape(a)}\s+{re.escape(b)}\)\s*([-\d.eE]+)\s*\)"
    )
    match = pattern.search(problem_text)
    return float(match.group(1)) if match else None


def _set_fact_value(problem_text: str, predicate: str, a: str, b: str, value: float) -> str:
    pattern = re.compile(
        rf"(\(=\s*\({re.escape(predicate)}\s+{re.escape(a)}\s+{re.escape(b)}\)\s*)"
        rf"[-\d.eE]+(\s*\))"
    )

    def _repl(match: re.Match) -> str:
        return f"{match.group(1)}{value:.2f}{match.group(2)}"

    return pattern.sub(_repl, problem_text, count=1)


def _set_latlon(problem_text: str, name: str, point: LatLon) -> str:
    for kind, value in (("latitude", point.lat), ("longitude", point.lon)):
        pattern = re.compile(
            rf"(\(=\s*\({kind}\s+{re.escape(name)}\)\s*)[-\d.eE]+(\s*\))"
        )
        problem_text = pattern.sub(
            lambda m, v=value: f"{m.group(1)}{v:.7f}{m.group(2)}", problem_text, count=1
        )
    return problem_text


# ---- Local-planar projection, for the rotate+scale+translate ----


def _metres_per_deg_lon(lat_deg: float) -> float:
    return METRES_PER_DEG_LAT * math.cos(math.radians(lat_deg))


def _to_local(point: LatLon, origin: LatLon) -> tuple[float, float]:
    """(east_m, north_m) of `point` relative to `origin`."""
    x = (point.lon - origin.lon) * _metres_per_deg_lon(origin.lat)
    y = (point.lat - origin.lat) * METRES_PER_DEG_LAT
    return x, y


def _from_local(offset: tuple[float, float], origin: LatLon) -> LatLon:
    x, y = offset
    return LatLon(
        lat=origin.lat + y / METRES_PER_DEG_LAT,
        lon=origin.lon + x / _metres_per_deg_lon(origin.lat),
    )


def _similarity_transform(old_source: LatLon, old_dest: LatLon, new_source: LatLon, new_dest: LatLon):
    """A function mapping any `LatLon` through the rotate+scale+translate that
    takes `old_source -> new_source` and `old_dest -> new_dest`."""
    old_vec = _to_local(old_dest, old_source)
    old_len = math.hypot(*old_vec)
    if old_len < 1.0:
        # Degenerate template (source == destination) - translate only.
        def translate_only(point: LatLon) -> LatLon:
            return _from_local(_to_local(point, old_source), new_source)

        return translate_only

    new_vec = _to_local(new_dest, new_source)
    scale = math.hypot(*new_vec) / old_len
    angle = math.atan2(new_vec[1], new_vec[0]) - math.atan2(old_vec[1], old_vec[0])
    cos_a, sin_a = math.cos(angle), math.sin(angle)

    def transform(point: LatLon) -> LatLon:
        x, y = _to_local(point, old_source)
        rotated = (x * cos_a - y * sin_a, x * sin_a + y * cos_a)
        scaled = (rotated[0] * scale, rotated[1] * scale)
        return _from_local(scaled, new_source)

    return transform


# ---- Segment/polygon intersection, for the restricted-area check ----

_Point = tuple[float, float]


def _convex_hull(points: list[_Point]) -> list[_Point]:
    """Andrew's monotone chain, counter-clockwise, duplicates removed.

    The 4 corners of a restricted area are clicked in whatever order feels
    natural to the user (e.g. top-left, top-right, bottom-left, bottom-right
    - not a walk around the boundary), which as a raw polygon is
    self-intersecting: a bowtie, not a rectangle. Every geometry check below
    assumes a simple polygon, so the hull - the one unambiguous simple
    polygon those 4 points define - is used instead of the raw click order.
    """
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o: _Point, a: _Point, b: _Point) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[_Point] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: list[_Point] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _offset_polygon(polygon: list[_Point], distance_m: float) -> list[_Point]:
    """Expand a convex `polygon` outward by `distance_m`, so a route planned
    against the result clears the true boundary by that margin rather than
    just grazing it.

    Standard edge-offset-and-reintersect construction: each edge's
    supporting line is pushed outward along its own normal by `distance_m`,
    and the new vertices are where each pair of consecutive offset lines
    meets. Exact for a convex polygon (this module's restricted areas always
    are - see `_convex_hull`), unlike inflating from the centroid, which
    over-expands corners on an elongated shape.
    """
    n = len(polygon)
    if n < 3 or distance_m <= 0:
        return list(polygon)

    centroid = (sum(p[0] for p in polygon) / n, sum(p[1] for p in polygon) / n)

    shifted_edges: list[tuple[_Point, _Point]] = []  # (point on the shifted line, its direction)
    for i in range(n):
        a, b = polygon[i], polygon[(i + 1) % n]
        direction = (b[0] - a[0], b[1] - a[1])
        length = math.hypot(*direction)
        if length < 1e-9:
            continue
        normal = (direction[1] / length, -direction[0] / length)
        mid = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
        to_centroid = (centroid[0] - mid[0], centroid[1] - mid[1])
        if normal[0] * to_centroid[0] + normal[1] * to_centroid[1] > 0:
            normal = (-normal[0], -normal[1])  # was pointing inward - flip it
        shifted_edges.append(((a[0] + normal[0] * distance_m, a[1] + normal[1] * distance_m), direction))

    m = len(shifted_edges)
    if m < 3:
        return list(polygon)

    def _line_intersection(p1: _Point, d1: _Point, p2: _Point, d2: _Point) -> _Point:
        denom = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(denom) < 1e-9:
            return p1  # parallel (degenerate polygon) - shouldn't happen for a simple hull
        t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / denom
        return (p1[0] + d1[0] * t, p1[1] + d1[1] * t)

    return [
        _line_intersection(*shifted_edges[i - 1], *shifted_edges[i])
        for i in range(m)
    ]


def _orientation(a: _Point, b: _Point, c: _Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a: _Point, b: _Point, p: _Point) -> bool:
    return min(a[0], b[0]) - 1e-9 <= p[0] <= max(a[0], b[0]) + 1e-9 and \
        min(a[1], b[1]) - 1e-9 <= p[1] <= max(a[1], b[1]) + 1e-9


def _segments_intersect(p1: _Point, p2: _Point, p3: _Point, p4: _Point) -> bool:
    d1, d2 = _orientation(p3, p4, p1), _orientation(p3, p4, p2)
    d3, d4 = _orientation(p1, p2, p3), _orientation(p1, p2, p4)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and d1 != 0 and d2 != 0 and d3 != 0 and d4 != 0:
        return True
    # Collinear/touching cases: only count them if the point genuinely lies
    # on the other segment's span, not just on its infinite line.
    if d1 == 0 and _on_segment(p3, p4, p1):
        return True
    if d2 == 0 and _on_segment(p3, p4, p2):
        return True
    if d3 == 0 and _on_segment(p1, p2, p3):
        return True
    if d4 == 0 and _on_segment(p1, p2, p4):
        return True
    return False


def _point_in_polygon(point: _Point, polygon: list[_Point]) -> bool:
    """Standard ray-casting point-in-polygon test."""
    x, y = point
    inside = False
    n = len(polygon)
    x2, y2 = polygon[-1]
    for x1, y1 in polygon:
        if (y1 > y) != (y2 > y):
            x_at_y = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < x_at_y:
                inside = not inside
        x2, y2 = x1, y1
    return inside


def _segment_crosses_polygon(seg_a: _Point, seg_b: _Point, polygon: list[_Point]) -> bool:
    """Whether the segment `seg_a`->`seg_b` passes through `polygon` at all -
    crossing an edge, or running entirely inside it."""
    if len(polygon) < 3:
        return False
    n = len(polygon)
    for i in range(n):
        if _segments_intersect(seg_a, seg_b, polygon[i], polygon[(i + 1) % n]):
            return True
    midpoint = ((seg_a[0] + seg_b[0]) / 2.0, (seg_a[1] + seg_b[1]) / 2.0)
    return _point_in_polygon(midpoint, polygon)


def _points_close(p1: _Point, p2: _Point, eps: float = 0.5) -> bool:
    return abs(p1[0] - p2[0]) < eps and abs(p1[1] - p2[1]) < eps


def _segment_blocked_by_polygon(a: _Point, b: _Point, polygon: list[_Point]) -> bool:
    """Whether the segment `a`->`b` cuts through `polygon`'s interior.

    Unlike `_segment_crosses_polygon`, this tolerates `a`/`b` themselves
    being polygon vertices (or lying on a polygon edge) - needed for the
    visibility graph below, where the candidate endpoints usually *are*
    polygon corners and merely touching one is not a real obstruction.
    """
    n = len(polygon)
    a_idx = next((i for i, p in enumerate(polygon) if _points_close(a, p)), None)
    b_idx = next((i for i, p in enumerate(polygon) if _points_close(b, p)), None)
    if a_idx is not None and b_idx is not None:
        # Both endpoints are (convex) polygon vertices: adjacent ones are a
        # real boundary edge (not blocked); anything else is a diagonal,
        # which for a convex polygon necessarily cuts through the interior.
        # Decided by exact adjacency rather than the general test below,
        # because its own fallback - "is the midpoint inside?" - is exactly
        # the ill-posed question for a point that sits precisely *on* the
        # boundary, which an edge's own midpoint always does.
        diff = abs(a_idx - b_idx)
        return diff != 1 and diff != n - 1

    for i in range(n):
        p, q = polygon[i], polygon[(i + 1) % n]
        a_shared = _points_close(a, p) or _points_close(a, q)
        b_shared = _points_close(b, p) or _points_close(b, q)
        if a_shared and b_shared:
            continue  # (a, b) is this polygon edge itself - travelling along the boundary is fine
        if _segments_intersect(a, b, p, q) and not (a_shared or b_shared):
            return True
    midpoint = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
    return _point_in_polygon(midpoint, polygon)


def _shortest_detour(source: _Point, destination: _Point, polygon: list[_Point]) -> list[_Point]:
    """Shortest path from `source` to `destination` that does not cut through
    `polygon`, via a visibility graph over {source, destination} + the
    polygon's own vertices - the classic construction for routing around
    polygonal obstacles, and exact (not a heuristic) for this few a corners.

    Returns the full path including both endpoints; `path[1:-1]` are the
    via-points to insert between them.
    """
    nodes = [source, destination] + list(polygon)
    n = len(nodes)
    edge_len: list[list[float]] = [[math.inf] * n for _ in range(n)]
    for i in range(n):
        edge_len[i][i] = 0.0
        for j in range(i + 1, n):
            if not _segment_blocked_by_polygon(nodes[i], nodes[j], polygon):
                d = math.hypot(nodes[i][0] - nodes[j][0], nodes[i][1] - nodes[j][1])
                edge_len[i][j] = edge_len[j][i] = d

    # Dijkstra from node 0 (source) to node 1 (destination); n <= 6 in
    # practice, so the plain O(n^2) scan is more than fast enough.
    best = [math.inf] * n
    best[0] = 0.0
    prev = [-1] * n
    visited = [False] * n
    for _ in range(n):
        u = min((i for i in range(n) if not visited[i]), key=lambda i: best[i], default=None)
        if u is None or best[u] == math.inf:
            break
        visited[u] = True
        for v in range(n):
            if not visited[v] and edge_len[u][v] < math.inf:
                candidate = best[u] + edge_len[u][v]
                if candidate < best[v]:
                    best[v] = candidate
                    prev[v] = u

    if best[1] == math.inf:
        return [source, destination]  # no path found (degenerate polygon) - fall back to direct

    path_idx = []
    cur = 1
    while cur != -1:
        path_idx.append(cur)
        cur = prev[cur]
    path_idx.reverse()
    return [nodes[i] for i in path_idx]


def _find_matching_paren(text: str, open_pos: int) -> int:
    depth = 0
    for i in range(open_pos, len(text)):
        if text[i] == "(":
            depth += 1
        elif text[i] == ")":
            depth -= 1
            if depth == 0:
                return i
    raise ValueError("unbalanced parentheses")


def _section_span(text: str, keyword: str) -> tuple[int, int]:
    """(open, close) character indices of a `(:keyword ...)` block."""
    match = re.search(rf"\(:{re.escape(keyword)}\b", text)
    if not match:
        raise ValueError(f"no (:{keyword} ...) section found")
    open_pos = match.start()
    return open_pos, _find_matching_paren(text, open_pos)


def _add_locations(problem_text: str, names: list[str]) -> str:
    """Declare new `- location` objects in the `(:objects ...)` block."""
    _, close_pos = _section_span(problem_text, "objects")
    addition = "".join(f"\n        {name} - location" for name in names)
    return problem_text[:close_pos] + addition + "\n    " + problem_text[close_pos:]


def _add_init_facts(problem_text: str, facts_text: str) -> str:
    """Append raw fact text into the `(:init ...)` block."""
    _, close_pos = _section_span(problem_text, "init")
    return problem_text[:close_pos] + facts_text + "\n    " + problem_text[close_pos:]


def _set_detour_route(
    problem_text: str,
    source_name: str,
    destination_name: str,
    new_source: LatLon,
    new_destination: LatLon,
    via_points: list[LatLon],
) -> str:
    """Make `source -> via_points... -> destination` the *only* legal route:
    strip every existing `safe-route` fact, declare the via-points as new
    location objects, and connect the chain with fresh
    `connected`/`safe-route`/`distance`/`energy-required` facts."""
    result = _strip_safe_routes(problem_text)
    if not via_points:
        return result  # nothing to chain through - caller keeps this a dead end deliberately

    names = [f"detour{i + 1}" for i in range(len(via_points))]
    result = _add_locations(result, names)

    chain_names = [source_name] + names + [destination_name]
    chain_coords = [new_source] + via_points + [new_destination]

    facts: list[str] = []
    for name, coord in zip(names, via_points):
        facts.append(f"\n        (= (latitude {name}) {coord.lat:.7f})")
        facts.append(f"\n        (= (longitude {name}) {coord.lon:.7f})")
    for a_name, a_pt, b_name, b_pt in zip(chain_names, chain_coords, chain_names[1:], chain_coords[1:]):
        distance_m = haversine_m(a_pt, b_pt)
        energy = distance_m * DEFAULT_ENERGY_RATE
        for x, y in ((a_name, b_name), (b_name, a_name)):
            facts.append(f"\n        (connected {x} {y})")
            facts.append(f"\n        (safe-route {x} {y})")
            facts.append(f"\n        (= (distance {x} {y}) {distance_m:.2f})")
            facts.append(f"\n        (= (energy-required {x} {y}) {energy:.2f})")

    return _add_init_facts(result, "".join(facts))


def _strip_safe_routes(problem_text: str) -> str:
    """Remove every existing `(safe-route A B)` fact and its line."""
    return re.sub(r"[ \t]*\(safe-route\s+\S+\s+\S+\)\n?", "", problem_text)


def _strip_pair_facts(problem_text: str, predicate: str) -> str:
    """Remove every existing `(predicate A B)` fact and its line - used by
    both `expand_grid_drones` (`grid-neighbor`) and `expand_formation_drones`
    (`wing-neighbor`) to clear out the template's own hardcoded rank-1-only
    pair(s), which only describe the smallest case and would otherwise sit
    alongside - duplicating, and for any larger formation misdescribing
    nothing but redundantly - the complete set each expander computes fresh
    for the actual requested shape."""
    return re.sub(rf"[ \t]*\({re.escape(predicate)}\s+\S+\s+\S+\)\n?", "", problem_text)


def _set_direct_route_only(problem_text: str, source_name: str, destination_name: str) -> str:
    """Make the direct leg the *only* legal route: strip every detour
    `safe-route` fact and add just `source<->destination`.

    ENHSP's default search is satisficing, not optimal - if both the direct
    leg and the detour were left legal, it could still return either one.
    Removing the alternative instead of merely permitting the direct leg
    makes the outcome deterministic.
    """
    stripped = _strip_safe_routes(problem_text)
    addition = (
        f"\n        (safe-route {source_name} {destination_name})"
        f"\n        (safe-route {destination_name} {source_name})\n"
    )
    matches = list(_CONNECTED_FACT.finditer(stripped))
    if not matches:
        return stripped + addition
    insert_at = matches[-1].end()
    return stripped[:insert_at] + addition + stripped[insert_at:]


def retarget_problem(
    problem_text: str,
    new_source: LatLon,
    new_destination: LatLon,
    *,
    no_fly_zone: list[LatLon] | None = None,
    source_name: str = "source",
    destination_name: str = "destination",
) -> str:
    """Relocate the whole waypoint graph to a newly picked source/destination.

    Every location is carried through the same rotate+scale+translate that
    maps the template's original source->destination segment onto the new
    one, preserving the detour's shape. Every edge's distance/energy-required
    is then recomputed from the moved coordinates, at that edge's own
    original energy-per-metre rate.

    Whether the mission is safe-routed at all is an explicit input, not
    something inferred from the template: `no_fly_zone` is `None`/empty
    unless the caller actually marked one, as a polygon (in the order its
    corners were picked - typically 4 points). Auto-detecting a restriction
    from where the template's own `(no-fly-zone W)` waypoint happened to
    land after the transform does not work - that point is carried along by
    the *same* proportional rotate+scale+translate as everything else, so
    its distance from the new line necessarily scales down with a short hop
    and up with a long one, making "blocked" depend on how far apart the two
    picked points are rather than on any real obstruction.

    - `no_fly_zone` given (>= 3 points) and the direct source->destination
      line actually crosses it (an edge, or running through its interior):
      the detour `safe-route` facts are left as the template wrote them; the
      direct leg stays illegal.
    - otherwise (no polygon, one with too few points, or one the direct line
      does not actually cross): every `safe-route` fact is replaced with
      just `source<->destination` - the detour is not merely more expensive,
      it does not exist as an option.

    Only one route is ever left legal, so ENHSP's (satisficing, not optimal)
    search has nothing to choose between and the outcome is deterministic.
    """
    locations = read_locations(problem_text)
    if source_name not in locations or destination_name not in locations:
        raise ValueError(
            f"template has no latitude/longitude for {source_name!r}/{destination_name!r}"
        )

    transform = _similarity_transform(
        locations[source_name], locations[destination_name], new_source, new_destination
    )
    moved = {name: transform(point) for name, point in locations.items()}
    # Pin the endpoints exactly to the picked points, rather than trusting
    # floating-point round-trip through the transform to land on them.
    moved[source_name] = new_source
    moved[destination_name] = new_destination

    result = problem_text
    for name, point in moved.items():
        result = _set_latlon(result, name, point)

    for a, b in _read_edges(problem_text):
        if a not in moved or b not in moved:
            continue
        old_distance = _fact_value(problem_text, "distance", a, b)
        old_energy = _fact_value(problem_text, "energy-required", a, b)
        rate = (
            old_energy / old_distance
            if old_distance and old_energy is not None
            else DEFAULT_ENERGY_RATE
        )
        new_distance = haversine_m(moved[a], moved[b])
        if old_distance is not None:
            result = _set_fact_value(result, "distance", a, b, new_distance)
        if old_energy is not None:
            result = _set_fact_value(result, "energy-required", a, b, new_distance * rate)

    blocked = False
    via_points: list[LatLon] = []
    if no_fly_zone and len(no_fly_zone) >= 3:
        seg_a = _to_local(new_source, new_source)  # (0, 0), by construction
        seg_b = _to_local(new_destination, new_source)
        # The hull, not the raw click order: 4 corners clicked in a natural
        # (non-boundary-walk) order are a self-intersecting bowtie as a raw
        # polygon, which breaks every check below.
        polygon_local = _convex_hull([_to_local(p, new_source) for p in no_fly_zone])
        endpoint_inside = _point_in_polygon(seg_a, polygon_local) or _point_in_polygon(seg_b, polygon_local)
        if len(polygon_local) >= 3 and not endpoint_inside:
            # "Route around it" is meaningless if the mission starts or ends
            # inside the marked area - fly direct rather than force a detour
            # that can't actually avoid the zone it's departing/arriving in.
            #
            # Routing is planned against the *buffered* boundary (expanded by
            # NO_FLY_BUFFER_M), not the true one, so the detour clears the
            # zone by that margin instead of merely grazing its corners - the
            # true boundary's own vertices are exactly what the unbuffered
            # visibility graph below would otherwise route through. Fall
            # back to the true boundary only if the buffer would swallow an
            # endpoint the caller picked close to the zone; a source/
            # destination can't be routed clear of a boundary grown to
            # include it.
            routing_polygon = _offset_polygon(polygon_local, NO_FLY_BUFFER_M)
            if _point_in_polygon(seg_a, routing_polygon) or _point_in_polygon(seg_b, routing_polygon):
                routing_polygon = polygon_local
            blocked = _segment_crosses_polygon(seg_a, seg_b, routing_polygon)
            if blocked:
                detour_local = _shortest_detour(seg_a, seg_b, routing_polygon)
                via_points = [_from_local(p, new_source) for p in detour_local[1:-1]]

    if not blocked:
        result = _set_direct_route_only(result, source_name, destination_name)
    elif via_points:
        # A real shortest path around the polygon exists (the common case) -
        # use it instead of the template's fixed, unrelated detour shape.
        result = _set_detour_route(
            result, source_name, destination_name, new_source, new_destination, via_points
        )
    # else: blocked, but no via-points came back (a degenerate polygon) -
    # leave the template's own detour facts as the fallback.

    return result


# ---- Swarm expansion: one reference drone -> N identical drones ------------


def _clone_named_lines(text: str, span: tuple[int, int], token: str, extra_names: list[str]) -> str:
    """Duplicate every *fact* line inside ``text[span[0]:span[1]]`` that
    mentions the word ``token`` - once per name in ``extra_names``, with
    ``token`` swapped for that name - inserting the copies just before the
    span's end (i.e. before the section's / the goal ``(and ...)``'s closing
    paren).

    Comment lines (``;;...``) are skipped even if they mention ``token``: an
    illustrative comment - e.g. a small ASCII-art diagram - can split a
    parenthetical across two lines where only one of them happens to mention
    the token, and cloning just that one line would inject an unmatched
    ``(`` into the file, throwing off every later `_find_matching_paren`
    call. Comments aren't data to expand per drone anyway.
    """
    token_re = re.compile(rf"\b{re.escape(token)}\b")
    _open, close_pos = span
    clones: list[str] = []
    for line in text[_open:close_pos].splitlines():
        if line.lstrip().startswith(";"):
            continue
        if token_re.search(line):
            clones.extend(token_re.sub(name, line) for name in extra_names)
    if not clones:
        return text
    head = text[:close_pos].rstrip()
    addition = "".join("\n" + line for line in clones)
    return f"{head}{addition}\n    {text[close_pos:]}"


def _clone_drone1_lines(text: str, span: tuple[int, int], extra_names: list[str]) -> str:
    return _clone_named_lines(text, span, "drone1", extra_names)


def expand_point_to_point_drones(problem_text: str, count: int) -> str:
    """Grow a point-to-point ``swarm-drone-mission`` problem from its single
    reference drone (``drone1``) to ``count`` identical drones
    (``drone1`` .. ``drone{count}``), so a swarm of that many checked drones
    all fly the planned route to the same destination.

    Every ``(:init ...)`` fact and numeric fluent that mentions ``drone1`` is
    duplicated for each extra drone (same start location, battery, health,
    GPS/comms state), the ``(:objects ...)`` declaration is widened, and the
    goal's ``(and ...)`` gains a ``(mission-completed droneN)`` /
    ``(at droneN destination)`` pair per drone.

    ``count <= 1`` returns the text unchanged - the ordinary single-drone
    problem. Purely additive text substitution over ``drone1``; the waypoint
    graph, routes and coordinates are untouched (retarget those first).
    """
    if count <= 1:
        return problem_text

    all_names = [f"drone{i}" for i in range(1, count + 1)]
    extra_names = all_names[1:]

    # 1. widen the (:objects ...) declaration: "drone1 - drone" -> "drone1 drone2 ... - drone"
    obj_open, obj_close = _section_span(problem_text, "objects")
    widened = re.sub(
        r"\bdrone1\b(\s*-\s*drone\b)",
        lambda m: " ".join(all_names) + m.group(1),
        problem_text[obj_open:obj_close],
        count=1,
    )
    if widened == problem_text[obj_open:obj_close]:
        raise ValueError("point-to-point template has no 'drone1 - drone' object to expand")
    problem_text = problem_text[:obj_open] + widened + problem_text[obj_close:]

    # 2. clone every (:init ...) fact that references drone1
    problem_text = _clone_drone1_lines(
        problem_text, _section_span(problem_text, "init"), extra_names
    )

    # 3. clone the goal's per-drone conjuncts, inside its (and ...) so the
    #    result stays a single well-formed goal expression
    goal_open, goal_close = _section_span(problem_text, "goal")
    and_match = re.search(r"\(and\b", problem_text[goal_open:goal_close])
    if and_match:
        and_open = goal_open + and_match.start()
        span = (and_open, _find_matching_paren(problem_text, and_open))
    else:
        span = (goal_open, goal_close)
    problem_text = _clone_drone1_lines(problem_text, span, extra_names)

    return problem_text


# ---- V-formation retargeting -------------------------------------------------

# The 8 compass objects declared in plans/vformation/problem.pddl's
# :objects, in bearing order starting from north (0 deg) and stepping 45 deg
# clockwise - index i is the nearest compass point to a bearing of i * 45 deg.
_COMPASS_POINTS = (
    "north", "north-east", "east", "south-east",
    "south", "south-west", "west", "north-west",
)

_SLOT_DIRECTION_FACT = re.compile(r"(\(slot-direction\s+(\S+)\s+)(\S+)(\))")


def _unary_fact_value(problem_text: str, predicate: str, obj: str) -> float | None:
    """`(= (predicate obj) v)` -> v - the one-argument counterpart to
    `_fact_value`, for per-drone fluents like `slot-along-offset`."""
    pattern = re.compile(
        rf"\(=\s*\({re.escape(predicate)}\s+{re.escape(obj)}\)\s*([-\d.eE]+)\s*\)"
    )
    match = pattern.search(problem_text)
    return float(match.group(1)) if match else None


def _set_unary_fact_value(problem_text: str, predicate: str, obj: str, value: float) -> str:
    """`(= (predicate obj) v)` -> `(= (predicate obj) value)` - the
    one-argument counterpart to `_set_fact_value`."""
    pattern = re.compile(
        rf"(\(=\s*\({re.escape(predicate)}\s+{re.escape(obj)}\)\s*)[-\d.eE]+(\s*\))"
    )
    return pattern.sub(lambda m: f"{m.group(1)}{value:.6f}{m.group(2)}", problem_text, count=1)


def _bearing_deg(a: LatLon, b: LatLon) -> float:
    """Compass bearing (0 = north, clockwise) from `a` to `b` - locally
    flat, accurate at the leg lengths a formation mission flies."""
    east, north = _to_local(b, a)
    return math.degrees(math.atan2(east, north)) % 360.0


def _nearest_compass_point(bearing_deg: float) -> str:
    return _COMPASS_POINTS[round(bearing_deg / 45.0) % 8]


def _wing_slot_direction(travel_bearing_deg: float, along_offset_m: float, cross_offset_m: float) -> str:
    """The real compass point a wing's fixed (along, cross) offset - in the
    frame of a formation travelling at `travel_bearing_deg` - points to,
    relative to the leader. `along` is measured backward along the
    direction of travel (negative = behind, the template's convention);
    `cross` is perpendicular to it (negative = left of travel, positive =
    right)."""
    rad = math.radians(travel_bearing_deg)
    east = along_offset_m * math.sin(rad) + cross_offset_m * math.cos(rad)
    north = along_offset_m * math.cos(rad) - cross_offset_m * math.sin(rad)
    return _nearest_compass_point(math.degrees(math.atan2(east, north)) % 360.0)


def _set_slot_direction(problem_text: str, drone_name: str, direction_name: str) -> str:
    def _repl(match: re.Match) -> str:
        if match.group(2) != drone_name:
            return match.group(0)
        return f"{match.group(1)}{direction_name}{match.group(4)}"

    return _SLOT_DIRECTION_FACT.sub(_repl, problem_text)


_WING_DRONE_FACT = re.compile(r"\(wing-drone\s+(\S+)\)")


def expand_formation_drones(
    problem_text: str,
    drone_count: int,
    *,
    left_wing_name: str = "drone-left",
    right_wing_name: str = "drone-right",
) -> str:
    """Grow the v-formation template's rank-1 wings (`drone_count == 3`: the
    apex plus one drone on each wing) to any larger odd `drone_count` - one
    more drone per wing per +2 - so a V can fly with 3, 5, 7, ... drones:
    one at the apex, the rest split evenly between both wings.

    Each new rank sits exactly as far again past the previous one as rank 1
    sits past the apex, so every consecutive pair along an arm (apex ->
    rank 1 -> rank 2 -> ...) is the same 20 m apart and the arm reads as a
    straight line running out from the apex - the two arms together the
    "V". A rank's `slot-direction` doesn't need a fresh value: it only
    depends on the along/cross *ratio* (see `_wing_slot_direction`), which
    scaling by rank leaves unchanged, so cloning rank 1's own fact for
    every new rank is already correct.

    domain.pddl's formation actions (ACTION 5/6b/7) are generic over
    however many drones carry `wing-drone`, not a fixed left/right pair of
    named parameters, so any odd count works with no domain change: this
    only has to grow the *problem* - `:objects`, the cloned `:init` facts,
    and the goal's per-drone conjuncts - the same way
    `expand_point_to_point_drones` and `expand_search_drones` grow their
    own templates.

    `drone_count` must be odd and >= 3 (the template's own apex + one wing
    drone per side); anything else raises `ValueError`. `drone_count == 3`
    returns the template unchanged.
    """
    if drone_count < 3 or drone_count % 2 == 0:
        raise ValueError(f"v-formation drone_count must be odd and >= 3, got {drone_count}")
    ranks = (drone_count - 1) // 2
    if ranks <= 1:
        return problem_text

    base_along = {
        name: _unary_fact_value(problem_text, "slot-along-offset", name)
        for name in (left_wing_name, right_wing_name)
    }
    base_cross = {
        name: _unary_fact_value(problem_text, "slot-cross-offset", name)
        for name in (left_wing_name, right_wing_name)
    }
    if any(v is None for v in (*base_along.values(), *base_cross.values())):
        raise ValueError(
            f"template has no slot-along-offset/slot-cross-offset for "
            f"{left_wing_name!r}/{right_wing_name!r}"
        )

    # 1. Objects: one "<name><rank> - drone" line per new rank per wing.
    obj_open, obj_close = _section_span(problem_text, "objects")
    extra_objects = "".join(
        f"\n        {name}{r} - drone"
        for r in range(2, ranks + 1)
        for name in (left_wing_name, right_wing_name)
    )
    result = problem_text[:obj_close] + extra_objects + "\n    " + problem_text[obj_close:]

    left_ranks = [f"{left_wing_name}{r}" for r in range(2, ranks + 1)]
    right_ranks = [f"{right_wing_name}{r}" for r in range(2, ranks + 1)]

    # 2. Clone every :init fact mentioning a rank-1 wing, once per new rank
    #    - everything (health, battery, slot-direction, wing-drone, ...) is
    #    identical across ranks except along/cross offset, fixed up after.
    result = _clone_named_lines(result, _section_span(result, "init"), left_wing_name, left_ranks)
    result = _clone_named_lines(result, _section_span(result, "init"), right_wing_name, right_ranks)
    for r, name in zip(range(2, ranks + 1), left_ranks):
        result = _set_unary_fact_value(result, "slot-along-offset", name, base_along[left_wing_name] * r)
        result = _set_unary_fact_value(result, "slot-cross-offset", name, base_cross[left_wing_name] * r)
    for r, name in zip(range(2, ranks + 1), right_ranks):
        result = _set_unary_fact_value(result, "slot-along-offset", name, base_along[right_wing_name] * r)
        result = _set_unary_fact_value(result, "slot-cross-offset", name, base_cross[right_wing_name] * r)

    # 3. Clone the goal's per-drone conjuncts the same way, inside its
    #    (and ...) so the result stays a single well-formed goal expression.
    def _goal_and_span(text: str) -> tuple[int, int]:
        goal_open, goal_close = _section_span(text, "goal")
        and_match = re.search(r"\(and\b", text[goal_open:goal_close])
        if not and_match:
            return goal_open, goal_close
        and_open = goal_open + and_match.start()
        return and_open, _find_matching_paren(text, and_open)

    result = _clone_named_lines(result, _goal_and_span(result), left_wing_name, left_ranks)
    result = _clone_named_lines(result, _goal_and_span(result), right_wing_name, right_ranks)

    # 4. Neighbor links: each drone's own left/right neighbor along its wing
    #    - leader <-> rank 1, then rank r <-> rank r+1 out to the tip, one
    #    symmetric pair of `wing-neighbor` facts per link, the same rebuild
    #    `expand_grid_drones` does for `grid-neighbor`. Step 2 already cloned
    #    the template's own rank-1-only pair (leader<->rank-1) onto every new
    #    rank name too, which would wire every further rank straight to the
    #    leader instead of to its own inward neighbor - strip all of it first
    #    and lay down the complete chain fresh.
    result = _strip_pair_facts(result, "wing-neighbor")
    chain_facts: list[str] = []
    for chain in (
        ["drone-lead", left_wing_name, *left_ranks],
        ["drone-lead", right_wing_name, *right_ranks],
    ):
        for a, b in zip(chain, chain[1:]):
            chain_facts.append(f"\n        (wing-neighbor {a} {b})")
            chain_facts.append(f"\n        (wing-neighbor {b} {a})")
    result = _add_init_facts(result, "".join(chain_facts))

    return result


def retarget_formation_problem(
    problem_text: str,
    new_source: LatLon,
    new_destination: LatLon,
    *,
    no_fly_zone: list[LatLon] | None = None,
    no_fly_buffer_m: float = NO_FLY_BUFFER_M,
    source_name: str = "source",
    destination_name: str = "destination",
    left_wing_name: str = "drone-left",
    right_wing_name: str = "drone-right",
) -> str:
    """Relocate a `v-formation-drone-mission` problem onto a picked
    source/destination.

    Same coordinate relocation as `retarget_problem` - every location is
    carried through the rotate+scale+translate that maps the template's
    `source->destination` onto the new one, and each edge's distance /
    energy-required is recomputed from the moved points at that edge's own
    original per-metre rate.

    `no_fly_zone` is handled the same way `retarget_problem` handles it: if
    the marked polygon (>= 3 points) actually crosses the direct
    source->destination line, a shortest-path detour around its buffered
    boundary (`no_fly_buffer_m` clearance, `NO_FLY_BUFFER_M` by default) is
    computed via the same visibility-graph construction, and the direct
    `safe-route` is replaced with a `source -> via-points... -> destination`
    chain of fresh `connected`/`safe-route`/`distance`/`energy-required`
    facts (`_set_detour_route`) - so the apex's `formation-cruise` corridor
    bends around the zone leg by leg instead of cutting straight through it,
    and both wings (held parallel to the apex by `formation_wing_routes`)
    bend with it. Only the apex's centreline is planned against the buffer,
    not the wings' own tracks - callers that hold drones off to the side
    (any `left_wing_name`/`right_wing_name` offset) should pass a
    `no_fly_buffer_m` padded by that offset so the whole formation, not just
    the centreline, clears the zone. If the polygon doesn't actually block
    the direct line (or is absent, too small, or one of the endpoints sits
    inside it), the template's own `source<->destination` route is left
    untouched - there is no alternate route to prune the way
    `retarget_problem`'s `_set_direct_route_only` prunes a point-to-point
    template's detour.

    Every wing drone's `slot-direction` (the real compass point - north/
    south/east/west/north-east/north-west/south-east/south-west - its fixed
    `slot-along-offset`/`slot-cross-offset` geometry actually points to) is
    also recomputed here, from the *new* source->destination bearing - a
    wing sitting behind and to the left of a formation heading east points
    north-west; the same offset heading north points north-east instead.
    The offsets themselves (what puts the leader and every wing drone at
    the corners/along the arms of a fixed-size V) never change; only which
    compass label that shape's wings happen to point at does. Which drones
    count as wings is read from the problem's own `(wing-drone ...)` facts
    - however many `expand_formation_drones` put there - falling back to
    just `left_wing_name`/`right_wing_name` if the template predates that
    predicate.
    """
    locations = read_locations(problem_text)
    if source_name not in locations or destination_name not in locations:
        raise ValueError(
            f"template has no latitude/longitude for {source_name!r}/{destination_name!r}"
        )

    transform = _similarity_transform(
        locations[source_name], locations[destination_name], new_source, new_destination
    )
    moved = {name: transform(point) for name, point in locations.items()}
    moved[source_name] = new_source
    moved[destination_name] = new_destination

    result = problem_text
    for name, point in moved.items():
        result = _set_latlon(result, name, point)

    for a, b in _read_edges(problem_text):
        if a not in moved or b not in moved:
            continue
        old_distance = _fact_value(problem_text, "distance", a, b)
        old_energy = _fact_value(problem_text, "energy-required", a, b)
        rate = (
            old_energy / old_distance
            if old_distance and old_energy is not None
            else DEFAULT_ENERGY_RATE
        )
        new_distance = haversine_m(moved[a], moved[b])
        if old_distance is not None:
            result = _set_fact_value(result, "distance", a, b, new_distance)
        if old_energy is not None:
            result = _set_fact_value(result, "energy-required", a, b, new_distance * rate)

    if no_fly_zone and len(no_fly_zone) >= 3:
        seg_a = _to_local(new_source, new_source)  # (0, 0), by construction
        seg_b = _to_local(new_destination, new_source)
        polygon_local = _convex_hull([_to_local(p, new_source) for p in no_fly_zone])
        endpoint_inside = _point_in_polygon(seg_a, polygon_local) or _point_in_polygon(seg_b, polygon_local)
        if len(polygon_local) >= 3 and not endpoint_inside:
            routing_polygon = _offset_polygon(polygon_local, no_fly_buffer_m)
            if _point_in_polygon(seg_a, routing_polygon) or _point_in_polygon(seg_b, routing_polygon):
                routing_polygon = polygon_local
            if _segment_crosses_polygon(seg_a, seg_b, routing_polygon):
                detour_local = _shortest_detour(seg_a, seg_b, routing_polygon)
                via_points = [_from_local(p, new_source) for p in detour_local[1:-1]]
                if via_points:
                    result = _set_detour_route(
                        result, source_name, destination_name, new_source, new_destination, via_points
                    )

    bearing = _bearing_deg(new_source, new_destination)
    wing_names = _WING_DRONE_FACT.findall(problem_text) or [left_wing_name, right_wing_name]
    for wing_name in wing_names:
        along = _unary_fact_value(problem_text, "slot-along-offset", wing_name)
        cross = _unary_fact_value(problem_text, "slot-cross-offset", wing_name)
        if along is not None and cross is not None:
            result = _set_slot_direction(result, wing_name, _wing_slot_direction(bearing, along, cross))

    return result


def _unit(v: tuple[float, float]) -> tuple[float, float] | None:
    norm = math.hypot(*v)
    return (v[0] / norm, v[1] / norm) if norm > 1e-9 else None


def formation_member_route(
    lead_route: list[LatLon], *, along_m: float, cross_m: float
) -> list[LatLon]:
    """One formation member's track, parallel to `lead_route`.

    Each waypoint - including the first (`source`) - sits `along_m` behind
    its lead waypoint (opposite the direction of travel; negative = ahead)
    and `cross_m` out to its right (negative = left), so the member-to-lead
    spacing is the fixed formation distance everywhere along the route,
    source to destination alike, not just once airborne. The single-offset
    counterpart of the old `formation_wing_routes` (now built on top of this
    - see below): a V's two wings are always a symmetric +-`side_m` pair
    around the lead track, but a grid's members generally aren't (a 3x4
    grid's twelve members each sit at their own distinct (row, column)
    offset), so each needs its own independent call.

    At an interior waypoint (a detour bend - see `retarget_formation_problem`'s
    `no_fly_zone` handling) "forward" is the turn bisector: the sum of the
    incoming leg's and outgoing leg's own unit directions, not the straight
    line from the point *before* the turn to the point *after* it. That
    straight-line shortcut skips the bend entirely - for a single bend
    (source -> via -> destination) it collapses right back to the original
    source->destination bearing regardless of how far `via` swings out to
    the side - so a member would hold the pre-detour heading exactly at the
    bend, pinching together or crossing the lead track instead of turning
    with it. An endpoint has only one leg, so that leg's own direction is
    used directly, same as before.

    Returns a route the same length as `lead_route`. A single-point (or
    empty) lead route is returned unchanged.
    """
    if len(lead_route) < 2:
        return list(lead_route)

    n = len(lead_route)
    route: list[LatLon] = []
    for i, here in enumerate(lead_route):
        incoming = _unit(_to_local(here, lead_route[i - 1])) if i > 0 else None
        outgoing = _unit(_to_local(lead_route[i + 1], here)) if i + 1 < n else None

        fwd = None
        if incoming and outgoing:
            fwd = _unit((incoming[0] + outgoing[0], incoming[1] + outgoing[1]))
        if fwd is None:
            fwd = incoming or outgoing

        if fwd is None:
            route.append(here)
            continue
        port = (-fwd[1], fwd[0])               # 90 deg left of forward
        back = (-fwd[0] * along_m, -fwd[1] * along_m)
        route.append(_from_local((back[0] + port[0] * cross_m, back[1] + port[1] * cross_m), here))
    return route


def formation_wing_routes(
    lead_route: list[LatLon], *, back_m: float, side_m: float
) -> tuple[list[LatLon], list[LatLon]]:
    """The two wing tracks of a V, parallel to `lead_route` - left wing to
    port, right wing to starboard, both `back_m` behind the leader and
    `side_m` out to their own side. A symmetric pair of `formation_member_route`
    calls (`side_m` and `-side_m`) - see that function for the geometry.

    Returns `(left_route, right_route)`, each the same length as
    `lead_route`.
    """
    left = formation_member_route(lead_route, along_m=back_m, cross_m=side_m)
    right = formation_member_route(lead_route, along_m=back_m, cross_m=-side_m)
    return left, right


# ---- Grid-formation expansion/retargeting ----------------------------------

# Spacing between adjacent drones (front/back and left/right alike) in a
# grid-formation mission, in metres. Must be kept in sync with nothing else -
# unlike the V's FORMATION_BACK_M/FORMATION_SIDE_M (services.plan_service),
# a grid's member offsets are computed straight from this one constant, both
# here (expand_grid_drones, writing the descriptive slot-along-offset/
# slot-cross-offset facts) and in services.plan_service (deriving each
# member's actual flown track via formation_member_route).
GRID_SPACING_M = 10.0

_GRID_MEMBER_TOKEN = "drone-member"


def grid_dimensions(drone_count: int) -> tuple[int, int]:
    """(rows, cols) for a grid of exactly `drone_count` drones.

    A perfect-square count forms a square grid (side x side, e.g. 4 -> 2x2,
    9 -> 3x3, 16 -> 4x4). Anything else forms the most-square rectangle that
    tiles it exactly: the largest divisor of `drone_count` that is <= its
    square root, paired with the matching quotient (8 -> 2x4, 12 -> 3x4).
    A count with no such divisor besides 1 (a prime, e.g. 7 or 11) has no
    rectangle to form and degenerates to a single line, 1 x `drone_count`.

    `rows <= cols` always; `expand_grid_drones` decides which physical axis
    is which (row 0 is the drone(s) nearest the direction of travel, i.e.
    the front row).
    """
    if drone_count < 1:
        raise ValueError(f"grid drone_count must be >= 1, got {drone_count}")
    rows = int(math.isqrt(drone_count))
    while rows > 1 and drone_count % rows != 0:
        rows -= 1
    return rows, drone_count // rows


def grid_cell_names(rows: int, cols: int) -> dict[tuple[int, int], str]:
    """(row, col) -> drone object name for a `rows x cols` grid-formation
    swarm, 0-indexed with row 0 the front row - the same leader/member
    naming `expand_grid_drones` builds into the problem file, exposed so a
    caller (`services.plan_service`) can derive every drone's name and its
    row/col without re-parsing the expanded text.

    The leader sits at the front row, in the column nearest the middle
    (`(cols - 1) // 2`) - exact for odd `cols`, off by at most half a slot
    for even `cols` (the same unavoidable asymmetry a V-formation has when
    it can't split its wings perfectly evenly). Every other cell is a
    `grid-member` drone: the first one encountered in row-major order keeps
    the template's own `drone-member` name (see `expand_grid_drones`); the
    rest are named `drone-r{row}c{col}`.
    """
    leader_cell = (0, (cols - 1) // 2)
    names = {leader_cell: "drone-lead"}
    member_cells = [
        (row, col) for row in range(rows) for col in range(cols) if (row, col) != leader_cell
    ]
    for i, cell in enumerate(member_cells):
        names[cell] = _GRID_MEMBER_TOKEN if i == 0 else f"drone-r{cell[0]}c{cell[1]}"
    return names


def expand_grid_drones(problem_text: str, drone_count: int) -> str:
    """Grow the grid-formation template's single reference member
    (`drone-member` - the smallest case, a 1x2 grid: the leader plus one
    drone at its side) into a full `rows x cols` grid sized for
    `drone_count` drones total (see `grid_dimensions`).

    Unlike `expand_formation_drones`'s V (which only ever grows two fixed
    wings further out along the same two lines), a grid's shape changes
    non-monotonically between drone counts - 9 drones is a 3x3 square, not
    "2x2 plus one more somewhere" - so this always rebuilds the full member
    list, offsets and neighbor links from `drone_count` directly, the same
    way for every count, rather than incrementally extending whatever the
    template already had.

    Every member's `slot-along-offset` is `-row * GRID_SPACING_M` (rows
    behind the leader trail it, matching the V wings' own trailing-negative
    convention; the leader's own row is always 0) and `slot-cross-offset` is
    `(col - leader_col) * GRID_SPACING_M` (negative = left of the leader,
    positive = right) - see `grid_cell_names` for the leader/column choice.

    Adjacent cells - front/back and left/right, not diagonal - get a
    symmetric pair of `grid-neighbor` facts each: descriptive structure
    only (domain.pddl's actions don't read it), recording that every drone
    has up to 4 neighbors. What those actions actually check is each
    drone's own `neighbor-comm-ok` (cloned/set per drone the same way every
    other per-drone :init fact is, by the `_clone_named_lines` calls above)
    - a per-drone flag, not a pairwise one, so the check stays O(count) per
    action instead of O(count^2) - see domain.pddl's own predicates comment
    for why that matters.

    `drone_count` must be >= 2 (a grid needs at least one neighbor pair);
    raises `ValueError` otherwise.
    """
    if drone_count < 2:
        raise ValueError(f"grid-formation drone_count must be >= 2, got {drone_count}")

    rows, cols = grid_dimensions(drone_count)
    leader_col = (cols - 1) // 2
    cell_name = grid_cell_names(rows, cols)
    member_cells = [cell for cell, name in cell_name.items() if name != "drone-lead"]
    member_names = [cell_name[cell] for cell in member_cells]
    first_name, rest_names = member_names[0], member_names[1:]

    # 1. Objects: widen "drone-member - drone" to every member name, the
    #    same way expand_point_to_point_drones widens "drone1 - drone" -
    #    the first name is the template's own "drone-member" (kept as-is,
    #    never renamed - it just represents whichever cell comes first in
    #    row-major order for this particular grid size); the rest are fresh
    #    clones.
    obj_open, obj_close = _section_span(problem_text, "objects")
    widened, n_matched = re.subn(
        rf"\b{re.escape(_GRID_MEMBER_TOKEN)}\b(\s*-\s*drone\b)",
        lambda m: " ".join([first_name, *rest_names]) + m.group(1),
        problem_text[obj_open:obj_close],
        count=1,
    )
    if not n_matched:
        raise ValueError("grid-formation template has no 'drone-member - drone' object to expand")
    result = problem_text[:obj_open] + widened + problem_text[obj_close:]

    # 2. Clone every :init fact and goal conjunct mentioning drone-member,
    #    once per further member (the same _clone_named_lines machinery
    #    expand_formation_drones uses for further V ranks).
    result = _clone_named_lines(result, _section_span(result, "init"), _GRID_MEMBER_TOKEN, rest_names)

    def _goal_and_span(text: str) -> tuple[int, int]:
        goal_open, goal_close = _section_span(text, "goal")
        and_match = re.search(r"\(and\b", text[goal_open:goal_close])
        if not and_match:
            return goal_open, goal_close
        and_open = goal_open + and_match.start()
        return and_open, _find_matching_paren(text, and_open)

    result = _clone_named_lines(result, _goal_and_span(result), _GRID_MEMBER_TOKEN, rest_names)

    # 3. Fix up every drone's along/cross offset for its actual cell.
    result = _set_unary_fact_value(result, "slot-along-offset", "drone-lead", 0.0)
    result = _set_unary_fact_value(result, "slot-cross-offset", "drone-lead", 0.0)
    for cell, name in zip(member_cells, member_names):
        row, col = cell
        result = _set_unary_fact_value(result, "slot-along-offset", name, -row * GRID_SPACING_M)
        result = _set_unary_fact_value(
            result, "slot-cross-offset", name, (col - leader_col) * GRID_SPACING_M
        )

    # 4. Neighbor links: front/back and left/right adjacency only (no
    #    diagonals), one symmetric pair of grid-neighbor facts per adjacent
    #    cell pair - descriptive structure only (domain.pddl's actions check
    #    the per-drone `neighbor-comm-ok`, already cloned/set for every
    #    drone by steps 1-2 above, not this). The template's own hardcoded
    #    drone-lead<->drone-member pair only describes the smallest (1x2)
    #    case - strip it first so it isn't left duplicating (or, for any
    #    other shape, redundant alongside) the freshly computed complete set.
    result = _strip_pair_facts(result, "grid-neighbor")
    neighbor_facts: list[str] = []
    for row in range(rows):
        for col in range(cols):
            here = cell_name[(row, col)]
            for neighbor_row, neighbor_col in ((row + 1, col), (row, col + 1)):
                if neighbor_row < rows and neighbor_col < cols:
                    there = cell_name[(neighbor_row, neighbor_col)]
                    neighbor_facts.append(f"\n        (grid-neighbor {here} {there})")
                    neighbor_facts.append(f"\n        (grid-neighbor {there} {here})")
    result = _add_init_facts(result, "".join(neighbor_facts))

    return result


def retarget_grid_problem(
    problem_text: str,
    new_source: LatLon,
    new_destination: LatLon,
    *,
    no_fly_zone: list[LatLon] | None = None,
    no_fly_buffer_m: float = NO_FLY_BUFFER_M,
    source_name: str = "source",
    destination_name: str = "destination",
) -> str:
    """Relocate a `grid-formation-drone-mission` problem onto a picked
    source/destination.

    The same coordinate relocation `retarget_formation_problem` does for a
    V: every location is carried through the rotate+scale+translate that
    maps the template's `source->destination` onto the new one, each edge's
    distance/energy-required is recomputed from the moved points at that
    edge's own original per-metre rate, and (if a marked restricted area - 3+
    points - actually crosses the direct line) the route is replaced with a
    shortest-path detour around its buffered boundary. See
    `retarget_formation_problem`'s docstring for the full detour mechanics;
    unlike that V-formation counterpart, a grid has no `slot-direction`/
    compass-label bookkeeping to recompute afterwards - a grid member's
    position is read off `slot-along-offset`/`slot-cross-offset` directly
    (see `expand_grid_drones`), not a labelled direction, so there is
    nothing else to do once the coordinates and route are relocated.

    Only the leader's corridor is planned against the buffer; a caller whose
    grid extends some members off to the side of the leader's own column
    should pad `no_fly_buffer_m` by that extra lateral reach (see
    `services.plan_service._run_grid_formation`), the same way a V-formation
    caller pads it by `FORMATION_SIDE_M * ranks`.
    """
    locations = read_locations(problem_text)
    if source_name not in locations or destination_name not in locations:
        raise ValueError(
            f"template has no latitude/longitude for {source_name!r}/{destination_name!r}"
        )

    transform = _similarity_transform(
        locations[source_name], locations[destination_name], new_source, new_destination
    )
    moved = {name: transform(point) for name, point in locations.items()}
    moved[source_name] = new_source
    moved[destination_name] = new_destination

    result = problem_text
    for name, point in moved.items():
        result = _set_latlon(result, name, point)

    for a, b in _read_edges(problem_text):
        if a not in moved or b not in moved:
            continue
        old_distance = _fact_value(problem_text, "distance", a, b)
        old_energy = _fact_value(problem_text, "energy-required", a, b)
        rate = (
            old_energy / old_distance
            if old_distance and old_energy is not None
            else DEFAULT_ENERGY_RATE
        )
        new_distance = haversine_m(moved[a], moved[b])
        if old_distance is not None:
            result = _set_fact_value(result, "distance", a, b, new_distance)
        if old_energy is not None:
            result = _set_fact_value(result, "energy-required", a, b, new_distance * rate)

    if no_fly_zone and len(no_fly_zone) >= 3:
        seg_a = _to_local(new_source, new_source)  # (0, 0), by construction
        seg_b = _to_local(new_destination, new_source)
        polygon_local = _convex_hull([_to_local(p, new_source) for p in no_fly_zone])
        endpoint_inside = _point_in_polygon(seg_a, polygon_local) or _point_in_polygon(seg_b, polygon_local)
        if len(polygon_local) >= 3 and not endpoint_inside:
            routing_polygon = _offset_polygon(polygon_local, no_fly_buffer_m)
            if _point_in_polygon(seg_a, routing_polygon) or _point_in_polygon(seg_b, routing_polygon):
                routing_polygon = polygon_local
            if _segment_crosses_polygon(seg_a, seg_b, routing_polygon):
                detour_local = _shortest_detour(seg_a, seg_b, routing_polygon)
                via_points = [_from_local(p, new_source) for p in detour_local[1:-1]]
                if via_points:
                    result = _set_detour_route(
                        result, source_name, destination_name, new_source, new_destination, via_points
                    )

    return result
