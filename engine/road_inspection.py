"""Road inspection: the road route between two clicks, and its PDDL problem.

    picked source + destination (anywhere on a road)
        -> fetch_road_route: the road's exact path between them, from an
           OSRM routing server over OpenStreetMap roads
        -> plans/Inspect/problem.pddl (template) -> retarget_inspection_problem
        -> ENHSP plans: inspect every road segment source -> destination,
           fly back along the same road, land at the source
        -> the drone flies the road's FULL geometry, every curve, out and back

The route comes from OSRM's `route` service (driving profile), which
follows real roads, curve by curve, and answers "no route" when the two
roads don't connect. OSRM always snaps a click to the nearest road - even
one hundreds of metres away - so a click further than `MAX_SNAP_M` from
any road is refused here too: inspecting "the road" from a point in a field
isn't what the operator meant.

The server is the public OSRM demo (`https://router.project-osrm.org`)
unless the `DRONE_SIM_OSRM_URL` environment variable points at another
(e.g. a self-hosted one for offline use). The two clicked points are sent
to it - the same kind of location the map's tile requests already reveal.
"""
from __future__ import annotations

import math
import os
import re
from dataclasses import dataclass
from typing import Optional

import requests

from engine.pddl_problem import (
    LatLon,
    _add_init_facts,
    _add_locations,
    _set_latlon,
    haversine_m,
)

OSRM_URL = os.environ.get("DRONE_SIM_OSRM_URL", "https://router.project-osrm.org")
USER_AGENT = "DroneSwarmSimulatorGUI/0.1"
MAX_SNAP_M = 30.0          # a click must be this close to a road
MIN_ROUTE_M = 20.0         # shorter than this isn't a road to inspect
MAX_PDDL_LEGS = 12         # the planner reasons over at most this many road segments
# The PDDL battery is abstract (100 units); the real per-drone battery check
# runs on each drone's own profile before flight, so the whole out-and-back
# inspection is budgeted this many units.
_PDDL_MISSION_ENERGY = 40.0


class RoadRouteError(ValueError):
    """There's no road route to inspect - the message says why, for the
    operator."""


@dataclass(frozen=True)
class RoadRoute:
    """The road between the two clicks, as the drone will fly it."""

    points: list[LatLon]        # source end -> destination end, every curve
    distance_m: float
    source_snap_m: float        # how far each click was from the road
    destination_snap_m: float
    road_names: tuple[str, ...]  # named roads along the way, in order (may be empty)

    def out_and_back(self) -> list[LatLon]:
        """Source -> destination along the road, then the same road back."""
        return [*self.points, *reversed(self.points[:-1])]


def _dedupe(points: list[LatLon], min_gap_m: float = 0.5) -> list[LatLon]:
    out = [points[0]]
    for p in points[1:]:
        if haversine_m(out[-1], p) >= min_gap_m:
            out.append(p)
    if len(out) == 1 and len(points) > 1:
        out.append(points[-1])
    return out


def fetch_road_route(
    source: LatLon,
    destination: LatLon,
    *,
    base_url: str = OSRM_URL,
    timeout_s: float = 15.0,
    session: Optional[requests.Session] = None,
) -> RoadRoute:
    """The road route from `source` to `destination`. Raises
    `RoadRouteError` with an operator-facing message when there isn't one
    to inspect."""
    url = (
        f"{base_url.rstrip('/')}/route/v1/driving/"
        f"{source.lon:.7f},{source.lat:.7f};{destination.lon:.7f},{destination.lat:.7f}"
    )
    http = session or requests
    try:
        response = http.get(
            url,
            params={"overview": "full", "geometries": "geojson", "steps": "true"},
            headers={"User-Agent": USER_AGENT},
            timeout=timeout_s,
        )
        body = response.json()
    except requests.RequestException as exc:
        raise RoadRouteError(
            "Couldn't reach the road-routing service, so the road between the two "
            "points can't be looked up.\n\n"
            f"Inspect needs an internet connection to {base_url} "
            f"({exc.__class__.__name__})."
        ) from exc
    except ValueError as exc:
        raise RoadRouteError(
            f"The road-routing service at {base_url} sent back something that isn't "
            f"a route (HTTP {response.status_code})."
        ) from exc

    code = body.get("code")
    if code == "NoRoute":
        raise RoadRouteError(
            "There is no road route between the two points you clicked - the roads "
            "they are on don't connect (for example, across a river with no bridge, "
            "or into a closed-off area).\n\nPick a source and destination on "
            "connected roads."
        )
    if code == "NoSegment":
        raise RoadRouteError(
            "One of the points you clicked isn't near any road the routing service "
            "knows. Click on a road for both the source and the destination."
        )
    if code != "Ok" or not body.get("routes"):
        raise RoadRouteError(
            f"The road-routing service couldn't find a route ({code}: "
            f"{body.get('message', 'no details')})."
        )

    snaps = [float(w.get("distance", 0.0)) for w in body.get("waypoints", [])]
    for label, snap in zip(("source", "destination"), snaps):
        if snap > MAX_SNAP_M:
            raise RoadRouteError(
                f"The {label} you clicked is {snap:.0f} m from the nearest road.\n\n"
                f"Inspect follows a road, so click on the road itself (within "
                f"{MAX_SNAP_M:.0f} m of it) for both the source and the destination."
            )

    route = body["routes"][0]
    coords = route["geometry"]["coordinates"]
    points = _dedupe([LatLon(lat=float(lat), lon=float(lon)) for lon, lat in coords])
    distance = float(route.get("distance", 0.0))
    if len(points) < 2 or distance < MIN_ROUTE_M:
        raise RoadRouteError(
            f"The source and destination are only {distance:.0f} m apart along the "
            f"road - pick two points further apart to inspect the road between them."
        )

    names: list[str] = []
    for leg in route.get("legs", []):
        for step in leg.get("steps", []):
            name = (step.get("name") or "").strip()
            if name and (not names or names[-1] != name):
                names.append(name)
    return RoadRoute(
        points=points,
        distance_m=distance,
        source_snap_m=snaps[0] if snaps else 0.0,
        destination_snap_m=snaps[1] if len(snaps) > 1 else 0.0,
        road_names=tuple(names),
    )


# ---- PDDL ---------------------------------------------------------------------


def inspection_key_points(route: RoadRoute) -> list[int]:
    """Indices into `route.points` of the planner's road waypoints: both
    ends, plus evenly spread points between - at most `MAX_PDDL_LEGS`
    segments. The planner reasons over these; the drone flies every point."""
    n = len(route.points)
    legs = min(MAX_PDDL_LEGS, n - 1)
    indices = sorted({round(k * (n - 1) / legs) for k in range(legs + 1)})
    return indices


def _arc_lengths(points: list[LatLon]) -> list[float]:
    out = [0.0]
    for a, b in zip(points, points[1:]):
        out.append(out[-1] + haversine_m(a, b))
    return out


_DROPPED_FACTS = re.compile(
    r"[ \t]*\((?:road-segment\s+\S+\s+\S+|=\s*\((?:distance|energy-required)\s+\S+\s+\S+\)\s*[-\d.eE]+)\)\n?"
)


def retarget_inspection_problem(problem_text: str, route: RoadRoute) -> str:
    """Put the template's source and destination on the road's two ends and
    replace its single road segment with the real road's segments (as
    `road1`, `road2`, ... waypoints between them), each with its
    along-the-road length and energy; `segments-total` says how many."""
    keys = inspection_key_points(route)
    cumulative = _arc_lengths(route.points)
    names = ["source", *[f"road{k}" for k in range(1, len(keys) - 1)], "destination"]

    result = _set_latlon(problem_text, "source", route.points[0])
    result = _set_latlon(result, "destination", route.points[-1])
    result = _DROPPED_FACTS.sub("", result)
    if len(names) > 2:
        result = _add_locations(result, names[1:-1])

    rate = _PDDL_MISSION_ENERGY / (2.0 * max(cumulative[-1], 1.0))
    facts: list[str] = []
    for name, index in zip(names[1:-1], keys[1:-1]):
        point = route.points[index]
        facts.append(f"\n        (= (latitude {name}) {point.lat:.7f})")
        facts.append(f"\n        (= (longitude {name}) {point.lon:.7f})")
    for (a, ia), (b, ib) in zip(zip(names, keys), zip(names[1:], keys[1:])):
        leg_m = cumulative[ib] - cumulative[ia]
        facts.append(f"\n        (road-segment {a} {b})")
        for x, y in ((a, b), (b, a)):
            facts.append(f"\n        (= (distance {x} {y}) {leg_m:.2f})")
            facts.append(f"\n        (= (energy-required {x} {y}) {leg_m * rate:.3f})")
    result = _add_init_facts(result, "".join(facts))
    result = re.sub(
        r"(\(=\s*\(segments-total\)\s*)[-\d.eE]+(\s*\))",
        lambda m: f"{m.group(1)}{len(keys) - 1}{m.group(2)}",
        result,
        count=1,
    )
    return result


def inspection_road_names(route: RoadRoute) -> list[str]:
    """The planner's road waypoints in flying order, for checking the plan."""
    keys = inspection_key_points(route)
    return ["source", *[f"road{k}" for k in range(1, len(keys) - 1)], "destination"]


def cross_track_m(position: LatLon, polyline: list[LatLon]) -> float:
    """How far `position` is from the nearest point of `polyline` (metres,
    locally flat) - used to check the drone really followed the road."""
    best = math.inf
    lat0 = math.radians(position.lat)
    kx = 111_320.0 * math.cos(lat0)
    ky = 111_320.0

    def xy(p):
        return ((p.lon - position.lon) * kx, (p.lat - position.lat) * ky)

    for a, b in zip(polyline, polyline[1:]):
        (ax, ay), (bx, by) = xy(a), xy(b)
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        t = 0.0 if length_sq == 0 else max(0.0, min(1.0, -(ax * dx + ay * dy) / length_sq))
        best = min(best, math.hypot(ax + t * dx, ay + t * dy))
    return best
