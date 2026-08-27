"""Synthetic terrain, for dynamic altitude planning.

No real elevation data is bundled or fetched - `elevation_m` is a
deterministic, purely-computed pseudo-terrain (a handful of smooth
low-frequency waves, summed in local metres around each point), so the same
(lat, lon) always yields the same height with no network access or external
dataset required. It is not real-world topography; it exists to give the
climb-over/route-around logic below actual hills to react to.

`plan_terrain_profile` is the entry point: given an already laterally-planned
route (e.g. the plain source->destination leg, or one already carrying
restricted-area detour points from `engine.pddl_problem`), it assigns each
leg a cruise altitude that clears its highest sampled point by
AGL_CLEARANCE_M ("go above that hill") - and where a leg's peak would need
climbing past MAX_CLIMB_AGL_M to do that, it instead searches for a nearby
lateral bypass point that both legs either side of it can clear within the
ceiling ("turn around the hill"), inserting it into the route.
"""
from __future__ import annotations

import math

from contracts.gui_orchestration import LatLon

METRES_PER_DEG_LAT = 111_320.0

AGL_CLEARANCE_M = 50.0    # how far above the ground the drone stays, per the ask
MAX_CLIMB_AGL_M = 120.0   # ceiling a leg can climb to before "go around" kicks in instead

# Fixed, arbitrary constants - not tuned to any real place. Chosen so a
# "hill" spans roughly a few hundred metres (comparable to this simulator's
# mission scale), not a planetary-scale feature.
_TERRAIN_BASELINE_M = 40.0
_TERRAIN_WAVES = [
    # (amplitude_m, wavelength_m, phase_x, phase_y)
    (70.0, 900.0, 0.7, 2.1),
    (40.0, 420.0, 3.4, 0.5),
    (22.0, 180.0, 1.1, 4.8),
]

_BYPASS_SEARCH_OFFSETS_M = (50.0, 100.0, 150.0, 220.0, 300.0, 400.0, 550.0, 750.0)
_PROFILE_SAMPLES_PER_LEG = 12


def _metres_per_deg_lon(lat_deg: float) -> float:
    return METRES_PER_DEG_LAT * math.cos(math.radians(lat_deg))


def _to_local(point: LatLon, origin: LatLon) -> tuple[float, float]:
    return (
        (point.lon - origin.lon) * _metres_per_deg_lon(origin.lat),
        (point.lat - origin.lat) * METRES_PER_DEG_LAT,
    )


def _from_local(offset: tuple[float, float], origin: LatLon) -> LatLon:
    x, y = offset
    return LatLon(
        lat=origin.lat + y / METRES_PER_DEG_LAT,
        lon=origin.lon + x / _metres_per_deg_lon(origin.lat),
    )


def elevation_m(lat: float, lon: float) -> float:
    """Deterministic pseudo-terrain height (metres), never negative.

    Worked in local metres around the origin `(0, 0)` (equator/prime
    meridian) purely so the waves' wavelengths mean an actual distance in
    metres rather than degrees, which would make a "hill" thousands of
    kilometres wide - not a real place, so there's no meaningful origin to
    prefer over any other.
    """
    x_m = lon * _metres_per_deg_lon(lat)
    y_m = lat * METRES_PER_DEG_LAT
    height = _TERRAIN_BASELINE_M
    for amplitude, wavelength_m, phase_x, phase_y in _TERRAIN_WAVES:
        height += amplitude * math.sin(2 * math.pi * x_m / wavelength_m + phase_x) \
                             * math.sin(2 * math.pi * y_m / wavelength_m + phase_y)
    return max(0.0, height)


def _peak_elevation_along(a: LatLon, b: LatLon, samples: int = _PROFILE_SAMPLES_PER_LEG) -> float:
    """Highest sampled terrain point on the straight leg `a`->`b`."""
    return max(
        elevation_m(a.lat + (b.lat - a.lat) * t, a.lon + (b.lon - a.lon) * t)
        for t in (i / samples for i in range(samples + 1))
    )


def _clears_ceiling(a: LatLon, b: LatLon) -> bool:
    return _peak_elevation_along(a, b) + AGL_CLEARANCE_M <= MAX_CLIMB_AGL_M


def _find_bypass(a: LatLon, b: LatLon) -> LatLon | None:
    """A point near the midpoint of `a`->`b`, offset perpendicular to it,
    from which both the approach (`a`->via) and departure (via->`b`) legs
    clear MAX_CLIMB_AGL_M - the "turn around the hill" fallback for a leg
    whose peak is too tall to simply climb over. Tries growing offsets on
    both sides and returns the first that works; `None` if nothing within
    the search radius does (the caller then falls back to climbing to the
    ceiling and flying direct anyway, rather than refusing the mission)."""
    ax, ay = 0.0, 0.0
    bx, by = _to_local(b, a)
    length = math.hypot(bx - ax, by - ay)
    if length < 1.0:
        return None
    ux, uy = (bx - ax) / length, (by - ay) / length
    perp = (uy, -ux)
    mid = ((ax + bx) / 2.0, (ay + by) / 2.0)

    for offset_m in _BYPASS_SEARCH_OFFSETS_M:
        for side in (1.0, -1.0):
            candidate = _from_local(
                (mid[0] + perp[0] * offset_m * side, mid[1] + perp[1] * offset_m * side), a
            )
            if _clears_ceiling(a, candidate) and _clears_ceiling(candidate, b):
                return candidate
    return None


FLAT_CRUISE_ALT_M = 50.0  # temporary fixed altitude, see DYNAMIC_ALTITUDE_ENABLED below

# Dynamic climb-over/route-around terrain planning is parked for now at the
# user's request ("dont use dynamic altitude just use 50m altitude for all
# the terrains, next i will give you update on this") - flip this back to
# True to re-enable the elevation_m/_find_bypass logic below with no other
# changes needed.
DYNAMIC_ALTITUDE_ENABLED = False


def plan_terrain_profile(waypoints: list[LatLon]) -> tuple[list[LatLon], list[float]]:
    """Terrain-aware pass over an already laterally-planned `waypoints`.

    Returns `(points, altitudes)`, aligned by index - `points` is `waypoints`
    with a bypass point inserted before any leg that needed one to clear
    ("turn around the hill" - see `_find_bypass`), and `altitudes` is the
    *same* single cruise altitude repeated for every point: the highest
    clearance any leg of the (possibly-bypassed) route needs, capped at
    MAX_CLIMB_AGL_M.

    Deliberately one constant altitude for the whole trip, not a per-leg
    profile that eases up and down with the terrain under each individual
    leg: climbing once to whatever the route's tallest point requires, and
    holding it, means the drone only ever comes down once - the ordinary
    vertical "helicopter" landing once it actually arrives - rather than
    gliding down early just because the final leg's own terrain happens to
    be lower than an earlier one's.

    While `DYNAMIC_ALTITUDE_ENABLED` is False, none of that runs: every
    waypoint is handed back unchanged with a flat `FLAT_CRUISE_ALT_M`
    altitude, no elevation sampling or bypass insertion at all.
    """
    if not waypoints:
        return [], []

    if not DYNAMIC_ALTITUDE_ENABLED:
        return list(waypoints), [FLAT_CRUISE_ALT_M] * len(waypoints)

    points = [waypoints[0]]
    for i in range(1, len(waypoints)):
        a, b = waypoints[i - 1], waypoints[i]
        if _clears_ceiling(a, b):
            points.append(b)
            continue

        via = _find_bypass(a, b)
        if via is not None:
            points.append(via)
        # else: no clearance found nearby either - best effort, fly this
        # leg direct anyway (still climbs to the ceiling below) rather than
        # refuse the mission.
        points.append(b)

    cruise_alt = AGL_CLEARANCE_M + elevation_m(points[0].lat, points[0].lon)
    for i in range(1, len(points)):
        cruise_alt = max(cruise_alt, _peak_elevation_along(points[i - 1], points[i]) + AGL_CLEARANCE_M)
    cruise_alt = min(MAX_CLIMB_AGL_M, cruise_alt)

    return points, [cruise_alt] * len(points)
