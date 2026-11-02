"""Battery-aware mission: source -> destination for ONE drone, charging on
the way only as much as it has to.

    picked source + destination + CHARGING_STATION_COUNT charging stations
        -> best_station_route: is there ANY chain of hops through them the
           drone can fly? (if not, why - before planning)
        -> retarget_battery_problem: every hop between source, stations and
           destination, with its distance and the battery it would use
        -> ENHSP (optimal, metric = metres flown + 1 per landing) picks the
           shortest chain:
             - the current charge reaches the destination -> straight there,
               no station used
             - otherwise take-off, fly-hop, land-at-station, charge-battery,
               take-off, ... land-at-destination - and from the first station
               the destination can be reached from, straight to it
        -> the drone flies each hop as its own mission: it lands at every
           station on the route and takes off again once charged

CHARGING: never to full. At a station the battery is charged to what the
NEXT hop needs plus CHARGE_MARGIN_PCT (`charge_target_pct`) - if the next
hop is the destination, just enough for the destination. So every hop lands
with about CHARGE_MARGIN_PCT left.

Battery use is modelled exactly the way the simulated autopilot draws it
(`engine.flight_controller`: a constant NOMINAL_DRAW_A while armed): climb
to the hop's cruise altitude, fly the hop, come down, plus
TRANSITION_OVERHEAD_S for the moments armed between those phases (arming,
the switch from climb to cruise, touchdown to disarm).

Landing with ~10% is below the app's ordinary low-battery prompt
(`engine.drone_thread.LOW_BATTERY_PCT`); a planned charging hop is exempt
from it (`services.thread_backend.ThreadSwarmBackend._check_exceptions`)
unless the battery gets down to the critical level.
"""
from __future__ import annotations

import heapq
import re
from dataclasses import dataclass

from contracts.gui_orchestration import DroneConfig
from contracts.gui_orchestration import LatLon as GuiLatLon
from engine.flight_controller import CLIMB_RATE_MPS, DESCENT_RATE_MPS, NOMINAL_DRAW_A
from engine.pddl_problem import (
    LatLon,
    _add_init_facts,
    _add_locations,
    _set_latlon,
    haversine_m,
)
from engine.terrain import plan_terrain_profile

CHARGING_STATION_COUNT = 5
CHARGE_MARGIN_PCT = 10.0      # charge this much more than the next hop needs
MAX_BATTERY_PCT = 100.0
TRANSITION_OVERHEAD_S = 10.0  # armed but not climbing, cruising or descending, per hop
# Simulated seconds a full 0 -> 100% charge takes at a station (a fast
# charger, ~3C); a partial charge takes its share of that. At the
# simulator's default 20x, a full charge is a minute on screen.
FULL_CHARGE_S = 1200.0


class BatteryRouteError(ValueError):
    """No route the drone can fly - the message says why, for the operator."""


def station_names(count: int) -> list[str]:
    return [f"station{i}" for i in range(1, count + 1)]


@dataclass(frozen=True)
class Hop:
    """One takeoff-to-landing flight between two places."""

    distance_m: float
    energy_pct: float      # battery it uses
    flight_s: float        # takeoff to touchdown, simulated seconds


def hop_estimate(drone: DroneConfig, a: LatLon, b: LatLon) -> Hop:
    """What flying straight from `a` to `b` (taking off and landing) costs
    `drone` - on the same terrain-derived cruise altitude and the same
    current draw the drone threads' autopilot will use."""
    points, altitudes = plan_terrain_profile([GuiLatLon(lat=a.lat, lon=a.lon), GuiLatLon(lat=b.lat, lon=b.lon)])
    distance = sum(
        haversine_m(LatLon(p.lat, p.lon), LatLon(q.lat, q.lon)) for p, q in zip(points, points[1:])
    )
    cruise_alt = max(altitudes) if altitudes else drone.cruise_altitude_m
    flight_s = (
        cruise_alt / CLIMB_RATE_MPS
        + distance / max(0.1, drone.max_velocity_mps)
        + cruise_alt / DESCENT_RATE_MPS
        + TRANSITION_OVERHEAD_S
    )
    used_mah = NOMINAL_DRAW_A * (flight_s / 3600.0) * 1000.0
    energy = 100.0 * used_mah / max(1.0, drone.battery_capacity_mah)
    return Hop(distance_m=distance, energy_pct=energy, flight_s=flight_s)


def charge_target_pct(hop: Hop) -> float:
    """What to charge to before flying `hop`: its need plus the margin."""
    return hop.energy_pct + CHARGE_MARGIN_PCT


def flyable(hop: Hop, battery_pct: float = MAX_BATTERY_PCT) -> bool:
    """Whether `hop` can be flown on `battery_pct`, landing with the margin left."""
    return charge_target_pct(hop) <= battery_pct


def max_hop_m(drone: DroneConfig) -> float:
    """Longest single hop `drone` can fly on a full charge, keeping the margin."""
    hop_1km = hop_estimate(drone, LatLon(0.0, 0.0), LatLon(0.0, 1000.0 / 111_320.0))
    zero = hop_estimate(drone, LatLon(0.0, 0.0), LatLon(0.0, 0.0))
    per_m = max(1e-9, (hop_1km.energy_pct - zero.energy_pct) / hop_1km.distance_m)
    return max(0.0, (MAX_BATTERY_PCT - CHARGE_MARGIN_PCT - zero.energy_pct) / per_m)


def direct_hop_check(
    drone: DroneConfig, source: LatLon, destination: LatLon, battery_pct: float = MAX_BATTERY_PCT
) -> tuple[bool, Hop]:
    """Can `drone` fly straight there on the charge it has now?"""
    hop = hop_estimate(drone, source, destination)
    return flyable(hop, battery_pct), hop


def charge_targets(drone: DroneConfig, stops: list[LatLon]) -> list[float]:
    """For a route `stops` (source, stations..., destination): the battery
    to charge to at each station - what the hop leaving it needs, plus the
    margin. One per station, in order."""
    return [
        min(MAX_BATTERY_PCT, charge_target_pct(hop_estimate(drone, a, b)))
        for a, b in zip(stops[1:-1], stops[2:])
    ]


def best_station_route(
    drone: DroneConfig,
    source: LatLon,
    destination: LatLon,
    stations: list[LatLon],
    battery_pct: float = MAX_BATTERY_PCT,
) -> list[str]:
    """The shortest chain of hops (metres flown, +1 per landing) from the
    source to the destination - direct if the current charge reaches it -
    each hop flyable: the first on `battery_pct`, the rest on at most a full
    charge. The same answer ENHSP's optimal search must find, used to check
    it. Raises `BatteryRouteError` with the reason when there is none."""
    names = ["source", *station_names(len(stations)), "destination"]
    points = dict(zip(names, [source, *stations, destination]))
    best = {name: float("inf") for name in names}
    previous: dict[str, str] = {}
    best["source"] = 0.0
    queue = [(0.0, "source")]
    while queue:
        cost, here = heapq.heappop(queue)
        if cost > best[here] or here == "destination":
            continue
        for there in names:
            if there in (here, "source"):
                continue
            hop = hop_estimate(drone, points[here], points[there])
            if not flyable(hop, battery_pct if here == "source" else MAX_BATTERY_PCT):
                continue
            step = hop.distance_m + (1.0 if there != "destination" else 0.0)
            if cost + step < best[there]:
                best[there] = cost + step
                previous[there] = here
                heapq.heappush(queue, (best[there], there))
    if best["destination"] == float("inf"):
        reach_km = max_hop_m(drone) / 1000.0
        reachable = [n for n in names[1:-1] if best[n] < float("inf")]
        detail = (
            "none of the charging stations is within reach of the source"
            if not reachable
            else "the drone can reach " + ", ".join(reachable)
            + " but no station within reach is close enough to the destination"
        )
        raise BatteryRouteError(
            f"{drone.name} can't reach the destination, even charging on the way: "
            f"{detail}.\n\nOn a full charge it flies at most about {reach_km:.1f} km per hop "
            f"(landing with {CHARGE_MARGIN_PCT:.0f}% battery left). Place the charging stations "
            f"so that no gap along the way - source to station, station to station, station to "
            f"destination - is longer than that."
        )
    route = ["destination"]
    while route[-1] != "source":
        route.append(previous[route[-1]])
    return list(reversed(route))


# ---- PDDL ---------------------------------------------------------------------


_DROPPED_FACTS = re.compile(
    r"[ \t]*\((?:hop\s+\S+\s+\S+|=\s*\((?:distance|hop-energy)\s+\S+\s+\S+\)\s*[-\d.eE]+)\)\n?"
)


def retarget_battery_problem(
    problem_text: str,
    drone: DroneConfig,
    source: LatLon,
    destination: LatLon,
    stations: list[LatLon],
    start_battery_pct: float = MAX_BATTERY_PCT,
) -> str:
    """Put the template on the picked source and destination, add the
    charging stations (`station1` ... `stationN`), and give every directed
    hop between them its distance and battery use for `drone`. Every hop is
    listed - which of them the battery allows is the planner's call."""
    names = ["source", *station_names(len(stations)), "destination"]
    points = dict(zip(names, [source, *stations, destination]))

    result = _set_latlon(problem_text, "source", source)
    result = _set_latlon(result, "destination", destination)
    result = _DROPPED_FACTS.sub("", result)
    if stations:
        result = _add_locations(result, names[1:-1])

    facts: list[str] = []
    for name in names[1:-1]:
        point = points[name]
        facts.append(f"\n        (charging-station {name})")
        facts.append(f"\n        (= (latitude {name}) {point.lat:.7f})")
        facts.append(f"\n        (= (longitude {name}) {point.lon:.7f})")
    for a in names:
        for b in names:
            if a == b or b == "source" or a == "destination":
                continue  # nobody flies back to the source or on from the destination
            hop = hop_estimate(drone, points[a], points[b])
            facts.append(f"\n        (hop {a} {b})")
            facts.append(f"\n        (= (distance {a} {b}) {hop.distance_m:.2f})")
            facts.append(f"\n        (= (hop-energy {a} {b}) {hop.energy_pct:.3f})")
    result = _add_init_facts(result, "".join(facts))
    for fluent, value in (("battery", start_battery_pct), ("charge-margin", CHARGE_MARGIN_PCT)):
        result = re.sub(
            rf"(\(=\s*\({fluent}\s+drone1\)\s*)[-\d.eE]+(\s*\))",
            lambda m, v=value: f"{m.group(1)}{v:g}{m.group(2)}",
            result,
            count=1,
        )
    return result


def plan_stops(steps) -> list[str]:
    """The places ENHSP's plan visits in order (source first), from its
    `fly-hop` actions."""
    ordered = sorted(steps, key=lambda s: s.index)
    hops = [s.args[1:3] for s in ordered if s.action == "fly-hop" and len(s.args) >= 3]
    if not hops:
        return []
    return [hops[0][0], *(to for _from, to in hops)]


def check_plan_charges(steps, stops: list[str]) -> None:
    """Every station the plan flies to must be landed at, charged at for
    the hop that actually leaves it, and taken off from. Raises ValueError
    if not."""
    ordered = sorted(steps, key=lambda s: s.index)
    actions = [(s.action, *s.args[1:]) for s in ordered]
    for station, next_stop in zip(stops[1:-1], stops[2:]):
        try:
            landed = actions.index(("land-at-station", station))
            charged = actions.index(("charge-battery", station, next_stop), landed)
            actions.index(("take-off", station), charged)
        except ValueError as exc:
            raise ValueError(
                f"ENHSP's plan flies to {station} but does not land, charge for the hop to "
                f"{next_stop} and take off there"
            ) from exc
