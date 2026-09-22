"""Live per-drone stats for an area-coverage (forest-search) mission, printed
to the terminal `python main.py` was launched from.

This is the GUI-side echo of what `scripts/pddl_search_to_mavlink.py` prints
when it flies the same kind of mission from the command line: one line per
drone, every few seconds, in the identical

    [   t.ts] <name> wp=<cur>/<last> (lat,lon) alt=..m hdg=.. gs=..m/s batt=..%

shape. That script is left untouched - this reproduces its output format from
the telemetry batches the GUI already receives, so selecting two drones and a
search area in the app gives you the same running read-out you would get from
the CLI tool, without leaving the GUI.

Framework-free on purpose (no Qt, no pymavlink): `MainWindow` owns one
instance, calls `start()` once an area-coverage mission actually launches,
feeds every telemetry batch through `update()`, and calls `stop()` on Stop.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

# How close (metres) a drone must get to its next route point before the
# printed `wp=` cursor ticks over to the following one. Matches the spirit of
# engine.flight_controller.WAYPOINT_RADIUS_M, loosened a little because GUI
# telemetry is sampled, not continuous.
_WAYPOINT_RADIUS_M = 20.0

_EARTH_RADIUS_M = 6_371_000.0


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dlat = p2 - p1
    dlon = math.radians(lon2 - lon1)
    h = math.sin(dlat / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlon / 2) ** 2
    return 2 * _EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


@dataclass
class _Track:
    """One drone's progress along its assigned lane route."""

    name: str
    sysid: int
    lane: int
    route: list[tuple[float, float]]  # [(lat, lon), ...] incl. base at 0 and end
    cursor: int = 1                   # index 0 is the base/takeoff point
    done: bool = False

    @property
    def last(self) -> int:
        return max(1, len(self.route) - 1)

    def advance(self, lat: float, lon: float) -> None:
        """Tick the waypoint cursor forward while the drone is within
        `_WAYPOINT_RADIUS_M` of the point it is currently heading for (can
        skip several at once if telemetry jumped)."""
        while self.cursor < self.last:
            tgt_lat, tgt_lon = self.route[self.cursor]
            if _haversine_m(lat, lon, tgt_lat, tgt_lon) <= _WAYPOINT_RADIUS_M:
                self.cursor += 1
            else:
                break


class SearchStatsConsole:
    """Prints interleaved per-drone stats for a running search mission.

    Usage from the GUI:
        console = SearchStatsConsole()
        console.start("Search", [(cfg1, route1), (cfg2, route2)])   # on launch
        console.update(batch)                                        # per telemetry batch
        console.stop()                                               # on Stop / teardown
    """

    def __init__(self, interval_s: float = 5.0, sink=print) -> None:
        self._interval_s = max(0.5, interval_s)
        self._sink = sink
        self._active = False
        self._plan_name = ""
        self._t0 = 0.0
        self._next_report = 0.0
        self._tracks: dict[int, _Track] = {}
        self._all_done = False

    # ---- lifecycle ------------------------------------------------------

    def start(self, plan_name: str, assignments, *, route_noun: str = "lane") -> None:
        """Begin printing. `assignments` is the same `[(DroneConfig, route),
        ...]` list `MainWindow._start_area_coverage_mission` hands to the
        backend - `route` being that drone's `[LatLon, ...]` lane."""
        self._active = True
        self._all_done = False
        self._plan_name = plan_name
        self._t0 = time.monotonic()
        self._next_report = 0.0
        self._tracks = {}
        for lane_idx, (cfg, route) in enumerate(assignments, start=1):
            pts = [(p.lat, p.lon) for p in route]
            self._tracks[cfg.sysid] = _Track(
                name=cfg.name, sysid=cfg.sysid, lane=lane_idx, route=pts
            )

        rule = "-" * 78
        self._emit(rule)
        self._emit(
            f" Search '{plan_name}' - {len(self._tracks)} drone(s), "
            f"live stats every {self._interval_s:g}s"
        )
        for tr in self._tracks.values():
            self._emit(
                f"   {tr.name} (sys{tr.sysid}) -> {route_noun} {tr.lane}: "
                f"{tr.last} waypoint(s)"
            )
        self._emit(rule)

    def update(self, batch) -> None:
        """Feed one `SwarmTelemetryBatch`. Waypoint-cursor and completion
        tracking run on every batch; the stats block is printed at most once
        per `interval_s`."""
        if not self._active or not self._tracks:
            return

        by_sysid = {d.sysid: d for d in batch.drones}
        t = time.monotonic() - self._t0

        # Cheap per-batch bookkeeping - never throttled, so an end-of-mission
        # batch that lands between two report ticks still registers.
        for sysid, tr in self._tracks.items():
            d = by_sysid.get(sysid)
            if d is None:
                continue
            tr.advance(d.lat, d.lon)
            status = self._status_of(d)
            landed = status.upper() in ("LANDED", "LANDING")
            settled = d.ground_speed_mps < 0.5 and d.altitude_m < 2.0
            if not tr.done and tr.cursor >= tr.last and (landed or settled):
                tr.done = True
                self._emit(f"[{t:6.1f}s] {tr.name} lane complete - back at base.")

        if not self._all_done and all(tr.done for tr in self._tracks.values()):
            self._all_done = True
            self._emit(
                f"[{t:6.1f}s] all {len(self._tracks)} drones complete - "
                f"search '{self._plan_name}' done."
            )

        if t < self._next_report:
            return
        self._next_report = t + self._interval_s

        for sysid, tr in self._tracks.items():
            d = by_sysid.get(sysid)
            if d is None:
                self._emit(f"[{t:6.1f}s] {tr.name} (sys{sysid}) - no telemetry this tick")
                continue
            faults = getattr(d, "active_faults", None) or []
            fault_txt = f" faults={','.join(str(f) for f in faults)}" if faults else ""
            self._emit(
                f"[{t:6.1f}s] {tr.name} wp={tr.cursor}/{tr.last} "
                f"({d.lat:.6f},{d.lon:.6f}) alt={d.altitude_m:5.1f}m "
                f"hdg={d.heading_deg:5.1f} gs={d.ground_speed_mps:4.1f}m/s "
                f"batt={d.battery_pct:5.1f}% link={d.link_quality_pct:3.0f}% "
                f"{self._status_of(d)}{fault_txt}"
            )

    @staticmethod
    def _status_of(d) -> str:
        return getattr(getattr(d, "status", None), "value", "") or str(getattr(d, "status", ""))

    def stop(self) -> None:
        if not self._active:
            return
        if not self._all_done:
            t = time.monotonic() - self._t0
            self._emit(f"[{t:6.1f}s] search '{self._plan_name}' stopped.")
        self._active = False
        self._tracks = {}

    # ---- internals ----------------------------------------------------

    def _emit(self, line: str) -> None:
        try:
            self._sink(line)
        except Exception:  # noqa: BLE001 - a broken stdout must never crash the GUI
            pass
