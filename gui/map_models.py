"""Qt list models exposing drone telemetry and map annotations to Map.qml
(SRS §3.2.2, §3.2.3)."""
from __future__ import annotations

import math
import time
from typing import Optional

from PySide6.QtCore import QAbstractListModel, QModelIndex, QTimer, Qt

from contracts.gui_orchestration import DroneStatus, DroneTelemetry

_METRES_PER_DEG_LAT = 111_320.0
_GLIDE_TICK_MS = 33  # ~30 Hz, smooth without being wasteful
_FALLBACK_GLIDE_MPS = 8.0  # used only before a drone's first real hop is observed
_FALLBACK_GLIDE_VPS = 3.0  # vertical equivalent, before a real climb/descent is observed


class DroneMarkerModel(QAbstractListModel):
    """Live drone positions rendered as moving map markers.

    Visualization only: the marker's on-screen position (and altitude, via
    `displayed_position`) is smoothed towards each new telemetry fix at a
    constant, adjustable crawl speed (see `set_glide_speed`) rather than
    snapping straight to it. Real telemetry - heading/status/faults, and the
    true fix for anything else that reads drone positions (`PathModel`'s
    trail, battery logic, etc.) - is untouched; only what this particular
    model hands out (to the map's marker delegate, and to `displayed_position`
    for the telemetry dashboard to match) is smoothed.
    """

    SYSID = Qt.UserRole + 1
    LAT = Qt.UserRole + 2
    LON = Qt.UserRole + 3
    HEADING = Qt.UserRole + 4
    STATUS = Qt.UserRole + 5
    HAS_FAULT = Qt.UserRole + 6

    _ROLE_NAMES = {
        SYSID: b"sysid",
        LAT: b"lat",
        LON: b"lon",
        HEADING: b"heading",
        STATUS: b"status",
        HAS_FAULT: b"hasFault",
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self._order: list[int] = []
        self._latest: dict[int, DroneTelemetry] = {}       # true telemetry, for non-position roles
        self._displayed: dict[int, tuple[float, float]] = {}  # smoothed (lat, lon) shown on the map
        self._displayed_alt: dict[int, float] = {}  # smoothed altitude, same crawl as lat/lon
        self._prior_fix: dict[int, tuple[tuple[float, float, float], float]] = {}  # last raw (lat, lon, alt) fix + when
        self._observed_mps: dict[int, float] = {}  # apparent real speed implied by the raw telemetry stream
        self._observed_vps: dict[int, float] = {}  # apparent real climb/descent rate, same idea vertically
        self._prev_status: dict[int, DroneStatus] = {}
        self._glide_speed = 1.0
        self._last_tick_s: float | None = None

        self._glide_timer = QTimer(self)
        self._glide_timer.setInterval(_GLIDE_TICK_MS)
        self._glide_timer.timeout.connect(self._advance_glide)
        self._glide_timer.start()

    def roleNames(self):
        return dict(self._ROLE_NAMES)

    def rowCount(self, parent=QModelIndex()) -> int:
        return len(self._order)

    def data(self, index, role):
        if not index.isValid() or not (0 <= index.row() < len(self._order)):
            return None
        sysid = self._order[index.row()]
        drone = self._latest[sysid]
        if role == self.SYSID:
            return sysid
        if role == self.LAT:
            return self._displayed[sysid][0]
        if role == self.LON:
            return self._displayed[sysid][1]
        if role == self.HEADING:
            return drone.heading_deg
        if role == self.STATUS:
            return drone.status.value
        if role == self.HAS_FAULT:
            return bool(drone.active_faults)
        return None

    def set_glide_speed(self, speed: float) -> None:
        """Visualization only: multiplier on the crawl speed a marker eases
        towards its latest fix at - 1.0x reproduces the apparent speed the
        raw telemetry stream itself implies (so the default looks the same
        as before), below 1x it visibly lags/crawls, above 1x it catches up
        and overtakes. Has no effect on the mission's actual timing - only
        on how this model smooths what it reports to the map."""
        self._glide_speed = max(0.01, speed)

    def clear(self) -> None:
        """Drop every tracked drone and all its glide state. Called on
        Stop/Reset and at the start of each new run so a stale icon can't
        linger on the map, and so the next mission's first fix is treated as
        a first sighting (placed straight at the new source) instead of the
        marker gliding in from wherever the previous run left it - the
        external-MAVLink path never reports TAKING_OFF, so `_is_fresh_launch`
        alone can't catch that case."""
        if self._order:
            self.beginRemoveRows(QModelIndex(), 0, len(self._order) - 1)
            self._order.clear()
            self._latest.clear()
            self._displayed.clear()
            self._displayed_alt.clear()
            self._prior_fix.clear()
            self._observed_mps.clear()
            self._observed_vps.clear()
            self._prev_status.clear()
            self.endRemoveRows()
        self._last_tick_s = None

    def displayed_position(self, sysid: int) -> Optional[tuple[float, float, float]]:
        """The (lat, lon, altitude_m) this model is currently showing for
        `sysid` - the same smoothed position/altitude the map marker itself
        is drawn at right now, not the latest raw fix. For anything else on
        screen that should visibly track the Speed slider along with the
        marker (the telemetry dashboard's table and inspector) instead of
        jumping straight to each new telemetry batch regardless of it.
        `None` if this sysid isn't currently tracked."""
        if sysid not in self._displayed:
            return None
        lat, lon = self._displayed[sysid]
        alt = self._displayed_alt.get(sysid, self._latest[sysid].altitude_m)
        return (lat, lon, alt)

    def set_drones(self, drones: list[DroneTelemetry]) -> None:
        """Update in place rather than reset - a reset tells `MapItemView` the
        whole model changed, so it destroys and recreates every delegate on
        every batch, which is wasteful and would fight any future per-item
        QML state. Keeping existing rows' indices stable (matched by sysid)
        keeps each drone's delegate alive across updates."""
        by_sysid = {d.sysid: d for d in drones}
        now = time.monotonic()

        for row in reversed(range(len(self._order))):
            sysid = self._order[row]
            if sysid not in by_sysid:
                self.beginRemoveRows(QModelIndex(), row, row)
                del self._order[row]
                del self._latest[sysid]
                del self._displayed[sysid]
                self._displayed_alt.pop(sysid, None)
                self._prior_fix.pop(sysid, None)
                self._observed_mps.pop(sysid, None)
                self._observed_vps.pop(sysid, None)
                self._prev_status.pop(sysid, None)
                self.endRemoveRows()

        for sysid in self._order:
            new_drone = by_sysid[sysid]
            if self._is_fresh_launch(sysid, new_drone):
                # A brand-new mission always reports TAKING_OFF the moment it
                # starts, at the new source - that's a reset, not a hop the
                # drone flew. Snap straight there instead of gliding in from
                # wherever the previous mission's flight left off, and drop
                # the old speed baseline so it doesn't leak into the new one.
                self._displayed[sysid] = (new_drone.lat, new_drone.lon)
                self._displayed_alt[sysid] = new_drone.altitude_m
                self._observed_mps.pop(sysid, None)
                self._observed_vps.pop(sysid, None)
                self._prior_fix[sysid] = ((new_drone.lat, new_drone.lon, new_drone.altitude_m), now)
            else:
                self._track_observed_speed(sysid, new_drone, now)
            self._prev_status[sysid] = new_drone.status
            self._latest[sysid] = new_drone
        if self._order:
            self.dataChanged.emit(
                self.index(0),
                self.index(len(self._order) - 1),
                [self.HEADING, self.STATUS, self.HAS_FAULT],
            )

        new_ones = [d for d in drones if d.sysid not in self._latest]
        if new_ones:
            start = len(self._order)
            self.beginInsertRows(QModelIndex(), start, start + len(new_ones) - 1)
            for drone in new_ones:
                self._order.append(drone.sysid)
                self._latest[drone.sysid] = drone
                self._prior_fix[drone.sysid] = ((drone.lat, drone.lon, drone.altitude_m), now)
                self._prev_status[drone.sysid] = drone.status
                # First sighting of this drone: show it where it really is,
                # not gliding in from nowhere.
                self._displayed[drone.sysid] = (drone.lat, drone.lon)
                self._displayed_alt[drone.sysid] = drone.altitude_m
            self.endInsertRows()

    def _is_fresh_launch(self, sysid: int, drone: DroneTelemetry) -> bool:
        """True exactly on the tick a drone's status first becomes
        TAKING_OFF - every fresh mission starts there (see `_Flight` in
        services/local_flight.py and the MissionState mapping in
        services/thread_backend.py), so it marks the drone's position having
        just been reset to a new launch point rather than having flown there
        from wherever the previous mission ended."""
        return drone.status == DroneStatus.TAKING_OFF and self._prev_status.get(sysid) != DroneStatus.TAKING_OFF

    def _track_observed_speed(self, sysid: int, drone: DroneTelemetry, now: float) -> None:
        """Estimate the apparent real speed (m/s horizontally, m/s vertically)
        the raw telemetry stream itself implies for this drone, from the gap
        between its last two reported fixes. Those figures are the crawl
        speeds at Speed slider 1.0x, so the default glide matches whatever
        pace the active flight source (local preview, drone threads, or
        eventually Renode) is already reporting, regardless of that source's
        own internal time-scaling."""
        prior = self._prior_fix.get(sysid)
        self._prior_fix[sysid] = ((drone.lat, drone.lon, drone.altitude_m), now)
        if prior is None:
            return
        (prev_lat, prev_lon, prev_alt), prev_t = prior
        dt_hop = now - prev_t
        if dt_hop <= 0.02:
            return
        dist_m = _distance_m((prev_lat, prev_lon), (drone.lat, drone.lon))
        observed = dist_m / dt_hop
        if observed > 0.05:  # ignore near-stationary noise so it doesn't zero out the baseline
            self._observed_mps[sysid] = observed
        observed_v = abs(drone.altitude_m - prev_alt) / dt_hop
        if observed_v > 0.05:
            self._observed_vps[sysid] = observed_v

    def _advance_glide(self) -> None:
        now = time.monotonic()
        dt = 0.0 if self._last_tick_s is None else max(0.0, now - self._last_tick_s)
        self._last_tick_s = now
        if dt <= 0.0 or not self._order:
            return

        changed_rows: list[int] = []
        for row, sysid in enumerate(self._order):
            drone = self._latest[sysid]
            target = (drone.lat, drone.lon)
            shown = self._displayed[sysid]
            target_alt = drone.altitude_m
            shown_alt = self._displayed_alt.get(sysid, target_alt)
            moved = False

            if shown != target:
                dist_m = _distance_m(shown, target)
                base_mps = self._observed_mps.get(sysid, _FALLBACK_GLIDE_MPS)
                step_m = base_mps * self._glide_speed * dt

                if dist_m <= step_m or dist_m < 0.1:
                    self._displayed[sysid] = target
                else:
                    frac = step_m / dist_m
                    dlat = target[0] - shown[0]
                    dlon = target[1] - shown[1]
                    self._displayed[sysid] = (shown[0] + dlat * frac, shown[1] + dlon * frac)
                moved = True

            if shown_alt != target_alt:
                # Same crawl treatment vertically, so the altitude the
                # dashboard reads via `displayed_position` eases towards each
                # new fix in step with the marker's lat/lon glide, instead of
                # jumping straight to it while the marker is still crawling.
                base_vps = self._observed_vps.get(sysid, _FALLBACK_GLIDE_VPS)
                step_alt = base_vps * self._glide_speed * dt
                dalt = target_alt - shown_alt
                if abs(dalt) <= step_alt or abs(dalt) < 0.05:
                    self._displayed_alt[sysid] = target_alt
                else:
                    self._displayed_alt[sysid] = shown_alt + math.copysign(step_alt, dalt)
                moved = True

            if moved:
                changed_rows.append(row)

        if changed_rows:
            self.dataChanged.emit(
                self.index(min(changed_rows)),
                self.index(max(changed_rows)),
                [self.LAT, self.LON],
            )


def _distance_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Flat-earth approximation, plenty accurate at the short ranges a
    single glide step covers."""
    dlat_m = (b[0] - a[0]) * _METRES_PER_DEG_LAT
    lon_scale = _METRES_PER_DEG_LAT * max(0.01, math.cos(math.radians(a[0])))
    dlon_m = (b[1] - a[1]) * lon_scale
    return math.hypot(dlat_m, dlon_m)


class PointMarkerModel(QAbstractListModel):
    """Click-to-set start (green) / destination (red) markers."""

    ROLE_KIND = Qt.UserRole + 1
    LAT = Qt.UserRole + 2
    LON = Qt.UserRole + 3

    _ROLE_NAMES = {ROLE_KIND: b"role", LAT: b"lat", LON: b"lon"}

    def __init__(self, parent=None):
        super().__init__(parent)
        self._points: list[tuple[str, float, float]] = []

    def roleNames(self):
        return dict(self._ROLE_NAMES)

    def rowCount(self, parent=QModelIndex()) -> int:
        return len(self._points)

    def data(self, index, role):
        if not index.isValid() or not (0 <= index.row() < len(self._points)):
            return None
        kind, lat, lon = self._points[index.row()]
        if role == self.ROLE_KIND:
            return kind
        if role == self.LAT:
            return lat
        if role == self.LON:
            return lon
        return None

    def set_point(self, kind: str, lat: float, lon: float) -> None:
        self.beginResetModel()
        self._points = [p for p in self._points if p[0] != kind] + [(kind, lat, lon)]
        self.endResetModel()

    def clear(self) -> None:
        self.beginResetModel()
        self._points = []
        self.endResetModel()


class RestrictedAreaModel(QAbstractListModel):
    """The polygon marking the restricted area for the plan currently being
    set up. Deliberately independent of `PathModel` (which live flight-path
    trails also use) so this shape survives `clear_paths()` when a mission
    starts, and only changes when a new plan is armed."""

    POINTS = Qt.UserRole + 1
    _ROLE_NAMES = {POINTS: b"points"}

    def __init__(self, parent=None):
        super().__init__(parent)
        self._points: list[tuple[float, float]] = []

    def roleNames(self):
        return dict(self._ROLE_NAMES)

    def rowCount(self, parent=QModelIndex()) -> int:
        return 1 if self._points else 0

    def data(self, index, role):
        if not index.isValid() or role != self.POINTS or not (0 <= index.row() < 1):
            return None
        return [{"latitude": lat, "longitude": lon} for lat, lon in self._points]

    def set_polygon(self, points: list[tuple[float, float]]) -> None:
        """`points` in click order. Closed automatically for rendering once
        there are at least 3 of them."""
        self.beginResetModel()
        self._points = list(points)
        if len(self._points) >= 3:
            self._points.append(self._points[0])
        self.endResetModel()

    def clear(self) -> None:
        self.set_polygon([])


class PathModel(QAbstractListModel):
    """Per-drone flight-path vector overlays (a trailing polyline of recent positions)."""

    SYSID = Qt.UserRole + 1
    POINTS = Qt.UserRole + 2

    _ROLE_NAMES = {SYSID: b"sysid", POINTS: b"points"}

    def __init__(self, parent=None, max_points: int = 200):
        super().__init__(parent)
        self._paths: dict[int, list[tuple[float, float]]] = {}
        self._order: list[int] = []
        self._max_points = max_points

    def roleNames(self):
        return dict(self._ROLE_NAMES)

    def rowCount(self, parent=QModelIndex()) -> int:
        return len(self._order)

    def data(self, index, role):
        if not index.isValid() or not (0 <= index.row() < len(self._order)):
            return None
        sysid = self._order[index.row()]
        if role == self.SYSID:
            return sysid
        if role == self.POINTS:
            return [{"latitude": lat, "longitude": lon} for lat, lon in self._paths[sysid]]
        return None

    def append_point(self, sysid: int, lat: float, lon: float) -> None:
        self.beginResetModel()
        if sysid not in self._paths:
            self._paths[sysid] = []
            self._order.append(sysid)
        points = self._paths[sysid]
        points.append((lat, lon))
        if len(points) > self._max_points:
            del points[: len(points) - self._max_points]
        self.endResetModel()

    def clear(self) -> None:
        self.beginResetModel()
        self._paths = {}
        self._order = []
        self.endResetModel()
