"""Flight Log panel: a live table of every drone (one row per drone, updated
in place) and, one click away, the plain scrolling log of drone movement,
mission-planner events and the solved PDDL action sequence.

    [ Table | Logs (3) ]                                    [Clear]
    +------------------------------------------------------------+
    | Drone  State  Mission  Lat  Lon  Alt  Speed  Hdg  Bat  ... |   <- Table
    +------------------------------------------------------------+

The log keeps receiving exactly what it always has (events and one
telemetry line per drone per batch), whichever view is showing, so toggling
never loses anything. The table reads the same telemetry batches.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

from PySide6.QtCore import QAbstractTableModel, QModelIndex, Qt, QTimer
from PySide6.QtGui import QColor, QFont, QFontDatabase
from PySide6.QtWidgets import (
    QAbstractItemView,
    QButtonGroup,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QStackedLayout,
    QStackedWidget,
    QTableView,
    QVBoxLayout,
    QWidget,
)

from contracts.gui_orchestration import DroneTelemetry
from gui.theme import theme_manager

MAX_LOG_LINES = 5000
TABLE_REFRESH_MS = 200     # table cells follow telemetry at up to 5 Hz
STALE_AFTER_S = 3.0        # "Updated" turns warning-coloured after this long without data
ROW_HEIGHT_PX = 27

COLUMNS = ("Drone", "State", "Mission", "Lat", "Lon", "Alt (m)", "Speed (m/s)",
           "Hdg (°)", "Battery", "Link", "Updated")
(COL_DRONE, COL_STATE, COL_MISSION, COL_LAT, COL_LON, COL_ALT, COL_SPEED,
 COL_HDG, COL_BATTERY, COL_LINK, COL_UPDATED) = range(len(COLUMNS))
NUMERIC_COLUMNS = {COL_LAT, COL_LON, COL_ALT, COL_SPEED, COL_HDG, COL_BATTERY, COL_LINK, COL_UPDATED}


@dataclass
class _Row:
    sysid: int
    name: str = ""
    telemetry: DroneTelemetry | None = None
    mission: str = ""
    updated_at: float | None = None  # time.monotonic() of the last telemetry
    # Set once the drone's flight has ended (see DroneTableModel.set_mission).
    # The last telemetry can still say IN_FLIGHT: a flight ends on the disarm
    # heartbeat and its instance is stopped before another position report.
    final_state: str | None = None


def _final_state(mission: str) -> str | None:
    """The State a finished flight's outcome text implies. "Completed" means
    it disarmed on its LAND point (engine.mavlink_mission), so it has landed."""
    if mission == "Completed":
        return "LANDED"
    if mission == "Stopped":
        return "STOPPED"
    if mission.startswith(("Failed", "Not flown")):
        return "FAILSAFE"
    return None


def _state_colour(row: _Row, palette) -> str:
    t = row.telemetry
    if row.final_state is not None:
        return palette.critical if row.final_state == "FAILSAFE" else palette.text_secondary
    if t is None:
        return palette.text_secondary
    status = getattr(t.status, "value", t.status)
    if status == "FAILSAFE" or t.active_faults:
        return palette.critical
    if status == "IN_FLIGHT":
        return palette.nominal
    if status in ("TAKING_OFF", "LANDING", "ARMED"):
        return palette.accent
    return palette.text_secondary  # LANDED, STANDBY


class DroneTableModel(QAbstractTableModel):
    """One row per drone, keyed and sorted by SYSID. Rows are only ever
    inserted or updated in place - never cleared per batch - except by
    `reset()` when a new run starts."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._rows: list[_Row] = []
        self._names: dict[int, str] = {}
        self.run_active = False
        self._fixed = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        self._bold = QFont()
        self._bold.setBold(True)

    # ---- structure ----

    def rowCount(self, parent=QModelIndex()) -> int:  # noqa: N802 (Qt API)
        return 0 if parent.isValid() else len(self._rows)

    def columnCount(self, parent=QModelIndex()) -> int:  # noqa: N802
        return 0 if parent.isValid() else len(COLUMNS)

    def headerData(self, section, orientation, role=Qt.DisplayRole):  # noqa: N802
        if orientation == Qt.Horizontal:
            if role == Qt.DisplayRole:
                return COLUMNS[section]
            if role == Qt.TextAlignmentRole:
                return int((Qt.AlignRight if section in NUMERIC_COLUMNS else Qt.AlignLeft) | Qt.AlignVCenter)
        return None

    def flags(self, index):
        return Qt.ItemIsEnabled | Qt.ItemIsSelectable

    def _row_for(self, sysid: int) -> int:
        """Index of `sysid`'s row, inserting it (in SYSID order) if new."""
        for i, row in enumerate(self._rows):
            if row.sysid == sysid:
                return i
            if row.sysid > sysid:
                break
        else:
            i = len(self._rows)
        self.beginInsertRows(QModelIndex(), i, i)
        self._rows.insert(i, _Row(sysid=sysid, name=self._names.get(sysid, "")))
        self.endInsertRows()
        return i

    # ---- updates ----

    def reset(self, names: dict[int, str] | None = None) -> None:
        self.beginResetModel()
        self._rows = []
        self._names = dict(names or {})
        self.endResetModel()

    def update_telemetry(self, drones: list[DroneTelemetry], now: float) -> None:
        touched = []
        for drone in drones:
            i = self._row_for(drone.sysid)
            self._rows[i].telemetry = drone
            self._rows[i].updated_at = now
            touched.append(i)
        if touched:
            self.dataChanged.emit(self.index(min(touched), 0), self.index(max(touched), len(COLUMNS) - 1))

    def set_mission(self, sysid: int, text: str) -> None:
        i = self._row_for(sysid)
        if self._rows[i].mission != text:
            self._rows[i].mission = text
            self._rows[i].final_state = _final_state(text)
            self.dataChanged.emit(self.index(i, COL_STATE), self.index(i, COL_MISSION))

    def tick(self) -> None:
        """Refresh only the "Updated" column (ages grow without new data)."""
        if self._rows:
            self.dataChanged.emit(self.index(0, COL_UPDATED), self.index(len(self._rows) - 1, COL_UPDATED))

    def restyle(self) -> None:
        if self._rows:
            self.dataChanged.emit(self.index(0, 0), self.index(len(self._rows) - 1, len(COLUMNS) - 1))

    # ---- cell contents ----

    def data(self, index, role=Qt.DisplayRole):
        if not index.isValid():
            return None
        row = self._rows[index.row()]
        col = index.column()
        t = row.telemetry
        palette = theme_manager.palette()

        if role == Qt.DisplayRole:
            return self._text(row, col)
        if role == Qt.TextAlignmentRole:
            return int((Qt.AlignRight if col in NUMERIC_COLUMNS else Qt.AlignLeft) | Qt.AlignVCenter)
        if role == Qt.FontRole:
            if col in NUMERIC_COLUMNS:
                return self._fixed
            if col == COL_STATE:
                return self._bold
            return None
        if role == Qt.ForegroundRole:
            if col == COL_STATE:
                return QColor(_state_colour(row, palette))
            if col == COL_BATTERY and t is not None:
                if t.battery_pct < 15:
                    return QColor(palette.critical)
                if t.battery_pct < 30:
                    return QColor(palette.warning)
            if col == COL_UPDATED and self._stale(row):
                return QColor(palette.warning)
            if t is None:
                return QColor(palette.text_secondary)
            return None
        if role == Qt.BackgroundRole:
            if t is not None and t.active_faults:
                return QColor(palette.fault_row_bg)
            return None
        if role == Qt.ToolTipRole:
            if col == COL_MISSION and row.mission:
                return row.mission
            if col == COL_STATE and t is not None and t.active_faults:
                faults = ", ".join(getattr(f, "value", str(f)) for f in t.active_faults)
                return f"Active faults: {faults}"
            return None
        return None

    def _stale(self, row: _Row) -> bool:
        return (self.run_active and row.updated_at is not None
                and time.monotonic() - row.updated_at > STALE_AFTER_S)

    def _text(self, row: _Row, col: int) -> str:
        t = row.telemetry
        if col == COL_DRONE:
            return f"{row.name} · {row.sysid}" if row.name else f"SYSID {row.sysid}"
        if t is None:
            if col == COL_STATE:
                return row.final_state or ""
            if col == COL_MISSION:
                return row.mission or "Waiting for telemetry"
            return ""
        status = getattr(t.status, "value", t.status)
        if col == COL_STATE:
            return row.final_state or status
        if col == COL_MISSION:
            return row.mission or status  # no mission text for this source: show the state
        if col == COL_LAT:
            return f"{t.lat:.6f}"
        if col == COL_LON:
            return f"{t.lon:.6f}"
        if col == COL_ALT:
            return f"{t.altitude_m:.1f}"
        if col == COL_SPEED:
            return f"{t.ground_speed_mps:.1f}"
        if col == COL_HDG:
            return f"{t.heading_deg:.0f}"
        if col == COL_BATTERY:
            return f"{t.battery_pct:.0f} %"
        if col == COL_LINK:
            return f"{t.link_quality_pct:.0f} %"
        if col == COL_UPDATED:
            return "-" if row.updated_at is None else f"{time.monotonic() - row.updated_at:.1f} s"
        return ""


class FlightLogPanel(QWidget):
    """Live drone table (default) and the running log, behind a
    [ Table | Logs ] toggle."""

    TABLE, LOGS = 0, 1

    def __init__(self, parent=None):
        super().__init__(parent)

        # ---- Logs page (the original log, unchanged) ----
        self.view = QPlainTextEdit()
        self.view.setReadOnly(True)
        self.view.setMaximumBlockCount(MAX_LOG_LINES)
        self.view.setFont(QFont("Consolas", 9))
        self.view.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.view.setPlaceholderText(
            "Drone travel log - one timestamped line per drone per telemetry "
            "update (lat / lon / altitude / status) once a flight is running."
        )

        # ---- Table page ----
        self.model = DroneTableModel(self)
        self.table = QTableView()
        self.table.setModel(self.model)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.setAlternatingRowColors(True)
        self.table.setShowGrid(False)
        self.table.setWordWrap(False)
        self.table.setTextElideMode(Qt.ElideRight)
        self.table.verticalHeader().setVisible(False)
        self.table.verticalHeader().setSectionResizeMode(QHeaderView.Fixed)
        self.table.verticalHeader().setDefaultSectionSize(ROW_HEIGHT_PX)
        header = self.table.horizontalHeader()
        header.setHighlightSections(False)
        widths = {COL_DRONE: 90, COL_STATE: 104, COL_LAT: 112, COL_LON: 112, COL_ALT: 64,
                  COL_SPEED: 88, COL_HDG: 58, COL_BATTERY: 66, COL_LINK: 58, COL_UPDATED: 72}
        for col, width in widths.items():
            header.setSectionResizeMode(col, QHeaderView.Interactive)
            header.resizeSection(col, width)
        header.setSectionResizeMode(COL_MISSION, QHeaderView.Stretch)

        self.placeholder = QLabel("No drones flying - the table fills in once a flight starts.")
        self.placeholder.setAlignment(Qt.AlignCenter)
        self.placeholder.setWordWrap(True)

        table_page = QWidget()
        self._table_stack = QStackedLayout(table_page)
        self._table_stack.addWidget(self.placeholder)
        self._table_stack.addWidget(self.table)
        self.model.rowsInserted.connect(self._update_placeholder)
        self.model.modelReset.connect(self._update_placeholder)

        self.pages = QStackedWidget()
        self.pages.addWidget(table_page)   # TABLE
        self.pages.addWidget(self.view)    # LOGS

        # ---- Header: [ Table | Logs ] ........ [Clear] ----
        self.table_btn = QPushButton("Table")
        self.logs_btn = QPushButton("Logs")
        self.toggle_group = QButtonGroup(self)
        self.toggle_group.setExclusive(True)
        for i, (button, side) in enumerate(((self.table_btn, "left"), (self.logs_btn, "right"))):
            button.setCheckable(True)
            button.setObjectName(f"segment_{side}")
            button.setCursor(Qt.PointingHandCursor)
            self.toggle_group.addButton(button, i)
        self.table_btn.setChecked(True)
        self.toggle_group.idClicked.connect(self.show_page)
        self._unread_events = 0

        self.clear_btn = QPushButton("Clear")
        self.clear_btn.clicked.connect(self.clear_log)
        self.clear_btn.setVisible(False)

        header_row = QHBoxLayout()
        header_row.setSpacing(0)
        header_row.addWidget(self.table_btn)
        header_row.addWidget(self.logs_btn)
        header_row.addStretch(1)
        header_row.addWidget(self.clear_btn)

        box = QGroupBox("Flight Log")
        box_layout = QVBoxLayout(box)
        box_layout.addLayout(header_row)
        box_layout.addWidget(self.pages, stretch=1)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(box)

        # Telemetry lands in _pending as fast as it arrives; the table is
        # refreshed from it at most TABLE_REFRESH_MS apart (see log_batch).
        self._pending: dict[int, DroneTelemetry] = {}
        self._pending_at: dict[int, float] = {}
        self._refresh_timer = QTimer(self)
        self._refresh_timer.setInterval(TABLE_REFRESH_MS)
        self._refresh_timer.timeout.connect(self._flush_pending)
        self._refresh_timer.start()
        self._age_timer = QTimer(self)
        self._age_timer.setInterval(1000)
        self._age_timer.timeout.connect(self.model.tick)
        self._age_timer.start()
        # Cumulative time spent updating the table, for measuring its cost.
        self.table_update_s = 0.0
        self.table_updates = 0

        self._restyle()
        theme_manager.theme_changed.connect(lambda _name: self._restyle())
        self._update_placeholder()

    # ---- View switching ----

    def show_page(self, page: int) -> None:
        self.pages.setCurrentIndex(page)
        self.clear_btn.setVisible(page == self.LOGS)
        (self.logs_btn if page == self.LOGS else self.table_btn).setChecked(True)
        if page == self.LOGS:
            self._unread_events = 0
        self._update_badge()

    def _update_badge(self) -> None:
        self.logs_btn.setText(f"Logs ({self._unread_events})" if self._unread_events else "Logs")

    def _update_placeholder(self, *_args) -> None:
        self._table_stack.setCurrentWidget(self.table if self.model.rowCount() else self.placeholder)

    def _restyle(self) -> None:
        p = theme_manager.palette()
        radius = 4
        segment = f"""
            QPushButton#segment_left, QPushButton#segment_right {{
                background-color: {p.panel_bg}; color: {p.text_primary};
                border: 1px solid {p.border}; padding: 3px 14px; min-width: 56px;
            }}
            QPushButton#segment_left {{ border-top-left-radius: {radius}px; border-bottom-left-radius: {radius}px;
                border-top-right-radius: 0; border-bottom-right-radius: 0; }}
            QPushButton#segment_right {{ border-top-right-radius: {radius}px; border-bottom-right-radius: {radius}px;
                border-top-left-radius: 0; border-bottom-left-radius: 0; border-left: none; }}
            QPushButton#segment_left:checked, QPushButton#segment_right:checked {{
                background-color: {p.accent}; color: {p.accent_text}; border-color: {p.accent}; font-weight: bold;
            }}
            QPushButton#segment_left:hover:!checked, QPushButton#segment_right:hover:!checked {{
                border-color: {p.accent};
            }}
        """
        self.table_btn.setStyleSheet(segment)
        self.logs_btn.setStyleSheet(segment)
        self.table.setStyleSheet(
            f"QTableView {{ background-color: {p.panel_bg}; alternate-background-color: {p.window_bg};"
            f" color: {p.text_primary}; gridline-color: {p.border}; border: 1px solid {p.border};"
            f" selection-background-color: {p.accent}; selection-color: {p.accent_text}; }}"
        )
        self.placeholder.setStyleSheet(f"color: {p.text_secondary}; font-style: italic;")
        self.model.restyle()

    # ---- Table feed (called by MainWindow) ----

    def reset_table(self, names: dict[int, str] | None = None) -> None:
        """A new mission or run is starting: drop the old rows. `names` maps
        SYSID -> profile name for the Drone column."""
        self._pending.clear()
        self._pending_at.clear()
        self.model.reset(names)

    def set_mission_text(self, sysid: int, text: str) -> None:
        self.model.set_mission(sysid, text)

    def set_run_active(self, active: bool) -> None:
        """While a run is active, a drone whose telemetry stops for longer
        than STALE_AFTER_S shows its "Updated" age in the warning colour."""
        self.model.run_active = active
        self.model.tick()

    def _flush_pending(self) -> None:
        if not self._pending:
            return
        start = time.perf_counter()
        drones = [self._pending[s] for s in sorted(self._pending)]
        now = max(self._pending_at.values())
        self._pending.clear()
        self._pending_at.clear()
        self.model.update_telemetry(drones, now)
        self.table_update_s += time.perf_counter() - start
        self.table_updates += 1

    # ---- Writing the log (public API unchanged) ----

    @staticmethod
    def _stamp() -> str:
        return time.strftime("%H:%M:%S")

    def log_line(self, text: str) -> None:
        """Append a raw line exactly as given (no timestamp) - used for the
        continuation lines of a multi-line entry such as a plan's steps."""
        self.view.appendPlainText(text)

    def log_event(self, text: str) -> None:
        """Append a timestamped one-off event (run started/stopped, plan
        ready, ...). Counted on the Logs button while the table is showing."""
        self.view.appendPlainText(f"[{self._stamp()}] === {text}")
        if self.pages.currentIndex() != self.LOGS:
            self._unread_events += 1
            self._update_badge()

    def log_batch(self, drones: list[DroneTelemetry]) -> None:
        """Append one line per drone in this telemetry batch, and queue the
        batch for the table's next refresh."""
        stamp = self._stamp()
        now = time.monotonic()
        for drone in sorted(drones, key=lambda d: d.sysid):
            status = getattr(drone.status, "value", drone.status)
            self.view.appendPlainText(
                f"[{stamp}] SYSID {drone.sysid:<4} "
                f"lat={drone.lat:10.5f}  lon={drone.lon:10.5f}  "
                f"alt={drone.altitude_m:7.1f} m  "
                f"bat={drone.battery_pct:3.0f}%  {status}"
            )
            self._pending[drone.sysid] = drone
            self._pending_at[drone.sysid] = now

    def clear_log(self) -> None:
        self.view.clear()
