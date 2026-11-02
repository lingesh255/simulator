"""Fault Injection Controls (SRS §3.2.4, §9): per-drone or swarm-wide GPS
loss, communications loss, and battery failure injection - and clearing
them again, per drone or all at once.
"""
from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QBrush, QColor
from PySide6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from contracts.gui_orchestration import FaultSeverity, FaultType, InjectFault

SYSID_ROLE = Qt.UserRole + 1

FAULT_LABELS = {
    FaultType.GPS_LOSS: "GPS Loss",
    FaultType.COMMS_LOSS: "Communications Loss",
    FaultType.BATTERY_FAIL: "Battery Failure",
}

FAULT_COLOUR = QColor("#e74c3c")
CHECK_ICON = (Path(__file__).resolve().parent / "icons" / "check.svg").as_posix()

# A big, high-contrast tick box: empty outline when unticked, filled green
# with a white tick when ticked - readable on the dark theme.
LIST_STYLE = f"""
QListWidget {{ font-size: 13px; }}
QListWidget::item {{ padding: 5px 4px; border-radius: 4px; }}
QListWidget::item:hover {{ background: rgba(255, 255, 255, 0.08); }}
QListWidget::indicator {{
    width: 18px; height: 18px; border-radius: 4px;
    border: 2px solid #9aa4ad; background: transparent;
}}
QListWidget::indicator:checked {{
    border: 2px solid #27ae60; background: #27ae60; image: url({CHECK_ICON});
}}
"""


class FaultInjectionPanel(QWidget):
    """Targets one or more active drones (SYSIDs) with an injected fault,
    and clears faults again."""

    fault_requested = Signal(object)  # InjectFault
    clear_requested = Signal(object)  # list[int] of SYSIDs, or None for every drone
    status_message = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        # Which faults each drone currently reports (from telemetry), for the
        # list's labels.
        self._active_faults: dict[int, list[FaultType]] = {}

        self.drone_list = QListWidget()
        self.drone_list.setSelectionMode(QAbstractItemView.NoSelection)
        self.drone_list.setFocusPolicy(Qt.NoFocus)
        self.drone_list.setStyleSheet(LIST_STYLE)
        # A click anywhere on a row ticks / unticks it (see _on_item_clicked).
        self.drone_list.itemClicked.connect(self._on_item_clicked)

        self.selection_label = QLabel("No active drones.")
        self.selection_label.setStyleSheet("color: #9aa4ad;")

        self.select_all_btn = QPushButton("Select All")
        self.select_none_btn = QPushButton("Select None")
        self.select_all_btn.clicked.connect(lambda: self._set_all_checked(True))
        self.select_none_btn.clicked.connect(lambda: self._set_all_checked(False))

        select_row = QHBoxLayout()
        select_row.addWidget(self.select_all_btn)
        select_row.addWidget(self.select_none_btn)

        target_group = QGroupBox("Target Drones")
        target_layout = QVBoxLayout(target_group)
        target_layout.addWidget(self.drone_list)
        target_layout.addWidget(self.selection_label)
        target_layout.addLayout(select_row)

        self.fault_combo = QComboBox()
        for fault_type, label in FAULT_LABELS.items():
            self.fault_combo.addItem(label, fault_type)

        self.severity_combo = QComboBox()
        for severity in FaultSeverity:
            self.severity_combo.addItem(severity.value.capitalize(), severity)

        self.duration_spin = QDoubleSpinBox()
        self.duration_spin.setRange(0, 3600)
        self.duration_spin.setSuffix(" s")
        self.duration_spin.setSpecialValueText("Indefinite")
        self.duration_spin.setValue(0)
        self.duration_spin.setToolTip(
            "How long the fault lasts before it clears itself.\n"
            "Indefinite (0): until you press Clear Fault / Clear All Faults."
        )

        form = QFormLayout()
        form.addRow("Fault type", self.fault_combo)
        form.addRow("Severity", self.severity_combo)
        form.addRow("Duration (0 = until cleared)", self.duration_spin)

        self.inject_btn = QPushButton("Inject Fault")
        self.inject_btn.setStyleSheet(
            "QPushButton { font-weight: bold; color: white; background: #c0392b; padding: 6px; }"
            "QPushButton:hover { background: #e74c3c; }"
            "QPushButton:disabled { background: #5a3a37; color: #b8a9a7; }"
        )
        self.inject_btn.setToolTip("Inject the fault above into every ticked drone.")
        self.inject_btn.clicked.connect(self._on_inject)

        self.clear_btn = QPushButton("Clear Fault")
        self.clear_btn.setStyleSheet(
            "QPushButton { font-weight: bold; color: white; background: #1e8449; padding: 6px; }"
            "QPushButton:hover { background: #27ae60; }"
            "QPushButton:disabled { background: #2f4a3a; color: #a9b8ae; }"
        )
        self.clear_btn.setToolTip("Clear the faults on every ticked drone.")
        self.clear_btn.clicked.connect(self._on_clear_selected)

        self.clear_all_btn = QPushButton("Clear All Faults")
        self.clear_all_btn.setToolTip("Clear the faults on every active drone, ticked or not.")
        self.clear_all_btn.clicked.connect(self._on_clear_all)

        # Inject and Clear side by side, half the width each.
        action_row = QHBoxLayout()
        action_row.addWidget(self.inject_btn, stretch=1)
        action_row.addWidget(self.clear_btn, stretch=1)

        fault_group = QGroupBox("Fault Injection")
        fault_layout = QVBoxLayout(fault_group)
        fault_layout.addLayout(form)
        fault_layout.addLayout(action_row)
        fault_layout.addWidget(self.clear_all_btn)

        layout = QVBoxLayout(self)
        layout.addWidget(target_group, stretch=1)
        layout.addWidget(fault_group)

        self._refresh_controls()

    # ---- The drone list ----

    def update_active_sysids(self, sysids: list[int]) -> None:
        """Show these drones. The list is only rebuilt when the set of drones
        changes - telemetry calls this many times a second, and rebuilding
        every time made ticks vanish or flip back while being clicked."""
        wanted = sorted(set(sysids))
        if wanted == self._listed_sysids():
            return
        checked = set(self._checked_sysids())
        self.drone_list.clear()
        for sysid in wanted:
            item = QListWidgetItem()
            # Not user-checkable through Qt's own indicator: every click on
            # the row goes through _on_item_clicked instead, so a click on
            # the box and a click on the text behave exactly the same.
            item.setFlags(Qt.ItemIsEnabled)
            item.setCheckState(Qt.Checked if sysid in checked else Qt.Unchecked)
            item.setData(SYSID_ROLE, sysid)
            self.drone_list.addItem(item)
        self._active_faults = {s: f for s, f in self._active_faults.items() if s in wanted}
        self._refresh_labels()
        self._refresh_controls()

    def update_drones(self, drones: list) -> None:
        """From a telemetry batch: which drones are active, and which faults
        each one currently reports."""
        self.update_active_sysids([d.sysid for d in drones])
        faults = {d.sysid: list(d.active_faults) for d in drones}
        if faults != self._active_faults:
            self._active_faults = faults
            self._refresh_labels()

    def _listed_sysids(self) -> list[int]:
        return [self.drone_list.item(i).data(SYSID_ROLE) for i in range(self.drone_list.count())]

    def _refresh_labels(self) -> None:
        for i in range(self.drone_list.count()):
            item = self.drone_list.item(i)
            sysid = item.data(SYSID_ROLE)
            faults = self._active_faults.get(sysid, [])
            if faults:
                names = ", ".join(FAULT_LABELS.get(f, str(f)) for f in faults)
                item.setText(f"SYSID {sysid}    ⚠ {names} active")
                item.setForeground(QBrush(FAULT_COLOUR))
            else:
                item.setText(f"SYSID {sysid}")
                item.setData(Qt.ForegroundRole, None)

    def _on_item_clicked(self, item: QListWidgetItem) -> None:
        item.setCheckState(Qt.Unchecked if item.checkState() == Qt.Checked else Qt.Checked)
        self._refresh_controls()

    def _set_all_checked(self, checked: bool) -> None:
        state = Qt.Checked if checked else Qt.Unchecked
        for i in range(self.drone_list.count()):
            self.drone_list.item(i).setCheckState(state)
        self._refresh_controls()

    def _checked_sysids(self) -> list[int]:
        return [
            self.drone_list.item(i).data(SYSID_ROLE)
            for i in range(self.drone_list.count())
            if self.drone_list.item(i).checkState() == Qt.Checked
        ]

    def _refresh_controls(self) -> None:
        """Say how many drones are ticked, and only enable what can act."""
        total = self.drone_list.count()
        ticked = len(self._checked_sysids())
        if total == 0:
            self.selection_label.setText("No active drones - start a flight first.")
        else:
            self.selection_label.setText(f"{ticked} of {total} drone(s) ticked.")
        self.inject_btn.setEnabled(ticked > 0)
        self.clear_btn.setEnabled(ticked > 0)
        self.clear_all_btn.setEnabled(total > 0)
        self.select_all_btn.setEnabled(total > 0 and ticked < total)
        self.select_none_btn.setEnabled(ticked > 0)

    # ---- Actions ----

    def _on_inject(self) -> None:
        sysids = self._checked_sysids()
        if not sysids:
            QMessageBox.information(self, "No target selected", "Tick at least one active drone to target.")
            return
        command = InjectFault(
            sysids=sysids,
            fault_type=self.fault_combo.currentData(),
            severity=self.severity_combo.currentData(),
            duration_s=self.duration_spin.value() or None,
        )
        # Announce the injection first: handling the fault may itself post a
        # status message (a fall-back decision, say), and that outcome is the
        # more useful thing to leave on screen.
        label = FAULT_LABELS[command.fault_type]
        self.status_message.emit(f"Injected '{label}' on SYSID(s): {', '.join(map(str, sysids))}.")
        self.fault_requested.emit(command)

    def _on_clear_selected(self) -> None:
        sysids = self._checked_sysids()
        if not sysids:
            QMessageBox.information(self, "No target selected", "Tick the drones whose faults to clear.")
            return
        self.clear_requested.emit(sysids)

    def _on_clear_all(self) -> None:
        self.clear_requested.emit(None)
