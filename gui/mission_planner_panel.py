"""Mission Planner panel: pick a named PDDL plan, arm the map for a
start/destination click, and show ENHSP's parsed action sequence once one
comes back.
"""
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from engine.pddl_planner import PlanStep
from services.plan_service import list_plans


class MissionPlannerPanel(QWidget):
    """Select a plan, request it, and read back its parsed step list."""

    plan_requested = Signal(str)  # plan name

    def __init__(self, parent=None):
        super().__init__(parent)

        self.plan_combo = QComboBox()
        self.refresh_btn = QPushButton("Refresh")
        self.plan_btn = QPushButton("Plan Mission")
        self.plan_btn.setStyleSheet("font-weight: bold;")
        self.refresh_btn.clicked.connect(self.reload_plans)
        self.plan_btn.clicked.connect(self._on_plan_clicked)

        pick_row = QHBoxLayout()
        pick_row.addWidget(self.plan_combo, stretch=1)
        pick_row.addWidget(self.refresh_btn)

        self.restricted_check = QCheckBox("Mark a restricted area (3rd click)")
        self.restricted_check.setToolTip(
            "Unchecked: the mission always flies direct.\n"
            "Checked: click a Start point, a Destination, then switch the map's "
            "Click mode to \"Set Restricted Area\" and click the area to avoid - "
            "the mission only detours around it if it actually sits in the way."
        )

        self.external_mavlink_check = QCheckBox("Fly via real MAVLink (SITL/hardware)")
        self.external_mavlink_check.setToolTip(
            "Unchecked (default): fly using this app's own in-process drone-thread "
            "pipeline - works with no extra setup.\n"
            "Checked: instead upload the planned route as one real MAVLink mission "
            "to the connection string below and fly it there - needs an actual "
            "ArduPilot/PX4 (SITL or hardware) already listening, or the mission "
            "fails immediately rather than hanging."
        )
        self.external_mavlink_check.toggled.connect(self._on_external_mavlink_toggled)

        self.mavlink_connection_edit = QLineEdit("udp:127.0.0.1:14550")
        self.mavlink_connection_edit.setEnabled(False)
        self.mavlink_connection_edit.setToolTip(
            "pymavlink connection string, e.g. udp:127.0.0.1:14550 for ArduPilot "
            "SITL's default output."
        )
        connection_row = QHBoxLayout()
        connection_row.addWidget(QLabel("Connect:"))
        connection_row.addWidget(self.mavlink_connection_edit, stretch=1)

        self.mock_vehicle_check = QCheckBox("Use built-in mock vehicle (auto-launch, for testing)")
        self.mock_vehicle_check.setEnabled(False)
        self.mock_vehicle_check.setToolTip(
            "Unchecked: you manage the vehicle yourself - point Connect at a real "
            "ArduPilot/PX4 (SITL or hardware) you already started.\n"
            "Checked: the app launches scripts/mock_sitl.py for you, seeded with "
            "the source point you click on the map - no separate terminal or "
            "manual --home needed. Only for testing; leave unchecked against a "
            "real vehicle (checking this alongside a real local SITL on the same "
            "port would run two vehicles at once)."
        )

        plan_group = QGroupBox("Named Plan (plans/<name>/)")
        plan_layout = QVBoxLayout(plan_group)
        plan_layout.addLayout(pick_row)
        plan_layout.addWidget(self.restricted_check)
        plan_layout.addWidget(self.external_mavlink_check)
        plan_layout.addLayout(connection_row)
        plan_layout.addWidget(self.mock_vehicle_check)
        plan_layout.addWidget(self.plan_btn)

        self.status_label = QLabel("Idle.")
        self.status_label.setWordWrap(True)

        self.output = QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setPlaceholderText(
            "Pick a plan, press \"Plan Mission\", then click a Start point and "
            "a Destination on the map. ENHSP's action sequence appears here."
        )

        layout = QVBoxLayout(self)
        layout.addWidget(plan_group)
        layout.addWidget(self.status_label)
        layout.addWidget(self.output, stretch=1)

        self.reload_plans()

    def reload_plans(self) -> None:
        current = self.plan_combo.currentText()
        self.plan_combo.clear()
        self.plan_combo.addItems(list_plans())
        index = self.plan_combo.findText(current)
        if index >= 0:
            self.plan_combo.setCurrentIndex(index)

    def _on_plan_clicked(self) -> None:
        name = self.plan_combo.currentText()
        if not name:
            self.set_status("No plans found under plans/ - add a domain.pddl + problem.pddl folder.")
            return
        self.plan_requested.emit(name)

    def mark_restricted_area(self) -> bool:
        return self.restricted_check.isChecked()

    def _on_external_mavlink_toggled(self, checked: bool) -> None:
        self.mavlink_connection_edit.setEnabled(checked)
        self.mock_vehicle_check.setEnabled(checked)
        if not checked:
            self.mock_vehicle_check.setChecked(False)

    def use_external_mavlink(self) -> bool:
        return self.external_mavlink_check.isChecked()

    def mavlink_connection_string(self) -> str:
        return self.mavlink_connection_edit.text().strip()

    def use_mock_vehicle(self) -> bool:
        return self.mock_vehicle_check.isChecked()

    def set_status(self, text: str) -> None:
        self.status_label.setText(text)

    def set_plan_steps(self, plan_name: str, steps: list[PlanStep]) -> None:
        lines = [f"Plan '{plan_name}' - {len(steps)} step(s):", ""]
        lines.extend(str(step) for step in sorted(steps, key=lambda s: s.index))
        self.output.setPlainText("\n".join(lines))

    def set_error(self, text: str) -> None:
        self.output.setPlainText(text)
