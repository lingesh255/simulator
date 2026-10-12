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
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from engine.figure8_problem import MAX_FIGURE8_LAPS
from engine.pddl_planner import PlanStep
from services.plan_service import list_plans, plan_kind

# Display-only labels for plan folders under plans/ - the underlying folder
# name (used everywhere else as the plan identifier) is unchanged.
PLAN_DISPLAY_NAMES = {
    "travell": "Travell",
    "Search": "Search Forest",
    "vformation": "V-Formation",
    "gridformation": "Grid-Formation",
    "3dgridformation": "3D-Grid Formation",
    "figure8formation": "Figure-8 Formation",
    "starformation": "Star Drone Show",
    "planets": "Planets",
    "Inspect": "Inspect",
}


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

        # Only meaningful for the figure-8, the star show and the planets -
        # shown just while one of them is picked.
        self.laps_spin = QSpinBox()
        self.laps_spin.setRange(1, MAX_FIGURE8_LAPS)
        self.laps_spin.setValue(1)
        self.laps_spin.setSuffix(" lap(s)")
        self.laps_spin.setToolTip(
            "Figure-8: how many times the drones fly the whole 8 before flying "
            "back from its middle point to the base.\n"
            "Star show: how many times the show runs (star, circle, spinning "
            "circle, star, spinning star, circle) before the drones land.\n"
            "Planets: how many times the OUTERMOST drone goes round its orbit "
            "(inner drones go round more often) before the drones land."
        )
        self.laps_label = QLabel("Laps:")
        laps_row = QHBoxLayout()
        laps_row.addWidget(self.laps_label)
        laps_row.addWidget(self.laps_spin, stretch=1)
        self.plan_combo.currentIndexChanged.connect(self._on_plan_selection_changed)

        # Only for the drone shows (star, planets): how fast they play. The simulator normally
        # runs at 20x real time, which turns a show into a blur.
        self.show_speed_combo = QComboBox()
        for label, scale in (
            ("Real time (1x)", 1.0), ("2x", 2.0), ("5x", 5.0), ("20x (simulator default)", 20.0),
        ):
            self.show_speed_combo.addItem(label, scale)
        self.show_speed_combo.setCurrentIndex(1)
        self.show_speed_combo.setToolTip(
            "How fast the drone show plays: 1x is real time - the drones fly at "
            "their real show speed (the star's tips about 6-7 m/s). The rest of "
            "the simulator runs at 20x."
        )
        self.show_speed_label = QLabel("Show speed:")
        show_speed_row = QHBoxLayout()
        show_speed_row.addWidget(self.show_speed_label)
        show_speed_row.addWidget(self.show_speed_combo, stretch=1)

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
        plan_layout.addLayout(laps_row)
        plan_layout.addLayout(show_speed_row)
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
        current = self.plan_combo.currentData()
        self.plan_combo.clear()
        for name in list_plans():
            self.plan_combo.addItem(PLAN_DISPLAY_NAMES.get(name, name), name)
        index = self.plan_combo.findData(current)
        if index >= 0:
            self.plan_combo.setCurrentIndex(index)
        self._on_plan_selection_changed()

    def _on_plan_selection_changed(self) -> None:
        name = self.plan_combo.currentData()
        kind = plan_kind(name) if name else None
        has_laps = kind in ("figure8_formation", "star_formation", "planets_formation")
        self.laps_label.setVisible(has_laps)
        self.laps_spin.setVisible(has_laps)
        is_show = kind in ("star_formation", "planets_formation")
        self.show_speed_label.setVisible(is_show)
        self.show_speed_combo.setVisible(is_show)
        # A figure-8 is picked from points of its own and never detours; a
        # drone show stays in one place; an inspection follows the road.
        self.restricted_check.setEnabled(not has_laps and kind != "road_inspection")

    def figure8_laps(self) -> int:
        return self.laps_spin.value()

    def show_time_scale(self) -> float:
        """How many times real time a drone show (star, planets) plays at."""
        return float(self.show_speed_combo.currentData())

    def _on_plan_clicked(self) -> None:
        name = self.plan_combo.currentData()
        if not name:
            self.set_status("No plans found under plans/ - add a domain.pddl + problem.pddl folder.")
            return
        self.plan_requested.emit(name)

    def mark_restricted_area(self) -> bool:
        # Greyed out for plans that never detour - a tick left over from an
        # earlier plan doesn't count.
        return self.restricted_check.isChecked() and self.restricted_check.isEnabled()

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
