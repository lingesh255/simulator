"""Mission Planner panel: pick a named PDDL plan, arm the map for a
start/destination click, and show ENHSP's parsed action sequence once one
comes back.
"""
from __future__ import annotations

from pathlib import Path

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

# Display-only labels for plan folders under plans/ - the underlying folder
# name (used everywhere else as the plan identifier) is unchanged.
PLAN_DISPLAY_NAMES = {
    "travell": "Travell",
    "Search": "Search Forest",
    "vformation": "V-Formation",
    "gridformation": "Grid-Formation",
}


class MissionPlannerPanel(QWidget):
    """Select a plan, request it, and read back its parsed step list."""

    plan_requested = Signal(str)  # plan name
    renode_launch_requested = Signal(str)  # standalone folder path

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

        # Empty with a placeholder, not a literal starting value: Renode
        # always serves tcp:127.0.0.1:5762, a real, fixed, different
        # protocol and port from the udp:... SITL/mock convention this
        # field used to show as an actual (not just hinted) value the whole
        # time - genuinely misleading while "Launch Renode" is the intended
        # path, since it looked like an already-made, submittable choice
        # rather than an example. mavlink_connection_string() already
        # returns "" for an empty field, and _start_external_mavlink_mission
        # already reports a clear "enter a connection string first" message
        # for that case - so this doesn't newly need any error handling of
        # its own, it just stops silently offering a wrong-looking default.
        self.mavlink_connection_edit = QLineEdit()
        self.mavlink_connection_edit.setPlaceholderText(
            "udp:127.0.0.1:14550 (SITL/mock convention - not used if launching Renode below)"
        )
        self.mavlink_connection_edit.setEnabled(False)
        self.mavlink_connection_edit.setToolTip(
            "pymavlink connection string. Renode always fills this in itself "
            "(tcp:127.0.0.1:5762) once launched below - only type here "
            "yourself for a real SITL/hardware connection instead, e.g. "
            "udp:127.0.0.1:14550 for ArduPilot SITL's default output."
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
            "port would run two vehicles at once). Mutually exclusive with "
            "launching Renode below - both auto-fill Connect themselves."
        )
        self.mock_vehicle_check.toggled.connect(self._on_mock_vehicle_toggled)

        self.renode_dir_edit = QLineEdit(
            str(Path(__file__).resolve().parent.parent / "pixhawk6c_renode_standalone")
        )
        self.renode_dir_edit.setEnabled(False)
        self.renode_dir_edit.setToolTip(
            "Path to the standalone Renode + Pixhawk6C/6X package (contains "
            "renode-bin/renode and launch.resc)."
        )
        self.renode_launch_btn = QPushButton("Launch Renode")
        self.renode_launch_btn.setEnabled(False)
        self.renode_launch_btn.clicked.connect(self._on_renode_launch_clicked)
        renode_row = QHBoxLayout()
        renode_row.addWidget(self.renode_dir_edit, stretch=1)
        renode_row.addWidget(self.renode_launch_btn)

        plan_group = QGroupBox("Named Plan (plans/<name>/)")
        plan_layout = QVBoxLayout(plan_group)
        plan_layout.addLayout(pick_row)
        plan_layout.addWidget(self.restricted_check)
        plan_layout.addWidget(self.external_mavlink_check)
        plan_layout.addLayout(connection_row)
        plan_layout.addWidget(self.mock_vehicle_check)
        plan_layout.addLayout(renode_row)
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

        # Whether the currently-launched Renode instance has reported ready
        # - only meaningful while external_mavlink_check is checked (see
        # _update_plan_gating). Reset to False by set_renode_ready(False),
        # called by MainWindow both while a launch is still booting and if
        # a mid-session MAVLink failure suggests the connection died.
        self._renode_ready = False
        # Whether at least one drone is checked in Drone Configuration
        # Profiles - updated by set_drone_selected(), called by MainWindow
        # via DroneManagementPanel.selection_changed. Starts False: a fresh
        # app has no drone checked by default (DroneManagementPanel adds
        # every row unchecked unless a prior session's checked state was
        # carried over a reload).
        self._drone_selected = False
        # Whether a launch is currently in flight (between the button click
        # and ready/failed) - distinct from _renode_ready is False in that
        # state too, but needs its own flag so the button's tooltip/gating
        # can distinguish "no drone selected yet" from "already launching".
        self._renode_launch_in_progress = False
        # Whether an external-MAVLink mission is actively flying right now
        # - set True by MainWindow at the same point it calls
        # set_renode_ready(False) for a fresh mission (see
        # _start_external_mavlink_mission), cleared alongside the
        # post-flight relaunch trigger (_restart_renode_after_mission).
        # Deliberately a SEPARATE flag from _renode_ready rather than
        # reusing it: _renode_ready is False both while a mission is
        # flying AND after a genuine failure/disconnect with nothing
        # flying, but those two situations need opposite "Launch Renode"
        # gating - a live, airborne vehicle must not be killed out from
        # under an active flight, while a failed/disconnected one (this
        # flag False) must still allow "Launch Renode" to recover, exactly
        # as before this flag existed.
        self._mission_active = False

        self.reload_plans()

    def reload_plans(self) -> None:
        current = self.plan_combo.currentData()
        self.plan_combo.clear()
        for name in list_plans():
            self.plan_combo.addItem(PLAN_DISPLAY_NAMES.get(name, name), name)
        index = self.plan_combo.findData(current)
        if index >= 0:
            self.plan_combo.setCurrentIndex(index)

    def _on_plan_clicked(self) -> None:
        name = self.plan_combo.currentData()
        if not name:
            self.set_status("No plans found under plans/ - add a domain.pddl + problem.pddl folder.")
            return
        self.plan_requested.emit(name)

    def mark_restricted_area(self) -> bool:
        return self.restricted_check.isChecked()

    def set_renode_ready(self, ready: bool) -> None:
        """Called by MainWindow when a launched Renode instance reports
        ready (True), or when a launch is (re)started, fails, or a
        mid-session MAVLink failure suggests the connection died (False -
        see main_window.py's wiring of RenodeLaunchService.failed and
        MavlinkFlightService.failed). Only affects "Plan Mission"/the plan
        combo's ENABLED state while external_mavlink_check is checked -
        unaffected otherwise, matching requirement C.1. Deliberately
        doesn't touch the status message itself: the caller already knows
        a more specific reason (a launch failure's real error text, a
        fresh boot just starting, ready with the real connection string)
        than this method could - callers should call set_status() with
        that themselves, same turn, after this. Also clears
        _renode_launch_in_progress - a launch has, one way or another,
        settled by the time this is called with either value (True: it
        finished; False: it failed, or a fresh launch is about to
        overwrite whatever was running before) - and refreshes "Launch
        Renode"'s own gating/tooltip, replacing what used to be a direct
        setEnabled(True) call in MainWindow's ready handler that didn't
        account for the drone-selection requirement."""
        self._renode_ready = ready
        self._renode_launch_in_progress = False
        self._update_plan_gating()
        self._update_mock_vehicle_gating()
        self._update_renode_launch_gating()

    def _update_plan_gating(self) -> None:
        """"Plan Mission" and the plan combo require a live, ready
        connection whenever real MAVLink is the selected path - clicking
        Plan Mission before Renode has finished booting produced a real,
        confusing "No heartbeat" failure previously; this prevents that at
        the UI level instead of relying on the user to wait for a status
        message. Enabled-state only - see set_renode_ready()'s docstring
        for why this doesn't also set a status message."""
        needs_renode = self.external_mavlink_check.isChecked() and not self._renode_ready
        self.plan_btn.setEnabled(not needs_renode)
        self.plan_combo.setEnabled(not needs_renode)

    def set_drone_selected(self, selected: bool) -> None:
        """Called by MainWindow via DroneManagementPanel.selection_changed
        whenever the checked-drone set changes. "Launch Renode" requires
        at least one drone checked first - launching against an empty
        swarm selection produces a live Renode instance with nothing to
        fly, which previously went unnoticed until well after the ~75s
        boot completed."""
        self._drone_selected = selected
        self._update_renode_launch_gating()

    def set_mission_active(self, active: bool) -> None:
        """Called by MainWindow at the same point it calls
        set_renode_ready(False) for a fresh mission (True), and cleared
        alongside the post-flight relaunch trigger (False) - see
        _mission_active's own comment in __init__ for why this has to be
        a separate flag from _renode_ready rather than reusing it."""
        self._mission_active = active
        self._update_renode_launch_gating()

    def _update_renode_launch_gating(self) -> None:
        """"Launch Renode" requires, in order: no mission actively flying
        right now (checked first and unconditionally - this is the one
        case that must NOT be overridden by anything below, since killing
        a live, airborne vehicle is a different, worse situation than a
        merely-not-yet-ready or failed one), real MAVLink selected, the
        mock vehicle NOT in use (Part D.1 - they're two mutually exclusive
        ways of filling the same Connect field), a drone actually checked
        in Drone Configuration Profiles, and no launch already in flight.
        Sets both the enabled state and an explanatory tooltip, so a
        disabled button doesn't leave the user guessing which condition is
        missing."""
        if self._mission_active:
            self.renode_launch_btn.setEnabled(False)
            self.renode_launch_btn.setToolTip(
                "A mission is actively flying - wait for it to finish (or "
                "Stop it) before launching a new Renode instance."
            )
        elif not self.external_mavlink_check.isChecked():
            self.renode_launch_btn.setEnabled(False)
            self.renode_launch_btn.setToolTip('Check "Fly via real MAVLink" above first.')
        elif self.mock_vehicle_check.isChecked():
            self.renode_launch_btn.setEnabled(False)
            self.renode_launch_btn.setToolTip(
                'Uncheck "Use built-in mock vehicle" first - it and Renode both '
                "fill in Connect themselves and can't be used together."
            )
        elif self._renode_launch_in_progress:
            self.renode_launch_btn.setEnabled(False)
            self.renode_launch_btn.setToolTip("Renode is already launching - please wait.")
        elif not self._drone_selected:
            self.renode_launch_btn.setEnabled(False)
            self.renode_launch_btn.setToolTip(
                "Check at least one drone profile in Drone Configuration Profiles first."
            )
        else:
            self.renode_launch_btn.setEnabled(True)
            self.renode_launch_btn.setToolTip("")

    def _update_mock_vehicle_gating(self) -> None:
        """Mirror of _update_renode_launch_gating()'s exclusivity (Part
        D.1): once a Renode launch is in flight or ready, "Use built-in
        mock vehicle" no longer makes sense against the same Connect
        field (which Renode now owns), so disable - and uncheck, not just
        gray out, since a checked-but-disabled box would still visually
        claim to be the active choice - rather than leaving both options
        looking simultaneously available."""
        enabled = (
            self.external_mavlink_check.isChecked()
            and not self._renode_launch_in_progress
            and not self._renode_ready
        )
        self.mock_vehicle_check.setEnabled(enabled)
        if not enabled:
            self.mock_vehicle_check.setChecked(False)

    def _on_mock_vehicle_toggled(self, checked: bool) -> None:
        # Checking the mock vehicle must disable the Renode controls
        # (Part D.1) - renode_dir_edit isn't covered by
        # _update_renode_launch_gating() (that only sets the button), so
        # it's handled directly here.
        self.renode_dir_edit.setEnabled(self.external_mavlink_check.isChecked() and not checked)
        self._update_renode_launch_gating()

    def _on_external_mavlink_toggled(self, checked: bool) -> None:
        self.mavlink_connection_edit.setEnabled(checked)
        self.renode_dir_edit.setEnabled(checked and not self.mock_vehicle_check.isChecked())
        self._update_plan_gating()
        self._update_mock_vehicle_gating()
        self._update_renode_launch_gating()
        if not checked:
            self.mock_vehicle_check.setChecked(False)
        elif not self._renode_ready:
            self.set_status("Launch Renode below, then plan a mission once it's ready.")

    def _on_renode_launch_clicked(self) -> None:
        # A re-launch (e.g. after the previous instance was closed) must
        # not leave "Plan Mission" enabled against an instance that's being
        # torn down and rebuilt. Also clears _renode_launch_in_progress -
        # set True right after - so this is purely "reset stale ready
        # state", not a statement about the new launch's progress.
        self.set_renode_ready(False)
        # Disabled immediately, re-enabled once this launch settles (ready
        # or failed, via set_renode_ready) - RenodeLaunchService itself
        # also now rejects a launch requested while one is already in
        # flight, but disabling the button too means a real double-click
        # never reaches that rejection path at all, matching what a user
        # actually expects to see (a busy button, not a click that
        # silently does nothing).
        self._renode_launch_in_progress = True
        self._update_mock_vehicle_gating()
        self._update_renode_launch_gating()
        # The auto-fill on ready already sets the real text - this just
        # makes the field's hint accurate for the boot window in between,
        # instead of still showing the generic SITL placeholder while
        # Renode (a fixed, different tcp: address) is what's actually
        # coming.
        self.mavlink_connection_edit.setPlaceholderText(
            "tcp:127.0.0.1:5762 (Renode - booting, fills in automatically once ready)"
        )
        self.renode_launch_requested.emit(self.renode_dir_edit.text().strip())

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
