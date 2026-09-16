"""Module 1 - GUI Application main window (SRS §3).

Ties together the Drone Management Panel, Map Viewer, Flight Log, Fault
Injection Controls and Mission Planner - laid out as one vertically
scrollable page - wiring user interaction through to the Renode Emulation
Backend's UDP contract via `services.api_client.OrchestrationClient`.
"""
from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtGui import QAction
from PySide6.QtWidgets import (
    QComboBox,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

from contracts.gui_orchestration import FlockCommand, LatLon
from engine.renode_launcher import RenodeLauncher
from gui.drone_management import DroneManagementPanel
from gui.fault_injection import FaultInjectionPanel
from gui.flight_log_panel import FlightLogPanel
from gui.map_viewer import MapViewer
from gui.mission_planner_panel import MissionPlannerPanel
from gui.telemetry_dashboard import TelemetryDashboard
from gui.theme import theme_manager
from services.api_client import OrchestrationClient
from services.mavlink_flight_service import MavlinkFlightService
from services.plan_service import PlanRunResult, PlanService, plan_kind
from services.renode_launch_service import RenodeLaunchService
from services.thread_backend import ThreadSwarmBackend
from services.local_flight import (
    LOW_BATTERY_PCT,
    LocalFlightSimulator,
    format_duration,
    haversine_m,
    nearest_point,
    plan_flights,
    plan_route_flights,
    plan_route_flights_by_drone,
)
from services.storage import ProfileStore

NFZ_CORNER_COUNT = 4  # a restricted area is a quadrilateral, click order = winding order
FOREST_CORNER_COUNT = 4  # a search area is a quadrilateral too - "dynamic dimensions"


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Swarm Simulator - GUI (Module 1)")
        self.resize(1440, 900)

        self.store = ProfileStore()
        self.client = OrchestrationClient(parent=self)

        # Start/destination picked on the map, and the local preview that flies
        # between them when the backend is not supplying telemetry.
        self._start_point: LatLon | None = None
        self._destination_point: LatLon | None = None
        # Every point the controller has placed, in order - an emergency
        # landing diverts to whichever of these is nearest.
        self._landing_candidates: list[LatLon] = []
        self._connected = False

        # PDDL mission planning: while armed, the next map clicks are a
        # plan's start, then destination, then (if the panel's checkbox asks
        # for one) the restricted area's corners - in that order, regardless
        # of the map's own "Click mode" dropdown, which this drives itself
        # so the user never has to operate it by hand mid-sequence.
        #
        # An area-coverage plan (see services.plan_service.plan_kind) uses
        # the same "Start" click for its base point, then collects
        # `FOREST_CORNER_COUNT` search-area corners instead of a destination
        # - `_plan_kind` (set once per `_on_plan_requested`) is what tells
        # `_handle_plan_click`/`_run_planned_mission` which sequence is live.
        self.plan_service = PlanService(self)
        self._planning_mode = False
        self._plan_mark_nfz = False
        self._pending_plan_name: str | None = None
        self._plan_kind = "point_to_point"
        self._plan_source: LatLon | None = None
        self._plan_destination: LatLon | None = None
        self._plan_no_fly_zone: list[LatLon] = []
        self._plan_forest_corners: list[LatLon] = []

        # Two interchangeable flight sources with the same surface: the local
        # kinematic preview, and independent drone threads commanded over UDP
        # (what PDDL-planned missions fly). `flight_sim` always points at
        # whichever is selected.
        self.local_sim = LocalFlightSimulator(self)
        self.thread_sim = ThreadSwarmBackend(self)
        self.flight_sim = self.local_sim

        # A third, opt-in path for a PDDL-planned point-to-point mission: fly
        # the solved route as one real MAVLink mission against an external
        # SITL/vehicle instead of thread_sim's in-process pipeline - see the
        # Mission Planner panel's "Fly via real MAVLink" toggle and
        # `_run_planned_mission`. Not part of the flight_sim/thread_sim
        # swap above since it isn't a telemetry source the rest of the GUI
        # polls; it just reports progress/finished/failed while it runs.
        self.mavlink_flight = MavlinkFlightService(self)
        # Opt-in fourth flight source: launches the standalone Renode +
        # Pixhawk6C/6X package and, once its MAVLink port is live, hands the
        # resulting connection string to the Mission Planner panel's
        # existing "Fly via real MAVLink" field - mavlink_flight above then
        # consumes it completely unchanged.
        self.renode_launch = RenodeLaunchService(self)
        # Where the CURRENTLY booted-or-booting Renode instance's vehicle
        # actually spawns - defaults to RenodeLauncher's own confirmed
        # Canberra default, matching a manual "Launch Renode" click's
        # existing behavior exactly (start_async() below falls back to
        # this same default internally when no override is passed).
        # Changed only by _start_external_mavlink_mission() when a
        # mission's own clicked Start point genuinely differs from this
        # (see there) - _restart_renode_after_mission()'s post-mission
        # relaunch intentionally reuses whatever this currently holds
        # rather than resetting it, so relaunching after a mission flown
        # at a custom location keeps that same location instead of
        # needlessly bouncing back to Canberra for the next one.
        self._renode_target_lat = RenodeLauncher.PHYSICS_LATITUDE_DEG
        self._renode_target_lon = RenodeLauncher.PHYSICS_LONGITUDE_DEG
        # A mission waiting on a location-changing Renode relaunch
        # (triggered by _start_external_mavlink_mission when the clicked
        # Start point doesn't match _renode_target_lat/lon) to finish
        # before it can actually start flying - (PlanRunResult, drones)
        # or None. Consumed by _on_renode_ready().
        self._pending_mavlink_mission: tuple[object, list] | None = None
        # Managed by `_start_external_mavlink_mission`/`_stop_mock_vehicle`
        # when the panel's "Use built-in mock vehicle" box is checked - a
        # `scripts/mock_sitl.py` subprocess this window owns the lifetime of.
        self._mock_sitl_proc: subprocess.Popen | None = None
        # Cleared on Stop so telemetry batches already queued from a worker
        # thread when Stop was pressed can't slip through and repopulate the
        # map after everything has been torn down. Re-armed by each run start.
        self._accepting_telemetry = True

        self.drone_management = DroneManagementPanel(self.store)
        self.map_viewer = MapViewer()
        self.flight_log = FlightLogPanel()
        self.fault_injection = FaultInjectionPanel()
        self.mission_planner = MissionPlannerPanel()

        self.setCentralWidget(self._build_page())

        self._build_connection_toolbar()
        self._build_flight_source_toolbar()
        self._build_appearance_toolbar()
        self._build_status_bar()
        self._wire_signals()
        self._apply_theme_colors()
        theme_manager.theme_changed.connect(lambda _name: self._apply_theme_colors())

    # ---- Layout ----

    # Side panels cap their width and never stretch; the map (top) and the
    # Flight Log (bottom) absorb all the horizontal slack, so the page
    # always spans exactly the viewport width - no left/right scrollbar,
    # only a vertical one when the window is too short for the fixed row
    # heights below.
    SIDE_PANEL_MAX_WIDTH = 340
    PLANNER_PANEL_MAX_WIDTH = 440
    MAP_MIN_WIDTH = 260
    MAP_MIN_HEIGHT = 460
    LOWER_MIN_HEIGHT = 380

    def _build_page(self) -> QScrollArea:
        """One page, vertical scroll only: Drone Management, the map and
        Fault Injection across the top; the Flight Log beside the Mission
        Planner below. Everything is width-elastic - the row grows and
        shrinks with the window - while the map and lower blocks keep
        minimum heights so a short window scrolls instead of squashing the
        log and the planner's PDDL output out of view."""
        top_row = QHBoxLayout()
        top_row.addWidget(
            self._titled("Drone Management", self.drone_management, self.SIDE_PANEL_MAX_WIDTH)
        )
        top_row.addWidget(self.map_viewer, stretch=1)
        top_row.addWidget(
            self._titled("Fault Injection", self.fault_injection, self.SIDE_PANEL_MAX_WIDTH)
        )

        bottom_row = QHBoxLayout()
        bottom_row.addWidget(self.flight_log, stretch=1)
        bottom_row.addWidget(
            self._titled("Mission Planner", self.mission_planner, self.PLANNER_PANEL_MAX_WIDTH)
        )

        self.map_viewer.setMinimumSize(self.MAP_MIN_WIDTH, self.MAP_MIN_HEIGHT)
        self.flight_log.setMinimumWidth(self.MAP_MIN_WIDTH)
        self.flight_log.setMinimumHeight(self.LOWER_MIN_HEIGHT)

        page = QWidget()
        page_layout = QVBoxLayout(page)
        page_layout.setContentsMargins(6, 6, 6, 6)
        page_layout.setSpacing(8)
        page_layout.addLayout(top_row)
        page_layout.addLayout(bottom_row)

        scroll = QScrollArea()
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidgetResizable(True)
        # Vertical scrolling only - the layout is sized to fit the width.
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setWidget(page)
        return scroll

    @staticmethod
    def _titled(title: str, widget: QWidget, max_width: int | None = None) -> QGroupBox:
        """Wrap a panel in a titled group box - the caption the dock title
        bars used to carry. `max_width` caps how wide it can get (it still
        shrinks with the window); it never stretches to eat slack, so the
        map / Flight Log beside it take the extra space."""
        box = QGroupBox(title)
        layout = QVBoxLayout(box)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.addWidget(widget)
        if max_width is not None:
            box.setMaximumWidth(max_width)
        box.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        return box

    # ---- Setup ----

    def _build_connection_toolbar(self) -> None:
        toolbar = QToolBar("Renode UDP Connection", self)
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        self.connect_btn = QPushButton("Connect to Telemetry")
        self.connect_btn.clicked.connect(self._on_connect_clicked)

        self.sysid_combo = QComboBox()
        self.sysid_combo.addItems(["1", "2", "3"])

        self.inject_isr_btn = QPushButton("Inject Hardware ISR")
        self.inject_isr_btn.clicked.connect(self._on_inject_isr_clicked)

        self.restore_isr_btn = QPushButton("Restore State")
        self.restore_isr_btn.clicked.connect(self._on_restore_isr_clicked)

        container = QWidget()
        row = QHBoxLayout(container)
        row.setContentsMargins(4, 0, 4, 0)
        row.addWidget(self.connect_btn)
        row.addSpacing(20)
        row.addWidget(QLabel("Target SysID:"))
        row.addWidget(self.sysid_combo)
        row.addWidget(self.inject_isr_btn)
        row.addWidget(self.restore_isr_btn)
        row.addStretch()
        toolbar.addWidget(container)

    def _build_flight_source_toolbar(self) -> None:
        """Which engine drives an "Emulate"/planned mission when the Renode
        backend above is not connected: the local kinematic preview, or
        independent drone threads commanded over UDP (what PDDL-planned
        missions fly)."""
        toolbar = QToolBar("Flight Source", self)
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        self.backend_combo = QComboBox()
        self.backend_combo.addItem("Local preview", "local")
        self.backend_combo.addItem("Drone threads (UDP)", "threads")
        self.backend_combo.currentIndexChanged.connect(self._on_backend_changed)

        container = QWidget()
        row = QHBoxLayout(container)
        row.setContentsMargins(4, 0, 4, 0)
        row.addWidget(QLabel("Flight source:"))
        row.addWidget(self.backend_combo)
        toolbar.addWidget(container)

    def _build_appearance_toolbar(self) -> None:
        """App-wide preference, not a per-panel setting - kept in its own
        globally accessible toolbar rather than buried in a settings dialog."""
        toolbar = QToolBar("Appearance", self)
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        self.dark_mode_action = QAction("Dark Mode", self)
        self.dark_mode_action.setCheckable(True)
        self.dark_mode_action.setChecked(theme_manager.current_theme() == "dark")
        self.dark_mode_action.toggled.connect(
            lambda checked: theme_manager.set_theme("dark" if checked else "light")
        )
        toolbar.addAction(self.dark_mode_action)

    def _apply_theme_colors(self) -> None:
        """Refreshes everything in this window styled via a per-widget
        `setStyleSheet()` call - those don't pick up the app-level QSS
        cascade (see main.py), so they have to be redone by hand on every
        theme change."""
        palette = theme_manager.palette()
        self.inject_isr_btn.setStyleSheet(f"background-color: {palette.critical}; color: white;")
        self.restore_isr_btn.setStyleSheet(f"background-color: {palette.nominal}; color: white;")
        self._update_connection_label()
        if self.dark_mode_action.isChecked() != (theme_manager.current_theme() == "dark"):
            self.dark_mode_action.setChecked(theme_manager.current_theme() == "dark")

    def _on_backend_changed(self) -> None:
        """Swap the active flight source. Both expose the same surface, so
        nothing else in the window changes."""
        self.local_sim.stop()
        self.thread_sim.stop()
        choice = self.backend_combo.currentData()
        self.flight_sim = self.thread_sim if choice == "threads" else self.local_sim
        self.statusBar().showMessage(
            "Flight source: independent drone threads, commanded over UDP."
            if choice == "threads"
            else "Flight source: local kinematic preview."
        )

    def _build_status_bar(self) -> None:
        self.flight_label = QLabel("No flight")
        self.statusBar().addPermanentWidget(self.flight_label)

        self.connection_label = QLabel("Disconnected")
        self.statusBar().addPermanentWidget(self.connection_label)
        self.statusBar().showMessage("Ready.")

    def _wire_signals(self) -> None:
        self.drone_management.emulate_requested.connect(self._on_emulate)
        self.drone_management.stop_requested.connect(self._on_stop)
        self.drone_management.status_message.connect(self.statusBar().showMessage)

        self.map_viewer.status_message.connect(self.statusBar().showMessage)
        self.map_viewer.point_picked.connect(self._on_point_picked)

        self.fault_injection.fault_requested.connect(self._on_fault_requested)
        self.fault_injection.status_message.connect(self.statusBar().showMessage)

        self.mission_planner.plan_requested.connect(self._on_plan_requested)
        self.plan_service.finished.connect(self._on_plan_ready)
        self.plan_service.failed.connect(self._on_plan_failed)

        self.client.telemetry_received.connect(self._on_telemetry)
        self.client.connection_state_changed.connect(self._on_connection_state_changed)
        self.client.request_failed.connect(self._on_request_failed)

        # Both flight sources feed the identical telemetry path as the backend.
        for backend in (self.local_sim, self.thread_sim):
            backend.batch_ready.connect(self._on_telemetry)
            backend.progress.connect(self._on_flight_progress)
            backend.finished.connect(self._on_flight_finished)
            backend.low_battery.connect(self._on_low_battery)
            backend.drone_lost.connect(self._on_drone_lost)
            backend.gps_lost.connect(self._on_gps_lost)
            backend.gps_restored.connect(self._on_gps_restored)
        self.thread_sim.link_failed.connect(
            lambda msg: self.statusBar().showMessage(f"Drone node: {msg}", 15000)
        )

        self.mavlink_flight.batch_ready.connect(self._on_telemetry)
        self.mavlink_flight.progress.connect(self._on_mavlink_flight_progress)
        self.mavlink_flight.finished.connect(self._on_mavlink_flight_finished)
        self.mavlink_flight.failed.connect(self._on_mavlink_flight_failed)

        # Not a direct connect(self.renode_launch.start_async) - every
        # Renode (re)launch, whether a manual button click or the
        # post-mission auto-relaunch (both go through this same signal,
        # via MissionPlannerPanel._on_renode_launch_clicked()), needs to
        # spawn the vehicle at whatever location is currently intended
        # (_renode_target_lat/lon), not always the class default.
        self.mission_planner.renode_launch_requested.connect(
            lambda standalone_dir: self.renode_launch.start_async(
                standalone_dir, latitude_deg=self._renode_target_lat, longitude_deg=self._renode_target_lon,
            )
        )
        self.renode_launch.progress.connect(self.mission_planner.set_status)
        self.renode_launch.failed.connect(
            lambda msg: self.mission_planner.set_status(f"Renode launch failed: {msg}")
        )
        self.renode_launch.failed.connect(
            lambda _msg: self.mission_planner.mavlink_connection_edit.setPlaceholderText(
                "udp:127.0.0.1:14550 (SITL/mock convention - not used if launching Renode below)"
            )
        )
        # set_renode_ready(False) also clears _renode_launch_in_progress and
        # re-runs "Launch Renode"'s own gating (see its docstring) - this
        # replaces a previous direct renode_launch_btn.setEnabled(True) here
        # that didn't account for the drone-selection requirement, and
        # doesn't clobber the "Renode launch failed: ..." message set by the
        # earlier connection above (set_renode_ready only touches
        # enabled-state, not status text).
        self.renode_launch.failed.connect(lambda _msg: self.mission_planner.set_renode_ready(False))
        self.renode_launch.ready.connect(self._on_renode_ready)
        self.drone_management.selection_changed.connect(self._on_drone_selection_changed)
        self._on_drone_selection_changed()

    # ---- Connection ----

    def _on_connect_clicked(self) -> None:
        self._accepting_telemetry = True
        sysids = self.drone_management.checked_sysids()
        if not sysids:
            sysids = [1, 2]  # Default if nothing selected
        self.client.connect_telemetry(sysids)
        self.statusBar().showMessage(f"Connecting to Renode UDP telemetry for sysids: {sysids}...")

    def _on_inject_isr_clicked(self) -> None:
        sysid = int(self.sysid_combo.currentText())
        from contracts.gui_orchestration import FaultType
        self.client.inject_isr(sysid, FaultType.GPS_LOSS)
        self.statusBar().showMessage(f"Injected Hardware ISR for sysid {sysid}")

    def _on_restore_isr_clicked(self) -> None:
        sysid = int(self.sysid_combo.currentText())
        self.client.restore_isr_state(sysid)
        self.statusBar().showMessage(f"Restored State for sysid {sysid}")

    def _on_connection_state_changed(self, connected: bool) -> None:
        self._connected = connected
        if connected:
            # Real telemetry supersedes the local preview.
            self.flight_sim.stop()
        self._update_connection_label()

    def _update_connection_label(self) -> None:
        palette = theme_manager.palette()
        if self._connected:
            self.connection_label.setText("Connected")
            self.connection_label.setStyleSheet(f"color: {palette.nominal}; font-weight: bold;")
        else:
            self.connection_label.setText("Disconnected")
            self.connection_label.setStyleSheet(f"color: {palette.critical}; font-weight: bold;")

    def _on_request_failed(self, tag: str, error: str) -> None:
        self.statusBar().showMessage(f"[{tag}] request failed: {error}", 5000)

    # ---- Drone management wiring ----

    def _warn_grounded(self, grounded: list, total_count: int, distance_km: float, label: str = "leg") -> None:
        """Tell the controller which drones the battery gate kept on the
        ground, if any. Shared by the straight-leg Emulate flow and the PDDL
        mission-planner flow."""
        if not grounded:
            return
        detail = "\n".join(
            f"- {f.config.name} (SYSID {f.config.sysid}): needs "
            f"{f.required_mah:,.0f} mAh but carries {f.config.battery_capacity_mah:,.0f} mAh "
            f"- range about {f.range_m / 1000:.1f} km at {f.cruise_speed_mps:.1f} m/s"
            for f in grounded
        )
        QMessageBox.warning(
            self,
            "Not enough battery",
            f"{len(grounded)} of {total_count} drone(s) cannot complete the "
            f"{distance_km:.1f} km {label} and will not take off:\n\n{detail}",
        )

    def _on_emulate(self, drones: list) -> None:
        self._accepting_telemetry = True
        self.flight_log.log_event(
            f"Emulate started - {len(drones)} drone(s): "
            f"{', '.join(f'SYSID {d.sysid}' for d in drones)}"
        )
        for drone in drones:
            self.client.register_drone(drone)
        self.client.connect_telemetry([d.sysid for d in drones])
        self.fault_injection.update_active_sysids([d.sysid for d in drones])

        if self._connected:
            self.statusBar().showMessage(
                f"Emulating {len(drones)} drone(s). Module 2 is driving telemetry."
            )
            return

        # No orchestrator: fly the picked course locally so the run is visible.
        if self._start_point is None or self._destination_point is None:
            self.statusBar().showMessage(
                "Emulating - set a Start Point and a Destination Point on the map to see the flight."
            )
            return

        # Pre-flight battery check: a drone whose profile would draw more than
        # it carries stays on the ground; the rest fly the mission.
        flights = plan_flights(drones, self._start_point, self._destination_point)
        cleared = [f for f in flights if f.feasible]
        grounded = [f for f in flights if not f.feasible]
        distance_km = flights[0].distance_m / 1000.0

        self._warn_grounded(grounded, len(flights), distance_km)

        if not cleared:
            # Nothing launched, so unlock the configuration inputs again.
            self.drone_management.set_running(False)
            self.fault_injection.update_active_sysids([])
            self.statusBar().showMessage(
                f"Flight not started - no drone has the battery for {distance_km:.1f} km."
            )
            return

        self.fault_injection.update_active_sysids([f.config.sysid for f in cleared])
        self.map_viewer.clear_paths()

        if self.flight_sim is self.thread_sim:
            # One thread per drone, commanded over UDP by an in-process GCS.
            if not self.thread_sim.start_mission(
                [f.config for f in cleared], self._start_point, self._destination_point
            ):
                self.drone_management.set_running(False)
                self.statusBar().showMessage("Could not start drone nodes - see the log.")
                return
            self.statusBar().showMessage(
                f"Launched {len(cleared)} drone thread(s); NAVIGATE sent over UDP "
                f"({self.thread_sim.time_scale:.0f}x speed)."
            )
            return

        if not self.flight_sim.start(cleared):
            return

        altitudes = {f.config.cruise_altitude_m for f in cleared}
        altitude_text = (
            f"{next(iter(altitudes)):.0f} m" if len(altitudes) == 1 else f"{min(altitudes):.0f}-{max(altitudes):.0f} m"
        )
        grounded_text = f", {len(grounded)} grounded on battery" if grounded else ""
        self.statusBar().showMessage(
            f"Flying {len(cleared)} drone(s) {distance_km:.1f} km at {altitude_text}{grounded_text} "
            f"(local preview, {self.flight_sim.time_scale:.0f}x speed)."
        )

    def _on_flight_progress(self, elapsed_s: float, total_s: float) -> None:
        self.flight_label.setText(
            f"Flight {format_duration(elapsed_s)} / {format_duration(total_s)}"
            f"  (ETA {format_duration(total_s - elapsed_s)})"
        )

    def _on_fault_requested(self, command) -> None:
        """Faults go to the backend, and to the local preview when it is flying."""
        self.client.inject_fault(command)
        if self.flight_sim.is_active():
            affected = self.flight_sim.inject_fault(command)
            if not affected:
                self.flight_sim.resume()

    def _on_gps_lost(self, sysid: int) -> None:
        """The drone reports GPS loss and is holding station pending orders."""
        flight = next((f for f in self.flight_sim.flights if f.config.sysid == sysid), None)
        home_note = ""
        if flight is not None:
            distance_km = haversine_m(flight.position, flight.start) / 1000.0
            reach = (
                "within reach on the remaining charge"
                if flight.can_reach(flight.start)
                else "TOO FAR on the remaining charge - it would land where it stands instead"
            )
            home_note = (
                f"\n\nStart point is {distance_km:.2f} km back - {reach}."
                f"\nBattery is at {flight.battery_pct:.0f}%."
            )

        granted = QMessageBox.question(
            self,
            "GPS lost - fall back?",
            f"SYSID {sysid} reports GPS LOSS and is holding position.{home_note}\n\n"
            "Grant permission to fall back to the start point and land?\n\n"
            "Yes - abandon the mission and return to the start point.\n"
            "No  - hold position until GPS returns (the battery keeps draining).",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes,
        )

        if granted == QMessageBox.Yes:
            landing = self.flight_sim.fall_back(sysid)
            if landing is None:
                self.statusBar().showMessage(f"SYSID {sysid}: unable to fall back.")
            elif landing.in_place:
                self.statusBar().showMessage(
                    f"SYSID {sysid}: start point out of range - landing immediately at "
                    f"({landing.target.lat:.5f}, {landing.target.lon:.5f})."
                )
            else:
                self.statusBar().showMessage(
                    f"SYSID {sysid} falling back {landing.distance_m / 1000:.2f} km to the start point."
                )
        else:
            self.statusBar().showMessage(
                f"SYSID {sysid}: fall-back denied - holding position until GPS returns."
            )
        self.flight_sim.resume()

    def _on_gps_restored(self, sysid: int) -> None:
        self.statusBar().showMessage(f"SYSID {sysid}: GPS reacquired - resuming the mission.", 8000)

    def _on_low_battery(self, sysid: int, battery_pct: float) -> None:
        """Battery hit the threshold: hold the mission and ask the controller.

        The simulator has already paused itself, so the drone is not draining
        while the dialog is up - at the compressed timescale the decision
        window would otherwise be a fraction of a second.
        """
        candidates = self._landing_candidates
        nearest_note = ""
        flight = next((f for f in self.flight_sim.flights if f.config.sysid == sysid), None)
        if flight is not None and candidates:
            target = nearest_point(flight.position, candidates)
            if target is not None:
                distance_km = haversine_m(flight.position, target) / 1000.0
                reach = (
                    "within reach on the remaining charge"
                    if flight.can_reach(target)
                    else "TOO FAR on the remaining charge - it would land where it stands instead"
                )
                nearest_note = (
                    f"\n\nNearest landing point is ({target.lat:.5f}, {target.lon:.5f}), "
                    f"{distance_km:.2f} km away - {reach}."
                )

        holding = flight is not None and flight.gps_hold
        if holding:
            title = "Low battery while holding - fall back?"
            headline = (
                f"SYSID {sysid} has been holding without GPS and is down to "
                f"{battery_pct:.0f}% battery (threshold {LOW_BATTERY_PCT:.0f}%)."
            )
            deny_line = "No  - keep holding for GPS; the drone will be lost when the battery runs flat."
        else:
            title = "Low battery - emergency landing?"
            headline = (
                f"SYSID {sysid} is down to {battery_pct:.0f}% battery "
                f"(threshold {LOW_BATTERY_PCT:.0f}%)."
            )
            deny_line = "No  - press on to the destination; the drone will be lost when the battery runs flat."

        granted = QMessageBox.question(
            self,
            title,
            f"{headline}{nearest_note}\n\n"
            "Grant permission to break off and land at the nearest point?\n\n"
            "Yes - divert and land now.\n"
            f"{deny_line}",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes,
        )

        if granted == QMessageBox.Yes:
            landing = self.flight_sim.emergency_land(sysid, candidates)
            if landing is None:
                self.statusBar().showMessage(
                    f"SYSID {sysid}: no landing point available - continuing on mission."
                )
            elif landing.in_place:
                self.statusBar().showMessage(
                    f"SYSID {sysid}: no landing point within range - landing immediately at "
                    f"({landing.target.lat:.5f}, {landing.target.lon:.5f})."
                )
            else:
                self.statusBar().showMessage(
                    f"SYSID {sysid} diverting {landing.distance_m / 1000:.2f} km to emergency "
                    f"landing at ({landing.target.lat:.5f}, {landing.target.lon:.5f})."
                )
        else:
            self.statusBar().showMessage(
                f"SYSID {sysid}: emergency landing denied - pressing on until the battery is flat."
            )
        self.flight_sim.resume()

    def _on_drone_lost(self, sysid: int) -> None:
        self.statusBar().showMessage(
            f"SYSID {sysid} LOST - battery exhausted in flight, drone sacrificed.", 15000
        )

    def _on_flight_finished(self, total_s: float) -> None:
        lost = [f.config.sysid for f in self.flight_sim.flights if f.is_lost]
        landed = [f.config.sysid for f in self.flight_sim.flights if not f.is_lost]
        self.flight_label.setText(f"Flight complete - {format_duration(total_s)}")
        summary = f"Mission over. Time consumed: {format_duration(total_s)} of flight time."
        if landed:
            summary += f" Landed: {', '.join(f'SYSID {s}' for s in landed)}."
        if lost:
            summary += f" Lost to flat battery: {', '.join(f'SYSID {s}' for s in lost)}."
        self.statusBar().showMessage(summary)

    def _on_stop(self) -> None:
        self._accepting_telemetry = False
        self.flight_sim.stop()
        self.mavlink_flight.abort()
        self._stop_mock_vehicle()
        self.flight_label.setText("No flight")
        # Markers are cleared below, so the points they stood for go too.
        self._landing_candidates.clear()
        self._start_point = None
        self._destination_point = None
        self._planning_mode = False
        self._pending_plan_name = None
        self._plan_kind = "point_to_point"
        self._plan_source = None
        self._plan_destination = None
        self._plan_no_fly_zone = []
        self._plan_forest_corners = []
        self.client.stop_swarm()
        self.client.disconnect_telemetry()
        self.map_viewer.clear_markers()
        self.map_viewer.clear_drones()
        self.map_viewer.clear_paths()
        self.map_viewer.clear_restricted_area()
        self.fault_injection.update_active_sysids([])
        self.flight_log.log_event("Run stopped - swarm torn down, inputs unlocked.")
        self.statusBar().showMessage("Swarm stopped. Configuration inputs unlocked.")

    # ---- Map wiring ----

    def _on_point_picked(self, role: str, lat: float, lon: float) -> None:
        point = LatLon(lat=lat, lon=lon)
        self._landing_candidates.append(point)

        if self._planning_mode:
            # Order-driven, not role-driven: this is the Nth click of the
            # plan's sequence regardless of what the map's "Click mode"
            # dropdown says, because this code is what drives that dropdown -
            # the user never has to operate it themselves mid-sequence.
            self._handle_plan_click(point)
            return

        if role != "destination":
            self._start_point = point
            self.statusBar().showMessage(f"Start point set at ({lat:.5f}, {lon:.5f}).")
            return

        self._destination_point = point

        # A destination placed mid-mission is a diversion order: the drones
        # already airborne turn for it from wherever they are.
        if self.flight_sim.is_active():
            diverted = self.flight_sim.retarget_all(point)
            self.flight_sim.resume()
            self.statusBar().showMessage(
                f"Re-routing {diverted} drone(s) to the new destination ({lat:.5f}, {lon:.5f})."
            )
            return

        sysids = self.drone_management.checked_sysids()
        if not sysids:
            QMessageBox.information(
                self, "No active drones", "Check drones in the Drone Management panel before issuing a flock command."
            )
            return
        self.client.flock(FlockCommand(sysids=sysids, destination=point))
        self.statusBar().showMessage(
            f"Destination set for {len(sysids)} drone(s) -> ({lat:.5f}, {lon:.5f})."
        )

    # ---- Mission planner wiring ----

    def _on_plan_requested(self, plan_name: str) -> None:
        """Arm the map: the next clicks belong to this plan, not to the
        ordinary flock-command flow. A point-to-point plan collects Start
        and Destination, then (if the panel's checkbox asks for one)
        `NFZ_CORNER_COUNT` restricted-area corners; an area-coverage plan
        (see `services.plan_service.plan_kind`) collects a base point
        instead, then `FOREST_CORNER_COUNT` search-area corners."""
        self._pending_plan_name = plan_name
        self._plan_kind = plan_kind(plan_name)
        self._plan_source = None
        self._plan_destination = None
        self._plan_no_fly_zone = []
        self._plan_forest_corners = []
        self._plan_mark_nfz = self.mission_planner.mark_restricted_area()
        self._planning_mode = True
        self.map_viewer.clear_markers()
        self.map_viewer.clear_restricted_area()
        self.map_viewer.mode_combo.setCurrentText("Set Start Point")
        if self._plan_kind == "area_coverage":
            self.statusBar().showMessage(
                f"Plan '{plan_name}' armed - click the drones' base point on the map."
            )
        else:
            self.statusBar().showMessage(f"Plan '{plan_name}' armed - click the Start point on the map.")

    def _handle_plan_click(self, point: LatLon) -> None:
        """Consume one click of the armed plan's sequence, driving the map's
        Click mode as it goes so the dropdown never has to be touched by
        hand - see `_on_plan_requested` for what that sequence is."""
        if self._plan_kind == "area_coverage":
            self._handle_area_plan_click(point)
            return

        name = self._pending_plan_name

        if self._plan_source is None:
            self._plan_source = point
            self.map_viewer.mode_combo.setCurrentText("Set Destination Point")
            self.statusBar().showMessage(
                f"Plan '{name}': start set at ({point.lat:.5f}, {point.lon:.5f}) - "
                f"now click the destination."
            )
            return

        if self._plan_destination is None:
            self._plan_destination = point
            if self._plan_mark_nfz:
                self.map_viewer.mode_combo.setCurrentText("Set Restricted Area")
                self.statusBar().showMessage(
                    f"Plan '{name}': destination set - click the restricted area's "
                    f"corner 1 of {NFZ_CORNER_COUNT}."
                )
                return
            self._planning_mode = False
            self._run_planned_mission()
            return

        # Collecting the restricted-area's corners.
        self._plan_no_fly_zone.append(point)
        self.map_viewer.set_restricted_area(self._plan_no_fly_zone)
        collected = len(self._plan_no_fly_zone)
        if collected < NFZ_CORNER_COUNT:
            self.statusBar().showMessage(
                f"Plan '{name}': restricted-area corner {collected} of {NFZ_CORNER_COUNT} set - "
                f"click the next corner."
            )
            return

        self._planning_mode = False
        self._run_planned_mission()

    def _handle_area_plan_click(self, point: LatLon) -> None:
        """Consume one click of an armed area-coverage plan's sequence: the
        drones' shared base point, then `FOREST_CORNER_COUNT` corners of the
        area to search - reusing the same corner-by-corner overlay the
        restricted-area flow uses (see `map_viewer.set_restricted_area`)."""
        name = self._pending_plan_name

        if self._plan_source is None:
            self._plan_source = point  # the base drones launch from and return to
            self.map_viewer.mode_combo.setCurrentText("Set Forest Area")
            self.statusBar().showMessage(
                f"Plan '{name}': base set at ({point.lat:.5f}, {point.lon:.5f}) - "
                f"now click the search area's corner 1 of {FOREST_CORNER_COUNT}."
            )
            return

        self._plan_forest_corners.append(point)
        self.map_viewer.set_restricted_area(self._plan_forest_corners)
        collected = len(self._plan_forest_corners)
        if collected < FOREST_CORNER_COUNT:
            self.statusBar().showMessage(
                f"Plan '{name}': search-area corner {collected} of {FOREST_CORNER_COUNT} set - "
                f"click the next corner."
            )
            return

        self._planning_mode = False
        self._run_planned_mission()

    def _run_planned_mission(self) -> None:
        if self._plan_source is None or self._pending_plan_name is None:
            return
        if self._plan_kind == "area_coverage":
            if len(self._plan_forest_corners) < FOREST_CORNER_COUNT:
                return
        elif self._plan_destination is None:
            return
        checked = self.drone_management.checked_drones()
        if not checked:
            QMessageBox.information(
                self, "No active drones",
                "Check drones in the Drone Management panel before planning a mission.",
            )
            return
        if self._plan_kind == "area_coverage" and len(checked) < 2:
            QMessageBox.information(
                self, "Area search needs at least 2 drones",
                "An area search splits the marked area into one lane per drone and "
                "keeps the drones separated, so it needs at least 2 checked drones. "
                "For a single drone, use the 'travell' plan instead.",
            )
            return
        self.mission_planner.set_status(f"Running ENHSP for plan '{self._pending_plan_name}'...")
        self.statusBar().showMessage(f"Running ENHSP for plan '{self._pending_plan_name}'...")
        if self._plan_kind == "area_coverage":
            # One lane (and one PDDL drone) per checked drone, so the marked
            # area is split across however many drones the swarm has.
            self.plan_service.run_area_async(
                self._pending_plan_name,
                self._plan_source,
                self._plan_forest_corners,
                drone_count=len(self.drone_management.checked_drones()),
            )
        else:
            # One PDDL drone per checked drone, so the solved plan sends the
            # whole swarm to the same destination together.
            self.plan_service.run_async(
                self._pending_plan_name,
                self._plan_source,
                self._plan_destination,
                self._plan_no_fly_zone,
                drone_count=len(self.drone_management.checked_drones()),
            )

    def _on_plan_ready(self, result: PlanRunResult) -> None:
        self._accepting_telemetry = True
        self.mission_planner.set_plan_steps(result.plan_name, result.steps)

        # Mirror ENHSP's solved action sequence into the log alongside the
        # travel lines, so the whole run reads top to bottom in one place.
        self.flight_log.log_event(f"Plan '{result.plan_name}' ready - {len(result.steps)} step(s):")
        for step in sorted(result.steps, key=lambda s: s.index):
            self.flight_log.log_line(f"    {step}")

        drones = self.drone_management.checked_drones()
        if not drones:
            self.mission_planner.set_status(f"Plan '{result.plan_name}' ready, but no drones are checked.")
            self.statusBar().showMessage("Plan ready, but no drones are checked to fly it.")
            return

        if result.per_drone_waypoints is not None:
            noun = "slot" if self._plan_kind == "formation" else "lane"
            self._start_area_coverage_mission(result, drones, route_noun=noun)
            return

        if self.mission_planner.use_external_mavlink():
            self._start_external_mavlink_mission(result, drones)
            return

        flights = plan_route_flights(drones, result.waypoints)
        cleared = [f for f in flights if f.feasible]
        grounded = [f for f in flights if not f.feasible]
        distance_km = flights[0].distance_m / 1000.0 if flights else 0.0
        route_text = " -> ".join(result.location_names)

        self._warn_grounded(grounded, len(flights), distance_km, label="route")

        if not cleared:
            self.mission_planner.set_status(
                f"Plan '{result.plan_name}' ({route_text}, {distance_km:.1f} km): "
                f"no drone has the battery for it."
            )
            self.statusBar().showMessage(
                f"Plan '{result.plan_name}' ready, but no drone has the battery for the {distance_km:.1f} km route."
            )
            return

        # A planned mission always flies the drone-thread pipeline: the plan's
        # waypoints are commanded over UDP to one thread per drone.
        threads_index = self.backend_combo.findData("threads")
        if threads_index >= 0 and self.backend_combo.currentIndex() != threads_index:
            self.backend_combo.setCurrentIndex(threads_index)

        self.drone_management.set_running(True)
        self.fault_injection.update_active_sysids([f.config.sysid for f in cleared])
        self.map_viewer.clear_paths()

        if not self.thread_sim.start_mission_path([f.config for f in cleared], result.waypoints):
            self.drone_management.set_running(False)
            self.statusBar().showMessage("Could not start drone nodes for the planned mission - see the log.")
            return

        self.mission_planner.set_status(
            f"Flying plan '{result.plan_name}' ({route_text}, {distance_km:.1f} km) "
            f"with {len(cleared)} drone(s)."
        )
        self.statusBar().showMessage(
            f"Flying {len(cleared)} drone(s) on plan '{result.plan_name}': "
            f"{distance_km:.1f} km via {max(0, len(result.waypoints) - 2)} waypoint(s)."
        )
        self._pending_plan_name = None
        self._plan_source = None

    # Renode's own reported location is only ever accurate to however
    # precisely a click can be made and read back - a real relaunch is
    # expensive (~75-160s), so two points within this real, if
    # judgment-call, distance count as "the same place" rather than
    # requiring exact equality. ~0.001 degrees is roughly 100m at this
    # latitude: materially the same takeoff area, but small enough that a
    # genuinely different real Start point (even just a few hundred
    # metres away) still triggers a relaunch.
    _RENODE_LOCATION_MATCH_TOLERANCE_DEG = 0.001

    def _renode_location_matches(self, point: LatLon) -> bool:
        return (
            abs(point.lat - self._renode_target_lat) <= self._RENODE_LOCATION_MATCH_TOLERANCE_DEG
            and abs(point.lon - self._renode_target_lon) <= self._RENODE_LOCATION_MATCH_TOLERANCE_DEG
        )

    def _start_external_mavlink_mission(self, result: PlanRunResult, drones: list) -> None:
        """Fly the solved route as one real MAVLink mission against an
        external SITL/vehicle (see MissionPlannerPanel's "Fly via real
        MAVLink" toggle), instead of thread_sim's in-process drone-thread
        pipeline. A point-to-point plan has exactly one shared route
        regardless of how many drones are checked, so this always flies one
        external connection - it isn't "one per checked drone".

        Skips the internal battery-feasibility check `_on_plan_ready`
        otherwise runs (`plan_route_flights`): that model is this app's own
        `DroneConfig` battery curve, which has nothing to do with whatever
        vehicle is actually listening on the far end of `connection`.

        For a real (non-mock) vehicle, a mission's Start point can now
        actually change where the vehicle takes off from - but only via a
        fresh Renode relaunch at that location (RenodeLauncher.start()
        parametrizes physics Connect's lat/lon per-instance; there is no
        live-relocation path). If this mission's Start point doesn't match
        wherever the current/intended instance already is, the mission is
        stashed in `_pending_mavlink_mission` and only actually starts once
        `_on_renode_ready()` sees that relaunch complete - see
        `_renode_location_matches()`. `renode_launch.stop()` is called
        FIRST, before triggering that relaunch - confirmed necessary by a
        real failed validation attempt (renode_firmware_guide.md §4.9.1):
        relying on RenodeLauncher.start()'s own defensive
        `_kill_any_stale_processes()` alone, without this explicit
        synchronous stop first, left a real race where the still-alive
        previous instance was still there when the new one tried to
        bind/connect, producing an infinite "EOF on TCP socket" loop that
        never reached ready(). Mirrors _restart_renode_after_mission()'s
        already-validated stop()-then-relaunch order exactly.
        """
        connection = self.mission_planner.mavlink_connection_string()
        if not connection:
            message = "Enter a MAVLink connection string first, e.g. udp:127.0.0.1:14550."
            self.mission_planner.set_status(message)
            self.statusBar().showMessage(message, 10000)
            return

        source = result.waypoints[0]  # the exact point this route was solved from

        if not self.mission_planner.use_mock_vehicle() and not self._renode_location_matches(source):
            self._pending_mavlink_mission = (result, drones)
            self._renode_target_lat = source.lat
            self._renode_target_lon = source.lon
            message = (
                f"Start point changed - relaunching Renode at ({source.lat:.5f}, "
                f"{source.lon:.5f}) before this mission can fly (a fresh boot, ~75-160s)..."
            )
            self.mission_planner.set_status(message)
            self.statusBar().showMessage(message, 10000)
            # Permanent record, not just the transient status label -
            # same reasoning as the flight_log fix a round ago: a launch
            # already in progress emits its own progress messages
            # (RenodeLaunchService.progress -> mission_planner.set_status)
            # that would otherwise clobber this within milliseconds.
            self.flight_log.log_event(message)
            # stop() FIRST - see this method's own docstring for why this
            # is load-bearing, not optional (§4.9.1's real failure).
            self.renode_launch.stop()
            # Reuses the exact gating/launch machinery a manual "Launch
            # Renode" click already goes through - set_renode_ready(False),
            # _renode_launch_in_progress, the Connect placeholder update,
            # and the real start_async() call (now using the just-updated
            # _renode_target_lat/lon via the wiring in _wire_signals()).
            self.mission_planner._on_renode_launch_clicked()
            return

        self._fly_external_mavlink_mission(result, drones, connection)

    def _fly_external_mavlink_mission(self, result: PlanRunResult, drones: list, connection: str) -> None:
        """The part of `_start_external_mavlink_mission()` that actually
        starts the flight - split out so `_on_renode_ready()` can resume
        a mission that was deferred for a location-changing relaunch
        (`_pending_mavlink_mission`) without duplicating any of this."""
        route_text = " -> ".join(result.location_names)
        altitude_m = drones[0].cruise_altitude_m
        sysid = drones[0].sysid
        source = result.waypoints[0]

        if self.mission_planner.use_mock_vehicle():
            if not self._start_mock_vehicle(connection, source):
                return  # status/console already explain why

        self.drone_management.set_running(True)
        # Drop any marker/trail left by a previous run so this mission's first
        # fix appears straight at its own source instead of the icon gliding
        # across the map from wherever the last one ended.
        self.map_viewer.clear_drones()
        self.map_viewer.clear_paths()
        # "Plan Mission"/the plan combo must stay disabled for the WHOLE
        # flight, not just the pre-flight boot wait set_renode_ready(False)
        # already covered - otherwise a second "Plan Mission" completed
        # while this one is still airborne would call
        # mavlink_flight.run_async() a second time on the same connection,
        # and/or land in the middle of _restart_renode_after_mission()'s
        # own relaunch window once this flight ends, double-triggering or
        # racing it. Re-enabled the same way the very first launch is:
        # _on_renode_ready() fires once _restart_renode_after_mission()'s
        # post-flight relaunch completes (see that method - every flight
        # end, including this one's, already triggers a relaunch) - so
        # there is no separate re-enable call needed here.
        self.mission_planner.set_renode_ready(False)
        # "Launch Renode" needs its OWN, separate flag rather than reusing
        # the line above - _renode_ready is False both while this mission
        # is flying AND after a genuine failure/disconnect with nothing
        # flying, but those need opposite gating: a live, airborne vehicle
        # must not be killed out from under an active flight, while a
        # failed/disconnected one must still allow "Launch Renode" to
        # recover. Cleared alongside the post-flight relaunch trigger, in
        # _restart_renode_after_mission().
        self.mission_planner.set_mission_active(True)
        status = f"Plan '{result.plan_name}' ({route_text}): connecting to {connection} ..."
        self.mission_planner.set_status(status)
        self.statusBar().showMessage(
            f"Flying plan '{result.plan_name}' via real MAVLink at {connection}."
        )
        print(f"[MAVLink] Plan '{result.plan_name}' ({route_text}): connecting to {connection} ...")
        self.mavlink_flight.run_async(result.waypoints, connection, altitude_m, sysid)
        self._pending_plan_name = None
        self._plan_source = None

    @staticmethod
    def _parse_udp_connection(connection: str) -> tuple[str, int] | None:
        """`"udp:host:port"` -> `(host, port)`, or None for anything else
        (tcp:/serial device/other transport) - the mock vehicle is only ever
        launched over UDP, matching what `engine.mavlink_mission` expects."""
        match = re.match(r"^udp:([^:]+):(\d+)$", connection.strip())
        return (match.group(1), int(match.group(2))) if match else None

    def _start_mock_vehicle(self, connection: str, source: LatLon) -> bool:
        """Launch `scripts/mock_sitl.py`, seeded with the plan's actual
        source point, so no one has to hand-run it or type --home themselves.
        Its own stdout/stderr are left un-redirected, so its prints land in
        this same console alongside ours."""
        parsed = self._parse_udp_connection(connection)
        if parsed is None:
            message = (
                f"Mock vehicle needs a udp:host:port connection string, got '{connection}'."
            )
            self.mission_planner.set_status(message)
            self.statusBar().showMessage(message, 10000)
            return False
        _host, port = parsed

        self._stop_mock_vehicle()
        script = Path(__file__).resolve().parent.parent / "scripts" / "mock_sitl.py"
        args = [
            sys.executable, str(script),
            "--port", str(port),
            "--home", f"{source.lat},{source.lon}",
        ]
        print(f"[MAVLink] Launching mock vehicle: {' '.join(args)}")
        try:
            self._mock_sitl_proc = subprocess.Popen(args, cwd=str(script.parent.parent))
        except OSError as exc:
            message = f"Could not launch mock vehicle: {exc}"
            self.mission_planner.set_status(message)
            self.statusBar().showMessage(message, 10000)
            return False
        return True

    def _stop_mock_vehicle(self) -> None:
        proc, self._mock_sitl_proc = self._mock_sitl_proc, None
        if proc is not None and proc.poll() is None:
            print("[MAVLink] Stopping mock vehicle.")
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()

    def _on_mavlink_flight_progress(self, message: str) -> None:
        print(f"[MAVLink] {message}")
        if not self._accepting_telemetry:
            return  # the run was stopped; don't clobber the "stopped" status
        self.mission_planner.set_status(message)
        self.statusBar().showMessage(message, 5000)

    def _on_mavlink_flight_finished(self) -> None:
        self._stop_mock_vehicle()
        if not self._accepting_telemetry:
            # Worker unwound because of a Stop - `_on_stop` already reported it.
            self._restart_renode_after_mission("Mission stopped.")
            return
        self.drone_management.set_running(False)
        self.statusBar().showMessage("External MAVLink mission complete.", 8000)
        print("[MAVLink] Mission complete.")
        self._restart_renode_after_mission("External MAVLink mission complete.")

    def _restart_renode_after_mission(self, reason: str) -> None:
        """Every "Plan Mission" flight over real MAVLink - success,
        failure, or a user-initiated Stop, all three funnel here (see
        call sites) - leaves the SAME live Renode vehicle running
        afterward, in whatever EKF3 state it ended in. Confirmed by
        reading the actual code: neither engine/mavlink_mission.py's
        abort/failure paths nor services/mavlink_flight_service.py's
        worker (both off-limits to edit) ever send an RTL/land/disarm/
        reboot command to the vehicle - "Mission stopped."/"mission
        failed" are purely local GCS-side messages. A real in-place
        firmware reboot was already tried and rejected for a different,
        already-documented reason (engine/renode_launcher.py's own
        comments: the GPS peripheral gets stuck renegotiating
        configuration indefinitely after a mid-session reboot) - so the
        only genuinely clean-state guarantee available is a fresh Renode
        *process*, exactly like a manual "Launch Renode" click produces.

        Every mission end triggers this, not just failures - a flight
        that lands without an explicit failure isn't reliable evidence of
        an uncorrupted internal EKF state either (a clean-looking landing
        tonight still logged transient EKF events), so there is no safe
        way to tell "reuse this" from "reset this" short of always
        resetting.

        Reuses the exact machinery a manual re-launch already goes
        through - MissionPlannerPanel._on_renode_launch_clicked() (resets
        set_renode_ready(False), marks a launch in-progress, updates the
        Connect placeholder, and triggers the real start_async() via the
        existing renode_launch_requested wiring) - rather than
        duplicating any of that gating here."""
        self.mission_planner.set_status(
            f"{reason} Restarting Renode for a clean vehicle state "
            "(EKF/GPS state is not reset between missions otherwise) - "
            "this normally takes 75-160s, same as the first launch."
        )
        self.statusBar().showMessage(
            "Restarting Renode for a clean vehicle state before the next mission...", 10000
        )
        # Cleared BEFORE triggering the relaunch below, not after: this
        # flight has genuinely ended by this point, so "Launch Renode"'s
        # own gating (_update_renode_launch_gating(), re-evaluated inside
        # _on_renode_launch_clicked() below) needs to see mission_active
        # already False, or it would report the wrong reason (still
        # "mission active") for the disabled state the in-progress launch
        # itself is about to set for a different, now-correct reason.
        self.mission_planner.set_mission_active(False)
        self.renode_launch.stop()
        self.mission_planner._on_renode_launch_clicked()

    def _on_drone_selection_changed(self) -> None:
        # "Launch Renode" requires a drone actually checked in Drone
        # Configuration Profiles first - checked_sysids() reflects the
        # live checked set right now, same source _on_connect_clicked and
        # _on_emulate already trust for "which drones are selected".
        # Called once at construction time too (see the wiring above) so a
        # fresh app with nothing checked starts with the button correctly
        # disabled, not just after the first checkbox click.
        self.mission_planner.set_drone_selected(bool(self.drone_management.checked_sysids()))

    def _on_renode_ready(self, connection_string: str) -> None:
        self.mission_planner.mavlink_connection_edit.setText(connection_string)
        # set_renode_ready() enables "Plan Mission"/the plan combo and
        # "Launch Renode" (once a drone is selected) and sets its own
        # generic ready status - overwritten right after with the more
        # specific connection-string message, which is more useful and
        # was the original wording here.
        self.mission_planner.set_renode_ready(True)
        self.mission_planner.set_status(f"Renode ready - connection set to {connection_string}.")
        # A real Renode/ArduPilot vehicle's GPS reports its own true
        # physical location - found and fixed during an earlier
        # verification pass: nothing ever panned the map there, so a real
        # flight's telemetry updated the drone model correctly but the
        # marker rendered far outside whatever the map's current viewport
        # happened to be - not broken, just invisible. Panning here, as
        # soon as the real location is known, means the vehicle is
        # visible from before a mission even starts, not just once
        # telemetry happens to arrive. Uses _renode_target_lat/lon (what
        # this specific instance was actually launched at - Canberra by
        # default, or a mission's own Start point once the location-
        # matching relaunch below has run at least once), not the
        # RenodeLauncher class constants directly - those are only the
        # default a launch falls back to when no override is given.
        self.map_viewer.pan_to(self._renode_target_lat, self._renode_target_lon, 15)

        # Resume a mission that was waiting on this exact relaunch (see
        # _start_external_mavlink_mission's location-matching check) -
        # the new instance just booted at _renode_target_lat/lon, which
        # is what that mission's own Start point needed.
        if self._pending_mavlink_mission is not None:
            result, drones = self._pending_mavlink_mission
            self._pending_mavlink_mission = None
            connection = self.mission_planner.mavlink_connection_string()
            self._fly_external_mavlink_mission(result, drones, connection)

    def _on_mavlink_flight_failed(self, message: str) -> None:
        self.drone_management.set_running(False)
        self.statusBar().showMessage(f"External MAVLink mission failed: {message}", 15000)
        print(f"[MAVLink] FAILED: {message}")
        self._stop_mock_vehicle()
        # A mid-session MAVLink failure means the connection this
        # session's Renode instance provided is no longer trustworthy -
        # _restart_renode_after_mission() re-disables "Plan Mission" (via
        # the same set_renode_ready(False) requirement C.5 already relied
        # on) AND now actually replaces the dead/corrupted instance,
        # rather than just leaving Plan Mission disabled with no way
        # forward except a manual re-launch.
        self._restart_renode_after_mission(f"External MAVLink mission failed: {message}")

    def _start_area_coverage_mission(
        self, result: PlanRunResult, drones: list, *, route_noun: str = "lane"
    ) -> None:
        """Assign each checked drone to one of the plan's per-drone routes,
        in order (the first checked drone flies `drone1`'s lane, the second
        `drone2`'s, ...), and fly them concurrently - every lane needs its
        own drone, so extra checked drones beyond the number of lanes simply
        sit this mission out, and fewer checked drones than lanes means it
        can't run at all yet.

        `route_noun` is just what these per-drone routes are called in the
        status line - "lane" for an area-coverage sweep, "slot" for a
        V-formation's apex/left/right tracks; the mechanism is identical."""
        routes = list(result.per_drone_waypoints.values())
        if len(drones) < len(routes):
            self.mission_planner.set_status(
                f"Plan '{result.plan_name}' needs {len(routes)} drone(s), "
                f"but only {len(drones)} are checked."
            )
            self.statusBar().showMessage(
                f"Check at least {len(routes)} drones before running plan '{result.plan_name}'."
            )
            return

        assignments = list(zip(drones, routes))
        flights = plan_route_flights_by_drone(assignments)
        cleared_sysids = {f.config.sysid for f in flights if f.feasible}
        grounded = [f for f in flights if not f.feasible]
        cleared_assignments = [(d, r) for d, r in assignments if d.sysid in cleared_sysids]
        total_distance_km = sum(f.distance_m for f in flights) / 1000.0

        self._warn_grounded(grounded, len(flights), total_distance_km, label="combined route")

        if len(cleared_assignments) < len(routes):
            self.mission_planner.set_status(
                f"Plan '{result.plan_name}': not every drone has the battery for its {route_noun}."
            )
            self.statusBar().showMessage(
                f"Plan '{result.plan_name}' ready, but not every drone has the battery for its {route_noun}."
            )
            return

        threads_index = self.backend_combo.findData("threads")
        if threads_index >= 0 and self.backend_combo.currentIndex() != threads_index:
            self.backend_combo.setCurrentIndex(threads_index)

        self.drone_management.set_running(True)
        self.fault_injection.update_active_sysids([d.sysid for d, _ in cleared_assignments])
        self.map_viewer.clear_paths()

        if not self.thread_sim.start_mission_paths(cleared_assignments):
            self.drone_management.set_running(False)
            self.statusBar().showMessage("Could not start drone nodes for the planned mission - see the log.")
            return

        self.mission_planner.set_status(
            f"Flying plan '{result.plan_name}' - {len(cleared_assignments)} drone(s), "
            f"{len(routes)} {route_noun}(s), {total_distance_km:.1f} km combined."
        )
        self.statusBar().showMessage(
            f"Flying {len(cleared_assignments)} drone(s) on plan '{result.plan_name}' "
            f"({len(routes)} {route_noun}(s))."
        )
        self._pending_plan_name = None
        self._plan_source = None

    def _on_plan_failed(self, message: str) -> None:
        plan_name = self._pending_plan_name or "?"
        self.mission_planner.set_error(f"Plan '{plan_name}' failed:\n\n{message}")
        self.mission_planner.set_status("Planning failed - see the log below.")
        QMessageBox.warning(self, "Mission planning failed", message)
        self.statusBar().showMessage("Mission planning failed - see the dialog.", 8000)

    # ---- Shutdown ----

    def closeEvent(self, event) -> None:
        # Pre-existing bug found and fixed during a verification pass (not
        # introduced tonight): three services (PlanService,
        # MavlinkFlightService, RenodeLaunchService) each start their own
        # QThread eagerly in __init__, and each already has a correct
        # shutdown()/stop() that quits it properly - none of the three were
        # actually being called here. Reproduced directly as a real crash
        # on window close ("QThread: Destroyed while thread '' is still
        # running", SIGABRT) that survived fixing renode_launch alone, and
        # again after also fixing mavlink_flight alone - only went away
        # once all three were covered.
        self.plan_service.shutdown()
        self.mavlink_flight.shutdown()
        self.renode_launch.stop()
        super().closeEvent(event)

    # ---- Telemetry wiring ----

    def _on_telemetry(self, batch) -> None:
        if not self._accepting_telemetry:
            # A run was stopped; this batch was already in the queue from a
            # worker thread. Dropping it keeps the torn-down map clear.
            return
        self.map_viewer.update_drones(batch.drones)
        self.flight_log.log_batch(self._glide_matched_telemetry(batch.drones))
        self.fault_injection.update_active_sysids([d.sysid for d in batch.drones])

    def _glide_matched_telemetry(self, drones: list) -> list:
        """The Flight Log reads the same glide-smoothed lat/lon/altitude the
        map marker is drawn at, not the raw telemetry batch directly -
        otherwise the numbers race ahead of
        (or lag behind) whatever the map's Speed slider makes the marker
        visually do, which reads as the two being unrelated. Purely
        cosmetic: `batch.drones` itself - the flight-path trail, battery/
        fault logic, everything else - still sees the true, untouched fix."""
        model = self.map_viewer.drone_model
        shown = []
        for drone in drones:
            position = model.displayed_position(drone.sysid)
            if position is None:
                shown.append(drone)
                continue
            lat, lon, altitude_m = position
            shown.append(drone.model_copy(update={"lat": lat, "lon": lon, "altitude_m": altitude_m}))
        return shown
