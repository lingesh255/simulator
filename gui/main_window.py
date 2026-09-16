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
from gui.drone_management import DroneManagementPanel
from gui.fault_injection import FaultInjectionPanel
from gui.flight_log_panel import FlightLogPanel
from gui.map_viewer import MapViewer
from gui.mission_planner_panel import MissionPlannerPanel
from services.api_client import OrchestrationClient
from services.mavlink_flight_service import MavlinkFlightService
from services.plan_service import MIN_GRID_DRONES, PlanRunResult, PlanService, plan_kind
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
        self._build_status_bar()
        self._wire_signals()

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
        self.inject_isr_btn.setStyleSheet("background-color: #e74c3c; color: white;")
        self.inject_isr_btn.clicked.connect(self._on_inject_isr_clicked)

        self.restore_isr_btn = QPushButton("Restore State")
        self.restore_isr_btn.setStyleSheet("background-color: #2ecc71; color: white;")
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
        self.connection_label.setStyleSheet("color: #e74c3c; font-weight: bold;")
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
            self.connection_label.setText("Connected")
            self.connection_label.setStyleSheet("color: #2ecc71; font-weight: bold;")
        else:
            self.connection_label.setText("Disconnected")
            self.connection_label.setStyleSheet("color: #e74c3c; font-weight: bold;")

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
        sysid_counts: dict[int, list[str]] = {}
        for d in checked:
            sysid_counts.setdefault(d.sysid, []).append(d.name)
        duplicate_sysids = {sysid: names for sysid, names in sysid_counts.items() if len(names) > 1}
        if duplicate_sysids:
            # The flight backend addresses every drone by SYSID (it's the UDP
            # link identity - see services.thread_backend._spawn_drone), so
            # two checked drones sharing one collapse onto the same node: the
            # second's link bind fails and it's silently dropped from the
            # swarm rather than flying its own route.
            detail = "; ".join(
                f"SYSID {sysid}: {', '.join(names)}" for sysid, names in duplicate_sysids.items()
            )
            QMessageBox.information(
                self, "Duplicate drone SYSIDs",
                "Checked drones must each have a unique SYSID - drones sharing one "
                "collapse onto the same flight node instead of flying separately, "
                "which silently shrinks the swarm.\n\n"
                f"{detail}\n\n"
                "Edit the affected profiles in Drone Management and give each a "
                "distinct SYSID.",
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
        if self._plan_kind == "formation" and (len(checked) < 3 or len(checked) % 2 == 0):
            QMessageBox.information(
                self, "V-formation needs an odd number of drones",
                "A V-formation has one drone at the apex and the rest split evenly "
                "between both wings, so it needs an odd number of checked drones "
                f"(3, 5, 7, ...). {len(checked)} are currently checked.",
            )
            return
        if self._plan_kind == "grid_formation" and len(checked) < MIN_GRID_DRONES:
            QMessageBox.information(
                self, "Grid formation needs more drones",
                "A grid formation needs at least "
                f"{MIN_GRID_DRONES} checked drones to read as a 2-D grid rather than "
                f"a single line. {len(checked)} are currently checked.\n\n"
                "Any count from there works - a perfect square (4, 9, 16, ...) forms "
                "a square grid, and any other count forms the closest rectangle that "
                "fits it exactly (e.g. 8 -> 2x4, 12 -> 3x4).",
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
            if self._plan_kind in ("formation", "grid_formation"):
                noun = "slot"
            else:
                noun = "lane"
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
        """
        connection = self.mission_planner.mavlink_connection_string()
        if not connection:
            message = "Enter a MAVLink connection string first, e.g. udp:127.0.0.1:14550."
            self.mission_planner.set_status(message)
            self.statusBar().showMessage(message, 10000)
            return

        route_text = " -> ".join(result.location_names)
        altitude_m = drones[0].cruise_altitude_m
        sysid = drones[0].sysid
        source = result.waypoints[0]  # the exact point this route was solved from

        if self.mission_planner.use_mock_vehicle():
            if not self._start_mock_vehicle(connection, source):
                return  # status/console already explain why

        self.drone_management.set_running(True)
        # Drop any marker/trail left by a previous run so this mission's first
        # fix appears straight at its own source instead of the icon gliding
        # across the map from wherever the last one ended.
        self.map_viewer.clear_drones()
        self.map_viewer.clear_paths()
        self.mission_planner.set_status(
            f"Plan '{result.plan_name}' ({route_text}): connecting to {connection} ..."
        )
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
            return
        self.drone_management.set_running(False)
        self.mission_planner.set_status("External MAVLink mission complete.")
        self.statusBar().showMessage("External MAVLink mission complete.", 8000)
        print("[MAVLink] Mission complete.")

    def _on_mavlink_flight_failed(self, message: str) -> None:
        self.drone_management.set_running(False)
        self.mission_planner.set_status(f"External MAVLink mission failed: {message}")
        self.statusBar().showMessage(f"External MAVLink mission failed: {message}", 15000)
        print(f"[MAVLink] FAILED: {message}")
        self._stop_mock_vehicle()

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
        V-formation's apex/wing tracks or a grid formation's leader/member
        tracks (however many drones the mission has); the mechanism is
        identical."""
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
