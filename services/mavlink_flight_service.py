"""GUI-facing: fly an already-solved plan route against a *real* external
MAVLink endpoint (SITL or actual hardware) instead of this repo's in-process
`DroneThread`/`SimulatedFlightController` pipeline (`services.thread_backend`).

    PlanRunResult.waypoints (from services.plan_service)
        -> engine.mavlink_mission.upload_and_fly
        -> a real pymavlink connection
        -> the vehicle's own GLOBAL_POSITION_INT/SYS_STATUS/GPS_RAW_INT/
           HEARTBEAT telemetry, turned back into a SwarmTelemetryBatch so
           the map/telemetry dashboard animate from genuine MAVLink state,
           the same way they do for `ThreadSwarmBackend`.

Runs on its own worker thread - `upload_and_fly` blocks on network I/O for the
whole flight - and reports back through Qt signals, the same shape as
`PlanService`/`ThreadSwarmBackend`'s workers. This is strictly opt-in (see the
Mission Planner panel's "Fly via real MAVLink" toggle): the default "Plan
Mission" behaviour is unchanged, and this path requires a real vehicle/SITL
already listening on the given connection string, or it reports a clear
failure rather than hanging the GUI.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

from PySide6.QtCore import QObject, QThread, Signal, Slot
from pymavlink.dialects.v20 import ardupilotmega as mav2

from contracts.gui_orchestration import (
    DroneStatus,
    DroneTelemetry,
    FaultType,
    LatLon,
    SwarmTelemetryBatch,
)
from engine.mavlink_mission import FlightAborted, upload_and_fly


@dataclass
class _VehicleState:
    """What's known about the external vehicle so far, updated as its own
    MAVLink messages arrive - nothing here is guessed or simulated."""

    lat: float
    lon: float
    altitude_m: float = 0.0
    heading_deg: float = 0.0
    ground_speed_mps: float = 0.0
    battery_pct: float = 100.0
    armed: bool = False
    gps_ok: bool = True
    custom_mode: int = 0
    have_position: bool = False


class _Worker(QObject):
    batch_ready = Signal(object)
    progress = Signal(str)
    finished = Signal()
    failed = Signal(str)

    def __init__(self):
        super().__init__()
        self._abort = False
        self._master = None

    def request_abort(self) -> None:
        self._abort = True

    @Slot(list, str, float, int)
    def run(
        self, waypoints: list[LatLon], connection_string: str, altitude_m: float, sysid: int
    ) -> None:
        self._abort = False
        state = _VehicleState(lat=waypoints[0].lat, lon=waypoints[0].lon)
        start_time = time.monotonic()
        tick = 0

        def emit_batch() -> None:
            nonlocal tick
            tick += 1
            payload = DroneTelemetry(
                sysid=sysid,
                status=DroneStatus.IN_FLIGHT if state.armed else DroneStatus.LANDED,
                lat=state.lat,
                lon=state.lon,
                altitude_m=state.altitude_m,
                heading_deg=state.heading_deg,
                ground_speed_mps=state.ground_speed_mps,
                battery_pct=max(0.0, min(100.0, state.battery_pct)),
                link_quality_pct=100.0,  # we're receiving telemetry at all, so the link is up
                active_faults=[] if state.gps_ok else [FaultType.GPS_LOSS],
                raw_mavlink=(
                    f"[external MAVLink] armed={state.armed} mode={state.custom_mode} "
                    f"pos=({state.lat:.6f},{state.lon:.6f},{state.altitude_m:.1f}) "
                    f"batt={state.battery_pct:.1f}%"
                ),
            )
            self.batch_ready.emit(
                SwarmTelemetryBatch(tick=tick, timestamp=time.monotonic() - start_time, drones=[payload])
            )

        def handle_message(message) -> None:
            kind = message.get_type()
            if kind == "GLOBAL_POSITION_INT":
                state.lat = message.lat / 1e7
                state.lon = message.lon / 1e7
                state.altitude_m = message.relative_alt / 1000.0
                state.heading_deg = (message.hdg / 100.0) % 360.0
                state.ground_speed_mps = math.hypot(message.vx, message.vy) / 100.0
                state.have_position = True
                emit_batch()  # position is what actually moves the map marker
            elif kind == "SYS_STATUS":
                if message.battery_remaining >= 0:
                    state.battery_pct = float(message.battery_remaining)
            elif kind == "GPS_RAW_INT":
                state.gps_ok = message.fix_type >= 2
            elif kind == "HEARTBEAT":
                state.armed = bool(message.base_mode & mav2.MAV_MODE_FLAG_SAFETY_ARMED)
                state.custom_mode = message.custom_mode

        try:
            from pymavlink import mavutil

            self.progress.emit(f"Connecting to {connection_string} ...")
            self._master = mavutil.mavlink_connection(connection_string)
            points = [(p.lat, p.lon) for p in waypoints]
            upload_and_fly(
                self._master, points, altitude_m,
                on_progress=self.progress.emit,
                should_abort=lambda: self._abort,
                on_message=handle_message,
            )
        except FlightAborted:
            # A deliberate Stop, not a failure - the GUI has already torn the
            # run down. Report it as a normal finish so no error banner shows.
            self.progress.emit("Mission stopped.")
            self.finished.emit()
        except (RuntimeError, TimeoutError, OSError) as exc:
            self.failed.emit(str(exc))
        else:
            self.finished.emit()
        finally:
            if self._master is not None:
                try:
                    self._master.close()
                except OSError:
                    pass
                self._master = None


class MavlinkFlightService(QObject):
    """GUI-facing handle: call `run_async`, get `batch_ready`/`progress`/
    `finished`/`failed`."""

    batch_ready = Signal(object)
    progress = Signal(str)
    finished = Signal()
    failed = Signal(str)

    _run_requested = Signal(list, str, float, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._thread = QThread(self)
        self._worker = _Worker()
        self._worker.moveToThread(self._thread)
        self._worker.batch_ready.connect(self.batch_ready)
        self._worker.progress.connect(self.progress)
        self._worker.finished.connect(self.finished)
        self._worker.failed.connect(self.failed)
        self._run_requested.connect(self._worker.run)
        self._thread.start()

    def run_async(
        self, waypoints: list[LatLon], connection_string: str, altitude_m: float, sysid: int
    ) -> None:
        self._run_requested.emit(waypoints, connection_string, altitude_m, sysid)

    def abort(self) -> None:
        self._worker.request_abort()

    def shutdown(self) -> None:
        self._worker.request_abort()
        self._thread.quit()
        self._thread.wait(2000)
