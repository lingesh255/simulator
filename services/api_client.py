"""UDP client for Module 1 <-> Module 2/3 (Renode) communication.

Replaces the REST/WS client. Uses QUdpSocket for telemetry ingestion and
command dispatch without blocking the GUI event loop.
"""
from __future__ import annotations

import json
from typing import Optional

from PySide6.QtCore import QObject, Signal, Slot, QTimer, QByteArray
from PySide6.QtNetwork import QUdpSocket, QHostAddress

from contracts.gui_orchestration import (
    DroneConfig,
    FlockCommand,
    SwarmTelemetryBatch,
    UdpTelemetryPayload,
    IsrCommandPayload,
    IsrCommandType,
    FaultType,
    FaultSeverity,
    InjectFault
)


class OrchestrationClient(QObject):
    """Talks to the Renode Emulation Backend via UDP."""

    request_succeeded = Signal(str, object)
    request_failed = Signal(str, str)
    telemetry_received = Signal(object)  # SwarmTelemetryBatch
    connection_state_changed = Signal(bool)

    def __init__(self, control_host: str = "127.0.0.1", control_port: int = 9000, parent=None):
        super().__init__(parent)
        self.control_host = QHostAddress(control_host)
        self.control_port = control_port
        
        # Socket for sending commands to the orchestrator
        self._cmd_socket = QUdpSocket(self)
        
        # Sockets for listening to telemetry from each drone
        self._telemetry_sockets: list[QUdpSocket] = []
        
        # Aggregation of telemetry
        self._latest_telemetry: dict[int, UdpTelemetryPayload] = {}
        self._batch_timer = QTimer(self)
        self._batch_timer.setInterval(20)  # 50Hz batch emission
        self._batch_timer.timeout.connect(self._emit_telemetry_batch)
        self._tick = 0
        self._telemetry_active = False
        self._mav_parsers = {}

    def connect_telemetry(self, sysids: list[int], base_port: int = 14550) -> None:
        """Bind sockets to listen to the specified drone telemetry ports."""
        self.disconnect_telemetry()
        
        for sysid in sysids:
            try:
                from pymavlink.mavutil import mavlink
                self._mav_parsers[sysid] = mavlink.MAVLink(None)
            except ImportError:
                pass
                
            port = base_port + sysid
            sock = QUdpSocket(self)
            sock.bind(QHostAddress.LocalHost, port)
            sock.readyRead.connect(lambda s=sock, sid=sysid: self._on_telemetry_datagram(s, sid))
            self._telemetry_sockets.append(sock)
            
        self._batch_timer.start()
        # Connection state is emitted when actual telemetry is received

    def disconnect_telemetry(self) -> None:
        self._batch_timer.stop()
        for sock in self._telemetry_sockets:
            sock.close()
            sock.deleteLater()
        self._telemetry_sockets.clear()
        self._latest_telemetry.clear()
        if self._telemetry_active:
            self._telemetry_active = False
            self.connection_state_changed.emit(False)

    @Slot()
    def _on_telemetry_datagram(self, sock: QUdpSocket, sysid: int) -> None:
        if not self._telemetry_active:
            self._telemetry_active = True
            self.connection_state_changed.emit(True)
            
        parser = self._mav_parsers.get(sysid)
            
        while sock.hasPendingDatagrams():
            datagram = sock.receiveDatagram()
            data = datagram.data().data()
            try:
                payload = UdpTelemetryPayload.model_validate_json(data.decode('utf-8'))
                self._latest_telemetry[payload.sysid] = payload
            except Exception:
                if parser:
                    try:
                        msgs = parser.parse_buffer(data)
                        if msgs:
                            self._update_telemetry_from_mavlink(sysid, msgs)
                    except Exception:
                        pass

    def _update_telemetry_from_mavlink(self, sysid: int, msgs: list) -> None:
        if sysid not in self._latest_telemetry:
            from contracts.gui_orchestration import DroneStatus
            self._latest_telemetry[sysid] = UdpTelemetryPayload(
                sysid=sysid,
                status=DroneStatus.STANDBY,
                lat=0.0,
                lon=0.0,
                altitude_m=0.0,
                heading_deg=0.0,
                ground_speed_mps=0.0,
                battery_pct=100.0,
                link_quality_pct=100.0,
                active_faults=[],
            )
        
        payload = self._latest_telemetry[sysid]
        
        for msg in msgs:
            msg_type = msg.get_type()
            
            if msg_type == 'GLOBAL_POSITION_INT':
                payload.lat = msg.lat / 1e7
                payload.lon = msg.lon / 1e7
                payload.altitude_m = msg.alt / 1000.0
                if hasattr(msg, 'hdg') and msg.hdg != 65535:
                    payload.heading_deg = msg.hdg / 100.0
            elif msg_type == 'VFR_HUD':
                payload.ground_speed_mps = msg.groundspeed
                payload.heading_deg = msg.heading
            elif msg_type == 'SYS_STATUS':
                if hasattr(msg, 'battery_remaining') and msg.battery_remaining != -1:
                    payload.battery_pct = float(msg.battery_remaining)
            elif msg_type == 'HEARTBEAT':
                from pymavlink import mavutil
                from contracts.gui_orchestration import DroneStatus
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    payload.status = DroneStatus.IN_FLIGHT
                else:
                    payload.status = DroneStatus.STANDBY

    def _emit_telemetry_batch(self) -> None:
        if not self._latest_telemetry:
            return
            
        import time
        batch = SwarmTelemetryBatch(
            tick=self._tick,
            timestamp=time.time(),
            drones=list(self._latest_telemetry.values())
        )
        self.telemetry_received.emit(batch)
        self._tick += 1

    # ---- Commands (Delegated to Orchestrator UDP port) ----

    def _send_command(self, payload: str, tag: str) -> None:
        datagram = QByteArray(payload.encode('utf-8'))
        bytes_sent = self._cmd_socket.writeDatagram(datagram, self.control_host, self.control_port)
        if bytes_sent == -1:
            self.request_failed.emit(tag, self._cmd_socket.errorString())
        else:
            self.request_succeeded.emit(tag, None)

    def inject_isr(self, sysid: int, fault_type: FaultType) -> None:
        """Triggers the hardware ISR via the orchestrator."""
        cmd = IsrCommandPayload(
            sysid=sysid,
            command_type=IsrCommandType.INJECT_FAULT,
            fault_type=fault_type,
            severity=FaultSeverity.FULL
        )
        self._send_command(cmd.model_dump_json(), f"inject_isr:{sysid}")

    def restore_isr_state(self, sysid: int) -> None:
        """Restores the snapshot state in the Renode VM."""
        cmd = IsrCommandPayload(
            sysid=sysid,
            command_type=IsrCommandType.RESTORE_STATE
        )
        self._send_command(cmd.model_dump_json(), f"restore_state:{sysid}")

    # Legacy commands mapped to simple success for GUI compatibility
    def register_drone(self, drone: DroneConfig) -> None:
        self.request_succeeded.emit("register_drone", None)

    def takeoff(self, sysid: int, target_altitude_m: float) -> None:
        self.request_succeeded.emit(f"takeoff:{sysid}", None)

    def flock(self, command: FlockCommand) -> None:
        self.request_succeeded.emit("flock", None)

    def stop_swarm(self, reason: Optional[str] = None) -> None:
        self.request_succeeded.emit("stop_swarm", None)

    def inject_fault(self, command: InjectFault) -> None:
        """Legacy inject fault adapter for GUI compatibility."""
        for sysid in command.sysids:
            cmd = IsrCommandPayload(
                sysid=sysid,
                command_type=IsrCommandType.INJECT_FAULT,
                fault_type=command.fault_type,
                severity=command.severity
            )
            self._send_command(cmd.model_dump_json(), f"inject_isr:{sysid}")
