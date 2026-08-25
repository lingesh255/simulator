"""Renode Orchestrator Backend.

Instantiates and manages virtual drone machines in Renode, bridging their
UARTs to host UDP ports for telemetry, and listening for GUI commands.
"""
from __future__ import annotations

import socket
import threading
import time
import sys
from pathlib import Path
from typing import Any

import pyrenode

# Ensure project root is in PYTHONPATH
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from contracts.gui_orchestration import IsrCommandPayload, IsrCommandType, UdpTelemetryPayload, DroneStatus
from backend.isr_manager import IsrManager
from backend.sensor_feeder import SensorFeeder
from backend.logger import setup_logger

logger = setup_logger("renode_orchestrator")





class RenodeOrchestrator:
    def __init__(self, base_telemetry_port: int = 14550, control_port: int = 9000):
        self.base_telemetry_port = base_telemetry_port
        self.control_port = control_port
        
        self.machines: dict[int, Any] = {}
        self.isr_managers: dict[int, IsrManager] = {}
        self.sensor_feeders: dict[int, SensorFeeder] = {}
        
        self._running = False
        self._control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._control_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._control_socket.bind(("127.0.0.1", self.control_port))
        
        self._mock_telemetry_thread: threading.Thread | None = None

    def start_orchestrator(self) -> None:
        """Initialize the Renode headless server."""
        logger.info("Starting Renode Headless Server...")
        pyrenode.connect_renode()
        
        self._running = True
        
        # Start command listener
        threading.Thread(target=self._command_listener_loop, daemon=True).start()
        
        logger.info(f"Orchestrator ready. Listening for commands on UDP {self.control_port}")

    def stop_orchestrator(self) -> None:
        logger.info("Stopping orchestrator and killing VMs...")
        self._running = False
        for feeder in self.sensor_feeders.values():
            feeder.stop_feeding()
        pyrenode.shutdown_renode()

    def instantiate_drone(self, sysid: int, firmware_path: str = "firmware/fmuv3.elf", trace: bool = False) -> None:
        """Create a new VM for the given sysid."""
        machine_name = f"Drone_{sysid}"
        telemetry_port = self.base_telemetry_port + sysid
        tcp_port = telemetry_port + 1000  # Offset for the Renode TCP terminal
        
        logger.info(f"Instantiating {machine_name} with firmware {firmware_path}")
        
        renode = pyrenode.get_renode()
        
        repl_path = Path(__file__).resolve().parent.parent / "platforms" / "fmuv3.repl"
        fw_path = Path(__file__).resolve().parent.parent / firmware_path
        
        renode.execute(f"mach create '{machine_name}'")
        renode.execute(f"machine LoadPlatformDescription @{repl_path}")
        renode.execute(f"sysbus LoadELF @{fw_path}")
        
        if trace:
            renode.execute("sysbus.cpu LogFunctionNames true")
        
        # Bridge UART to UDP via TCP terminal and Python relay
        # As requested, using socket terminal and relay to bridge MAVLink to GUI
        renode.execute(f"emulation CreateServerSocketTerminal {tcp_port} 'telemetry_hub_{sysid}'")
        renode.execute(f"connector Connect sysbus.usart2 telemetry_hub_{sysid}")
        
        # Start Relay Thread to convert TCP stream to UDP datagrams
        threading.Thread(target=self._uart_to_udp_relay, args=(tcp_port, telemetry_port), daemon=True).start()
        
        self.machines[sysid] = machine_name
        self.isr_managers[sysid] = IsrManager(machine_name)
        
        feeder = SensorFeeder(machine_name)
        self.sensor_feeders[sysid] = feeder
        feeder.start_feeding()
        
        renode.execute("start")
        logger.info(f"{machine_name} running. Telemetry mapped to UDP {telemetry_port}")

    def _command_listener_loop(self) -> None:
        """Listen for UDP commands from the GUI."""
        while self._running:
            try:
                data, addr = self._control_socket.recvfrom(4096)
                payload_dict = json.loads(data.decode("utf-8"))
                cmd = IsrCommandPayload(**payload_dict)
                self._handle_command(cmd)
            except Exception as e:
                logger.error(f"Error parsing UDP command: {e}")

    def _handle_command(self, cmd: IsrCommandPayload) -> None:
        if cmd.sysid not in self.isr_managers:
            logger.warning(f"Received command for unknown sysid {cmd.sysid}")
            return
            
        manager = self.isr_managers[cmd.sysid]
        if cmd.command_type == IsrCommandType.INJECT_FAULT:
            manager.trigger_failsafe_isr(cmd.fault_type.value if cmd.fault_type else "unknown")
        elif cmd.command_type == IsrCommandType.RESTORE_STATE:
            manager.restore_state()

    def _uart_to_udp_relay(self, tcp_port: int, udp_port: int) -> None:
        """Relay raw UART MAVLink from Renode TCP terminal to Host UDP port."""
        logger.info(f"Starting UDP relay from TCP {tcp_port} to UDP {udp_port}")
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        while self._running:
            try:
                tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                tcp_sock.connect(("127.0.0.1", tcp_port))
                logger.info(f"Relay connected to Renode TCP {tcp_port}")
                
                while self._running:
                    data = tcp_sock.recv(4096)
                    if not data:
                        break
                    udp_sock.sendto(data, ("127.0.0.1", udp_port))
                    
            except ConnectionRefusedError:
                time.sleep(1)
            except Exception as e:
                logger.error(f"Relay error on TCP {tcp_port}: {e}")
                time.sleep(1)
            finally:
                try:
                    tcp_sock.close()
                except Exception:
                    pass


if __name__ == "__main__":
    orchestrator = RenodeOrchestrator()
    orchestrator.start_orchestrator()
    # Instantiate two mock drones
    orchestrator.instantiate_drone(1)
    orchestrator.instantiate_drone(2)
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        orchestrator.stop_orchestrator()
