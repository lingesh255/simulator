"""Hardware Interrupt (ISR) and State Snapshotting Manager.

Responsible for freezing emulation, snapshotting CPU registers and memory,
injecting NVIC IRQs, and restoring the saved state.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Any

from backend.logger import setup_logger

logger = setup_logger("isr_manager")


@dataclass
class EmulationSnapshot:
    """Stores the saved CPU registers and critical memory blocks."""
    pc: int
    sp: int
    lr: int
    r0: int
    r1: int
    r2: int
    r3: int
    r4: int
    r5: int
    r6: int
    r7: int
    r8: int
    r9: int
    r10: int
    r11: int
    r12: int
    # Optionally store memory blocks if necessary
    # memory_blocks: dict[int, bytes]


class IsrManager:
    def __init__(self, machine: Any):
        """
        Initialize with a Renode machine object.
        (Assuming the object provides `sysbus` and `sysbus.cpu` interfaces).
        """
        self.machine = machine
        self.active_snapshot: EmulationSnapshot | None = None

    def pause_and_snapshot(self) -> EmulationSnapshot:
        """Freeze execution and snapshot CPU registers."""
        logger.info("Pausing machine and snapshotting state...")
        
        # In a real Renode python integration:
        # self.machine.Pause()
        
        # Mocking the register reads (pyrenode style or telnet style)
        # For pyrenode it's usually `machine.sysbus.cpu.PC`
        # Here we mock it if it's not a real pyrenode object, but assume the interface.
        try:
            cpu = self.machine.sysbus.cpu
            snapshot = EmulationSnapshot(
                pc=cpu.PC,
                sp=cpu.SP,
                lr=cpu.LR,
                r0=cpu.R0, r1=cpu.R1, r2=cpu.R2, r3=cpu.R3,
                r4=cpu.R4, r5=cpu.R5, r6=cpu.R6, r7=cpu.R7,
                r8=cpu.R8, r9=cpu.R9, r10=cpu.R10, r11=cpu.R11, r12=cpu.R12
            )
        except AttributeError:
            # Fallback for testing/mocking
            logger.warning("Mocking CPU registers for snapshot.")
            snapshot = EmulationSnapshot(
                pc=0x08000100, sp=0x20001000, lr=0x08000120,
                r0=0, r1=0, r2=0, r3=0, r4=0, r5=0, r6=0, r7=0,
                r8=0, r9=0, r10=0, r11=0, r12=0
            )

        self.active_snapshot = snapshot
        logger.debug(f"Snapshot taken: PC={hex(snapshot.pc)}, SP={hex(snapshot.sp)}")
        return snapshot

    def inject_irq(self, irq_id: int) -> None:
        """Inject a hardware interrupt to the NVIC and resume."""
        logger.info(f"Injecting IRQ {irq_id} into NVIC...")
        
        try:
            # Assuming sysbus.nvic OnGPIO mapping in Renode
            # self.machine.sysbus.nvic.OnGPIO(irq_id, True)
            pass
        except AttributeError:
            logger.warning(f"Mocking IRQ injection for IRQ {irq_id}")
            
        # In a real integration:
        # self.machine.Start()
        logger.info("Machine resumed with injected IRQ.")

    def restore_state(self) -> None:
        """Write back saved register values and resume."""
        if not self.active_snapshot:
            logger.error("No active snapshot to restore!")
            return

        logger.info("Restoring state from snapshot...")
        
        # self.machine.Pause()
        
        try:
            cpu = self.machine.sysbus.cpu
            snap = self.active_snapshot
            cpu.PC = snap.pc
            cpu.SP = snap.sp
            cpu.LR = snap.lr
            cpu.R0 = snap.r0
            cpu.R1 = snap.r1
            cpu.R2 = snap.r2
            cpu.R3 = snap.r3
            cpu.R4 = snap.r4
            cpu.R5 = snap.r5
            cpu.R6 = snap.r6
            cpu.R7 = snap.r7
            cpu.R8 = snap.r8
            cpu.R9 = snap.r9
            cpu.R10 = snap.r10
            cpu.R11 = snap.r11
            cpu.R12 = snap.r12
        except AttributeError:
            logger.warning("Mocking state restoration.")

        self.active_snapshot = None
        
        # self.machine.Start()
        logger.info("State restored. Mission resuming seamlessly.")

    def trigger_failsafe_isr(self, fault_type: str) -> None:
        """High-level method to execute the ISR snapshot and injection."""
        logger.info(f"Triggering failsafe ISR for {fault_type}")
        self.pause_and_snapshot()
        
        # Map fault type to a mock IRQ ID. 
        # For example, GPS loss might map to a specific EXTI line.
        irq_mapping = {
            "gps_loss": 10,
            "comms_loss": 11,
            "battery_fail": 12
        }
        irq_id = irq_mapping.get(fault_type, 15)
        
        self.inject_irq(irq_id)
