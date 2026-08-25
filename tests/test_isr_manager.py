"""Tests for the ISR Manager (snapshotting and restoring Renode CPU state)."""
from __future__ import annotations

import pytest
from backend.isr_manager import IsrManager, EmulationSnapshot

class MockCPU:
    def __init__(self):
        self.PC = 0x08000000
        self.SP = 0x20000000
        self.LR = 0x08000010
        self.R0 = 1
        self.R1 = 2
        self.R2 = 3
        self.R3 = 4
        self.R4 = 5
        self.R5 = 6
        self.R6 = 7
        self.R7 = 8
        self.R8 = 9
        self.R9 = 10
        self.R10 = 11
        self.R11 = 12
        self.R12 = 13

class MockSysBus:
    def __init__(self):
        self.cpu = MockCPU()

class MockRenodeMachine:
    def __init__(self):
        self.sysbus = MockSysBus()

def test_pause_and_snapshot():
    machine = MockRenodeMachine()
    manager = IsrManager(machine)
    
    snapshot = manager.pause_and_snapshot()
    
    assert snapshot is not None
    assert snapshot.pc == 0x08000000
    assert snapshot.sp == 0x20000000
    assert snapshot.r1 == 2
    
    assert manager.active_snapshot == snapshot

def test_restore_state():
    machine = MockRenodeMachine()
    manager = IsrManager(machine)
    
    manager.pause_and_snapshot()
    
    # Mutate the machine state to simulate execution during ISR
    machine.sysbus.cpu.PC = 0x11111111
    machine.sysbus.cpu.R0 = 999
    
    assert machine.sysbus.cpu.PC == 0x11111111
    assert machine.sysbus.cpu.R0 == 999
    
    manager.restore_state()
    
    # State should be restored back to original
    assert machine.sysbus.cpu.PC == 0x08000000
    assert machine.sysbus.cpu.R0 == 1
    assert manager.active_snapshot is None
