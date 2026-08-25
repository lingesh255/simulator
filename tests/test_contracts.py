"""Tests for the UDP datagram contracts (Pydantic schemas)."""
from __future__ import annotations

import json
from contracts.gui_orchestration import (
    UdpTelemetryPayload,
    DroneStatus,
    IsrCommandPayload,
    IsrCommandType,
    FaultType,
    FaultSeverity
)


def test_udp_telemetry_payload_serialization():
    payload = UdpTelemetryPayload(
        sysid=1,
        status=DroneStatus.IN_FLIGHT,
        lat=47.397742,
        lon=8.545594,
        altitude_m=50.0,
        heading_deg=180.5,
        ground_speed_mps=12.3,
        battery_pct=85.5,
        link_quality_pct=99.0
    )
    
    json_data = payload.model_dump_json()
    assert isinstance(json_data, str)
    
    parsed = UdpTelemetryPayload.model_validate_json(json_data)
    assert parsed.sysid == 1
    assert parsed.status == DroneStatus.IN_FLIGHT
    assert parsed.lat == 47.397742
    assert parsed.battery_pct == 85.5
    assert not parsed.active_faults


def test_isr_command_payload_serialization():
    payload = IsrCommandPayload(
        sysid=2,
        command_type=IsrCommandType.INJECT_FAULT,
        fault_type=FaultType.GPS_LOSS,
        severity=FaultSeverity.FULL
    )
    
    json_data = payload.model_dump_json()
    parsed = IsrCommandPayload.model_validate_json(json_data)
    
    assert parsed.sysid == 2
    assert parsed.command_type == IsrCommandType.INJECT_FAULT
    assert parsed.fault_type == FaultType.GPS_LOSS
    assert parsed.severity == FaultSeverity.FULL

def test_isr_command_restore_serialization():
    payload = IsrCommandPayload(
        sysid=1,
        command_type=IsrCommandType.RESTORE_STATE
    )
    
    json_data = payload.model_dump_json()
    parsed = IsrCommandPayload.model_validate_json(json_data)
    
    assert parsed.sysid == 1
    assert parsed.command_type == IsrCommandType.RESTORE_STATE
    assert parsed.fault_type is None
