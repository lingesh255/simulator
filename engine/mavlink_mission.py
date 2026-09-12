"""Route -> real MAVLink mission, shared by the standalone CLI tool
(`scripts/pddl_to_mavlink.py`) and the GUI's optional external-MAVLink flight
path (`services/mavlink_flight_service.py`), so the two never drift apart.

    [(lat, lon), ...] route
        -> MISSION_ITEM_INT sequence      (NAV_TAKEOFF, NAV_WAYPOINT x N, NAV_LAND)
        -> uploaded + flown against a real ArduPilot/PX4 (SITL or hardware)
           over an actual `pymavlink.mavutil` connection

Nothing here talks to this repo's own in-process `SimulatedFlightController` -
that path (this repo's default "Plan Mission" behaviour) goes through
`engine.mavlink_interpreter.CommandInterpreter` instead, one leg at a time.
This module is only for a genuine external MAVLink endpoint, uploaded as one
full mission the way a real GCS does it.
"""
from __future__ import annotations

import time
from typing import Callable, Optional

from pymavlink.dialects.v20 import ardupilotmega as mav2


def build_mission_items(
    mav: "mav2.MAVLink",
    target_system: int,
    target_component: int,
    waypoints: list[tuple[float, float]],
    altitude_m: float,
):
    """[(lat, lon), ...] -> MISSION_ITEM_INT messages: HOME placeholder,
    TAKEOFF, WAYPOINT x N, LAND.

    Mirrors the shape ArduPilot expects for a normal auto mission: item 0 is
    a takeoff (climbs from wherever the vehicle currently is - lat/lon are
    unused for it), every point in between is an ordinary waypoint, and the
    last point is a landing rather than just another waypoint. Intermediate
    points are exactly the route the PDDL planner worked out - including any
    detour it planned around a restricted area - so flying them in order
    clears that area by construction; nothing here re-checks it.

    Item 0 in the list this returns is NOT the takeoff, though - it's an
    unused placeholder. Confirmed by direct empirical test (uploading a
    mission, then reading it straight back over MAVLink): mission sequence
    0 is unconditionally reserved for home by ArduPilot's own mission
    storage and is never actually readable/usable as a real command -
    AP_Mission::read_cmd_from_storage() (AP_Mission.cpp:832) hard-codes
    "if (index == 0) { ... return home ...}" with no way to opt out, and
    MissionItemProtocol_Waypoints::get_item() (MissionItemProtocol_
    Waypoints.cpp:69) passes the wire seq straight through with no offset
    ("seq != 0 && // always allow HOME to be read"). A real run proved this
    the hard way: with takeoff at seq 0, a readback showed seq 0 coming
    back as a synthetic NAV_WAYPOINT at (0,0) - not our uploaded
    NAV_TAKEOFF at all - and ModeAuto::init() then failed every time with
    "Missing Takeoff Cmd", because the mission's first REAL command was our
    seq-1 item, which was a plain waypoint/land, not a takeoff. Real GCS
    implementations (QGroundControl, Mission Planner) always upload an
    explicit, effectively-unused item at seq 0 for exactly this reason -
    this mirrors that, not a Renode-specific workaround.
    """
    if len(waypoints) < 2:
        raise ValueError("need at least a source and a destination")

    items = [
        mav.mission_item_int_encode(
            target_system, target_component,
            0,  # seq - reserved for home, see docstring; content is unused
            mav2.MAV_FRAME_GLOBAL,
            mav2.MAV_CMD_NAV_WAYPOINT,
            0, 1,  # current, autocontinue
            0, 0, 0, 0,
            0, 0, 0,
        ),
        mav.mission_item_int_encode(
            target_system, target_component,
            1,  # seq
            mav2.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mav2.MAV_CMD_NAV_TAKEOFF,
            0, 1,  # current, autocontinue
            0, 0, 0, 0,
            0, 0, altitude_m,
        ),
    ]
    last_seq = len(waypoints)
    for offset, (lat, lon) in enumerate(waypoints[1:], start=1):
        seq = offset + 1
        is_final = seq == last_seq
        items.append(
            mav.mission_item_int_encode(
                target_system, target_component,
                seq,
                mav2.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mav2.MAV_CMD_NAV_LAND if is_final else mav2.MAV_CMD_NAV_WAYPOINT,
                0, 1,
                0, 0, 0, 0,
                int(lat * 1e7), int(lon * 1e7), 0.0 if is_final else altitude_m,
            )
        )
    return items


class FlightAborted(RuntimeError):
    """Raised when `should_abort` reports true mid-flight."""


TELEMETRY_TYPES = ("GLOBAL_POSITION_INT", "SYS_STATUS", "GPS_RAW_INT", "HEARTBEAT")
_TRACKED_TYPES = ("MISSION_ITEM_REACHED", "STATUSTEXT") + TELEMETRY_TYPES


def upload_and_fly(
    master,
    waypoints: list[tuple[float, float]],
    altitude_m: float,
    *,
    heartbeat_timeout_s: float = 10.0,
    item_timeout_s: float = 10.0,
    mission_timeout_s: float = 600.0,
    on_progress: Optional[Callable[[str], None]] = None,
    should_abort: Optional[Callable[[], bool]] = None,
    on_message: Optional[Callable[[object], None]] = None,
) -> None:
    """Connect, upload the route as one mission, arm, fly it, and return once
    the final waypoint is reached.

    `master` is an already-constructed `pymavlink.mavutil.mavlink_connection`
    (not created here, so the caller controls the connection string and its
    lifetime). Raises `RuntimeError`/`TimeoutError` on any protocol failure,
    or `FlightAborted` if `should_abort` reports true. Blocking throughout -
    callers on a GUI thread should run this on a worker thread.

    `on_message`, if given, is called with every raw MAVLink message seen
    once the mission is airborne - in particular `GLOBAL_POSITION_INT` /
    `SYS_STATUS` / `GPS_RAW_INT` / `HEARTBEAT` (see `TELEMETRY_TYPES`), which
    carry the vehicle's actual position/battery/GPS/armed state. This is how
    a caller (e.g. `services.mavlink_flight_service`) drives a live map
    marker from the real vehicle's own telemetry instead of just knowing
    when the mission starts and ends.
    """
    report = on_progress or (lambda _msg: None)
    aborted = should_abort or (lambda: False)

    report(f"Waiting for heartbeat (timeout {heartbeat_timeout_s:.0f}s) ...")
    if master.wait_heartbeat(timeout=heartbeat_timeout_s) is None:
        raise TimeoutError(
            f"No heartbeat within {heartbeat_timeout_s:.0f}s - is a vehicle/SITL "
            f"actually listening on this connection?"
        )
    report(f"Heartbeat OK - system {master.target_system}, component {master.target_component}.")

    items = build_mission_items(
        master.mav, master.target_system, master.target_component, waypoints, altitude_m
    )

    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    master.mav.mission_count_send(
        master.target_system, master.target_component, len(items), mav2.MAV_MISSION_TYPE_MISSION
    )
    for _ in range(len(items)):
        if aborted():
            raise FlightAborted("aborted during mission upload")
        request = master.recv_match(
            type=["MISSION_REQUEST_INT", "MISSION_REQUEST"], blocking=True, timeout=item_timeout_s
        )
        if request is None:
            raise TimeoutError("autopilot never asked for the next mission item")
        master.mav.send(items[request.seq])
    ack = master.recv_match(type="MISSION_ACK", blocking=True, timeout=item_timeout_s)
    if ack is None or ack.type != mav2.MAV_MISSION_ACCEPTED:
        raise RuntimeError(f"mission upload rejected: {ack}")
    report(f"Mission uploaded and accepted ({len(items)} items).")

    modes = master.mode_mapping() or {}

    def set_mode(name: str) -> None:
        mode_id = modes.get(name)
        if mode_id is None:
            raise RuntimeError(f"vehicle has no '{name}' mode in its mode_mapping()")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mav2.MAV_CMD_DO_SET_MODE, 0,
            mav2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0,
        )

    set_mode("GUIDED")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mav2.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0,
    )
    ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=item_timeout_s)
    report(f"Arm ack: {ack}")

    # DO_SET_MODE's own COMMAND_ACK does not guarantee the mode actually
    # took - AUTO mode's init() can reject the switch internally (e.g. a
    # mission-state race right after upload) with no NACK, leaving the
    # vehicle silently still in GUIDED. Confirm via HEARTBEAT.custom_mode
    # actually reflecting AUTO - a small, bounded number of retries with a
    # real delay between them, not a single blind attempt or a fixed sleep
    # standing in for confirmation.
    auto_mode_id = modes.get("AUTO")
    if auto_mode_id is None:
        raise RuntimeError("vehicle has no 'AUTO' mode in its mode_mapping()")
    auto_confirmed = False
    max_attempts = 5
    for attempt in range(1, max_attempts + 1):
        set_mode("AUTO")
        attempt_deadline = time.monotonic() + 1.0
        while time.monotonic() < attempt_deadline:
            message = master.recv_match(
                type=["HEARTBEAT", "STATUSTEXT"], blocking=True,
                timeout=max(0.0, attempt_deadline - time.monotonic()),
            )
            if message is None:
                continue
            # Route every message seen during this retry loop back through
            # the same callbacks the main poll loop below uses, so a caller
            # tracking armed state / STATUSTEXT (e.g. for real evidence of
            # what happened here) has no blind spot just because this
            # specific wait happened inside a retry rather than the main
            # loop.
            if on_message is not None:
                on_message(message)
            kind = message.get_type()
            if kind == "STATUSTEXT":
                report(f"[FC] {message.text}")
            elif kind == "HEARTBEAT" and message.custom_mode == auto_mode_id:
                auto_confirmed = True
                break
        if auto_confirmed:
            break
        report(f"AUTO mode not confirmed yet (attempt {attempt}/{max_attempts}), retrying.")
        time.sleep(0.5)
    if not auto_confirmed:
        raise RuntimeError(
            f"AUTO mode never confirmed via HEARTBEAT.custom_mode after "
            f"{max_attempts} attempts"
        )

    report("Flying mission.")
    last_seq = len(items) - 1
    last_message_at = time.monotonic()
    while True:
        if aborted():
            raise FlightAborted("aborted mid-flight")
        # Poll in short slices so an abort (or a vehicle that goes silent -
        # e.g. the mock subprocess being killed on Stop) is noticed within a
        # second instead of blocking this worker thread until the full
        # mission timeout elapses, which would wedge the next mission.
        message = master.recv_match(type=list(_TRACKED_TYPES), blocking=True, timeout=1.0)
        if message is None:
            if time.monotonic() - last_message_at >= mission_timeout_s:
                raise TimeoutError("no telemetry received - link lost?")
            continue
        last_message_at = time.monotonic()
        kind = message.get_type()
        if on_message is not None:
            on_message(message)
        if kind == "MISSION_ITEM_REACHED":
            report(f"Reached waypoint {message.seq}/{last_seq}")
            if message.seq >= last_seq:
                report("Mission complete.")
                return
        elif kind == "STATUSTEXT":
            report(f"[FC] {message.text}")
        # else: a telemetry type (see TELEMETRY_TYPES) - handed to on_message
        # above only, no progress line of its own to avoid flooding the log.
