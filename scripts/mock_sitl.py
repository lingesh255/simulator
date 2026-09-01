"""A minimal fake autopilot for VERIFYING what the GUI's "Fly via real
MAVLink (SITL/hardware)" toggle actually sends and does. It prints every
command and mission item it receives, decoded (command name, lat, lon, alt),
so you can watch in real time that Plan Mission is sending exactly the
PDDL-solved route and nothing else - and once armed and switched to AUTO, it
actually flies the mission (linear interpolation at a fixed cruise speed,
same shape as this repo's own `SimulatedFlightController`), streaming real
GLOBAL_POSITION_INT/SYS_STATUS/GPS_RAW_INT telemetry back - which is what
lets you confirm the GUI's map marker genuinely moves from this vehicle's
own reported position, not a shortcut.

How the wiring works: the GUI's MAVLink connection string (`udp:HOST:PORT`,
pymavlink's default "udpin" direction - see engine.mavlink_mission /
services.mavlink_flight_service) *binds and listens* on that port, exactly
like a real GCS waiting for an autopilot's telemetry stream. This script is
the other half: it *sends to* that port, starting with periodic HEARTBEATs
(a real ArduPilot SITL does the same), which is what lets the GUI's
`wait_heartbeat()` discover this script's address and start talking to it.

Usage:
    python scripts/mock_sitl.py --port 14550 --home 18.393304,79.074399

`--home` should be the plan's actual source point (see the run's
problem.pddl `(= (latitude source) ...)`/`(= (longitude source) ...)`) -
the mission's own items never carry it (its TAKEOFF item is "wherever you
already are", same as a real vehicle), so without it this script just
starts at the first real waypoint instead, which skips visualising the
takeoff leg but still flies every leg after it correctly.

Then in the GUI: check "Fly via real MAVLink (SITL/hardware)", leave the
connection field at its default `udp:127.0.0.1:14550` (or match --port), and
click Plan Mission. Everything the GUI sends prints here, live - compare the
lat/lon against `plans/<name>/runs/<timestamp>/problem.pddl` for that run to
confirm they match exactly.

This also proves the *internal* pipeline (services.thread_backend /
DroneThread) is NOT what's running: that path never touches the network, so
if you see this script receiving MAVLink at all, you're watching the
external-MAVLink path with certainty. As a second confirmation, the app's
own console stays silent about "Drone N Thread started" while this path is
in use - that message only ever comes from `engine.drone_thread`, which the
external-MAVLink path never spawns.
"""
from __future__ import annotations

import argparse
import math
import socket
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from pymavlink.dialects.v20 import ardupilotmega as mav2  # noqa: E402

SYSID = 1
COMPID = 1
HEARTBEAT_INTERVAL_S = 1.0
TELEMETRY_INTERVAL_S = 0.2  # ~5 Hz, a realistic autopilot stream rate
MODE_AUTO = 3  # ArduCopter AUTO - matches engine.mavlink_interpreter.MODE_AUTO
CRUISE_SPEED_MPS = 15.0
CLIMB_RATE_MPS = 3.0
ARRIVAL_RADIUS_M = 5.0
EARTH_RADIUS_M = 6_371_000.0
_CMD_NAMES = mav2.enums["MAV_CMD"]


def _cmd_name(command_id: int) -> str:
    entry = _CMD_NAMES.get(command_id)
    return entry.name if entry is not None else str(command_id)


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dlat = p2 - p1
    dlon = math.radians(lon2 - lon1)
    h = math.sin(dlat / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    y = math.sin(dlon) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dlon)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def _parse_latlon(text: str) -> tuple[float, float]:
    lat_str, lon_str = text.split(",")
    return float(lat_str), float(lon_str)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--host", default="127.0.0.1", help="address the GUI's connection field binds on")
    parser.add_argument("--port", type=int, default=14550, help="port the GUI's connection field binds on")
    parser.add_argument(
        "--home", type=_parse_latlon, default=None,
        help="lat,lon to start at - the plan's actual source point, if known (see problem.pddl); "
             "defaults to the first real waypoint if omitted",
    )
    parser.add_argument(
        "--no-fake-flight", action="store_true",
        help="don't actually fly the mission after upload - just verify the upload itself",
    )
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    dest = (args.host, args.port)

    mav = mav2.MAVLink(None, srcSystem=SYSID, srcComponent=COMPID)
    mav.robust_parsing = True

    def send(message) -> None:
        sock.sendto(message.pack(mav), dest)

    def send_heartbeat(armed: bool) -> None:
        base_mode = mav2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if armed:
            base_mode |= mav2.MAV_MODE_FLAG_SAFETY_ARMED
        send(mav.heartbeat_encode(
            mav2.MAV_TYPE_QUADROTOR, mav2.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode, mode, mav2.MAV_STATE_ACTIVE,
        ))

    def send_telemetry(lat: float, lon: float, alt_m: float, heading_deg: float, ground_speed_mps: float) -> None:
        send(mav.global_position_int_encode(
            0, int(lat * 1e7), int(lon * 1e7), int(alt_m * 1000), int(alt_m * 1000),
            int(ground_speed_mps * 100), 0, 0, int(heading_deg * 100),
        ))
        send(mav.sys_status_encode(0, 0, 0, 0, 12000, -1, 95, 0, 0, 0, 0, 0, 0))
        send(mav.gps_raw_int_encode(
            0, 3, int(lat * 1e7), int(lon * 1e7), int(alt_m * 1000), 100, 100,
            int(ground_speed_mps * 100), int(heading_deg * 100), 12,
        ))

    print(f"Mock SITL listening for a GCS and sending heartbeats to {dest}")
    print("Point the GUI's 'Connect' field at udp:%s:%d and click Plan Mission.\n" % (args.host, args.port))

    mission: list[tuple[float, float, float, str]] = []
    expecting_count = 0
    last_heartbeat = 0.0
    last_telemetry = 0.0
    armed = False
    mode = 0
    mission_uploaded = False

    # Flight state - only meaningful once armed and in AUTO.
    pos_lat = pos_lon = pos_alt = 0.0
    heading_deg = 0.0
    leg_index = 0  # index into `mission` of the waypoint currently headed for
    flying = False

    def start_flight() -> None:
        nonlocal pos_lat, pos_lon, pos_alt, leg_index, flying
        if args.home is not None:
            pos_lat, pos_lon = args.home
        elif len(mission) > 1:
            pos_lat, pos_lon = mission[1][0], mission[1][1]
            print("  (no --home given - starting at the first real waypoint instead of the true source)")
        pos_alt = 0.0
        leg_index = 0
        flying = True
        print(f"  (fake) airborne at ({pos_lat:.6f}, {pos_lon:.6f}) - flying {len(mission)} item(s)")

    while True:
        now = time.monotonic()
        if now - last_heartbeat >= HEARTBEAT_INTERVAL_S:
            send_heartbeat(armed)
            last_heartbeat = now

        if flying and now - last_telemetry >= TELEMETRY_INTERVAL_S:
            dt = now - last_telemetry
            last_telemetry = now
            ground_speed = 0.0

            if leg_index == 0:
                # mission[0] is the TAKEOFF item: lat/lon are unused ("wherever
                # you already are"), only its altitude means anything - climb
                # in place, then advance straight to the first real waypoint.
                climb_target_alt = mission[0][2]
                if pos_alt < climb_target_alt - 0.1:
                    pos_alt = min(climb_target_alt, pos_alt + CLIMB_RATE_MPS * dt)
                else:
                    leg_index = 1
            else:
                target_lat, target_lon, target_alt, target_name = mission[leg_index]
                remaining = _haversine_m(pos_lat, pos_lon, target_lat, target_lon)
                if remaining <= ARRIVAL_RADIUS_M:
                    send(mav.mission_item_reached_encode(leg_index))
                    print(f"  (fake) reached waypoint {leg_index}/{len(mission) - 1}  [{target_name}]")
                    leg_index += 1
                    if leg_index >= len(mission):
                        flying = False
                        armed = False
                        print("  (fake) mission complete - disarmed")
                else:
                    step = CRUISE_SPEED_MPS * dt
                    fraction = min(1.0, step / remaining)
                    heading_deg = _bearing_deg(pos_lat, pos_lon, target_lat, target_lon)
                    pos_lat += (target_lat - pos_lat) * fraction
                    pos_lon += (target_lon - pos_lon) * fraction
                    pos_alt += (target_alt - pos_alt) * fraction
                    ground_speed = CRUISE_SPEED_MPS

            send_telemetry(pos_lat, pos_lon, pos_alt, heading_deg, ground_speed)

        try:
            raw, _sender = sock.recvfrom(4096)
        except (BlockingIOError, OSError):
            time.sleep(0.02)
            continue

        for message in mav.parse_buffer(raw) or []:
            kind = message.get_type()

            if kind == "HEARTBEAT":
                continue  # the GCS's own heartbeat - not interesting here

            elif kind == "COMMAND_LONG":
                name = _cmd_name(message.command)
                print(f"COMMAND_LONG: {name}  (param1={message.param1}, param2={message.param2})")
                if message.command == mav2.MAV_CMD_COMPONENT_ARM_DISARM:
                    armed = bool(message.param1)
                    print(f"  -> {'ARMED' if armed else 'DISARMED'}")
                elif message.command == mav2.MAV_CMD_DO_SET_MODE:
                    mode = int(message.param2)
                    print(f"  -> mode set to {mode}")
                    if not args.no_fake_flight and mode == MODE_AUTO and armed and mission_uploaded and not flying:
                        start_flight()
                send(mav.command_ack_encode(message.command, mav2.MAV_RESULT_ACCEPTED))

            elif kind == "MISSION_COUNT":
                expecting_count = message.count
                mission = []
                mission_uploaded = False
                flying = False
                print(f"\nMISSION_COUNT: {expecting_count} item(s) incoming")
                send(mav.mission_request_int_encode(
                    message.get_srcSystem(), message.get_srcComponent(), 0,
                    mav2.MAV_MISSION_TYPE_MISSION,
                ))

            elif kind in ("MISSION_ITEM_INT", "MISSION_ITEM"):
                lat = message.x / 1e7 if abs(message.x) > 1000 else float(message.x)
                lon = message.y / 1e7 if abs(message.y) > 1000 else float(message.y)
                name = _cmd_name(message.command)
                mission.append((lat, lon, message.z, name))
                print(f"  seq={message.seq}  {name:<20}  lat={lat:.6f}  lon={lon:.6f}  alt={message.z}")

                next_seq = len(mission)
                if next_seq < expecting_count:
                    send(mav.mission_request_int_encode(
                        message.get_srcSystem(), message.get_srcComponent(), next_seq,
                        mav2.MAV_MISSION_TYPE_MISSION,
                    ))
                else:
                    send(mav.mission_ack_encode(
                        message.get_srcSystem(), message.get_srcComponent(),
                        mav2.MAV_MISSION_ACCEPTED, mav2.MAV_MISSION_TYPE_MISSION,
                    ))
                    mission_uploaded = True
                    print(f"Mission accepted - compare these {len(mission)} points against "
                          f"the plan's problem.pddl/plan.txt for this run:")
                    for i, (m_lat, m_lon, m_alt, m_name) in enumerate(mission):
                        print(f"    {i}: {m_name:<20} ({m_lat:.6f}, {m_lon:.6f}) alt={m_alt}")


if __name__ == "__main__":
    main()
