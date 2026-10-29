"""upload_and_fly's landing detection, against a fake `master` that replays a
scripted MAVLink conversation (plain unittest - also collected by pytest)."""
import unittest

from pymavlink.dialects.v20 import ardupilotmega as mav2

from engine.mavlink_mission import upload_and_fly

VEHICLE_SYSID = 1
MODE_AUTO = 3
# source -> one intermediate waypoint -> destination: items are HOME(0),
# TAKEOFF(1), WAYPOINT(2), LAND(3), so last_seq = 3.
ROUTE = [(-35.0, 149.0), (-35.001, 149.0), (-35.002, 149.0)]
# A plain two-point route: HOME(0), TAKEOFF(1), LAND(2).
TWO_POINT = [(-35.0, 149.0), (-35.003, 149.0)]
METRES_PER_DEG = 6_371_000.0 * 3.141592653589793 / 180


def _decode(message, src_system=VEHICLE_SYSID, src_component=mav2.MAV_COMP_ID_AUTOPILOT1):
    """Encode `message` as if sent by (src_system, src_component) and decode it
    back, so get_srcSystem()/get_srcComponent() work like a received message."""
    sender = mav2.MAVLink(None, srcSystem=src_system, srcComponent=src_component)
    return mav2.MAVLink(None).parse_buffer(message.pack(sender))[0]


_ENC = mav2.MAVLink(None)


def heartbeat(armed, src_system=VEHICLE_SYSID, mode=MODE_AUTO):
    base_mode = mav2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    if armed:
        base_mode |= mav2.MAV_MODE_FLAG_SAFETY_ARMED
    return _decode(_ENC.heartbeat_encode(
        mav2.MAV_TYPE_QUADROTOR, mav2.MAV_AUTOPILOT_ARDUPILOTMEGA, base_mode, mode,
        mav2.MAV_STATE_ACTIVE), src_system=src_system)


def reached(seq, src_system=VEHICLE_SYSID):
    return _decode(_ENC.mission_item_reached_encode(seq), src_system=src_system)


def position(lat, lon, src_system=VEHICLE_SYSID):
    return _decode(_ENC.global_position_int_encode(
        0, int(lat * 1e7), int(lon * 1e7), 0, 0, 0, 0, 0, 0), src_system=src_system)


def ack(command, result=mav2.MAV_RESULT_ACCEPTED):
    return _decode(_ENC.command_ack_encode(command, result))


def upload_handshake(route, arm_acks=(mav2.MAV_RESULT_ACCEPTED,)):
    """What the autopilot says from mission upload up to AUTO being confirmed
    and the cruise speed acknowledged. `arm_acks`: the result of each arm
    attempt, in order (upload_and_fly retries until one is ACCEPTED)."""
    items = len(route) + 1  # HOME placeholder + TAKEOFF + the rest
    script = [_decode(_ENC.mission_request_int_encode(255, 0, seq, 0)) for seq in range(items)]
    script.append(_decode(_ENC.mission_ack_encode(255, 0, mav2.MAV_MISSION_ACCEPTED, 0)))
    script.append(ack(mav2.MAV_CMD_DO_SET_MODE))  # GUIDED
    script += [ack(mav2.MAV_CMD_COMPONENT_ARM_DISARM, result) for result in arm_acks]
    script.append(heartbeat(armed=True))  # confirms AUTO
    script.append(ack(mav2.MAV_CMD_DO_CHANGE_SPEED))
    return script


class FakeMaster:
    """Just enough of a mavutil connection for upload_and_fly."""

    def __init__(self, script):
        self.target_system = VEHICLE_SYSID
        self.target_component = 0
        self.mav = mav2.MAVLink(_Sink(), srcSystem=255, srcComponent=190)
        self._script = list(script)

    def wait_heartbeat(self, timeout=None):
        return heartbeat(armed=False)

    def mode_mapping(self):
        return {"GUIDED": 4, "AUTO": MODE_AUTO}

    def recv_match(self, type=None, blocking=False, timeout=None):
        wanted = [type] if isinstance(type, str) else list(type or [])
        for i, message in enumerate(self._script):
            if not wanted or message.get_type() in wanted:
                return self._script.pop(i)
        if wanted == ["COMMAND_ACK"]:
            return None   # an ack that never comes: upload_and_fly times out and retries / reports
        raise AssertionError(f"script exhausted while waiting for {wanted}")


class _Sink:
    def write(self, _data):
        pass


def fly(flying_script, route=ROUTE, arm_acks=(mav2.MAV_RESULT_ACCEPTED,)):
    progress = []
    upload_and_fly(FakeMaster(upload_handshake(route, arm_acks) + flying_script), route, 10.0,
                   on_progress=progress.append, item_timeout_s=0.1)
    return progress


AT_LAND = position(*ROUTE[-1])


class LandingDetection(unittest.TestCase):
    def test_route_completed_then_disarm_on_land_point_returns(self):
        progress = fly([heartbeat(True), reached(1), reached(2), AT_LAND, heartbeat(True), heartbeat(False)])
        self.assertEqual(progress[-1], "Mission complete (landed and disarmed).")

    def test_disarm_after_only_takeoff_raises(self):
        with self.assertRaises(RuntimeError) as ctx:
            fly([heartbeat(True), reached(1), position(*ROUTE[0]), heartbeat(False)])
        self.assertIn("reached waypoint 1/3", str(ctx.exception))
        self.assertIn("m from the LAND point", str(ctx.exception))

    def test_disarmed_heartbeat_before_any_armed_is_ignored(self):
        # Were the first (disarmed) heartbeat counted, there is no position
        # yet and this would raise instead of completing.
        progress = fly([heartbeat(False), heartbeat(True), reached(1), reached(2), AT_LAND, heartbeat(False)])
        self.assertEqual(progress[-1], "Mission complete (landed and disarmed).")

    def test_heartbeats_from_another_sysid_are_ignored(self):
        # Another vehicle arming then disarming (at our LAND point, even)
        # must neither end nor fail this flight - otherwise this would raise
        # with our own vehicle still at the start.
        progress = fly([
            heartbeat(True), reached(1), position(*ROUTE[0]),
            position(*ROUTE[-1], src_system=2), heartbeat(True, src_system=2), heartbeat(False, src_system=2),
            reached(2), AT_LAND, heartbeat(False),
        ])
        self.assertEqual(progress[-1], "Mission complete (landed and disarmed).")

    def test_two_point_route_disarming_170_m_short_raises(self):
        # The "item before LAND reached" rule would call this complete: on a
        # two-point route that item is TAKEOFF. Only the position shows the
        # vehicle came down 170 m short of its destination.
        land_lat, land_lon = TWO_POINT[-1]
        short = (land_lat + 170.0 / METRES_PER_DEG, land_lon)
        with self.assertRaises(RuntimeError) as ctx:
            fly([heartbeat(True), reached(1), position(*short), heartbeat(False)], route=TWO_POINT)
        self.assertRegex(str(ctx.exception), r"\b170 m from the LAND point - reached waypoint 1/2\b")


class Arming(unittest.TestCase):
    def test_a_rejected_arm_is_retried(self):
        progress = fly([heartbeat(True), reached(1), reached(2), AT_LAND, heartbeat(False)],
                       arm_acks=(mav2.MAV_RESULT_TEMPORARILY_REJECTED, mav2.MAV_RESULT_ACCEPTED))
        self.assertTrue(any(line.startswith("Arm not confirmed yet (attempt 1/5") for line in progress), progress)
        self.assertEqual(progress[-1], "Mission complete (landed and disarmed).")

    def test_arm_never_accepted_raises(self):
        with self.assertRaises(RuntimeError) as ctx:
            fly([], arm_acks=(mav2.MAV_RESULT_DENIED,) * 5)
        self.assertIn("arm never accepted", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
