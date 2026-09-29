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
LAST_SEQ = len(ROUTE)


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


def upload_handshake():
    """What the autopilot says from mission upload up to AUTO being confirmed."""
    items = LAST_SEQ + 1
    script = [_decode(_ENC.mission_request_int_encode(255, 0, seq, 0)) for seq in range(items)]
    script.append(_decode(_ENC.mission_ack_encode(255, 0, mav2.MAV_MISSION_ACCEPTED, 0)))
    script.append(_decode(_ENC.command_ack_encode(mav2.MAV_CMD_DO_SET_MODE, 0)))
    script.append(heartbeat(armed=True))  # confirms AUTO
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
        raise AssertionError(f"script exhausted while waiting for {wanted}")


class _Sink:
    def write(self, _data):
        pass


def fly(flying_script):
    progress = []
    upload_and_fly(FakeMaster(upload_handshake() + flying_script), ROUTE, 10.0,
                   on_progress=progress.append)
    return progress


class LandingDetection(unittest.TestCase):
    def test_route_completed_then_disarm_returns(self):
        progress = fly([heartbeat(True), reached(1), reached(2), heartbeat(True), heartbeat(False)])
        self.assertEqual(progress[-1], "Mission complete (landed and disarmed).")

    def test_disarm_after_only_takeoff_raises(self):
        with self.assertRaises(RuntimeError) as ctx:
            fly([heartbeat(True), reached(1), heartbeat(False)])
        self.assertIn(f"reached waypoint 1/{LAST_SEQ}", str(ctx.exception))
        self.assertIn("likely a failsafe landing", str(ctx.exception))

    def test_disarmed_heartbeat_before_any_armed_is_ignored(self):
        # Were the first (disarmed) heartbeat counted, nothing has been reached
        # yet and this would raise instead of completing.
        progress = fly([heartbeat(False), heartbeat(True), reached(1), reached(2), heartbeat(False)])
        self.assertEqual(progress[-1], "Mission complete (landed and disarmed).")

    def test_heartbeats_from_another_sysid_are_ignored(self):
        # Another vehicle arming then disarming early must neither end nor fail
        # this flight - otherwise this would raise after only the takeoff item.
        progress = fly([
            heartbeat(True), reached(1),
            heartbeat(True, src_system=2), heartbeat(False, src_system=2),
            reached(2), heartbeat(False),
        ])
        self.assertEqual(progress[-1], "Mission complete (landed and disarmed).")


if __name__ == "__main__":
    unittest.main()
