"""gui.mission_steps.mission_step_label: the Flight Log Mission column's mapping."""
import unittest

from gui.mission_steps import mission_step_label


class MissionStepLabel(unittest.TestCase):
    def check(self, text, label, level=None):
        self.assertEqual(mission_step_label(text), (label, level), text)

    def test_boot_phases_pass_through(self):
        for phase in ("Booting Renode", "GPS fix - waiting for armable", "Armable - waiting for the fleet"):
            self.check(phase, phase)

    def test_connecting_and_connected(self):
        self.check("Connecting to tcp:127.0.0.1:5763 ...", "Connecting")
        self.check("Heartbeat OK - system 1, component 0.", "Connected")

    def test_mission_uploaded(self):
        self.check("Mission uploaded and accepted (3 items).", "Mission uploaded")

    def test_arm_ack(self):
        self.check("Arm ack: COMMAND_ACK {command : 176, result : 0, progress : 0, result_param2 : 0, "
                   "target_system : 255, target_component : 0}", "Armed")
        self.check("Arm ack: COMMAND_ACK {command : 400, result : 4, progress : 0}", "Arm refused (result 4)", "warning")
        self.check("Arm ack: None", "Arm not acknowledged", "warning")

    def test_flying_mission(self):
        self.check("Flying mission.", "Starting mission")

    def test_takeoff_waypoint_land(self):
        self.check("[FC] Mission: 1 Takeoff", "Takeoff")
        self.check("Mission: 1 Takeoff", "Takeoff")
        self.check("Reached waypoint 1/2", "Reached waypoint 1/2")
        self.check("[FC] Mission: 2 Land", "Landing")
        self.check("[FC] Mission: 3 WP", "Waypoint")

    def test_takeoff_after_landing_is_the_firmware_mission_reset(self):
        self.assertEqual(mission_step_label("[FC] Mission: 1 Takeoff", previous="Landing"), (None, None))
        self.assertEqual(mission_step_label("[FC] Mission: 1 Takeoff", previous="Landed"), (None, None))
        self.assertEqual(mission_step_label("[FC] Mission: 1 Takeoff", previous="Armed"), ("Takeoff", None))

    def test_a_final_outcome_is_not_replaced_by_a_later_step(self):
        failed = "Failed - its physics sidecar exited mid-flight (exit code -9)"
        self.assertEqual(mission_step_label("Mission stopped.", previous=failed), (None, None))
        self.assertEqual(mission_step_label("[FC] Mission: 2 Land", previous=failed), (None, None))
        self.assertEqual(mission_step_label("Mission stopped.", previous="Completed"), (None, None))
        self.assertEqual(mission_step_label("Stopped", previous=failed), ("Stopped", None))
        self.assertEqual(mission_step_label("Mission stopped.", previous="Takeoff"), ("Stopping", None))

    def test_other_mission_command(self):
        self.check("[FC] Mission: 4 Loiter_Time", "Loiter_Time (item 4)")

    def test_final_outcomes_pass_through(self):
        self.check("Completed", "Completed")
        self.check("Failed - vehicle landed and disarmed 60 m from the LAND point", "Failed - vehicle landed and disarmed 60 m from the LAND point")
        self.check("Stopped", "Stopped")
        self.check("Not flown - fleet did not boot: port in use", "Not flown - fleet did not boot: port in use")
        self.check("Mission complete (landed and disarmed).", "Landed")
        self.check("Mission stopped.", "Stopping")

    def test_unmatched_lines_do_not_change_the_step(self):
        self.check("[FC] EKF3 IMU1 MAG0 in-flight yaw alignment complete", None)
        self.check("EKF3 IMU0 MAG0 in-flight yaw alignment complete", None)
        self.check("Waiting for heartbeat (timeout 10s) ...", None)
        self.check("[FC] Reached command #2", None)

    def test_trouble_is_flagged_without_a_step(self):
        self.check("[FC] EKF Failsafe: changed to Land Mode", None, "critical")
        self.check("[FC] PreArm: AHRS: EKF3 Yaw inconsistent 20 deg. Wait o", None, "warning")
        self.check("[FC] EKF variance: over thresholds", None, "warning")
        self.check("[FC] GPS Glitch or Compass error", None, "critical")  # "error" outranks "glitch"
        self.check("[FC] Potential Thrust Loss (3)", None, "critical")
        self.check("[FC] EKF Failsafe Cleared", None)  # a recovery, not trouble
        self.check("[FC] Glitch cleared", None)


if __name__ == "__main__":
    unittest.main()
