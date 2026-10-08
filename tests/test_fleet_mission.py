"""services.fleet_mission.FleetMission: how the fleet reacts when a failed
drone's machine can't be confirmed halted in the shared Renode. No Renode:
the flights are stand-ins."""
import unittest

from PySide6.QtCore import QCoreApplication

from services.fleet_mission import DroneOutcome, FleetMission

_app = QCoreApplication.instance() or QCoreApplication([])


class FakeFlight:
    def __init__(self):
        self.aborted = False

    def abort(self):
        self.aborted = True

    def shutdown(self):
        pass

    def deleteLater(self):
        pass


class StopFailed(unittest.TestCase):
    def setUp(self):
        self.fleet = FleetMission()
        self.progress, self.phases, self.finished = [], [], []
        self.fleet.progress.connect(self.progress.append)
        self.fleet.drone_phase.connect(lambda sysid, text: self.phases.append((sysid, text)))
        self.fleet.finished.connect(lambda outcomes, ok: self.finished.append((outcomes, ok)))
        self.fleet._active = True
        self.fleet._outcomes = {n: DroneOutcome(n, f"D{n}", state="flying") for n in (1, 2, 3)}
        self.fleet._flights = {n: FakeFlight() for n in (1, 2, 3)}
        # what the watchdog has already done to drone 2 when its sidecar died
        self.fleet._outcomes[2].state = "failed"
        self.fleet._outcomes[2].reason = "its physics sidecar exited mid-flight (exit code -9)"
        self.fleet._flights[2].abort()

    def tearDown(self):
        self.fleet.shutdown()

    def test_the_whole_fleet_is_stopped_and_told_why(self):
        flights = dict(self.fleet._flights)
        self.fleet._on_stop_failed(2, "could not confirm that drone2 (SYSID 2) is halted ... Connection refused")
        self.assertTrue(self.progress[0].startswith("ERROR: D2 (SYSID 2) COULD NOT BE STOPPED - could not confirm"))
        self.assertIn("Stopping the other 2 drone(s)", self.progress[1])
        self.assertEqual(self.fleet._outcomes[2].reason,
                         "its physics sidecar exited mid-flight (exit code -9); and it could not be halted")
        for n in (1, 3):
            self.assertEqual(self.fleet._outcomes[n].state, "failed")
            self.assertEqual(self.fleet._outcomes[n].reason, "fleet stopped because D2 (SYSID 2) could not be halted")
            self.assertTrue(flights[n].aborted)
        self.assertIn((2, "Failed - its physics sidecar exited mid-flight (exit code -9); and it could not be halted"),
                      self.phases)
        self.assertIn((1, "Failed - fleet stopped because D2 (SYSID 2) could not be halted"), self.phases)
        # the aborted flights then end; the fleet wraps up as NOT complete, keeping the reasons
        for n in (1, 2, 3):
            self.fleet._on_drone_ended(n, None)
        outcomes, ok = self.finished[0]
        self.assertFalse(ok)
        self.assertEqual([o.state for o in outcomes], ["failed", "failed", "failed"])
        self.assertFalse(self.fleet.active)

    def test_ignored_once_the_fleet_has_ended(self):
        self.fleet._active = False
        self.fleet._on_stop_failed(2, "late")
        self.assertEqual(self.progress, [])
        self.assertEqual(self.fleet._outcomes[1].state, "flying")


if __name__ == "__main__":
    unittest.main()
