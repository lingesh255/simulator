"""Run the real main.main(); once the window shows, tick D1 + 'Fly via real
MAVLink' and click Launch Renode, and print READY when it boots. If the file
out/close_now appears, call window.close() (the title-bar X's closeEvent path).
Driven by step1_test.py."""
import os
import sys
from pathlib import Path

HARNESS = Path(__file__).resolve().parent
REPO = HARNESS.parents[1]
sys.path.insert(0, str(REPO))
os.chdir(REPO)

import main as app_main  # noqa: E402
from PySide6.QtCore import Qt, QTimer  # noqa: E402
from gui.main_window import MainWindow  # noqa: E402

_orig_show = MainWindow.show


def _show(self):
    _orig_show(self)

    def setup():
        lst = self.drone_management.profile_list
        for i in range(lst.count()):
            it = lst.item(i)
            it.setCheckState(Qt.Checked if it.text().startswith("D1 ") else Qt.Unchecked)
        self.mission_planner.external_mavlink_check.setChecked(True)
        self.renode_launch.ready.connect(lambda c: print(f"READY {c}", flush=True))
        self.renode_launch.failed.connect(lambda m: print(f"LAUNCH FAILED {m}", flush=True))
        print(f"APP PID {os.getpid()} - clicking Launch Renode", flush=True)
        self.mission_planner.renode_launch_btn.click()

    def poll_close():
        trigger = HARNESS / "out" / "close_now"
        if trigger.exists():
            trigger.unlink()
            print("closing window (trigger file)", flush=True)
            self.close()

    QTimer.singleShot(500, setup)
    self._close_poll = QTimer(self)
    self._close_poll.timeout.connect(poll_close)
    self._close_poll.start(200)


MainWindow.show = _show
app_main.main()
