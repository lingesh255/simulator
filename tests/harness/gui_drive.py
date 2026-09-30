"""Drive the real MainWindow through its widgets and the map's own click
handler, timestamping every event. Scenario chosen by argv[1].

Run from the repo root with the venv python.
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

HARNESS = Path(__file__).resolve().parent
REPO = Path(os.environ.get("SIM_REPO", HARNESS.parents[1]))
OUT = HARNESS / "out"
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(HARNESS))
os.chdir(REPO)

from PySide6 import QtLocation, QtPositioning  # noqa: F401,E402
from PySide6.QtCore import Qt, QTimer  # noqa: E402
from PySide6.QtWidgets import QApplication, QMessageBox  # noqa: E402

from gui.main_window import MainWindow  # noqa: E402
from gui.theme import build_stylesheet, theme_manager  # noqa: E402

# Default Renode spawn (RenodeLauncher.PHYSICS_LATITUDE/LONGITUDE_DEG).
CANBERRA = (-35.363261, 149.165230)
# ~330 m north of it (the Phase 0 reference route).
NORTH = (-35.360300, 149.165230)
# A second site ~1.1 km east of the default, and a point ~110 m north of it.
EAST = (-35.363261, 149.177300)
EAST_DEST = (-35.362261, 149.177300)

T0 = time.monotonic()
LOG: list[str] = []


def log(msg: str) -> None:
    line = f"[{time.monotonic() - T0:8.1f}s] {msg}"
    LOG.append(line)
    print(line, flush=True)


# Never block on a modal dialog: record it and answer the default.
def _auto_box(kind):
    def box(parent, title, text, *a, **k):
        log(f"DIALOG {kind}: {title} :: {text[:400]!r}")
        return QMessageBox.Yes if kind == "question" else QMessageBox.Ok
    return box


QMessageBox.information = _auto_box("information")
QMessageBox.warning = _auto_box("warning")
QMessageBox.question = _auto_box("question")


def after(seconds):
    """A predicate that becomes true `seconds` after it is first polled."""
    start = []

    def pred():
        if not start:
            start.append(time.monotonic())
        return time.monotonic() - start[0] >= seconds
    return pred


class Driver:
    def __init__(self, w: MainWindow):
        self.w = w
        self.steps = []
        self.flights = []         # finished single-drone flights
        self.cur = None           # the single-drone flight in progress
        self.flight_starts = 0
        self.ready_count = 0
        self.launch_fail_count = 0
        w.renode_launch.ready.connect(self._on_ready)
        w.renode_launch.failed.connect(self._on_launch_failed)
        w.renode_launch.progress.connect(lambda m: log(f"renode.progress: {m}"))
        w.mavlink_flight.progress.connect(self._on_flight_progress)
        w.mavlink_flight.finished.connect(lambda: self._end_flight("finished"))
        w.mavlink_flight.failed.connect(lambda m: self._end_flight(f"FAILED: {m}"))
        w.mavlink_flight.batch_ready.connect(self._on_single_batch)
        w.plan_service.finished.connect(lambda r: log(f"plan ready: {r.plan_name} {' -> '.join(r.location_names)} "
                                                      f"per_drone={list((r.per_drone_waypoints or {}).keys())}"))
        w.plan_service.failed.connect(lambda m: log(f"plan FAILED: {m[:300]}"))
        orig_status = w.mission_planner.set_status

        def status(text):
            log(f"panel.status: {text}")
            orig_status(text)
        w.mission_planner.set_status = status
        orig_event = w.flight_log.log_event

        def event(text):
            log(f"flight_log: {text}")
            orig_event(text)
        w.flight_log.log_event = event
        orig_run = w.mavlink_flight.run_async

        def run_async(waypoints, conn, alt, sysid):
            self.flight_starts += 1
            self.cur = {"n": self.flight_starts, "start": time.monotonic(), "conn": conn,
                        "src": (waypoints[0].lat, waypoints[0].lon)}
            log(f"FLIGHT #{self.flight_starts} START conn={conn} src=({waypoints[0].lat:.6f},{waypoints[0].lon:.6f}) alt={alt}")
            orig_run(waypoints, conn, alt, sysid)
        w.mavlink_flight.run_async = run_async
        self.timer = QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.start(250)
        # GUI responsiveness: how late a 100 ms timer fires (event-loop lag),
        # and the process's CPU time over the run.
        self.lags_ms: list[float] = []
        self._probe_last = time.monotonic()
        self._probe = QTimer()
        self._probe.timeout.connect(self._probe_tick)
        self._probe.start(100)
        self._cpu0 = os.times()
        self._wall0 = time.monotonic()

    def _probe_tick(self):
        now = time.monotonic()
        self.lags_ms.append(max(0.0, (now - self._probe_last) * 1000 - 100))
        self._probe_last = now

    def responsiveness(self) -> str:
        lags = sorted(self.lags_ms) or [0.0]
        cpu = os.times()
        cpu_s = (cpu.user - self._cpu0.user) + (cpu.system - self._cpu0.system)
        wall = time.monotonic() - self._wall0
        fl = self.w.flight_log
        per = fl.table_update_s / fl.table_updates * 1000 if fl.table_updates else 0.0
        return (f"GUI event-loop lag on a 100 ms timer: median {lags[len(lags) // 2]:.1f} ms, "
                f"p99 {lags[int(len(lags) * 0.99)]:.1f} ms, max {lags[-1]:.1f} ms over {len(lags)} ticks; "
                f"GUI process CPU {cpu_s:.1f} s over {wall:.0f} s wall ({100 * cpu_s / max(wall, 1):.0f}% of one core); "
                f"table refreshes {fl.table_updates}, {fl.table_update_s * 1000:.0f} ms total, {per:.2f} ms each")

    def grab(self, name: str) -> str:
        OUT.mkdir(exist_ok=True)
        path = OUT / name
        self.w.flight_log.grab().save(str(path))
        log(f"SCREENSHOT {path.name} ({self.w.flight_log.width()}x{self.w.flight_log.height()}, "
            f"{theme_manager.current_theme()} theme)")
        return path.name

    def table_rows(self) -> list[list[str]]:
        m = self.w.flight_log.model
        return [[m.data(m.index(r, c)) or "" for c in range(m.columnCount())] for r in range(m.rowCount())]

    # ---- event capture ----
    def _on_ready(self, conn):
        self.ready_count += 1
        log(f"RENODE READY #{self.ready_count}: {conn}")

    def _on_launch_failed(self, msg):
        self.launch_fail_count += 1
        log(f"RENODE LAUNCH FAILED #{self.launch_fail_count}: {msg}")

    def _on_flight_progress(self, msg):
        log(f"flight.progress: {msg}")
        if self.cur is not None and msg.startswith("Arm ack") and "arm_ack" not in self.cur:
            self.cur["arm_ack"] = time.monotonic()

    def _on_single_batch(self, batch):
        if self.cur is None:
            return
        d = batch.drones[0]
        now = time.monotonic()
        armed = d.status.name == "IN_FLIGHT"
        if armed and "armed" not in self.cur:
            self.cur["armed"] = now
            log(f"  telemetry: ARMED (alt {d.altitude_m:.1f})")
        alt = d.altitude_m
        self.cur["max_alt"] = max(self.cur.get("max_alt", 0.0), alt)
        self.cur["last_pos"] = (d.lat, d.lon, alt)
        if alt >= 47.5 and "alt_reached" not in self.cur:
            self.cur["alt_reached"] = now
            log(f"  telemetry: takeoff altitude reached ({alt:.1f} m)")
        if "armed" in self.cur and not armed and "disarmed" not in self.cur and self.cur.get("max_alt", 0) > 5:
            self.cur["disarmed"] = now
            log(f"  telemetry: DISARMED after flight (alt {alt:.1f} m) at ({d.lat:.6f},{d.lon:.6f})")
        n = self.cur.setdefault("batches", 0) + 1
        self.cur["batches"] = n
        if n % 40 == 0:
            log(f"  telemetry: alt={alt:.1f} armed={armed} pos=({d.lat:.6f},{d.lon:.6f})")

    def _end_flight(self, how):
        if self.cur is None:
            log(f"flight end ({how}) with no current flight")
            return
        self.cur["end"] = time.monotonic()
        self.cur["how"] = how
        log(f"FLIGHT #{self.cur['n']} END: {how}")
        self.flights.append(self.cur)
        self.cur = None

    # ---- scripting ----
    def then(self, desc, predicate, action, timeout_s=600):
        self.steps.append((desc, predicate, action, timeout_s))
        return self

    def _tick(self):
        if not self.steps:
            return
        desc, pred, action, timeout_s = self.steps[0]
        if not hasattr(self, "_step_t"):
            self._step_t = time.monotonic()
        if time.monotonic() - self._step_t > timeout_s:
            log(f"STEP TIMEOUT: {desc}")
            self.steps.clear()
            self.finish()
            return
        try:
            ok = pred()
        except Exception as exc:  # noqa: BLE001
            log(f"predicate error in {desc}: {exc!r}")
            ok = False
        if ok:
            self.steps.pop(0)
            del self._step_t
            log(f"STEP: {desc}")
            action()

    def finish(self):
        self.report()
        log("closing window")
        self.w.close()
        QTimer.singleShot(500, QApplication.instance().quit)

    def report(self):
        log("==== single-drone flight timings (s, relative to each flight's START) ====")
        for f in self.flights:
            s = f["start"]

            def rel(k):
                return f"{f[k] - s:.1f}" if k in f else "-"
            log(f"flight #{f['n']} conn={f['conn']} src={f['src']} arm_ack={rel('arm_ack')} "
                f"armed={rel('armed')} alt_reached={rel('alt_reached')} disarmed/landed={rel('disarmed')} "
                f"end={rel('end')} ({f['how']}) max_alt={f.get('max_alt', 0):.1f} last_pos={f.get('last_pos')}")
        log(f"ready_count={self.ready_count} launch_fail_count={self.launch_fail_count} flights_started={self.flight_starts}")
        log(f"RESPONSIVENESS: {self.responsiveness()}")

    # ---- UI helpers ----
    def check_drones(self, names=("D1",)):
        lst = self.w.drone_management.profile_list
        for i in range(lst.count()):
            it = lst.item(i)
            it.setCheckState(Qt.Checked if any(it.text().startswith(n + " ") for n in names) else Qt.Unchecked)
        log(f"checked drones: sysids {self.w.drone_management.checked_sysids()}")

    def select_plan(self, name="travell"):
        combo = self.w.mission_planner.plan_combo
        combo.setCurrentIndex(combo.findData(name))
        log(f"selected plan {combo.currentData()!r}")

    def click_map(self, pt):
        log(f"map click at {pt} (mode={self.w.map_viewer.current_mode()})")
        self.w.map_viewer._on_map_clicked(*pt)

    def plan_btn_state(self):
        b = self.w.mission_planner.plan_btn
        return f"plan_btn enabled={b.isEnabled()} tooltip={b.toolTip()!r}"

    def click_plan(self):
        log(self.plan_btn_state())
        self.w.mission_planner.plan_btn.click()

    def click_launch(self):
        b = self.w.mission_planner.renode_launch_btn
        log(f"Launch Renode button enabled={b.isEnabled()} tooltip={b.toolTip()!r}")
        b.click()

    def click_stop(self):
        log("clicking Stop / Reset")
        self.w.drone_management.stop_btn.click()


def fly(d: Driver, start, dest, label, press_stop_after_landing=True):
    """Plan Mission -> start click -> destination click, then wait for that flight to end."""
    before = {}
    state = {}
    d.then(f"{label}: click Plan Mission", lambda: True, d.click_plan)
    d.then(f"{label}: click Start {start}", lambda: True, lambda: d.click_map(start))
    d.then(f"{label}: click Destination {dest}", lambda: True,
           lambda: (before.setdefault("n", len(d.flights)), d.click_map(dest)))

    def ended_or_landed():
        if len(d.flights) > before.get("n", 10**6):
            return True
        cur = d.cur
        if press_stop_after_landing and cur is not None and "disarmed" in cur:
            if time.monotonic() - cur["disarmed"] > 30 and not state.get("stopped"):
                state["stopped"] = True
                log("landed but the flight has not ended 30s later - pressing Stop like a user would")
                d.click_stop()
        return False

    d.then(f"{label}: flight ended", ended_or_landed, lambda: None, timeout_s=1200)


# Screenshot name prefix for table_demo (TABLE_DEMO_PREFIX=s12 reproduces Task 12's names).
SHOT_PREFIX = os.environ.get("TABLE_DEMO_PREFIX", "s13")


def table_demo(d: Driver, w: MainWindow) -> None:
    """Task 12: three drones on the local preview (the fastest flight), checking
    the Flight Log table, the Table/Logs toggle, the Logs badge and the log."""
    fl = w.flight_log
    st = {}

    def row_summary():
        return [f"{r[0]} | {r[1]} | {r[2]} | alt {r[5]} | spd {r[6]} | bat {r[8]} | upd {r[10]}" for r in d.table_rows()]

    d.then("dark theme", lambda: True, lambda: theme_manager.set_theme("dark"))
    d.then("check D1, D2, D3", lambda: True, lambda: d.check_drones(("D1", "D2", "D3")))
    d.then("empty state", lambda: True, lambda: (
        log(f"table page shown: {fl.pages.currentIndex() == fl.TABLE}; placeholder visible: "
            f"{fl._table_stack.currentWidget() is fl.placeholder}; rows {fl.model.rowCount()}"),
        d.grab(f"{SHOT_PREFIX}_empty_dark.png")))
    d.then("click Start", lambda: True, lambda: (w.map_viewer.mode_combo.setCurrentText("Set Start Point"),
                                                 d.click_map(CANBERRA)))
    d.then("click Destination", lambda: True, lambda: (w.map_viewer.mode_combo.setCurrentText("Set Destination Point"),
                                                       d.click_map(NORTH)))
    d.then("click Emulate", lambda: True, lambda: (log(f"Logs button before: {fl.logs_btn.text()!r}"),
                                                   w.drone_management.emulate_btn.click()))
    d.then("3 rows, all above 20 m", lambda: fl.model.rowCount() == 3 and all(
        float(r[5] or 0) > 20 for r in d.table_rows()), lambda: (
        st.__setitem__("first", d.table_rows()), log("table (t1):\n  " + "\n  ".join(row_summary()))),
        timeout_s=120)
    d.then("2 s later", after(2), lambda: (
        st.__setitem__("second", d.table_rows()), log("table (t2):\n  " + "\n  ".join(row_summary())),
        log(f"values changed between t1 and t2: {[a[3:8] != b[3:8] for a, b in zip(st['first'], st['second'])]}")))
    d.then("screenshot table dark", lambda: True, lambda: d.grab(f"{SHOT_PREFIX}_table_dark.png"))
    d.then("light theme", lambda: True, lambda: theme_manager.set_theme("light"))
    d.then("screenshot table light", after(0.5), lambda: d.grab(f"{SHOT_PREFIX}_table_light.png"))
    d.then("dark theme again", lambda: True, lambda: theme_manager.set_theme("dark"))

    def before_logs():
        st["badge"] = fl.logs_btn.text()
        st["log_lines_before"] = fl.view.blockCount()
        log(f"Logs button while on Table: {st['badge']!r}; Clear visible: {fl.clear_btn.isVisible()}")
        fl.logs_btn.click()
    d.then("click Logs", after(0.5), before_logs)

    def in_logs():
        text = fl.view.toPlainText()
        log(f"after clicking Logs: page={'Logs' if fl.pages.currentIndex() == fl.LOGS else 'Table'}, "
            f"button {fl.logs_btn.text()!r}, Clear visible: {fl.clear_btn.isVisible()}; log has "
            f"{text.count(' SYSID ')} telemetry lines and {text.count(' === ')} events")
        fl.log_event("(harness) an event while Logs is open")
        log(f"after an event while on Logs: button {fl.logs_btn.text()!r}")
        d.grab(f"{SHOT_PREFIX}_logs_dark.png")
    d.then("logs view", after(0.5), in_logs)

    def back_to_table():
        fl.table_btn.click()
        n0 = fl.view.blockCount()
        fl.log_event("(harness) an event while Table is showing")
        fl.log_event("(harness) another")
        log(f"back on Table: page={'Table' if fl.pages.currentIndex() == fl.TABLE else 'Logs'}, "
            f"Clear visible: {fl.clear_btn.isVisible()}, button after 2 events {fl.logs_btn.text()!r}; "
            f"log grew by {fl.view.blockCount() - n0} lines")
    d.then("back to Table", after(1), back_to_table)
    d.then("flight finished", lambda: not w.flight_sim.is_active(), lambda: None, timeout_s=180)
    d.then("1 s later (last refresh landed)", after(1.0), lambda: (
        log("table (final):\n  " + "\n  ".join(row_summary())), d.grab(f"{SHOT_PREFIX}_table_final_dark.png")))


def main():
    scenario = sys.argv[1]
    app = QApplication(sys.argv)
    theme_manager.set_theme("dark")
    app.setStyleSheet(build_stylesheet(theme_manager.palette()))
    # As main.py does: the app-wide stylesheet follows the theme.
    theme_manager.theme_changed.connect(lambda _n: app.setStyleSheet(build_stylesheet(theme_manager.palette())))
    w = MainWindow()
    w.show()
    d = Driver(w)
    mp = w.mission_planner
    log(f"scenario={scenario} pid={os.getpid()} repo={REPO}")

    if scenario in ("baseline", "step4_mock"):
        d.then("check D1", lambda: True, lambda: d.check_drones(("D1",)))
        d.then("select travell", lambda: True, d.select_plan)
        d.then("check 'Fly via real MAVLink'", lambda: True, lambda: mp.external_mavlink_check.setChecked(True))

    if scenario == "baseline":
        # Phase 0 flow: Launch Renode by hand, wait for ready, then Plan Mission at its location.
        d.then("click Launch Renode", lambda: True, d.click_launch)
        d.then("Renode ready", lambda: d.ready_count >= 1, lambda: log(d.plan_btn_state()), timeout_s=600)
        fly(d, CANBERRA, NORTH, "baseline")
        d.then("post-flight relaunch settled", lambda: d.ready_count >= 2 or d.launch_fail_count, lambda: None, timeout_s=600)
    elif scenario == "step4_mock":
        d.then("check mock vehicle + connection", lambda: True, lambda: (
            mp.mock_vehicle_check.setChecked(True), mp.mavlink_connection_edit.setText("udp:127.0.0.1:14550")))
        fly(d, EAST, EAST_DEST, "mock flight")
        d.then("settle 20s", after(20),
               lambda: log(f"after 20s: ready_count={d.ready_count} launch_in_progress={mp.renode_launch_in_progress} "
                           f"renode.progress lines={sum('renode.progress' in l for l in LOG)} {d.plan_btn_state()}"))
    elif scenario == "table_demo":
        table_demo(d, w)
    else:
        import fleet_scenarios  # noqa: E402  (Step 6 scenarios, next to this file)
        fleet_scenarios.build(scenario, d, w, log, fly, after, LOG, {"CANBERRA": CANBERRA, "NORTH": NORTH,
                                                                   "EAST": EAST, "EAST_DEST": EAST_DEST})

    d.then("done", lambda: True, d.finish)
    rc = app.exec()
    OUT.mkdir(exist_ok=True)
    (OUT / f"{scenario}.log").write_text("\n".join(LOG))
    sys.exit(rc)


if __name__ == "__main__":
    main()
