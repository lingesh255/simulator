"""Fleet scenarios for gui_drive.py (Steps 6-8).

    fleet_travell / fleet_search / fleet_stop / fleet_bootfail       2 drones
    fleet_travell3 / fleet_search3 / fleet_stop3 / fleet_bootfail3   3 drones
    fleet_kill2      3 drones, drone 2's Renode killed mid-flight
    fleet_dupsysid   two checked profiles sharing a SYSID - must be refused
    fleet_vform3     3 drones, V-formation
    fleet_grid4      4 drones, grid formation (adds a temporary harness_grid_d4, SYSID 4)
    fleet_sidecar3   3 drones, drone 2's physics sidecar killed mid-flight
    fleet_twice3     two fleet missions back to back in ONE app session
    fleet_switch3    per-drone -> shared -> per-drone fleets in one app session
    fleet_stopthen3  Stop mid-flight, then a new mission straight away
    fleet_travell5 / fleet_travell6   5 / 6 drones (temporary harness_dN profiles)
    fleet_long3      3 drones on a ~3 km route (about 15 minutes of flying)
    fleet_sidecar4   4 drones, drone 2's physics sidecar killed mid-flight
    fleet_close3 / fleet_sigterm3 / fleet_sigkill3   the app window closed /
                     SIGTERM / SIGKILL to the app with 3 drones airborne
    fleet_single1    ONE drone checked (must use the single-drone path)
    fleet_norenode1  a typed address, then the mock vehicle: no Renode
    fleet_stopboot3  3 drones, Stop pressed 60 s into the fleet boot
    fleet_renodekill3  3 drones, every Renode process killed mid-flight (in the
                     shared mode that is the one Renode)

FLEET_EMULATION=shared in the environment flies any of them with "Fleet
emulation" set to "Shared Renode (low memory)" (not saved to the settings
file); the default is whatever the settings file says.
"""
import itertools
import math
import os
import subprocess

from PySide6.QtCore import QTimer
import sys
import time
from pathlib import Path

HARNESS = Path(__file__).resolve().parent
NAMES = ("D1", "D2", "D3", "D4")
# Temporary extra drones for fleets of more than 3 (travell5/6, sidecar4): their
# own names and files (data/profiles/harness_d<N>.json), SYSID N.
EXTRA = {4: "harness_d4", 5: "harness_d5", 6: "harness_d6"}
# fleet_grid4's temporary fourth drone. Its own name - and so its own file,
# data/profiles/harness_grid_d4.json - so a real D4.json is never touched.
GRID_D4 = "harness_grid_d4"


def build(scenario, d, w, log, fly, after, LOG, pts):
    mp = w.mission_planner
    n = 3 if scenario == "fleet_kill2" else (int(scenario[-1]) if scenario[-1].isdigit() else 2)
    emulation = os.environ.get("FLEET_EMULATION")   # "shared" / "per_drone" / unset = the saved setting
    st = {"finished": [], "boot_failed": [], "flying": 0, "all_sysid_batch": None, "free": None,
          "max_alt": {}, "last": {}, "dummy": None, "boot_started": None}

    w.fleet.finished.connect(lambda outcomes, ok: (
        st["finished"].append((outcomes, ok)),
        log(f"FLEET FINISHED signal: ok={ok} " + "; ".join(f"{o.name}/{o.sysid}={o.state} {o.reason}" for o in outcomes))))
    w.fleet.boot_failed.connect(lambda m: (st["boot_failed"].append(m), log(
        f"FLEET BOOT_FAILED signal ({time.monotonic() - (st['boot_started'] or time.monotonic()):.1f}s after the "
        f"fleet boot started): {m}")))
    w.fleet.flying.connect(lambda: (st.__setitem__("flying", st["flying"] + 1), log("FLEET FLYING signal")))
    w.fleet.progress.connect(lambda m: st.__setitem__("boot_started", st["boot_started"] or time.monotonic())
                             if m.startswith("Booting a fleet") else None)

    def resident_sets():
        from engine.renode_launcher import _renode_processes
        for pid, argv in _renode_processes():
            rss = subprocess.run(["ps", "-o", "rss=", "-p", str(pid)], capture_output=True, text=True).stdout.strip()
            port = next((argv[i + 1] for i, a in enumerate(argv[:-1]) if a == "--physics-port"), None) or \
                next((p.split()[2] for p in " ".join(argv).split("; ") if p.startswith("physics Connect")), None) or \
                ("all - the shared Renode" if "fleet.resc" in " ".join(argv) else "?")
            log(f"  pid {pid} {argv[0].rsplit('/', 1)[-1]} (physics port {port}): RSS {int(rss) / 1024:.0f} MB"
                if rss else f"  pid {pid}: gone")

    def on_fleet_batch(batch):
        sysids = sorted(x.sysid for x in batch.drones)
        for x in batch.drones:
            st["max_alt"][x.sysid] = max(st["max_alt"].get(x.sysid, 0.0), x.altitude_m)
            st["last"][x.sysid] = (round(x.lat, 6), round(x.lon, 6), round(x.altitude_m, 1))
        if len(sysids) >= n and st["all_sysid_batch"] is None:
            st["all_sysid_batch"] = sysids
            log(f"first merged batch with {len(sysids)} sysids: " + ", ".join(
                f"sysid {x.sysid}: ({x.lat:.6f},{x.lon:.6f}) alt {x.altitude_m:.1f} {x.status.name}" for x in batch.drones))
        if batch.tick % 90 == 0:
            log("  fleet telemetry: " + ", ".join(
                f"{x.sysid}: ({x.lat:.6f},{x.lon:.6f}) alt {x.altitude_m:.1f} {x.status.name}" for x in batch.drones))
        if st["free"] is None and len(batch.drones) >= n and all(x.altitude_m > 20 for x in batch.drones):
            st["free"] = subprocess.run(["free", "-h"], capture_output=True, text=True).stdout
            log(f"free -h while all {n} drones are airborne:\n" + st["free"])
            resident_sets()
            QTimer.singleShot(3000, lambda: (d.grab(f"{scenario}_table_midflight.png"), log_table("mid-flight")))
    w.fleet.batch_ready.connect(on_fleet_batch)

    def log_table(when):
        rows = d.table_rows()
        log(f"Flight Log table ({when}):\n  " + "\n  ".join(
            f"{r[0]} | {r[1]} | {r[2]} | alt {r[5]} | bat {r[8]} | upd {r[10]}" for r in rows))
    w.fleet.finished.connect(lambda *_: QTimer.singleShot(
        1500, lambda: (d.grab(f"{scenario}_table_final.png"), log_table("after the finish"))))

    def common(plan, names=NAMES[:n]):
        d.then(f"check {', '.join(names)}", lambda: True, lambda: d.check_drones(names))
        d.then(f"select {plan}", lambda: True, lambda: d.select_plan(plan))
        d.then("check 'Fly via real MAVLink'", lambda: True, lambda: mp.external_mavlink_check.setChecked(True))

        def pick_emulation():
            combo = mp.fleet_emulation_combo
            if emulation:
                combo.blockSignals(True)   # this run only - don't write the settings file
                combo.setCurrentIndex(combo.findData(emulation))
                combo.blockSignals(False)
            log(f"fleet emulation: {combo.currentText()!r} (enabled={combo.isEnabled()}, "
                f"shared_renode_fleet()={mp.shared_renode_fleet()})")
        d.then("fleet emulation", lambda: True, pick_emulation)

    def plan_travell(dest):
        d.then("click Plan Mission", lambda: True, d.click_plan)
        d.then("click Start", lambda: True, lambda: d.click_map(pts["CANBERRA"]))
        d.then("click Destination", lambda: True, lambda: d.click_map(dest))

    def plan_search():
        base = pts["CANBERRA"]
        # a ~45 m x 45 m search area just north-east of the base
        dlat, dlon = 0.0004, 0.0005
        corners = [(base[0] + 0.0002, base[1] + 0.0001), (base[0] + 0.0002 + dlat, base[1] + 0.0001),
                   (base[0] + 0.0002 + dlat, base[1] + 0.0001 + dlon), (base[0] + 0.0002, base[1] + 0.0001 + dlon)]
        d.then("click Plan Mission", lambda: True, d.click_plan)
        d.then("click base", lambda: True, lambda: d.click_map(base))
        for i, c in enumerate(corners):
            d.then(f"click search-area corner {i + 1}", lambda: True, lambda c=c: d.click_map(c))

    def landing_spread():
        landed = {s: p for s, p in st["last"].items() if p[2] < 1.0}
        pairs = []
        for (a, pa), (b, pb) in itertools.combinations(sorted(landed.items()), 2):
            dy = (pa[0] - pb[0]) * 111320
            dx = (pa[1] - pb[1]) * 111320 * math.cos(math.radians(pa[0]))
            pairs.append(f"{a}-{b}: {math.hypot(dx, dy):.1f} m")
        return ", ".join(pairs) or "-"

    def summary():
        log(f"landing points pairwise: {landing_spread()}")
        log(f"SUMMARY: finished={len(st['finished'])} boot_failed={len(st['boot_failed'])} flying_signals={st['flying']} "
            f"all_sysid_batch={st['all_sysid_batch']} max_alt={ {k: round(v, 1) for k, v in st['max_alt'].items()} } "
            f"last_pos={st['last']} single-drone flights_started={d.flight_starts} single ready_count={d.ready_count}")
        pg = subprocess.run(["pgrep", "-af", "[r]enode-bin/[r]enode|[r]enode-physics"], capture_output=True, text=True).stdout
        log(f"pgrep (Renode executables) at summary: {pg.strip() or '(nothing)'}")

    def all_above(m):
        return len(st["max_alt"]) >= n and all(st["last"][s][2] > m for s in st["last"])

    short_north = (pts["CANBERRA"][0] + 0.001, pts["CANBERRA"][1])  # ~110 m north

    def set_emulation(mode):
        combo = mp.fleet_emulation_combo
        combo.blockSignals(True)   # this run only - don't write the settings file
        combo.setCurrentIndex(combo.findData(mode))
        combo.blockSignals(False)
        log(f"fleet emulation set to {combo.currentText()!r} (combo enabled={combo.isEnabled()})")

    def processes():
        return subprocess.run(["pgrep", "-af", "[r]enode-bin/[r]enode|[r]enode-physics"], capture_output=True,
                              text=True).stdout.strip().replace("\n", " || ")[:700] or "(nothing)"

    def extra_profiles(count):
        """Temporary profiles for drones 4..count; returns (names, add step, remove function)."""
        from contracts.gui_orchestration import DroneConfig
        paths = {k: w.store.profiles_dir / f"{EXTRA[k]}.json" for k in range(4, count + 1)}
        for path in paths.values():
            if path.exists():
                raise SystemExit(f"{scenario}: {path} already exists (left over from an earlier run?) - "
                                 "not touching it; remove it by hand and rerun")

        def add():
            for k in paths:
                w.store.save_profile(DroneConfig(name=EXTRA[k], sysid=k, mass_kg=1.5, max_velocity_mps=15.0,
                                                 battery_capacity_mah=15200.0, cruise_altitude_m=50.0))
            w.drone_management.reload_profiles()
            log(f"added temporary profiles {[p.name for p in paths.values()]}")

        def remove():
            for path in paths.values():
                if path.exists():
                    path.unlink()
            w.drone_management.reload_profiles()
            log(f"removed the temporary profiles; profile files now "
                f"{sorted(q.name for q in w.store.profiles_dir.glob('*.json'))}")
        return (*NAMES[:3], *(EXTRA[k] for k in paths)), add, remove

    def one_mission(k, mode, dest=None, stop_above=None):
        """Mission number k (1-based) of this app session: set the mode, plan, fly to the end
        (or press Stop once every drone is above `stop_above` m), then log what a user sees."""
        def reset_trackers():
            st["free"] = None
            st["all_sysid_batch"] = None
            st["max_alt"].clear()
            st["last"].clear()
            st["boot_started"] = None
            set_emulation(mode)
            log(f"MISSION {k} ({mode}): {d.plan_btn_state()}; table rows before planning: {len(d.table_rows())}; "
                f"processes: {processes()}")
        d.then(f"mission {k}: prepare ({mode})", lambda: True, reset_trackers)
        d.then(f"mission {k}: click Plan Mission", lambda: mp.plan_btn.isEnabled(), d.click_plan, timeout_s=120)
        d.then(f"mission {k}: click Start", lambda: True, lambda: d.click_map(pts["CANBERRA"]))
        d.then(f"mission {k}: click Destination", lambda: True, lambda: d.click_map(dest or short_north))
        d.then(f"mission {k}: fleet boot under way", lambda: st["boot_started"] or st["boot_failed"], lambda: log(
            f"mission {k}: while booting, fleet emulation combo enabled = {mp.fleet_emulation_combo.isEnabled()} "
            f"(must be False), table rows = {len(d.table_rows())}"), timeout_s=120)
        if stop_above is not None:
            d.then(f"mission {k}: all {n} drones above {stop_above} m", lambda: all_above(stop_above), lambda: None,
                   timeout_s=1500)
            d.then(f"mission {k}: click Stop mid-flight", lambda: True, d.click_stop)
        d.then(f"mission {k}: fleet finished", lambda: len(st["finished"]) + len(st["boot_failed"]) >= k,
               lambda: None, timeout_s=2400)

        def after_mission():
            outcomes = st["finished"][-1] if st["finished"] else None
            log(f"MISSION {k} ({mode}) ENDED: finished={len(st['finished'])} boot_failed={len(st['boot_failed'])} "
                f"ok={outcomes[1] if outcomes else None}; processes 3 s later: {processes()}; "
                f"combo enabled={mp.fleet_emulation_combo.isEnabled()}; {d.plan_btn_state()}")
            log_table(f"after mission {k}")
            d.grab(f"{scenario}_mission{k}_table.png")
        d.then(f"mission {k}: 3 s later", after(3), after_mission)

    if scenario in ("fleet_travell", "fleet_travell3"):
        common("travell")
        plan_travell(short_north)
        d.then("fleet finished", lambda: st["finished"] or st["boot_failed"], lambda: None, timeout_s=1800)
        d.then("settle 10s", after(10), summary)
    elif scenario in ("fleet_search", "fleet_search3"):
        common("Search")
        plan_search()
        d.then("fleet finished", lambda: st["finished"] or st["boot_failed"], lambda: None, timeout_s=2400)
        d.then("settle 10s", after(10), summary)
    elif scenario in ("fleet_stop", "fleet_stop3"):
        common("travell")
        plan_travell(short_north)
        d.then(f"all {n} drones above 15 m", lambda: all_above(15), lambda: None, timeout_s=1200)
        d.then("click Stop mid-flight", lambda: True, d.click_stop)
        d.then("fleet finished after Stop", lambda: st["finished"], lambda: None, timeout_s=120)
        d.then("settle 5s", after(5), summary)
    elif scenario == "fleet_twice3":
        mode = emulation or mp.fleet_emulation_combo.currentData()
        common("travell")
        one_mission(1, mode)
        one_mission(2, mode)
        d.then("settle 5s", after(5), summary)
    elif scenario == "fleet_switch3":
        common("travell")
        one_mission(1, "per_drone")
        one_mission(2, "shared")
        one_mission(3, "per_drone")
        d.then("settle 5s", after(5), summary)
    elif scenario == "fleet_stopthen3":
        mode = emulation or mp.fleet_emulation_combo.currentData()
        common("travell")
        one_mission(1, mode, stop_above=15)
        one_mission(2, mode)
        d.then("settle 5s", after(5), summary)
    elif scenario in ("fleet_travell5", "fleet_travell6", "fleet_sidecar4"):
        names, add, remove = extra_profiles(n)
        d.then("add temporary profiles", lambda: True, add)
        common("travell", names=names)
        plan_travell(short_north)
        if scenario == "fleet_sidecar4":
            d.then("all 4 drones above 20 m", lambda: all_above(20), lambda: None, timeout_s=1500)

            def kill_sidecar2():
                from engine.renode_launcher import _renode_processes
                victims = [pid for pid, argv in _renode_processes()
                           if argv[0].endswith("renode-physics") and "9004" in argv]
                log(f"SIGKILL drone 2's physics sidecar only: pids {victims} (processes before: {processes()})")
                for pid in victims:
                    subprocess.run(["kill", "-9", str(pid)])
            d.then("kill drone 2's physics sidecar", lambda: True, kill_sidecar2)
        d.then("fleet finished", lambda: st["finished"] or st["boot_failed"], lambda: None, timeout_s=3000)
        d.then("settle 10s", after(10), lambda: (summary(), remove()))
    elif scenario == "fleet_long3":
        common("travell")
        far_north = (pts["CANBERRA"][0] + 0.027, pts["CANBERRA"][1])   # ~3 km north
        plan_travell(far_north)
        d.then("fleet finished", lambda: st["finished"] or st["boot_failed"], lambda: None, timeout_s=3000)
        d.then("settle 10s", after(10), summary)
    elif scenario in ("fleet_close3", "fleet_sigterm3", "fleet_sigkill3"):
        common("travell")
        plan_travell(short_north)
        d.then("all 3 drones above 20 m", lambda: all_above(20), lambda: None, timeout_s=1500)

        def end_the_app():
            import signal
            log(f"processes with 3 airborne: {processes()}")
            if scenario == "fleet_close3":
                log("closing the app window mid-flight")
                w.close()
                QTimer.singleShot(500, lambda: __import__("PySide6.QtWidgets").QtWidgets.QApplication.instance().quit())
            else:
                sig = signal.SIGTERM if scenario == "fleet_sigterm3" else signal.SIGKILL
                log(f"sending {sig.name} to the app itself (pid {os.getpid()}) mid-flight")
                os.kill(os.getpid(), sig)
        d.then("end the app mid-flight", lambda: True, end_the_app)
        d.then("(the app should be gone before this)", after(60), lambda: log("STILL ALIVE 60 s later"))
    elif scenario == "fleet_single1":
        # Shared selected (FLEET_EMULATION=shared) but only ONE drone checked: the normal
        # single-drone path, instance 0 on tcp:127.0.0.1:5762, no fleet.
        common("travell", names=("D1",))
        fly(d, pts["CANBERRA"], short_north, "single drone")
        d.then("settle 5s", after(5), lambda: (
            log(f"single-drone check: fleet finished signals={len(st['finished'])} boot_failed={len(st['boot_failed'])} "
                f"flying signals={st['flying']}; single flights={[(f['conn'], f['how']) for f in d.flights]}; "
                f"shared_renode_fleet()={mp.shared_renode_fleet()}"), summary()))
    elif scenario == "fleet_norenode1":
        # Shared selected, but the vehicle is not Renode: a typed address, then the mock vehicle.
        common("travell", names=("D1",))
        d.then("type an address of our own", lambda: True, lambda: (
            mp.mavlink_connection_edit.setText("udp:127.0.0.1:14599"),
            log(f"typed address: combo enabled={mp.fleet_emulation_combo.isEnabled()} "
                f"shared_renode_fleet()={mp.shared_renode_fleet()}")))
        fly(d, pts["EAST"], pts["EAST_DEST"], "typed address (nothing listens there)")
        d.then("after the typed-address attempt", after(3), lambda: log(
            f"typed address: flights={[(f['conn'], f['how'][:90]) for f in d.flights]} ready_count={d.ready_count} "
            f"processes: {processes()}"))
        d.then("switch to the mock vehicle", lambda: True, lambda: (
            mp.mock_vehicle_check.setChecked(True), mp.mavlink_connection_edit.setText("udp:127.0.0.1:14550"),
            log(f"mock vehicle: combo enabled={mp.fleet_emulation_combo.isEnabled()}")))
        fly(d, pts["EAST"], pts["EAST_DEST"], "mock vehicle")
        d.then("settle 10s", after(10), lambda: log(
            f"no-Renode check: flights={[(f['conn'], f['how'][:60]) for f in d.flights]} ready_count={d.ready_count} "
            f"renode.progress lines={sum('renode.progress' in l for l in LOG)} fleet signals="
            f"{len(st['finished']) + len(st['boot_failed'])} processes: {processes()}"))
    elif scenario == "fleet_stopboot3":
        common("travell")
        plan_travell(short_north)
        # 60 s into the boot: Renode is up and the firmware is booting, no drone is armable yet
        d.then("60s into the fleet boot", lambda: st["boot_started"] and time.monotonic() - st["boot_started"] > 60,
               lambda: log("Renode processes 60s into the boot: " + (subprocess.run(
                   ["pgrep", "-af", "[r]enode-bin/[r]enode|[r]enode-physics"], capture_output=True, text=True
               ).stdout.strip().replace("\n", " || ")[:500] or "(nothing)")), timeout_s=300)
        d.then("click Stop mid-boot", lambda: True, d.click_stop)
        d.then("fleet boot reported failed/stopped", lambda: st["boot_failed"] or st["finished"], lambda: None,
               timeout_s=120)
        d.then("settle 10s", after(10), summary)
    elif scenario == "fleet_renodekill3":
        common("travell")
        plan_travell(short_north)
        d.then("all 3 drones above 20 m", lambda: all_above(20), lambda: None, timeout_s=1500)

        def kill_renode():
            from engine.renode_launcher import _renode_processes
            procs = [(pid, argv[0].rsplit("/", 1)[-1]) for pid, argv in _renode_processes()]
            victims = [pid for pid, name in procs if name == "renode"]
            log(f"SIGKILL every Renode (not the sidecars): pids {victims} (processes before: {procs})")
            for pid in victims:
                subprocess.run(["kill", "-9", str(pid)])
        d.then("kill Renode", lambda: True, kill_renode)
        d.then("fleet finished", lambda: st["finished"], lambda: None, timeout_s=300)
        d.then("settle 10s", after(10), summary)
    elif scenario in ("fleet_bootfail", "fleet_bootfail3"):
        port = 5762 + n  # the last drone's MAVLink port (instance n)

        def occupy():
            st["dummy"] = subprocess.Popen([sys.executable, str(HARNESS / "port_squatter.py"), str(port)])
            time.sleep(1)
            ss = subprocess.run(f"ss -ltnp | grep ':{port} '", shell=True, capture_output=True, text=True).stdout
            log(f"dummy listener pid {st['dummy'].pid} on {port}: {ss.strip()}")
        d.then(f"pre-occupy port {port}", lambda: True, occupy)
        common("travell")
        plan_travell(short_north)
        d.then("fleet boot failed", lambda: st["boot_failed"] or st["finished"], lambda: None, timeout_s=900)
        d.then("wait 20s for any stray mission", after(20), lambda: (
            summary(), st["dummy"].kill(), log("dummy listener killed")))
    elif scenario == "fleet_kill2":
        common("travell")
        plan_travell(short_north)
        d.then("all 3 drones above 20 m", lambda: all_above(20), lambda: None, timeout_s=1200)

        def kill_drone2():
            from engine.renode_launcher import _renode_processes
            victims = [(pid, argv) for pid, argv in _renode_processes()
                       if argv[0].endswith("renode-bin/renode") and "physics Connect 9004 " in " ".join(argv)]
            for pid, _ in victims:
                log(f"SIGKILL drone 2's Renode only: pid {pid}")
                subprocess.run(["kill", "-9", str(pid)])
            log(f"Renode processes right after the kill: {[(p, a[0].rsplit('/', 1)[-1]) for p, a in _renode_processes()]}")
        d.then("kill drone 2's Renode", lambda: True, kill_drone2)
        d.then("5s later: drone 2's sidecar", after(5), lambda: log(
            "Renode processes 5s after the kill: " + (subprocess.run(
                ["pgrep", "-af", "[r]enode-bin/[r]enode|[r]enode-physics"], capture_output=True, text=True
            ).stdout.strip().replace("\n", " || ") or "(nothing)")))
        d.then("fleet finished", lambda: st["finished"], lambda: None, timeout_s=1800)
        d.then("settle 10s", after(10), summary)
    elif scenario == "fleet_sidecar3":
        common("travell")
        plan_travell(short_north)
        d.then("all 3 drones above 20 m", lambda: all_above(20), lambda: None, timeout_s=1500)

        def kill_sidecar2():
            from engine.renode_launcher import _renode_processes
            before = [(pid, argv[0].rsplit("/", 1)[-1]) for pid, argv in _renode_processes()]
            victims = [pid for pid, argv in _renode_processes()
                       if argv[0].endswith("renode-physics") and "9004" in argv]
            for pid in victims:
                log(f"SIGKILL drone 2's physics sidecar only: pid {pid} (processes before: {before})")
                subprocess.run(["kill", "-9", str(pid)])
        d.then("kill drone 2's physics sidecar", lambda: True, kill_sidecar2)
        d.then("10s later: processes", after(10), lambda: log(
            "Renode processes 10s after the kill: " + (subprocess.run(
                ["pgrep", "-af", "[r]enode-bin/[r]enode|[r]enode-physics"], capture_output=True, text=True
            ).stdout.strip().replace("\n", " || ")[:600] or "(nothing)")))
        d.then("fleet finished", lambda: st["finished"], lambda: None, timeout_s=1800)
        d.then("settle 10s", after(10), summary)
    elif scenario in ("fleet_vform3", "fleet_grid4"):
        if scenario == "fleet_grid4":
            from contracts.gui_orchestration import DroneConfig
            grid_path = w.store.profiles_dir / f"{GRID_D4}.json"
            if grid_path.exists():
                raise SystemExit(f"fleet_grid4: {grid_path} already exists (left over from an earlier run?) - "
                                 "not touching it; remove it by hand and rerun")

            def add_d4():
                w.store.save_profile(DroneConfig(name=GRID_D4, sysid=4, mass_kg=1.5, max_velocity_mps=15.0,
                                                 battery_capacity_mah=15200.0, cruise_altitude_m=50.0))
                w.drone_management.reload_profiles()
                log(f"added temporary profile {GRID_D4} (SYSID 4) as {grid_path.name}; "
                    f"profile files now {sorted(p.name for p in w.store.profiles_dir.glob('*.json'))}")
            d.then(f"add temporary {GRID_D4}", lambda: True, add_d4)
        common("vformation" if scenario == "fleet_vform3" else "gridformation",
               names=NAMES[:3] if scenario == "fleet_vform3" else (*NAMES[:3], GRID_D4))
        plan_travell(short_north)
        d.then("fleet finished", lambda: st["finished"] or st["boot_failed"], lambda: None, timeout_s=2400)

        def finish():
            summary()
            if scenario == "fleet_grid4":
                grid_path.unlink()
                w.drone_management.reload_profiles()
                log(f"removed {grid_path.name}; profile files now "
                    f"{sorted(p.name for p in w.store.profiles_dir.glob('*.json'))}")
        d.then("settle 10s", after(10), finish)
    elif scenario == "fleet_dupsysid":
        from contracts.gui_orchestration import DroneConfig
        store = w.store

        def add_duplicate():
            store.save_profile(DroneConfig(name="DUP", sysid=1, mass_kg=1.5, max_velocity_mps=15.0,
                                           battery_capacity_mah=15200.0, cruise_altitude_m=50.0))
            w.drone_management.reload_profiles()
            log(f"added temporary profile DUP (SYSID 1); profiles now {[p.name for p in store.list_profiles()]}")
        d.then("add profile DUP sharing SYSID 1", lambda: True, add_duplicate)
        common("travell", names=("D1", "DUP"))
        plan_travell(short_north)

        def check_and_restore():
            from engine.renode_launcher import _renode_processes
            log(f"after the clicks: fleet active={w.fleet.active} boot signals={len(st['boot_failed'])} "
                f"Renode processes={_renode_processes()} dialogs={[l for l in LOG if 'DIALOG' in l][-1:]}")
            store.delete_profile("DUP")
            w.drone_management.reload_profiles()
            log(f"removed DUP; profiles now {[p.name for p in store.list_profiles()]}")
        d.then("wait 15s, check nothing launched, restore profiles", after(15), check_and_restore)
    else:
        raise SystemExit(f"unknown scenario {scenario}")
