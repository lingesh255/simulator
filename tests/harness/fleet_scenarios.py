"""Fleet scenarios for gui_drive.py (Steps 6-8).

    fleet_travell / fleet_search / fleet_stop / fleet_bootfail       2 drones
    fleet_travell3 / fleet_search3 / fleet_stop3 / fleet_bootfail3   3 drones
    fleet_kill2      3 drones, drone 2's Renode killed mid-flight
    fleet_dupsysid   two checked profiles sharing a SYSID - must be refused
"""
import signal
import subprocess
import sys
import time
from pathlib import Path

HARNESS = Path(__file__).resolve().parent
NAMES = ("D1", "D2", "D3")


def build(scenario, d, w, log, fly, after, LOG, pts):
    mp = w.mission_planner
    n = 3 if scenario.endswith("3") or scenario == "fleet_kill2" else 2
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
                next((p.split()[2] for p in " ".join(argv).split("; ") if p.startswith("physics Connect")), "?")
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
    w.fleet.batch_ready.connect(on_fleet_batch)

    def common(plan, names=NAMES[:n]):
        d.then(f"check {', '.join(names)}", lambda: True, lambda: d.check_drones(names))
        d.then(f"select {plan}", lambda: True, lambda: d.select_plan(plan))
        d.then("check 'Fly via real MAVLink'", lambda: True, lambda: mp.external_mavlink_check.setChecked(True))

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

    def summary():
        log(f"SUMMARY: finished={len(st['finished'])} boot_failed={len(st['boot_failed'])} flying_signals={st['flying']} "
            f"all_sysid_batch={st['all_sysid_batch']} max_alt={ {k: round(v, 1) for k, v in st['max_alt'].items()} } "
            f"last_pos={st['last']} single-drone flights_started={d.flight_starts} single ready_count={d.ready_count}")
        pg = subprocess.run(["pgrep", "-af", "[r]enode-bin/[r]enode|[r]enode-physics"], capture_output=True, text=True).stdout
        log(f"pgrep (Renode executables) at summary: {pg.strip() or '(nothing)'}")

    def all_above(m):
        return len(st["max_alt"]) >= n and all(st["last"][s][2] > m for s in st["last"])

    short_north = (pts["CANBERRA"][0] + 0.001, pts["CANBERRA"][1])  # ~110 m north

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
