"""Task 21d probe: can Renode's emulation Save / Load skip the boot?

Boots ONE drone in a shared Renode to armable, pauses, saves the emulation,
stops everything, then starts a fresh physics sidecar and a fresh Renode,
loads the snapshot, reconnects physics, starts, and checks whether the
drone answers on MAVLink, is still armable, and flies. Every step's monitor
reply is printed; the first step that fails ends the probe.

    python experiments/shared_speed/snapshot_probe.py
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import speed_run as sr  # noqa: E402
from engine.renode_launcher import RenodeLauncher, _renode_processes  # noqa: E402

SNAPSHOT = sr.OUT / "drone1.snapshot"


def log(msg):
    sr.log(msg)


def options():
    return argparse.Namespace(quantum="0.01", master_quantum="0.1", advance_immediately=False, gc="7")


def show(label, replies):
    for reply in replies:
        text = " | ".join(line.strip() for line in reply.splitlines() if line.strip())
        log(f"  {label}: {text[:500] or '(no output)'}")


def heartbeat_and_arm_check(launcher, seconds=40.0):
    """After the load: is there a vehicle on the MAVLink port, with which sysid, and is it armable?"""
    from pymavlink import mavutil
    try:
        master = mavutil.mavlink_connection(launcher.connection_string)
    except OSError as exc:
        log(f"  MAVLink connect failed: {exc}")
        return False
    try:
        hb = master.wait_heartbeat(timeout=seconds)
        if hb is None:
            log(f"  no heartbeat on {launcher.connection_string} within {seconds:.0f} s")
            return False
        log(f"  heartbeat from sysid {master.target_system}, base_mode {hb.base_mode}, status {hb.system_status}")
        master.mav.request_data_stream_send(master.target_system, master.target_component,
                                            mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
        seen = {}
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            msg = master.recv_match(type=["GPS_RAW_INT", "EKF_STATUS_REPORT", "STATUSTEXT", "SYSTEM_TIME"],
                                    blocking=True, timeout=1)
            if msg is not None:
                seen[msg.get_type()] = msg
        gps, ekf, clock = seen.get("GPS_RAW_INT"), seen.get("EKF_STATUS_REPORT"), seen.get("SYSTEM_TIME")
        log(f"  after 30 s: GPS fix_type {getattr(gps, 'fix_type', None)}, EKF flags {getattr(ekf, 'flags', None)}, "
            f"time_boot_ms {getattr(clock, 'time_boot_ms', None)}, last text "
            f"{getattr(seen.get('STATUSTEXT'), 'text', None)!r}")
        return gps is not None and gps.fix_type >= 3
    finally:
        master.close()


def main() -> int:
    sr.OUT.mkdir(exist_ok=True)
    if SNAPSHOT.exists():
        SNAPSHOT.unlink()
    drone = sr.Drone(1)
    fleet = sr.SpeedFleet(str(sr.STANDALONE), [(1, *drone.start)])
    fleet.configure(1, options())
    fleet.renode_cpus = None
    t0 = time.monotonic()
    try:
        fleet.start(on_phase=lambda s, text: log(f"drone {s}: {text}"))
        boot_s = time.monotonic() - t0
        log(f"STEP 1 ok: armable after {boot_s:.0f}s (this is what a snapshot would skip)")

        log("STEP 2: pause, then Save")
        show("pause", fleet.monitor(["pause"], reply_timeout_s=30))
        if os.environ.get("SNAP_DISCONNECT_PHYSICS"):
            # the first probe's blocker: AP_Physics holds a live TcpClient to its sidecar
            show("physics Disconnect", fleet.monitor(['mach set "drone1"', "physics Disconnect", "sysbus.physics Connected"],
                                                     reply_timeout_s=30))
        t = time.monotonic()
        try:
            replies = fleet.monitor([f"Save @{SNAPSHOT}"], reply_timeout_s=300)
        except OSError as exc:
            log(f"  Save: monitor error {exc!r}")
            replies = []
        show("Save", replies)
        size = SNAPSHOT.stat().st_size if SNAPSHOT.exists() else 0
        log(f"  Save took {time.monotonic() - t:.1f}s; snapshot file: {size / 1e6:.1f} MB")
        errors = subprocess.run(["grep", "-a", "-i", "-m", "8", "error\\|exception\\|serializ", str(fleet.log_path)],
                                capture_output=True, text=True).stdout
        if errors.strip():
            log("  Renode console on Save:\n    " + "\n    ".join(line[:300] for line in errors.strip().splitlines()))
        if size == 0 or any("error" in r.lower() for r in replies):
            log("RESULT: Save did not produce a usable snapshot - blocked at step 2")
            return 1
    finally:
        fleet.stop_all()
        time.sleep(2)

    log("STEP 3: fresh sidecar + fresh Renode, Load the snapshot")
    launcher = RenodeLauncher(str(sr.STANDALONE), instance=1, latitude_deg=drone.start[0], longitude_deg=drone.start[1])
    launcher.check_ports_free()
    launcher.start_physics()          # NOT prepare(): the snapshot refers to the SD/FRAM files as they are
    work = fleet.work_dir
    console = open(work / "load-console.log", "wb")
    # a second fleet object, used only for its monitor helpers against the fresh Renode
    loader = sr.SpeedFleet(str(sr.STANDALONE), [(1, *drone.start)])
    loader.configure(1, options())
    loader.log_path = work / "load-console.log"
    from engine.shared_renode import _free_port
    loader.monitor_port = _free_port()
    t_load = time.monotonic()
    proc = subprocess.Popen([str(loader.renode_bin), "--disable-xwt", "-P", str(loader.monitor_port)],
                            cwd=loader.standalone_dir, preexec_fn=os.setsid, stdin=subprocess.DEVNULL,
                            stdout=console, stderr=subprocess.STDOUT, env={**os.environ, **RenodeLauncher.RENODE_ENV})
    loader._proc = proc
    ok = False
    try:
        loader._wait_for_monitor()
        t = time.monotonic()
        show("Load", loader.monitor([f"Load @{SNAPSHOT}"], reply_timeout_s=300))
        log(f"  Load took {time.monotonic() - t:.1f}s")
        show("mach", loader.monitor(["mach"], reply_timeout_s=20))
        show("select", loader.monitor(['mach set "drone1"', "machine ElapsedVirtualTime", "sysbus.physics Connected",
                                       "cpu IsHalted", "cpu PC"], reply_timeout_s=20))
        log("STEP 4: reconnect physics, start")
        show("physics Connect", loader.monitor(
            ['mach set "drone1"', 'physics Connect %d "%s" %.6f %.6f %.1f %.1f %d' % (
                launcher.physics_port, launcher.PHYSICS_MODEL, drone.start[0], drone.start[1],
                launcher.PHYSICS_ALTITUDE_M, launcher.PHYSICS_HEADING_DEG, launcher.PHYSICS_RATE_HZ),
             "sysbus.physics Connected", "sysbus.physics LastError"], reply_timeout_s=60))
        show("start", loader.monitor(["start"], reply_timeout_s=60))
        time.sleep(10)
        show("10 s later", loader.monitor(['mach set "drone1"', "machine ElapsedVirtualTime", "cpu PC",
                                           "sysbus.physics Connected", "sysbus.physics Steps"], reply_timeout_s=20))
        log("  ports listening: " + subprocess.run("ss -ltn | grep -E ':(5763|9003) ' | tr -s ' ' | cut -c1-60 | tr '\\n' ';'",
                                                    shell=True, capture_output=True, text=True).stdout)
        log("STEP 5: MAVLink after the load")
        if heartbeat_and_arm_check(launcher):
            ready_s = time.monotonic() - t_load
            log(f"  vehicle alive with a GPS fix {ready_s:.0f}s after starting the fresh Renode")
            launcher.attach(proc, work / "load-console.log")
            try:
                launcher.wait_until_armable(timeout_s=240)
                log(f"  ARMABLE {time.monotonic() - t_load:.0f}s after starting the fresh Renode (a normal boot took {boot_s:.0f}s)")
                log("STEP 6: fly")
                flight = threading.Thread(target=drone.fly, args=(launcher.connection_string,))
                flight.start()
                flight.join()
                ok = drone.r.get("outcome") == "completed"
            except Exception as exc:  # noqa: BLE001
                log(f"  not armable after the load: {exc}")
        errors = subprocess.run(["grep", "-a", "-i", "-m", "12", "error\\|exception\\|abort", str(work / "load-console.log")],
                                capture_output=True, text=True).stdout
        if errors.strip():
            log("  fresh Renode's console:\n    " + "\n    ".join(line[:300] for line in errors.strip().splitlines()))
    except Exception as exc:  # noqa: BLE001
        log(f"  probe step failed: {type(exc).__name__}: {exc}")
    finally:
        RenodeLauncher._terminate(proc)
        console.close()
        launcher.stop()
        time.sleep(2)
        log(f"left running: {[(p, Path(a[0]).name) for p, a in _renode_processes()] or '(nothing)'}")
    log("RESULT: " + ("the loaded drone armed and flew" if ok else "the snapshot did not give a flyable drone (see the steps above)"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
