"""Step 3: two RenodeLauncher instances at once, headless.

Boots instances 1 and 2 together, shows their ports / files / sysids, sets a
param on instance 1 only and shows it survives a restart there but never
appears on instance 2, then takes both to ~10 m at once and lands them.

    python tests/harness/multi_check.py
"""
import subprocess
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from pymavlink import mavutil  # noqa: E402
from pymavlink.dialects.v20 import ardupilotmega as mav2  # noqa: E402

from engine.mavlink_mission import FlightAborted, upload_and_fly  # noqa: E402
from engine.renode_launcher import RenodeLauncher, kill_all_renode_processes  # noqa: E402

STANDALONE = REPO / "pixhawk6c_renode_standalone"
T0 = time.monotonic()
PRINT_LOCK = threading.Lock()


def log(msg):
    with PRINT_LOCK:
        print(f"[{time.monotonic() - T0:7.1f}s] {msg}", flush=True)


def pgrep():
    out = subprocess.run(["pgrep", "-af", "[r]enode-bin/[r]enode|[r]enode-physics"], capture_output=True, text=True).stdout
    return out.strip() or "(nothing)"


def concurrently(fn, launchers):
    errors = {}

    def run(l):
        try:
            fn(l)
        except Exception as exc:  # noqa: BLE001
            errors[l.instance] = repr(exc)
            log(f"instance {l.instance}: ERROR {exc!r}")

    threads = [threading.Thread(target=run, args=(l,)) for l in launchers]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    return errors


def boot(l):
    t = time.monotonic()
    l.start()
    log(f"instance {l.instance}: GPS 3D fix after {time.monotonic() - t:.1f}s")
    l.provision_first_boot_params()
    l.wait_until_armable()
    log(f"instance {l.instance}: armable after {time.monotonic() - t:.1f}s")


def connect(l):
    c = mavutil.mavlink_connection(l.connection_string)
    return c, c.wait_heartbeat(timeout=30)


def read_param(c, name, timeout=10):
    c.mav.param_request_read_send(c.target_system, c.target_component, name.encode(), -1)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        m = c.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if m is not None and m.param_id.rstrip("\x00") == name:
            return m.param_value
    return None


def heartbeats_and_param(launchers, param):
    for l in launchers:
        c, hb = connect(l)
        log(f"instance {l.instance} ({l.connection_string}): HEARTBEAT srcSystem={hb.get_srcSystem()}; "
            f"{param}={read_param(c, param)}")
        c.close()


def fly(l, samples):
    master = mavutil.mavlink_connection(l.connection_string)
    home = (l.latitude_deg, l.longitude_deg)
    dest = (l.latitude_deg + 0.0001, l.longitude_deg)  # ~11 m north
    last = [0.0]
    landed = {"flag": False}
    armed_seen = [False]

    def on_msg(m):
        if m.get_type() == "GLOBAL_POSITION_INT" and m.get_srcSystem() == l.sysid:
            alt = m.relative_alt / 1000.0
            samples[l.instance].append((time.monotonic() - T0, alt))
            if time.monotonic() - last[0] >= 3:
                last[0] = time.monotonic()
                log(f"  instance {l.instance}: alt={alt:5.1f} m")
        elif m.get_type() == "HEARTBEAT" and m.get_srcSystem() == l.sysid:
            armed = bool(m.base_mode & mav2.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed != armed_seen[0]:
                armed_seen[0] = armed
                log(f"  instance {l.instance}: {'ARMED' if armed else 'DISARMED'}")
                if not armed and samples[l.instance] and max(a for _, a in samples[l.instance]) > 5:
                    landed["flag"] = True

    try:
        upload_and_fly(master, [home, dest], 10.0, on_progress=lambda s: log(f"  instance {l.instance}: {s}"),
                       on_message=on_msg, should_abort=lambda: landed["flag"])
    except FlightAborted:
        log(f"  instance {l.instance}: landed and disarmed")
    finally:
        master.close()


def run(launchers):
    log(f"kill_all_renode_processes() killed: {kill_all_renode_processes()}")
    launchers.extend(RenodeLauncher(str(STANDALONE), instance=i) for i in (1, 2))
    for l in launchers:
        log(f"instance {l.instance}: mavlink={l.connection_string} physics_port={l.physics_port} sysid={l.sysid} "
            f"sdcard={l.sdcard_path} fram={l.fram_path} flash={l.persistent_flash_path} log={l.renode_log_path}")
    log("== booting both instances concurrently ==")
    concurrently(boot, launchers)
    log("listening sockets:\n" + subprocess.run("ss -ltnp | grep -E 'renode|physics'", shell=True,
                                                capture_output=True, text=True).stdout)
    PARAM, NEW = "DISARM_DELAY", 7.0
    log("== HEARTBEAT sysid and DISARM_DELAY (before) ==")
    heartbeats_and_param(launchers, PARAM)
    c, _ = connect(launchers[0])
    RenodeLauncher._confirm_param_set(c, PARAM, NEW, mav2.MAV_PARAM_TYPE_INT8, 15)
    log(f"instance 1: set {PARAM}={NEW}; read back {read_param(c, PARAM)}")
    c.close()
    time.sleep(30)  # let the firmware write it to FRAM before stopping
    log("== restarting both ==")
    for l in launchers:
        l.stop()
    concurrently(boot, launchers)
    log("== HEARTBEAT sysid and DISARM_DELAY (after restart) ==")
    heartbeats_and_param(launchers, PARAM)
    c, _ = connect(launchers[0])
    RenodeLauncher._confirm_param_set(c, PARAM, 10.0, mav2.MAV_PARAM_TYPE_INT8, 15)  # put it back
    c.close()
    log("== both take off to 10 m at the same time ==")
    samples = {1: [], 2: []}
    concurrently(lambda l: fly(l, samples), launchers)
    for i, s in samples.items():
        above = [t for t, a in s if a >= 9.0]
        log(f"instance {i}: peak {max((a for _, a in s), default=0):.1f} m, >=9 m from "
            f"{above[0] if above else '-'}s to {above[-1] if above else '-'}s")


def main():
    launchers = []
    try:
        run(launchers)
    finally:
        for l in launchers:
            l.stop()
        time.sleep(2)
        log(f"pgrep after stop(): {pgrep()}")


if __name__ == "__main__":
    main()
