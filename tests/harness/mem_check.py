"""Step 5: one instance's RSS idle (booted, armable) and during a 50 m flight.

    python tests/harness/mem_check.py <label> [VAR=VALUE ...]

VAR=VALUE pairs are added to Renode's own environment on top of the
launcher's RENODE_ENV (subprocess.Popen is wrapped here; the launcher code is
untouched). Pass DOTNET_GCConserveMemory=0 to measure without the kept setting.
"""
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
from pymavlink import mavutil  # noqa: E402

import engine.renode_launcher as rl  # noqa: E402
from engine.mavlink_mission import upload_and_fly  # noqa: E402

LABEL = sys.argv[1]
EXTRA_ENV = dict(arg.split("=", 1) for arg in sys.argv[2:])
T0 = time.monotonic()
_real_popen = subprocess.Popen


def log(msg):
    print(f"[{time.monotonic() - T0:7.1f}s] {msg}", flush=True)


class _Popen(_real_popen):
    def __init__(self, args, *a, **kw):
        if EXTRA_ENV and isinstance(args, list) and args and args[0].endswith("renode-bin/renode"):
            kw["env"] = {**kw.get("env", os.environ), **EXTRA_ENV}
            log(f"Renode Popen env += {EXTRA_ENV}")
        super().__init__(args, *a, **kw)


rl.subprocess.Popen = _Popen


def rss_mb(pid):
    out = subprocess.run(["ps", "-o", "rss=", "-p", str(pid)], capture_output=True, text=True).stdout.strip()
    return int(out) / 1024 if out else None


def status(pid):
    return {l.split(":")[0]: l.split(":")[1].strip() for l in Path(f"/proc/{pid}/status").read_text().splitlines()
            if l.startswith(("RssAnon", "RssFile", "VmSwap"))}


def main():
    launcher = rl.RenodeLauncher(str(REPO / "pixhawk6c_renode_standalone"), instance=1)
    t = time.monotonic()
    try:
        launcher.start()
        t_fix = time.monotonic() - t
        launcher.provision_first_boot_params()
        launcher.wait_until_armable()
        t_armable = time.monotonic() - t
        pid = launcher._proc.pid
        for p in (launcher.sdcard_path, launcher.fram_path, launcher.persistent_flash_path):
            log(f"{p.name}: {p.stat().st_size / 2**20:.2f} MiB; memory-mapped: {p.name in Path(f'/proc/{pid}/maps').read_text()}")
        idle = []
        for _ in range(6):
            idle.append(rss_mb(pid))
            time.sleep(5)
        log(f"IDLE RSS MB: {[round(x) for x in idle]} max {max(idle):.0f}; {status(pid)}")

        flying, stop = [], threading.Event()

        def sampler():
            while not stop.is_set():
                flying.append(rss_mb(pid))
                stop.wait(5)
        threading.Thread(target=sampler, daemon=True).start()

        home = (launcher.latitude_deg, launcher.longitude_deg)
        dest = (home[0] + 0.001, home[1])
        st = {"alt50": None, "pos": None}
        tf = time.monotonic()

        def on_msg(m):
            if m.get_type() == "GLOBAL_POSITION_INT":
                st["pos"] = (m.lat / 1e7, m.lon / 1e7)
                if m.relative_alt >= 47500 and st["alt50"] is None:
                    st["alt50"] = time.monotonic() - tf

        master = mavutil.mavlink_connection(launcher.connection_string)
        try:
            upload_and_fly(master, [home, dest], 50.0, on_message=on_msg,
                           on_progress=lambda s: log(f"  {s}") if s.startswith(("Mission", "[FC] EKF Failsafe")) else None)
            result = "completed"
        except Exception as exc:  # noqa: BLE001
            result = f"FAILED: {exc}"
        finally:
            master.close()
        stop.set()
        dy = (st["pos"][0] - dest[0]) * 111320
        dx = (st["pos"][1] - dest[1]) * 111320 * math.cos(math.radians(dest[0]))
        log(f"SUMMARY {LABEL}: idle_max={max(idle):.0f}MB flying_max={max(x for x in flying if x):.0f}MB "
            f"fix={t_fix:.0f}s armable={t_armable:.0f}s alt50={st['alt50'] and round(st['alt50'], 1)}s "
            f"landed {math.hypot(dx, dy):.1f} m from the destination; {result}")
    finally:
        launcher.stop()


if __name__ == "__main__":
    main()
