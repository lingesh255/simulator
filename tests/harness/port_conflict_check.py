"""Step 7 item 4: a taken MAVLink port fails that instance within seconds.

(a) the pre-boot check: a squatter on instance 2's port 5764 -> start() fails.
(b) the boot-time log watch: the same, with the pre-boot check bypassed, so
    only Renode's own "AddressAlreadyInUse" log line can catch it.

    python tests/harness/port_conflict_check.py
"""
import subprocess
import sys
import time
from pathlib import Path

HARNESS = Path(__file__).resolve().parent
REPO = HARNESS.parents[1]
sys.path.insert(0, str(REPO))
from engine.renode_launcher import RenodeLauncher  # noqa: E402

STANDALONE = str(REPO / "pixhawk6c_renode_standalone")


def pgrep():
    out = subprocess.run(["pgrep", "-af", "[r]enode-bin/[r]enode|[r]enode-physics"], capture_output=True, text=True).stdout
    return out.strip() or "(nothing)"


squatter = subprocess.Popen([sys.executable, str(HARNESS / "port_squatter.py"), "5764"])
time.sleep(1)
try:
    for label, bypass in (("(a) pre-boot port check", False), ("(b) Renode log watch only", True)):
        launcher = RenodeLauncher(STANDALONE, instance=2)
        if bypass:
            launcher._check_ports_free = lambda: None
        t = time.monotonic()
        try:
            launcher.start()
            print(f"{label}: started?! (unexpected)")
        except Exception as exc:  # noqa: BLE001
            print(f"{label}: failed after {time.monotonic() - t:.1f}s - {type(exc).__name__}: {exc}")
        finally:
            launcher.stop()
        time.sleep(2)
        print(f"  pgrep after: {pgrep()}")
finally:
    squatter.kill()
