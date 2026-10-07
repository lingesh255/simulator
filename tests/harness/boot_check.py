"""Boot one RenodeLauncher instance to armable and stop it (no flight).

    python tests/harness/boot_check.py [instance=0]
"""
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from engine.renode_launcher import RenodeLauncher, _renode_processes  # noqa: E402

instance = int(sys.argv[1]) if len(sys.argv) > 1 else 0
launcher = RenodeLauncher(str(REPO / "pixhawk6c_renode_standalone"), instance=instance)
t = time.monotonic()
try:
    print(f"instance {instance}: start() -> {launcher.start()} (GPS fix) after {time.monotonic() - t:.0f}s", flush=True)
    launcher.provision_first_boot_params()
    print(f"provisioned after {time.monotonic() - t:.0f}s", flush=True)
    launcher.wait_until_armable()
    print(f"ARMABLE after {time.monotonic() - t:.0f}s; physics pid {launcher.physics_pid}", flush=True)
finally:
    launcher.stop()
    time.sleep(2)
    print("processes after stop:", [(pid, Path(argv[0]).name) for pid, argv in _renode_processes()] or "(nothing)")
