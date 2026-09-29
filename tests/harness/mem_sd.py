"""Step 5: does Renode's RSS track the SD image size? Boots a scratch
instance to its GPS fix with the given SD image, waits 60 s, samples RSS.

    python tests/harness/mem_sd.py <instance> normal|small64

small64 swaps in a blank 64 MiB image after the launcher's fresh copy (for
measurement only); the scratch instance's work dir can be deleted afterwards.
"""
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
from engine.renode_launcher import RenodeLauncher  # noqa: E402

instance, kind = int(sys.argv[1]), sys.argv[2]
launcher = RenodeLauncher(str(REPO / "pixhawk6c_renode_standalone"), instance=instance)
if kind == "small64":
    original_prepare = launcher._prepare_work_dir

    def prepare_then_shrink():
        original_prepare()
        with open(launcher.sdcard_path, "wb") as f:
            f.write(b"\0" * (64 * 2**20))
    launcher._prepare_work_dir = prepare_then_shrink
t = time.monotonic()
try:
    launcher.start()
    sd = launcher.sdcard_path
    print(f"{kind}: SD image {sd.stat().st_size / 2**20:.0f} MiB; GPS fix after {time.monotonic() - t:.0f}s", flush=True)
    time.sleep(60)
    pid = launcher._proc.pid
    rss = int(subprocess.run(["ps", "-o", "rss=", "-p", str(pid)], capture_output=True, text=True).stdout) / 1024
    print(f"RESULT {kind}: SD {sd.stat().st_size / 2**20:.0f} MiB -> RSS {rss:.0f} MB at GPS fix + 60 s", flush=True)
except Exception as exc:  # noqa: BLE001
    print(f"RESULT {kind}: boot failed after {time.monotonic() - t:.0f}s: {exc}", flush=True)
finally:
    launcher.stop()
