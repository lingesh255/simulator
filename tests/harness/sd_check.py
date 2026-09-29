"""Step 7 item 2: every launch boots from a fresh copy of the golden SD image.

Shows: the golden image passes `fsck.fat -n` (the original's result next to
it, and that the original is unchanged); instance 0's launch command differs
from main's only in the SD path; and for instances 0 and 1, after a real boot
(which writes to the SD copy) and a relaunch, the copy matches the golden
image again (mtime + md5) at the moment the relaunch hands it to Renode.

    python tests/harness/sd_check.py
"""
import hashlib
import importlib.util
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
from engine.renode_launcher import GOLDEN_SDCARD_NAME, RenodeLauncher, ensure_golden_sdcard  # noqa: E402

STANDALONE = REPO / "pixhawk6c_renode_standalone"


def md5(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 22), b""):
            h.update(chunk)
    return h.hexdigest()


def mtime(path):
    return time.strftime("%H:%M:%S", time.localtime(path.stat().st_mtime))


launcher0 = RenodeLauncher(str(STANDALONE))
original = launcher0.original_sdcard
before = md5(original)
golden = ensure_golden_sdcard(original, launcher0.work_root)
print(f"original {original}: md5 {before}")
print(f"golden   {golden}: md5 {md5(golden)}")
for name, path in (("original (read-only check)", original), ("golden", golden)):
    r = subprocess.run(["fsck.fat", "-n", str(path)], capture_output=True, text=True)
    print(f"fsck.fat -n {name}: exit {r.returncode}; " + " | ".join(r.stdout.strip().splitlines()[-3:]))
print(f"original unchanged: {md5(original) == before}")

# instance 0's command vs main's (e4488e2)
main_src = subprocess.run(["git", "-C", str(REPO), "show", "e4488e2:engine/renode_launcher.py"],
                          capture_output=True, text=True, check=True).stdout
with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as f:
    f.write(main_src)
spec = importlib.util.spec_from_file_location("launcher_main", f.name)
launcher_main = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher_main)
old = launcher_main.RenodeLauncher(str(STANDALONE))._build_launch_command().split("; ")
new = launcher0._build_launch_command().split("; ")
print(f"\ninstance 0 command: {len(old)} vs {len(new)} commands; differences from main:")
for a, b in zip(old, new):
    if a != b:
        print(f"  - {a}\n  + {b}")

# fresh copy on relaunch, instances 0 and 1
for instance in (0, 1):
    l = RenodeLauncher(str(STANDALONE), instance=instance)
    seen = {}
    real_prepare = l._prepare_work_dir

    def prepare_and_record(l=l, seen=seen, real_prepare=real_prepare):
        real_prepare()
        seen.setdefault("copies", []).append((mtime(l.sdcard_path), md5(l.sdcard_path)))
    l._prepare_work_dir = prepare_and_record
    try:
        print(f"\ninstance {instance}: boot 1 ...", flush=True)
        l.start()
        l.stop()
        after_boot = (mtime(l.sdcard_path), md5(l.sdcard_path))
        print(f"  SD after boot 1 + stop: mtime {after_boot[0]} md5 {after_boot[1]} (== golden: {after_boot[1] == md5(golden)})")
        time.sleep(2)
        print(f"instance {instance}: relaunch ...", flush=True)
        l.start()
    finally:
        l.stop()
    for n, (mt, digest) in enumerate(seen["copies"], 1):
        print(f"  SD as handed to Renode on launch {n}: mtime {mt} md5 {digest} (== golden: {digest == md5(golden)})")
print(f"\ngolden image name: {GOLDEN_SDCARD_NAME}")
