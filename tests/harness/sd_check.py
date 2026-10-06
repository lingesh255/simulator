"""Step 7 item 2: every launch boots from a fresh copy of the golden SD image.

Shows: the golden image (a fresh mkfs.fat image holding the original card's
files, copied in with mcopy) passes `fsck.fat -n`, with used/free clusters
next to the original's and the old fsck-salvaged image's, and the same file
list and contents as the original (which is unchanged); instance 0's launch command differs
from main's only in the SD path; and for instances 0 and 1, after a real boot
(which writes to the SD copy) and a relaunch, the copy matches the golden
image again (mtime + md5) at the moment the relaunch hands it to Renode.

    python tests/harness/sd_check.py
"""
import hashlib
import importlib.util
import re
import shutil
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
built_at = time.monotonic()
golden = ensure_golden_sdcard(original, launcher0.work_root)
print(f"golden image ready in {time.monotonic() - built_at:.1f}s: {golden}")
salvaged = launcher0.work_root / "golden-sdcard.salvaged.img"


def fsck(path):
    r = subprocess.run(["fsck.fat", "-n", str(path)], capture_output=True, text=True)
    m = re.search(r"(\d+) files, (\d+)/(\d+) clusters", r.stdout)
    files, used, total = (int(x) for x in m.groups()) if m else (None, None, None)
    return r.returncode, files, used, total


def paths(path):
    """Sorted full paths inside a FAT image (mtools reads a throwaway copy)."""
    with tempfile.TemporaryDirectory() as tmp:
        copy = Path(tmp) / "copy.img"
        shutil.copyfile(path, copy)
        out = subprocess.run(["mdir", "-/", "-b", "-a", "-i", str(copy), "::"], capture_output=True, text=True).stdout
    return sorted(out.split())


def extract(path, dest):
    copy = Path(dest) / "copy.img"
    shutil.copyfile(path, copy)
    (Path(dest) / "files").mkdir()
    subprocess.run(["mcopy", "-s", "-m", "-n", "-i", str(copy), "::/*", str(Path(dest) / "files")], check=True)
    return Path(dest) / "files"


print(f"{'image':<32} {'fsck -n':>7} {'files':>6} {'used clusters':>14} {'free clusters':>14}")
for name, path in (("original (read via a copy)", original), ("salvaged (old fsck -a golden)", salvaged),
                   ("golden (mkfs.fat + mcopy)", golden)):
    if path.exists():
        rc, files, used, total = fsck(path)
        print(f"{name:<32} {'exit ' + str(rc):>7} {files:>6} {used:>14} {total - used:>14}")
original_paths, golden_paths = paths(original), paths(golden)
print(f"\nentries: original {len(original_paths)}, golden {len(golden_paths)}; "
      f"identical path lists: {original_paths == golden_paths}")
if salvaged.exists():
    extra = sorted(set(paths(salvaged)) - set(original_paths))
    print(f"salvaged image has {len(extra)} entries the original doesn't, e.g. {extra[:3]}")
with tempfile.TemporaryDirectory() as a, tempfile.TemporaryDirectory() as b:
    same = subprocess.run(["diff", "-r", str(extract(original, a)), str(extract(golden, b))],
                          capture_output=True, text=True).returncode == 0
print(f"file contents byte-identical to the original's: {same}")
print(f"original unchanged: {md5(original) == before} (md5 {before})")

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
