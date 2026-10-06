"""Step 4d: Renode cleanup hits the real executables only.

Dummies whose command lines mention renode - including one instance's exact
cleanup markers, and one whose argv[0] is spoofed to "renode" - must survive
kill_all_renode_processes() and an instance's own cleanup; a real renode and
renode-physics must not.

    python tests/harness/kill_match_check.py
"""
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
from engine.renode_launcher import RenodeLauncher, _renode_processes, kill_all_renode_processes  # noqa: E402

STANDALONE = str(REPO / "pixhawk6c_renode_standalone")
SLEEP = "import time; time.sleep(60)"

dummies = {
    'python3 -c "..." renode': subprocess.Popen(["python3", "-c", SLEEP, "renode"]),
    'python3 -c "..." renode-physics --physics-port 9003': subprocess.Popen(
        ["python3", "-c", SLEEP, "renode-physics", "--physics-port", "9003"]),
    'python3 -c "..." "physics Connect 9003 quad"': subprocess.Popen(["python3", "-c", SLEEP, "physics Connect 9003 quad"]),
    "bash: exec -a renode sleep 60 (argv[0] spoofed)": subprocess.Popen(["bash", "-c", "exec -a renode sleep 60"]),
}
time.sleep(0.5)
for name, p in dummies.items():
    cmd = Path(f"/proc/{p.pid}/cmdline").read_bytes().replace(b"\0", b" ").decode().strip()
    print(f"dummy pid={p.pid} comm={Path(f'/proc/{p.pid}/comm').read_text().strip()!r} cmdline={cmd!r}")
print(f"\nkill_all_renode_processes() killed: {kill_all_renode_processes()}")
RenodeLauncher(STANDALONE, instance=1)._kill_own_stale_processes()
print("instance 1 _kill_own_stale_processes() ran")
time.sleep(0.5)
ok = True
for name, p in dummies.items():
    print(f"  {'ALIVE' if p.poll() is None else 'KILLED'}: {name}")
    ok &= p.poll() is None
for p in dummies.values():
    p.kill()

print()
launcher = RenodeLauncher(STANDALONE, instance=1)
launcher._prepare_work_dir()
launcher._start_physics(30.0)
launcher._start_renode()
time.sleep(5)
for pid in (launcher._physics_proc.pid, launcher._proc.pid):
    print(f"real pid {pid}: comm={Path(f'/proc/{pid}/comm').read_text().strip()!r} exe={Path(f'/proc/{pid}/exe').resolve()}")
print("matched:", [(pid, Path(argv[0]).name) for pid, argv in _renode_processes()])
print("kill_all_renode_processes() killed:", kill_all_renode_processes())
for proc in (launcher._physics_proc, launcher._proc):
    proc.wait(timeout=5)
    print(f"  real pid {proc.pid}: exit code {proc.returncode}")
    ok &= proc.returncode is not None
launcher.stop()
print(f"\nRESULT: {'PASS' if ok else 'FAIL'}")
