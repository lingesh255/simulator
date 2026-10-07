"""engine.shared_renode.SharedRenodeFleet against the real Renode, headless.

Boots N drones in one Renode to armable, shows the pids and memory, stops
the last drone with stop_drone() and checks the others still send
heartbeats with their own sysid while it has gone quiet (at most one
already-queued heartbeat), then stop_all().

    python tests/harness/shared_fleet_check.py [n=2]
"""
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from pymavlink import mavutil  # noqa: E402

from engine.renode_launcher import _renode_processes, kill_all_renode_processes  # noqa: E402
from engine.shared_renode import SharedRenodeFleet  # noqa: E402

T0 = time.monotonic()


def log(msg):
    print(f"[{time.monotonic() - T0:6.1f}s] {msg}", flush=True)


def rss_mb(pid):
    for line in Path(f"/proc/{pid}/status").read_text().splitlines():
        if line.startswith("VmRSS:"):
            return int(line.split()[1]) // 1024
    return 0


def heartbeats(connection, seconds):
    """(count, sysids seen) of autopilot heartbeats on `connection` within `seconds`."""
    count, seen = 0, set()
    try:
        master = mavutil.mavlink_connection(connection)
    except OSError as exc:
        return 0, {f"connect failed: {exc}"}
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is not None and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            count += 1
            seen.add(msg.get_srcSystem())
    master.close()
    return count, seen


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    log(f"cleared {len(kill_all_renode_processes())} stray process(es)")
    fleet = SharedRenodeFleet(str(REPO / "pixhawk6c_renode_standalone"),
                              [(i, -35.363261, 149.165230 + 0.0002 * (i - 1)) for i in range(1, n + 1)])
    log(f"timeout scale x{fleet.timeout_scale}")
    ok = True
    try:
        connections = fleet.start(on_phase=lambda sysid, text: log(f"  drone {sysid}: {text}"))
        pids = fleet.pids()
        log(f"ALL ARMABLE: {connections}")
        log(f"pids {pids}; monitor port {fleet.monitor_port}; Renode RSS {rss_mb(pids['renode'])} MB (one process)")
        log(f"processes: {[(pid, Path(argv[0]).name) for pid, argv in _renode_processes()]}")
        results = {}
        threads = [threading.Thread(target=lambda s=s, c=c: results.__setitem__(s, heartbeats(c, 8)))
                   for s, c in connections.items()]
        [t.start() for t in threads]
        [t.join() for t in threads]
        log(f"heartbeats in 8 s before the stop (count, sysids): {results}")
        # (heartbeats are 1 Hz in virtual time; the emulation runs below real time)
        ok &= all(seen == {s} and count >= 2 for s, (count, seen) in results.items())

        victim = n
        log(f"stop_drone({victim})")
        fleet.stop_drone(victim)
        log(f"Renode alive: {fleet.is_running}; pids now {fleet.pids()}; dead sidecars reported: {fleet.dead_sidecars()}")
        time.sleep(3)
        results = {}
        threads = [threading.Thread(target=lambda s=s, c=c: results.__setitem__(s, heartbeats(c, 10)))
                   for s, c in connections.items()]
        [t.start() for t in threads]
        [t.join() for t in threads]
        log(f"heartbeats in 10 s after the stop (count, sysids): {results}")
        # at most one heartbeat that was already queued for the socket when the CPU halted
        ok &= fleet.is_running and results[victim][0] <= 1
        ok &= all(seen == {s} and count >= 3 for s, (count, seen) in results.items() if s != victim)
    finally:
        fleet.stop_all()
        time.sleep(2)
        left = [(pid, Path(argv[0]).name) for pid, argv in _renode_processes()]
        log(f"after stop_all: {left or '(nothing)'}")
        ok &= not left
    log("RESULT: " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
