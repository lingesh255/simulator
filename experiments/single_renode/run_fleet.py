"""Task 16 experiment driver: boot N drones, fly every one, measure.

    run_fleet.py --mode single   --n 2            # N machines in ONE Renode
    run_fleet.py --mode separate --n 2            # today's one process per drone
    run_fleet.py --mode single --n 4 --telnet --slow 16
    run_fleet.py --mode single --n 2 --telnet --slow 8 --fail physics|pause|halt|killall --fail-on-ground

Both modes prepare each drone with RenodeLauncher(instance=N) (ports
5762+N / 9002+N, sysid N, its own SD/FRAM/flash copies) and use its own
GPS-fix wait, first-boot provisioning and armable wait. `separate` simply
calls launcher.start(); `single` starts the physics sidecars the same way,
then runs ONE Renode on the script make_fleet_resc.py generates and hands
that one process to every launcher object.

Each drone then flies start -> 60 m north at 50 m with
engine.mavlink_mission.upload_and_fly (which itself fails a landing more
than 10 m from the LAND point). Everything measured is printed and saved as
out/<tag>.json; the Renode console goes to out/<tag>_renode.log.

Run tests/harness/rclean.sh first: this never kills anything it didn't start.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(HERE))

from pymavlink import mavutil  # noqa: E402

from engine.mavlink_mission import upload_and_fly  # noqa: E402
from engine.renode_launcher import RenodeLauncher, _renode_processes  # noqa: E402
import make_fleet_resc  # noqa: E402

STANDALONE = REPO / "pixhawk6c_renode_standalone"
OUT = HERE / "out"
BASE_LAT, BASE_LON = -35.363261, 149.165230
SPACING_M = 18.0        # east, between drones (the fleet's own travell offset)
LEG_NORTH_M = 60.0
ALTITUDE_M = 50.0
MONITOR_PORT = 4455     # Renode's telnet monitor, single mode only (to address one machine)
T0 = time.monotonic()
_print_lock = threading.Lock()


def log(msg: str) -> None:
    with _print_lock:
        print(f"[{time.monotonic() - T0:7.1f}s] {msg}", flush=True)


def offset(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    return (lat + north_m / 111320.0, lon + east_m / (111320.0 * math.cos(math.radians(lat))))


def distance_m(a, b) -> float:
    return math.hypot((a[0] - b[0]) * 111320.0, (a[1] - b[1]) * 111320.0 * math.cos(math.radians(a[0])))


def rss_mb(pid: int) -> float:
    try:
        for line in Path(f"/proc/{pid}/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024.0
    except OSError:
        pass
    return 0.0


def cpu_seconds(pid: int) -> float:
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")
    except OSError:
        return 0.0


def renode_pids() -> list[int]:
    return [pid for pid, argv in _renode_processes() if Path(argv[0]).name == "renode"]


def memory_snapshot(label: str) -> dict:
    procs = {pid: Path(argv[0]).name for pid, argv in _renode_processes()}
    renode = {pid: round(rss_mb(pid)) for pid, name in procs.items() if name == "renode"}
    physics = {pid: round(rss_mb(pid)) for pid, name in procs.items() if name == "renode-physics"}
    free = subprocess.run(["free", "-m"], capture_output=True, text=True).stdout
    snap = {"label": label, "renode_rss_mb": renode, "renode_total_mb": sum(renode.values()),
            "physics_rss_mb": physics, "free_m": free}
    log(f"MEMORY {label}: Renode RSS {renode} = {snap['renode_total_mb']} MB total; "
        f"sidecars {sum(physics.values())} MB\n{free.rstrip()}")
    return snap


def cpu_snapshot(seconds: float = 20.0) -> dict:
    """CPU use of every Renode process over `seconds`, in cores, plus the
    busiest threads (are the machines on separate host threads?)."""
    pids = renode_pids()
    before = {pid: cpu_seconds(pid) for pid in pids}
    t = time.monotonic()
    time.sleep(seconds)
    elapsed = time.monotonic() - t
    cores = {pid: round((cpu_seconds(pid) - before[pid]) / elapsed, 2) for pid in pids}
    threads = {}
    for pid in pids:
        out = subprocess.run(["ps", "-L", "-o", "pcpu=,comm=", "-p", str(pid)], capture_output=True, text=True).stdout
        rows = sorted(((float(r.split(None, 1)[0]), r.split(None, 1)[1].strip()) for r in out.splitlines() if r.strip()),
                      reverse=True)
        threads[pid] = {"count": len(rows), "busiest": rows[:8]}
    log(f"CPU over {elapsed:.0f}s: cores used per Renode process {cores} (total {sum(cores.values()):.2f}); "
        + "; ".join(f"pid {p}: {v['count']} threads, busiest {v['busiest'][:6]}" for p, v in threads.items()))
    return {"cores": cores, "total_cores": round(sum(cores.values()), 2), "threads": threads}


def monitor(command: str, wait_s: float = 3.0) -> str:
    """One command to the single Renode's telnet monitor; returns what it printed."""
    try:
        with socket.create_connection(("127.0.0.1", MONITOR_PORT), timeout=5) as sock:
            sock.settimeout(0.5)
            time.sleep(0.5)
            try:
                sock.recv(65536)  # banner / prompt
            except OSError:
                pass
            sock.sendall(command.encode() + b"\n")
            deadline, chunks = time.monotonic() + wait_s, []
            while time.monotonic() < deadline:
                try:
                    data = sock.recv(65536)
                except socket.timeout:
                    continue
                if not data:
                    break
                chunks.append(data)
            text = b"".join(chunks).decode(errors="replace")
    except OSError as exc:
        text = f"(monitor connection failed: {exc})"
    clean = " | ".join(line.strip() for line in text.replace("\r", "").split("\n") if line.strip())
    log(f"MONITOR> {command}  ->  {clean[:400]}")
    return clean


class Drone:
    def __init__(self, n: int):
        self.n = n
        self.start = offset(BASE_LAT, BASE_LON, 0.0, SPACING_M * (n - 1))
        self.dest = offset(*self.start, LEG_NORTH_M, 0.0)
        self.launcher = RenodeLauncher(str(STANDALONE), instance=n, latitude_deg=self.start[0],
                                       longitude_deg=self.start[1])
        self.r: dict = {"drone": n, "port": self.launcher.port, "physics_port": self.launcher.physics_port,
                        "sysids_seen": [], "failsafes": 0, "max_alt_m": 0.0, "progress": []}
        self.abort = threading.Event()
        self.last_msg_at = 0.0
        self.alt_m = 0.0
        self.pos = None
        self.armed_at = None

    # -- boot --
    def boot(self, t_launch: float, single: bool, slow: float = 1.0, gps_only: bool = False) -> None:
        l = self.launcher
        try:
            if single:
                l._wait_for_gps_fix(600.0 * slow)
            else:
                l.start(gps_ready_timeout_s=600.0)
            self.r["gps_fix_s"] = round(time.monotonic() - t_launch, 1)
            log(f"drone {self.n}: GPS 3D fix {self.r['gps_fix_s']}s after launch")
            if gps_only:
                return
            l.provision_first_boot_params(timeout_s=120.0 * slow)
            self.r["provisioned_s"] = round(time.monotonic() - t_launch, 1)
            l.wait_until_armable(timeout_s=400.0 * slow)
            self.r["armable_s"] = round(time.monotonic() - t_launch, 1)
            log(f"drone {self.n}: armable {self.r['armable_s']}s after launch")
        except Exception as exc:  # noqa: BLE001
            self.r["boot_error"] = repr(exc)
            log(f"drone {self.n}: BOOT ERROR {exc!r}")

    # -- flight --
    def _on_message(self, msg) -> None:
        self.last_msg_at = time.monotonic()
        sysid = msg.get_srcSystem()
        if sysid not in self.r["sysids_seen"]:
            self.r["sysids_seen"].append(sysid)
        kind = msg.get_type()
        if kind == "GLOBAL_POSITION_INT":
            self.alt_m = msg.relative_alt / 1000.0
            self.pos = (msg.lat / 1e7, msg.lon / 1e7)
            self.r["max_alt_m"] = max(self.r["max_alt_m"], round(self.alt_m, 1))
            if self.armed_at is not None and "to_50m_s" not in self.r and self.alt_m >= ALTITUDE_M - 0.5:
                self.r["to_50m_s"] = round(time.monotonic() - self.armed_at, 1)
                log(f"drone {self.n}: 50 m reached {self.r['to_50m_s']}s after arming")
        elif kind == "HEARTBEAT" and self.armed_at is None and msg.base_mode & 128:
            self.armed_at = time.monotonic()

    def _on_progress(self, text: str) -> None:
        self.r["progress"].append(f"{time.monotonic() - T0:.1f}s {text}")
        if "failsafe" in text.lower():
            self.r["failsafes"] += 1
        if text.startswith("Arm ack") and self.armed_at is None:
            self.armed_at = time.monotonic()
        if any(key in text for key in ("Heartbeat OK", "Arm ack", "Mission:", "complete", "ailsafe", "Reached")):
            log(f"drone {self.n}: {text[:110]}")

    def listen(self) -> None:
        """Ground fault tests: just follow this drone's telemetry until told to stop."""
        master = None
        try:
            master = mavutil.mavlink_connection(self.launcher.connection_string)
            while not self.abort.is_set():
                msg = master.recv_match(blocking=True, timeout=1.0)
                if msg is not None and msg.get_type() != "BAD_DATA":
                    self._on_message(msg)
        except Exception as exc:  # noqa: BLE001
            self.r["listen_error"] = repr(exc)
            log(f"drone {self.n}: listener ended: {exc!r}")
        finally:
            if master is not None:
                master.close()

    def fly(self) -> None:
        t = time.monotonic()
        master = None
        try:
            master = mavutil.mavlink_connection(self.launcher.connection_string)
            upload_and_fly(master, [self.start, self.dest], ALTITUDE_M, mission_timeout_s=1500.0,
                           on_progress=self._on_progress, on_message=self._on_message,
                           should_abort=self.abort.is_set)
            self.r["outcome"] = "completed"
        except Exception as exc:  # noqa: BLE001
            self.r["outcome"] = f"FAILED {type(exc).__name__}: {exc}"
        finally:
            if master is not None:
                master.close()
        self.r["flight_s"] = round(time.monotonic() - t, 1)
        if self.pos is not None:
            self.r["final_pos"] = [round(self.pos[0], 7), round(self.pos[1], 7)]
            self.r["landing_error_m"] = round(distance_m(self.pos, self.dest), 2)
        self.r["final_alt_m"] = round(self.alt_m, 1)
        log(f"drone {self.n}: flight {self.r['outcome']} after {self.r['flight_s']}s, sysids seen {self.r['sysids_seen']}, "
            f"max alt {self.r['max_alt_m']} m, landing error {self.r.get('landing_error_m')} m")


def pgrep() -> str:
    found = _renode_processes()
    return "; ".join(f"{pid} {Path(argv[0]).name}" for pid, argv in found) or "(nothing)"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=("single", "separate"), required=True)
    ap.add_argument("--n", type=int, default=2)
    ap.add_argument("--gc", default="7", help="DOTNET_GCConserveMemory value, or 'default' to leave it unset")
    ap.add_argument("--fail", choices=("none", "physics", "pause", "halt", "killall"), default="none")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--cs-after-first-mach", action="store_true")
    ap.add_argument("--start-per-machine", action="store_true")
    ap.add_argument("--quantum", default="0.0001", help="single mode: global quantum in seconds")
    ap.add_argument("--debug-command", action="append", default=[],
                    help="extra monitor command run on every machine before start (diagnostics)")
    ap.add_argument("--unused", type=int, nargs="*", default=[],
                    help="drone numbers that are created as machines but not booted over MAVLink or flown "
                         "(in serial mode the FIRST machine never gets past early boot - see RESULTS.md)")
    ap.add_argument("--local-time", action="store_true",
                    help="single mode: give every machine its own time source (needed for --parallel)")
    ap.add_argument("--parallel", action="store_true",
                    help="single mode: leave Renode's default parallel machine execution on (it crashes)")
    ap.add_argument("--telnet", action="store_true",
                    help="monitor on telnet port %d instead of --console (needed for --fail pause)" % MONITOR_PORT)
    ap.add_argument("--fail-on-ground", action="store_true",
                    help="inject --fail once every drone has a GPS fix, on the ground, watching heartbeats "
                         "(no provisioning, no flight) - for setups too slow to fly")
    ap.add_argument("--slow", type=float, default=1.0, help="multiply the boot timeouts (slow emulation)")
    ap.add_argument("--boot-only", action="store_true", help="stop after the armable wait (no flight)")
    args = ap.parse_args()
    tag = args.tag or f"{args.mode}{args.n}" + ("" if args.fail == "none" else f"_{args.fail}")
    OUT.mkdir(exist_ok=True)
    single = args.mode == "single"
    drones = [Drone(n) for n in range(1, args.n + 1)]
    result: dict = {"tag": tag, "mode": args.mode, "n": args.n, "gc": args.gc, "fail": args.fail,
                    "options": {"cs_after_first_mach": args.cs_after_first_mach,
                                "start_per_machine": args.start_per_machine, "quantum": args.quantum,
                                "serial_execution": single and not args.parallel,
                                "local_time_sources": args.local_time},
                    "memory": [], "events": []}
    env = dict(os.environ)
    if args.gc != "default":
        env["DOTNET_GCConserveMemory"] = args.gc
    else:
        RenodeLauncher.RENODE_ENV = {}
    log(f"==== {tag}: {args.n} drone(s), mode {args.mode}, GCConserveMemory {args.gc}; before: {pgrep()}")
    result["memory"].append(memory_snapshot("before launch"))

    shared = None
    shared_log = None
    t_launch = time.monotonic()
    try:
        if single:
            for d in drones:
                d.launcher._check_ports_free()
                d.launcher._prepare_work_dir()
                d.launcher._start_physics(30.0)
            log(f"{args.n} physics sidecar(s) listening; work dirs prepared in {time.monotonic() - t_launch:.1f}s")
            script = OUT / f"{tag}.resc"
            script.write_text(make_fleet_resc.generate(
                [d.launcher for d in drones], cs_after_first_mach=args.cs_after_first_mach,
                start_per_machine=args.start_per_machine, quantum=args.quantum,
                debug_commands=tuple(args.debug_command), serial=not args.parallel,
                local_time=args.local_time))
            log_path = OUT / f"{tag}_renode.log"
            shared_log = open(log_path, "wb")
            t_launch = time.monotonic()
            shared = subprocess.Popen(
                [str(drones[0].launcher.renode_bin), "--disable-xwt",
                 *(["-P", str(MONITOR_PORT)] if args.telnet else ["--console"]),
                 "-e", f"include @{script}"],
                cwd=STANDALONE, preexec_fn=os.setsid, stdin=subprocess.DEVNULL, stdout=shared_log,
                stderr=subprocess.STDOUT, env=env)
            log(f"ONE Renode started, pid {shared.pid}, script {script.name}, console {log_path.name}")
            for d in drones:
                d.launcher._proc = shared
                d.launcher._renode_log_path = log_path

        machines = drones
        drones = [d for d in drones if d.n not in args.unused]
        boots = [threading.Thread(target=d.boot, args=(t_launch, single, args.slow, args.fail_on_ground))
                 for d in drones]
        for b in boots:
            b.start()
        for b in boots:
            b.join()
        result["boot_wall_s"] = round(time.monotonic() - t_launch, 1)
        result["memory"].append(memory_snapshot("after boot (all armable)"))
        booted = [d for d in drones if "armable_s" in d.r]
        if args.fail_on_ground and all("gps_fix_s" in d.r for d in drones):
            listeners = [threading.Thread(target=d.listen) for d in drones]
            for t in listeners:
                t.start()
            time.sleep(20 * args.slow)
            log("ground fault test; telemetry before: " + "; ".join(
                f"drone {d.n} last message {time.monotonic() - d.last_msg_at:.1f}s ago, sysids {d.r['sysids_seen']}"
                for d in drones))
            inject(args.fail, drones, shared, result)
            for d in drones:
                d.abort.set()
            for t in listeners:
                t.join()
        elif len(booted) < len(drones) or args.boot_only:
            log(f"{len(booted)} of {len(drones)} armable" + (" - boot only, not flying" if args.boot_only else " - not flying"))
        else:
            flights = [threading.Thread(target=d.fly) for d in drones]
            for f in flights:
                f.start()
            # once every drone is above 20 m: memory, CPU, then any fault injection
            deadline = time.monotonic() + 900
            while time.monotonic() < deadline and any(f.is_alive() for f in flights) and not all(d.alt_m > 20 for d in drones):
                time.sleep(1)
            if all(d.alt_m > 20 for d in drones):
                result["memory"].append(memory_snapshot(f"all {args.n} airborne (>20 m)"))
                result["cpu"] = cpu_snapshot()
                inject(args.fail, drones, shared, result)
            else:
                log("NOT every drone got above 20 m - no in-flight sample")
            for f in flights:
                f.join()
    finally:
        result["memory"].append(memory_snapshot("before stop"))
        result["renode_alive_at_end"] = bool(shared is not None and shared.poll() is None) if single else \
            [d.launcher.is_running for d in drones]
        if shared is not None:
            RenodeLauncher._terminate(shared)
            for d in drones:
                d.launcher._proc = None
        if shared_log is not None:
            shared_log.close()
        for d in locals().get("machines", drones):
            d.launcher._proc = None
            d.launcher.stop()
        time.sleep(2)
        result["pgrep_after_stop"] = pgrep()
        log(f"stopped; Renode/physics processes now: {result['pgrep_after_stop']}")

    result["drones"] = [d.r for d in drones]
    (OUT / f"{tag}.json").write_text(json.dumps(result, indent=1))
    log("==== RESULT " + tag)
    for d in drones:
        r = d.r
        log(f"  drone {d.n}: sysids {r['sysids_seen']} gps {r.get('gps_fix_s')}s armable {r.get('armable_s')}s "
            f"arm->50m {r.get('to_50m_s')}s max alt {r['max_alt_m']} m landing err {r.get('landing_error_m')} m "
            f"failsafes {r['failsafes']} -> {r.get('outcome', r.get('boot_error', 'not flown'))}")
    for m in result["memory"]:
        log(f"  {m['label']}: Renode total {m['renode_total_mb']} MB {m['renode_rss_mb']}")
    if "cpu" in result:
        log(f"  CPU in flight: {result['cpu']['total_cores']} cores {result['cpu']['cores']}")
    result["unused_machines"] = args.unused
    ok = all(d.r.get("outcome") == "completed" and d.r["sysids_seen"] == [d.n] for d in drones)
    log(f"  ALL COMPLETED WITH OWN SYSID: {ok}")
    return 0 if ok or args.fail != "none" or args.boot_only else 1


def inject(kind: str, drones: list[Drone], shared, result: dict) -> None:
    """16d. The victim is always the LAST drone; the others should keep flying."""
    if kind == "none":
        return
    victim, others = drones[-1], drones[:-1]
    events = result["events"]

    def note(text: str) -> None:
        events.append(f"{time.monotonic() - T0:.1f}s {text}")
        log("INJECT: " + text)

    def watch(seconds: float, label: str) -> None:
        marks = {d.n: (d.last_msg_at, d.alt_m) for d in drones}
        time.sleep(seconds)
        now = time.monotonic()
        note(f"{label} (+{seconds:.0f}s): " + "; ".join(
            f"drone {d.n} alt {marks[d.n][1]:.1f}->{d.alt_m:.1f} m, last telemetry {now - d.last_msg_at:.1f}s ago"
            for d in drones) + f"; Renode alive: {shared is None or shared.poll() is None}")

    if kind == "physics":
        pid = victim.launcher._physics_proc.pid
        note(f"SIGKILL drone {victim.n}'s renode-physics sidecar (pid {pid}) at alt {victim.alt_m:.1f} m")
        os.kill(pid, signal.SIGKILL)
        watch(15, "after the sidecar kill")
        watch(30, "later")
        note(f"aborting drone {victim.n}'s flight thread (its physics is gone)")
        victim.abort.set()
    elif kind == "pause":
        name = make_fleet_resc.machine_name(victim.launcher)
        note(f"pausing only machine {name} through the monitor, alt {victim.alt_m:.1f} m")
        result["monitor_pause"] = monitor(f'mach set "{name}"') + " || " + monitor("machine Pause")
        watch(20, f"with {name} paused")
        result["monitor_resume"] = monitor(f'mach set "{name}"') + " || " + monitor("machine Resume")
        watch(15, f"after resuming {name}")
        note(f"now removing machine {name}: mach rem")
        result["monitor_rem"] = monitor(f'mach rem "{name}"', wait_s=8.0)
        result["monitor_mach_list"] = monitor("mach")
        watch(20, f"after removing {name}")
        note(f"sidecar of drone {victim.n} still running: {victim.launcher._physics_proc.poll() is None}; "
             f"its MAVLink port {victim.launcher.port} still accepts: {port_open(victim.launcher.port)}")
        victim.abort.set()
    elif kind == "halt":
        name = make_fleet_resc.machine_name(victim.launcher)
        note(f"halting only {name}'s CPU through the monitor (cpu IsHalted true), alt {victim.alt_m:.1f} m")
        result["monitor_halt"] = monitor(f'mach set "{name}"') + " || " + monitor("cpu IsHalted true")
        watch(30, f"with {name}'s CPU halted")
        note(f"also disconnecting its physics and stopping its sidecar")
        result["monitor_disconnect"] = monitor("physics Disconnect")
        RenodeLauncher._terminate(victim.launcher._physics_proc)
        watch(30, f"with {name} halted and its sidecar stopped")
        note(f"sidecar of drone {victim.n} still running: {victim.launcher._physics_proc.poll() is None}; "
             f"its MAVLink port {victim.launcher.port} still accepts: {port_open(victim.launcher.port)}")
        result["monitor_unhalt"] = monitor("cpu IsHalted false")
        watch(15, f"after un-halting {name} (no physics)")
        victim.abort.set()
    elif kind == "killall":
        note(f"SIGKILL the whole Renode process (pid {shared.pid}); alts " + ", ".join(f"{d.alt_m:.1f}" for d in drones))
        os.killpg(os.getpgid(shared.pid), signal.SIGKILL)
        watch(10, "after killing Renode")
        note(f"processes left before cleanup: {pgrep()}")
        for d in drones:
            d.abort.set()
    del others


def port_open(port: int) -> bool:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=2):
            return True
    except OSError:
        return False


if __name__ == "__main__":
    sys.exit(main())
