"""Task 21 experiment runner: N drones in K shared-Renode processes, with
speed knobs, measuring where the time goes.

    speed_run.py --groups 1,2,3,4            # 4 drones in one Renode (today's settings)
    speed_run.py --groups 1,2,3,4/5,6,7,8    # 8 drones as 2 Renode processes x 4
    speed_run.py --groups 1,2,3,4 --boot-only --advance-immediately --tag ai4

Each group is one engine.shared_renode.SharedRenodeFleet (subclassed here
only to pass experiment options to the generator and to Renode's
environment - the engine is not changed). All groups boot in parallel; the
flights start once EVERY drone is armable, as the app does. Each drone
flies start -> 60 m north at 50 m with engine.mavlink_mission.upload_and_fly.

Measured: per-drone GPS fix / armable / arm->50 m / flight time; Renode RSS
and CPU per process, sidecar CPU; the real-time factor of every machine
(virtual seconds per wall second, from the monitor's `machine
ElapsedVirtualTime`, sampled every 30 s); the lowest MemAvailable.
Result: out/<tag>.json and a one-line summary. Nothing is left running.

Headless: drones are plain sysids, no GUI profiles are involved.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO))

from pymavlink import mavutil  # noqa: E402

from engine.mavlink_mission import upload_and_fly  # noqa: E402
from engine.renode_launcher import RenodeLauncher, _renode_processes  # noqa: E402
from engine.shared_renode import SharedRenodeFleet, generate_fleet_script, machine_name  # noqa: E402

STANDALONE = REPO / "pixhawk6c_renode_standalone"
OUT = HERE / "out"
BASE_LAT, BASE_LON = -35.363261, 149.165230
SPACING_M, LEG_NORTH_M, ALTITUDE_M = 18.0, 60.0, 50.0
P_CORE_CPUS = set(range(0, 12))      # i7-12650H: 6 P-cores x 2 threads
E_CORE_CPUS = set(range(12, 16))     # 4 E-cores
RENODE_GB, EXTRA_DRONE_GB = 2.2, 0.15
T0 = time.monotonic()
_lock = threading.Lock()


def log(msg: str) -> None:
    with _lock:
        print(f"[{time.monotonic() - T0:7.1f}s] {msg}", flush=True)


def mem_available_mb() -> int:
    for line in Path("/proc/meminfo").read_text().splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) // 1024
    return 0


def rss_mb(pid: int) -> int:
    try:
        for line in Path(f"/proc/{pid}/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) // 1024
    except OSError:
        pass
    return 0


def cpu_seconds(pid: int) -> float:
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")
    except OSError:
        return 0.0


def per_cpu_busy() -> dict[int, float]:
    """Cumulative busy jiffies per logical CPU."""
    busy = {}
    for line in Path("/proc/stat").read_text().splitlines():
        m = re.match(r"cpu(\d+) (.*)", line)
        if m:
            v = [int(x) for x in m.group(2).split()]
            busy[int(m.group(1))] = (sum(v) - v[3] - v[4], sum(v))
    return busy


def offset(lat, lon, north_m, east_m):
    return (lat + north_m / 111320.0, lon + east_m / (111320.0 * math.cos(math.radians(lat))))


def distance_m(a, b):
    return math.hypot((a[0] - b[0]) * 111320.0, (a[1] - b[1]) * 111320.0 * math.cos(math.radians(a[0])))


class SpeedFleet(SharedRenodeFleet):
    """SharedRenodeFleet with the experiment's knobs. Only _start_renode
    differs: generator options, extra script lines, Renode's environment
    and CPU affinity."""

    GPS_TIMEOUT_S = 600.0          # generous: slow configurations must still be measurable
    PROVISION_TIMEOUT_S = 120.0
    ARMABLE_TIMEOUT_S = 600.0

    def configure(self, index: int, options: argparse.Namespace) -> None:
        self.options = options
        self.work_dir = self.work_dir.parent / f"speed-group-{index}"
        self.script_path = self.work_dir / "fleet.resc"
        self.log_path = self.work_dir / "renode-console.log"

    def _start_renode(self) -> None:
        o = self.options
        self.work_dir.mkdir(parents=True, exist_ok=True)
        script = generate_fleet_script(list(self.launchers.values()), quantum=o.quantum,
                                       master_quantum=o.master_quantum)
        if o.advance_immediately:
            script = script.replace("emulation SetGlobalAdvanceImmediately false",
                                    "emulation SetGlobalAdvanceImmediately true")
            assert "SetGlobalAdvanceImmediately true" in script
        self.script_path.write_text(script)
        self._log = open(self.log_path, "wb")
        env = dict(os.environ)
        if o.gc == "7":
            env.update(RenodeLauncher.RENODE_ENV)
        elif o.gc != "default":
            for item in o.gc.split(","):
                key, value = item.split("=", 1)
                env[key] = value
        cpus = o.renode_cpus

        def setup():
            os.setsid()
            if cpus:
                os.sched_setaffinity(0, cpus)   # inherited by every thread Renode starts
        self._proc = subprocess.Popen(
            [str(self.renode_bin), "--disable-xwt", "-P", str(self.monitor_port), "-e", f"include @{self.script_path}"],
            cwd=self.standalone_dir, preexec_fn=setup, stdin=subprocess.DEVNULL, stdout=self._log,
            stderr=subprocess.STDOUT, env=env)

    def virtual_times(self) -> dict[int, float]:
        """{sysid: that machine's elapsed virtual time in seconds}."""
        commands = []
        for sysid in self.launchers:
            commands += [f'mach set "{machine_name(sysid)}"', "machine ElapsedVirtualTime"]
        replies = self.monitor(commands, reply_timeout_s=8.0)
        times = {}
        for sysid, reply in zip(self.launchers, replies[1::2]):
            m = re.search(r"Elapsed Virtual Time: (\d+):(\d+):([\d.]+)", reply)
            if m:
                times[sysid] = int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3))
        return times


class Drone:
    def __init__(self, sysid: int):
        self.n = sysid
        self.start = offset(BASE_LAT, BASE_LON, 0.0, SPACING_M * (sysid - 1))
        self.dest = offset(*self.start, LEG_NORTH_M, 0.0)
        self.r = {"drone": sysid, "sysids_seen": [], "failsafes": 0, "max_alt_m": 0.0}
        self.alt_m, self.pos, self.armed_at = 0.0, None, None
        self.abort = threading.Event()

    def on_message(self, msg) -> None:
        sysid = msg.get_srcSystem()
        if sysid not in self.r["sysids_seen"]:
            self.r["sysids_seen"].append(sysid)
        if msg.get_type() == "GLOBAL_POSITION_INT":
            self.alt_m = msg.relative_alt / 1000.0
            self.pos = (msg.lat / 1e7, msg.lon / 1e7)
            self.r["max_alt_m"] = max(self.r["max_alt_m"], round(self.alt_m, 1))
            if self.armed_at is not None and "to_50m_s" not in self.r and self.alt_m >= ALTITUDE_M - 0.5:
                self.r["to_50m_s"] = round(time.monotonic() - self.armed_at, 1)

    def on_progress(self, text: str) -> None:
        if "failsafe" in text.lower():
            self.r["failsafes"] += 1
            log(f"drone {self.n}: {text[:100]}")
        if text.startswith("Arm ack") and self.armed_at is None:
            self.armed_at = time.monotonic()

    def fly(self, connection: str) -> None:
        t, master = time.monotonic(), None
        try:
            master = mavutil.mavlink_connection(connection)
            upload_and_fly(master, [self.start, self.dest], ALTITUDE_M, mission_timeout_s=1800.0,
                           on_progress=self.on_progress, on_message=self.on_message, should_abort=self.abort.is_set)
            self.r["outcome"] = "completed"
        except Exception as exc:  # noqa: BLE001
            self.r["outcome"] = f"FAILED {type(exc).__name__}: {exc}"
        finally:
            if master is not None:
                master.close()
        self.r["flight_s"] = round(time.monotonic() - t, 1)
        if self.pos is not None:
            self.r["landing_error_m"] = round(distance_m(self.pos, self.dest), 2)
        log(f"drone {self.n}: {self.r['outcome']} after {self.r['flight_s']}s, sysids {self.r['sysids_seen']}, "
            f"arm->50m {self.r.get('to_50m_s')}s, landing error {self.r.get('landing_error_m')} m")


def thread_snapshot(pid: int) -> list:
    out = subprocess.run(["ps", "-L", "-o", "psr=,pcpu=,comm=", "-p", str(pid)], capture_output=True, text=True).stdout
    rows = []
    for line in out.splitlines():
        parts = line.split(None, 2)
        if len(parts) == 3 and float(parts[1]) >= 2.0:
            rows.append((int(parts[0]), float(parts[1]), parts[2].strip()))
    return sorted(rows, key=lambda r: -r[1])[:12]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--groups", required=True, help='sysids per Renode process, e.g. "1,2,3,4/5,6,7,8"')
    ap.add_argument("--tag", required=True)
    ap.add_argument("--quantum", default="0.01", help="each machine's quantum (s)")
    ap.add_argument("--master-quantum", default="0.1")
    ap.add_argument("--advance-immediately", action="store_true")
    ap.add_argument("--gc", default="7", help="'7' (DOTNET_GCConserveMemory=7), 'default', or VAR=VALUE[,VAR=VALUE]")
    ap.add_argument("--affinity", choices=("none", "renode-p", "renode-p1", "renode-e"), default="none",
                    help="renode-p: Renode on the P-core CPUs (0-11), sidecars on the E-cores; renode-p1: Renode on one "
                         "logical CPU per P-core (0,2,..,10), sidecars on the E-cores; renode-e: Renode on the E-cores, "
                         "sidecars on the P-cores (control)")
    ap.add_argument("--boot-only", action="store_true", help="stop once every drone is armable (plus 30 s)")
    args = ap.parse_args()
    groups = [[int(x) for x in g.split(",")] for g in args.groups.split("/")]
    sysids = [s for g in groups for s in g]
    args.renode_cpus = {"none": None, "renode-p": P_CORE_CPUS, "renode-p1": set(range(0, 12, 2)),
                        "renode-e": E_CORE_CPUS}[args.affinity]
    sidecar_cpus = {"none": None, "renode-p": E_CORE_CPUS, "renode-p1": E_CORE_CPUS,
                    "renode-e": P_CORE_CPUS}[args.affinity]
    OUT.mkdir(exist_ok=True)

    estimate_mb = int(1024 * sum(RENODE_GB + EXTRA_DRONE_GB * (len(g) - 1) for g in groups))
    available = mem_available_mb()
    result = {"tag": args.tag, "groups": groups, "drones": len(sysids), "processes": len(groups),
              "options": {k: (sorted(v) if isinstance(v, set) else v) for k, v in vars(args).items()},
              "estimate_mb": estimate_mb, "mem_available_before_mb": available}
    log(f"==== {args.tag}: {len(sysids)} drones in {len(groups)} Renode process(es) {groups}; "
        f"estimated Renode RAM {estimate_mb} MB, MemAvailable {available} MB; before: "
        f"{[(p, Path(a[0]).name) for p, a in _renode_processes()] or '(nothing)'}")
    if available - estimate_mb < 1500:
        log(f"SKIPPED: would leave {available - estimate_mb} MB available (< 1500 MB)")
        result["skipped"] = f"would leave {available - estimate_mb} MB available"
        (OUT / f"{args.tag}.json").write_text(json.dumps(result, indent=1))
        return 2

    drones = {s: Drone(s) for s in sysids}
    fleets = []
    for i, group in enumerate(groups, 1):
        fleet = SpeedFleet(str(STANDALONE), [(s, *drones[s].start) for s in group])
        fleet.configure(i, args)
        fleets.append(fleet)
    connections: dict[int, str] = {}
    boot_errors: list[str] = []
    t_launch = time.monotonic()
    stop_sampling = threading.Event()
    samples: list[dict] = []
    low_memory = threading.Event()
    state = {"min_avail": available, "phase": "boot"}

    def phase_cb(sysid: int, text: str) -> None:
        key = {"GPS fix": "gps_fix_s", "Armable": "armable_s"}.get(text.split(" - ")[0])
        if key:
            drones[sysid].r[key] = round(time.monotonic() - t_launch, 1)
            log(f"drone {sysid}: {text.split(' - ')[0]} at {drones[sysid].r[key]}s")

    def boot(fleet: SpeedFleet) -> None:
        try:
            connections.update(fleet.start(on_phase=phase_cb))
            if sidecar_cpus:
                for pid in fleet.pids()["physics"].values():
                    for task in Path(f"/proc/{pid}/task").iterdir():
                        os.sched_setaffinity(int(task.name), sidecar_cpus)
        except Exception as exc:  # noqa: BLE001
            boot_errors.append(str(exc))
            log(f"BOOT ERROR: {exc}")

    def sampler() -> None:
        """Every 30 s: each machine's virtual time (-> real-time factor), memory, CPU."""
        last_wall, last_virtual = time.monotonic(), {}
        last_cpu, last_percpu = {}, per_cpu_busy()
        while not stop_sampling.wait(30.0):
            now = time.monotonic()
            avail = mem_available_mb()
            state["min_avail"] = min(state["min_avail"], avail)
            if avail < 1000:
                log(f"ABORT: MemAvailable {avail} MB < 1000 MB")
                low_memory.set()
                for d in drones.values():
                    d.abort.set()
                for f in fleets:
                    f.stop_all()
                return
            virtual, rtf = {}, {}
            for fleet in fleets:
                if fleet.is_running:
                    try:
                        virtual.update(fleet.virtual_times())
                    except OSError as exc:
                        log(f"(virtual time sample failed: {exc})")
            for s, v in virtual.items():
                if s in last_virtual:
                    rtf[s] = round((v - last_virtual[s]) / (now - last_wall), 3)
            procs = {pid: Path(argv[0]).name for pid, argv in _renode_processes()}
            cpu_now = {pid: cpu_seconds(pid) for pid in procs}
            cores = {pid: round((cpu_now[pid] - last_cpu[pid]) / (now - last_wall), 2) for pid in procs if pid in last_cpu}
            renode_cores = {pid: c for pid, c in cores.items() if procs[pid] == "renode"}
            sidecar_cores = round(sum(c for pid, c in cores.items() if procs[pid] == "renode-physics"), 2)
            percpu = per_cpu_busy()
            util = {c: round(100 * (percpu[c][0] - last_percpu[c][0]) / max(1, percpu[c][1] - last_percpu[c][1]))
                    for c in percpu if c in last_percpu}
            sample = {"t": round(now - t_launch, 1), "phase": state["phase"], "rtf": rtf, "renode_cores": renode_cores,
                      "sidecar_cores": sidecar_cores, "mem_available_mb": avail, "cpu_util_pct": util,
                      "renode_rss_mb": {pid: rss_mb(pid) for pid in renode_cores}}
            samples.append(sample)
            if rtf:
                log(f"sample {state['phase']}: RTF {rtf} (mean {sum(rtf.values()) / len(rtf):.3f}); Renode cores "
                    f"{renode_cores}, sidecars {sidecar_cores}; avail {avail} MB; "
                    f"P-core CPUs {sum(util.get(c, 0) for c in P_CORE_CPUS) // 12}% E-core CPUs "
                    f"{sum(util.get(c, 0) for c in E_CORE_CPUS) // 4}%")
            last_wall, last_virtual, last_cpu, last_percpu = now, virtual, cpu_now, percpu

    try:
        sampling = threading.Thread(target=sampler, daemon=True)
        sampling.start()
        boots = [threading.Thread(target=boot, args=(f,)) for f in fleets]
        for b in boots:
            b.start()
        for b in boots:
            b.join()
        result["boot_wall_s"] = round(time.monotonic() - t_launch, 1)
        renode_pids = [f.pids()["renode"] for f in fleets if f.pids()["renode"]]
        result["rss_after_boot_mb"] = {pid: rss_mb(pid) for pid in renode_pids}
        if boot_errors or low_memory.is_set() or len(connections) < len(sysids):
            result["boot_errors"] = boot_errors
            log(f"NOT flying: {len(connections)} of {len(sysids)} armable; errors {boot_errors}")
        elif args.boot_only:
            log(f"all {len(sysids)} armable after {result['boot_wall_s']}s; Renode RSS {result['rss_after_boot_mb']} - "
                "boot only: sampling 35 s more")
            result["threads_armable"] = {pid: thread_snapshot(pid) for pid in renode_pids}
            log(f"hot threads (cpu, %, name): {result['threads_armable']}")
            state["phase"] = "idle-armable"
            time.sleep(35)
        else:
            log(f"all {len(sysids)} armable after {result['boot_wall_s']}s; Renode RSS {result['rss_after_boot_mb']} - flying")
            state["phase"] = "flight"
            flights = [threading.Thread(target=drones[s].fly, args=(connections[s],)) for s in sysids]
            for f in flights:
                f.start()
            deadline = time.monotonic() + 1200
            while time.monotonic() < deadline and any(f.is_alive() for f in flights) and \
                    not all(d.alt_m > 20 for d in drones.values()):
                time.sleep(1)
            if all(d.alt_m > 20 for d in drones.values()):
                result["rss_airborne_mb"] = {pid: rss_mb(pid) for pid in renode_pids}
                result["threads_airborne"] = {pid: thread_snapshot(pid) for pid in renode_pids}
                log(f"all airborne: Renode RSS {result['rss_airborne_mb']} = {sum(result['rss_airborne_mb'].values())} MB; "
                    f"hot threads (cpu, %, name): {result['threads_airborne']}")
            for f in flights:
                f.join()
            result["launch_to_all_landed_s"] = round(time.monotonic() - t_launch, 1)
            result["rss_end_mb"] = {pid: rss_mb(pid) for pid in renode_pids}
    finally:
        stop_sampling.set()
        for f in fleets:
            f.stop_all()
        time.sleep(2)
        left = [(pid, Path(argv[0]).name) for pid, argv in _renode_processes()]
        result["left_running"] = left
        log(f"stopped; left running: {left or '(nothing)'}")

    result["drone_results"] = [drones[s].r for s in sysids]
    result["samples"] = samples
    result["min_mem_available_mb"] = state["min_avail"]
    result["aborted_low_memory"] = low_memory.is_set()

    def mean_rtf(phase: str):
        values = [v for s in samples if s["phase"] == phase for v in s["rtf"].values()]
        return (round(sum(values) / len(values), 3), round(min(values), 3)) if values else (None, None)
    result["rtf_boot_mean_min"] = mean_rtf("boot")
    result["rtf_flight_mean_min"] = mean_rtf("flight")
    flight_samples = [s for s in samples if s["phase"] == ("flight" if not args.boot_only else "boot")]
    if flight_samples:
        result["renode_cores_mean"] = round(sum(sum(s["renode_cores"].values()) for s in flight_samples) / len(flight_samples), 2)
        result["sidecar_cores_mean"] = round(sum(s["sidecar_cores"] for s in flight_samples) / len(flight_samples), 2)
    ok = (not left and not low_memory.is_set() and not boot_errors and
          (args.boot_only or all(d.r.get("outcome") == "completed" and d.r["sysids_seen"] == [d.n] and
                                 d.r["failsafes"] == 0 for d in drones.values())))
    result["success"] = bool(ok)
    (OUT / f"{args.tag}.json").write_text(json.dumps(result, indent=1))

    def span(key):
        values = [d.r[key] for d in drones.values() if key in d.r]
        return f"{min(values):.0f}-{max(values):.0f}" if values else "-"
    log(f"==== RESULT {args.tag}: {len(sysids)} drones x {len(groups)} proc | success={ok} | gps {span('gps_fix_s')} | "
        f"armable {span('armable_s')} | arm->50m {span('to_50m_s')} | flight {span('flight_s')} | "
        f"landed {result.get('launch_to_all_landed_s')} | RSS boot {sum(result.get('rss_after_boot_mb', {}).values())} "
        f"air {sum(result.get('rss_airborne_mb', {}).values())} end {sum(result.get('rss_end_mb', {}).values())} MB | "
        f"RTF boot {result['rtf_boot_mean_min']} flight {result['rtf_flight_mean_min']} | cores renode "
        f"{result.get('renode_cores_mean')} sidecars {result.get('sidecar_cores_mean')} | min avail {state['min_avail']} MB")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
