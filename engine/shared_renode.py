"""A fleet of drones as machines inside ONE Renode process.

The alternative to one RenodeLauncher (one Renode, about 2.2 GB) per drone:
the .NET runtime, Renode itself and the compiled peripherals are paid for
once, so four drones take about 2.5 GB instead of about 9 GB, for roughly
20 % more time (experiments/single_renode/RESULTS.md).

Each drone is still prepared by a RenodeLauncher(instance=sysid): the same
MAVLink port (5762+N), physics port (9002+N), sysid, SD / FRAM / flash
copies and physics sidecar as in the per-process fleet. Only Renode itself
is shared: `generate_fleet_script` writes one script that creates a machine
per drone, and `SharedRenodeFleet` runs it and takes every drone through the
launcher's own GPS-fix, provisioning and armable steps.

The standalone folder's scripts each assume one machine per Renode (`mach
create "ardupilot"`, a server socket terminal called "serial", CAN hubs
called "can1Hub" / "can2Hub"), so they can't be included once per drone.
The generated script copies their content instead - the folder's files are
only ever read.
"""
from __future__ import annotations

import os
import re
import socket
import subprocess
import threading
import time
from pathlib import Path
from typing import Callable

from engine.renode_launcher import RenodeLauncher, RenodeLauncherError

# Emulation-level objects the board script names "serial", "can1Hub" and
# "can2Hub" (and the launcher's "gpsHub"): one per drone here.
_PER_DRONE_NAMES = ("serial", "can1Hub", "can2Hub")

_ANSI = re.compile(r"\x1b\[[0-9;]*m")
# The monitor's prompt: "(monitor) " or "(<current machine>) " at the end of its output.
_PROMPT = re.compile(r"\(\S+\) ?$")
# How the monitor reports a command it could not run.
_MONITOR_ERROR = re.compile(r"There was an error executing command|Bad parameters for command|"
                            r"No such command or device|Could not tokenize|does not provide a field",
                            re.IGNORECASE)

# Each machine's own quantum - the standalone scripts' value.
MACHINE_QUANTUM_S = "0.01"
# How often the machines are brought back in step with each other. They
# never talk to each other inside Renode, so this can be coarser than their
# own quantum; it is also the most (virtual) time a MAVLink byte from
# outside can wait before its machine sees it.
MASTER_QUANTUM_S = "0.1"


class DroneStopError(RenodeLauncherError):
    """stop_drone() could not confirm that the drone's machine is halted,
    although the shared Renode is still running."""


def machine_name(sysid: int) -> str:
    return f"drone{sysid}"


def _cs_includes(script: Path) -> list[str]:
    return [line.strip() for line in script.read_text().splitlines()
            if re.match(r"^include \S+\.cs$", line.strip())]


def _common_script(launcher: RenodeLauncher) -> Path:
    """The script the board script includes for everything common to the
    MCU family (ardupilot_h743.resc): the peripheral sources and the
    `mach create` this module replaces."""
    for line in launcher.board_script.read_text().splitlines():
        match = re.match(r"^include @(\S+\.resc)$", line.strip())
        if match:
            return Path(match.group(1))
    raise RenodeLauncherError(f"{launcher.board_script.name} includes no common .resc - not a layout this knows")


def _board_body(launcher: RenodeLauncher, tag: str) -> list[str]:
    """The board script's own per-machine lines (everything after its
    include of the common script), with the MAVLink port and the
    emulation-level names made this drone's own, and its `logLevel` line
    left out (set once for the whole emulation)."""
    lines = launcher.board_script.read_text().splitlines()
    start = next(i for i, line in enumerate(lines) if re.match(r"^include @\S+\.resc$", line.strip()))
    body = []
    for line in lines[start + 1:]:
        stripped = line.strip()
        if stripped.startswith("#") or stripped.startswith("logLevel") or stripped.startswith("$sdcard?="):
            continue
        line = line.replace(f"CreateServerSocketTerminal {launcher.declared_port} ",
                            f"CreateServerSocketTerminal {launcher.port} ")
        for name in _PER_DRONE_NAMES:
            line = re.sub(rf'(?<![\w.]){name}(?![\w])', f"{name}_{tag}", line)
        line = line.replace("$sdcard", f"@{launcher.sdcard_path}")
        body.append(line)
    return body


def generate_fleet_script(
    launchers: list[RenodeLauncher], *, local_time: bool = True, serial: bool = False,
    quantum: str = MACHINE_QUANTUM_S, master_quantum: str | None = MASTER_QUANTUM_S,
    cs_after_first_mach: bool = False, start_per_machine: bool = False,
    debug_commands: tuple[str, ...] = (),
) -> str:
    """One Renode script that runs every launcher's drone as its own machine.

    `launchers` are RenodeLauncher(instance=N >= 1) objects whose work dirs
    are already prepared (launcher.prepare()). The script has:

      1. the variables and every `include *.cs` line, once (the peripherals
         are compiled once for the whole process);
      2. per drone: a machine "droneN" with its own platform (its own FRAM /
         persistent-flash copies), ELF, vector table, reset macro, MAVLink
         server socket, CAN hubs, SD card, hooks, physics connection and GPS
         UART hub - every emulation-level object gets a per-drone name;
      3. the emulation-wide settings and one `start`.

    `local_time` (the default, and what the app uses) creates each machine
    with its own time source. The monitor's `mach create` instead makes every
    machine a direct sink of the emulation's one master time source, and
    then one machine's timers are advanced from another machine's CPU
    thread: with two of these machines a CPU takes a spurious interrupt or
    the emulation freezes within a second. Everything after `master_quantum`
    is for experiments/single_renode only.
    """
    first = launchers[0]
    launch = {m.group(1): m.group(2) for line in first._launch_lines
              if (m := re.match(r"^\$(\w+)=(\S+)$", line.strip()))}
    board_vars = {m.group(1): m.group(2) for line in first.board_script.read_text().splitlines()
                  if (m := re.match(r"^\$(\w+)\??=(\S+)$", line.strip()))}
    vector_base = launch.get("vector_base", board_vars["app_base"])
    binaries = [line.strip() for line in first._launch_lines if line.strip().startswith("sysbus LoadBinary ")]

    includes = _cs_includes(first.board_script) + _cs_includes(_common_script(first))
    out = [
        f"# GENERATED by engine/shared_renode.py - {len(launchers)} drone(s) as machines in one Renode.",
        f"$repo={launch['repo']}",
        f"$elf={launch['elf']}",
        f"$renode_data={launch['renode_data']}",
        f"$mcu_svd={launch['mcu_svd']}",
        f"$vector_base={vector_base}",
        "",
    ]
    if not cs_after_first_mach:
        out += ["# every peripheral source, compiled once for the whole process", *includes, ""]

    for index, launcher in enumerate(launchers):
        name = machine_name(launcher.instance)
        tag = f"d{launcher.instance}"
        platform = launcher.work_dir / launcher.platform_repl.name
        persistent = launcher.work_dir / launcher.persistent_repl.name
        out.append(f"# ---- {name}: MAVLink {launcher.port}, physics {launcher.physics_port} ----")
        if local_time:
            # Machine(createLocalTimeSource=True): the monitor has no command for it.
            out += ['python "from Antmicro.Renode.Core import Machine, EmulationManager; '
                    f"EmulationManager.Instance.CurrentEmulation.AddMachine(Machine(True), '{name}')\"",
                    f'mach set "{name}"']
        else:
            out.append(f'mach create "{name}"')
        if cs_after_first_mach and index == 0:
            out += includes
        out += [
            f"machine LoadPlatformDescription @{platform}",
            "sysbus ApplySVD $mcu_svd",
            "sysbus LoadELF $elf",
            "cpu VectorTableOffset $vector_base",
            "cpu PerformanceInMips 300",
            *_board_body(launcher, tag),
            *binaries,
            f"machine LoadPlatformDescription @{persistent}",
            'physics Connect %d "%s" %.6f %.6f %.1f %.1f %d' % (
                launcher.physics_port, launcher.PHYSICS_MODEL, launcher.latitude_deg, launcher.longitude_deg,
                launcher.PHYSICS_ALTITUDE_M, launcher.PHYSICS_HEADING_DEG, launcher.PHYSICS_RATE_HZ),
            f'emulation CreateUARTHub "gpsHub_{tag}"',
            f"connector Connect sysbus.{launcher.GPS_UART_HOST} gpsHub_{tag}",
            f"connector Connect sysbus.gps gpsHub_{tag}",
            "",
        ]

    out += [
        "# emulation-wide (the standalone scripts set these once per process)",
        f'emulation SetGlobalQuantum "{quantum}"',
        "emulation SetGlobalAdvanceImmediately false",
        "logLevel 3",
    ]
    if master_quantum and local_time:
        out.append(f'emulation SetQuantum "{master_quantum}"')
    if serial:
        # Experiments only: with `mach create` machines the one setting in
        # which every machine runs is serial execution with a 100 us quantum.
        out.append("emulation SetGlobalSerialExecution true")
    for launcher in launchers if debug_commands else ():
        out += [f'mach set "{machine_name(launcher.instance)}"', *debug_commands]
    if start_per_machine:
        for launcher in launchers:
            out += [f'mach set "{machine_name(launcher.instance)}"', "machine Start"]
    else:
        out.append("start")
    return "\n".join(out) + "\n"


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


class SharedRenodeFleet:
    """N drones in one Renode. `start()` boots them all to armable and
    returns {sysid: MAVLink connection string}; `stop_drone()` takes one out
    while the rest keep flying; `stop_all()` ends everything.

    `drones` is [(sysid, latitude_deg, longitude_deg)] - where each drone
    spawns. Sysids are 1 or higher and unique; each is that drone's
    RenodeLauncher instance number (ports, files, MAV_SYSID).
    """

    # Boot waits. With N machines in one process a boot takes longer than a
    # lone Renode's: measured GPS fix 55 s (2 drones) and 71-75 s (4), all
    # armable after 174 s (2) and 230-244 s (4), against 192-195 s for 4
    # separate processes. Each of the launcher's own timeouts is stretched
    # by 25 % per extra drone (x1.75 for 4), about three times that slowdown.
    GPS_TIMEOUT_S = 240.0
    PROVISION_TIMEOUT_S = 45.0
    ARMABLE_TIMEOUT_S = 180.0
    TIMEOUT_GROWTH_PER_DRONE = 0.25
    # Renode announces its monitor about a second after it starts, before it
    # reads the fleet script; 30 s is ample and still far from a boot wait.
    MONITOR_READY_TIMEOUT_S = 30.0
    # One monitor command normally answers within tens of milliseconds.
    MONITOR_REPLY_TIMEOUT_S = 5.0
    STOP_DRONE_ATTEMPTS = 2

    def __init__(self, standalone_dir: str, drones: list[tuple[int, float, float]],
                 work_root: str | None = None):
        sysids = [sysid for sysid, _lat, _lon in drones]
        if not drones:
            raise RenodeLauncherError("a shared-Renode fleet needs at least one drone")
        if len(set(sysids)) != len(sysids) or min(sysids) < 1:
            raise RenodeLauncherError(f"shared-Renode fleet sysids must be unique and >= 1, got {sysids}")
        self.launchers: dict[int, RenodeLauncher] = {
            sysid: RenodeLauncher(standalone_dir, instance=sysid, latitude_deg=lat, longitude_deg=lon,
                                  work_root=work_root)
            for sysid, lat, lon in drones
        }
        first = next(iter(self.launchers.values()))
        self.standalone_dir = first.standalone_dir
        self.renode_bin = first.renode_bin
        self.work_dir = first.work_root / "shared"
        self.script_path = self.work_dir / "fleet.resc"
        self.log_path = self.work_dir / "renode-console.log"
        self.monitor_port: int | None = None
        self._proc: subprocess.Popen | None = None
        self._log = None
        self._lock = threading.Lock()
        self._stopped_drones: set[int] = set()

    @property
    def timeout_scale(self) -> float:
        return 1.0 + self.TIMEOUT_GROWTH_PER_DRONE * (len(self.launchers) - 1)

    def connection_string(self, sysid: int) -> str:
        return self.launchers[sysid].connection_string

    # ---- start ----

    def start(
        self,
        cancel: threading.Event | None = None,
        on_phase: Callable[[int, str], None] | None = None,
    ) -> dict[int, str]:
        """Sidecars, one Renode, then every drone in parallel through GPS
        fix, provisioning and the armable wait. Returns {sysid: connection
        string} once all are armable. If any drone fails, everything is
        stopped and one RenodeLauncherError naming that drone is raised.
        `on_phase(sysid, text)` reports each drone's boot phase; `cancel`
        aborts the boot at any stage."""
        if self._proc is not None:
            raise RenodeLauncherError("already running - call stop_all() first")
        phase = on_phase or (lambda _sysid, _text: None)
        self._stopped_drones.clear()

        def check_cancel() -> None:
            if cancel is not None and cancel.is_set():
                raise RenodeLauncherError("shared-Renode fleet launch cancelled")

        try:
            # Every port first: nothing is started if any is taken.
            for sysid, launcher in self.launchers.items():
                try:
                    launcher.check_ports_free()
                except RenodeLauncherError as exc:
                    raise RenodeLauncherError(f"drone with SYSID {sysid}: {exc}") from exc
            self.monitor_port = _free_port()
            check_cancel()
            for sysid in self.launchers:
                phase(sysid, "Booting Renode")   # all at once: they boot together
            for sysid, launcher in self.launchers.items():
                launcher.prepare()
                check_cancel()
                launcher.start_physics()
                check_cancel()
            self._start_renode()
            self._wait_for_monitor(check_cancel)
            for launcher in self.launchers.values():
                launcher.attach(self._proc, self.log_path, cancel)
            return self._boot_all(phase)
        except Exception:
            self.stop_all()
            raise

    def _start_renode(self) -> None:
        self.work_dir.mkdir(parents=True, exist_ok=True)
        self.script_path.write_text(generate_fleet_script(list(self.launchers.values())))
        self._log = open(self.log_path, "wb")
        # As RenodeLauncher._start_renode, except -P instead of --console:
        # the telnet monitor is how one drone is stopped later (stop_drone).
        self._proc = subprocess.Popen(
            [str(self.renode_bin), "--disable-xwt", "-P", str(self.monitor_port),
             "-e", f"include @{self.script_path}"],
            cwd=self.standalone_dir,
            preexec_fn=os.setsid,
            stdin=subprocess.DEVNULL,
            stdout=self._log,
            stderr=subprocess.STDOUT,
            env={**os.environ, **RenodeLauncher.RENODE_ENV},
        )

    def _wait_for_monitor(self, check_cancel: Callable[[], None] = lambda: None) -> None:
        """Don't let a fleet boot (let alone fly) without the monitor that
        stop_drone() needs. The port was only known to be free a moment
        before Renode started, so confirm that it is THIS Renode listening
        on it: its own log line announcing the monitor on that port, and a
        connection that is accepted. Otherwise the launch fails here."""
        announcement = f"Monitor available in telnet mode on port {self.monitor_port}".encode()
        deadline = time.monotonic() + self.MONITOR_READY_TIMEOUT_S
        why = "Renode never announced it"
        while time.monotonic() < deadline:
            check_cancel()
            if self._proc is None or self._proc.poll() is not None:
                code = None if self._proc is None else self._proc.returncode
                raise RenodeLauncherError(
                    f"the shared Renode exited (code {code}) before its monitor came up on port "
                    f"{self.monitor_port} - see {self.log_path}")
            try:
                announced = announcement in self.log_path.read_bytes()
            except OSError:
                announced = False
            if announced:
                try:
                    socket.create_connection(("127.0.0.1", self.monitor_port), timeout=2).close()
                    return
                except OSError as exc:
                    why = f"it was announced but does not accept connections ({exc})"
            time.sleep(0.25)
        raise RenodeLauncherError(
            f"Renode's monitor didn't come up on port {self.monitor_port} within "
            f"{self.MONITOR_READY_TIMEOUT_S:.0f} s ({why}) - the fleet was not started")

    def _boot_all(self, phase: Callable[[int, str], None]) -> dict[int, str]:
        scale = self.timeout_scale
        errors: dict[int, str] = {}
        failed = threading.Event()

        def boot_one(sysid: int, launcher: RenodeLauncher) -> None:
            try:
                launcher.wait_for_gps_fix(self.GPS_TIMEOUT_S * scale)
                phase(sysid, "GPS fix - waiting for armable")
                launcher.provision_first_boot_params(self.PROVISION_TIMEOUT_S * scale)
                launcher.wait_until_armable(self.ARMABLE_TIMEOUT_S * scale)
                phase(sysid, "Armable - waiting for the fleet")
            except Exception as exc:  # noqa: BLE001 - any boot failure fails the fleet
                with self._lock:
                    # Only the first failure is the cause; stopping the
                    # shared Renode then fails the others too.
                    if not failed.is_set():
                        errors[sysid] = str(exc)
                        failed.set()
                self.stop_all()

        threads = [threading.Thread(target=boot_one, args=item, name=f"shared-boot-{item[0]}", daemon=True)
                   for item in self.launchers.items()]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        if errors:
            sysid, message = next(iter(errors.items()))
            raise RenodeLauncherError(f"drone with SYSID {sysid} did not boot in the shared Renode: {message}")
        return {sysid: launcher.connection_string for sysid, launcher in self.launchers.items()}

    # ---- while running ----

    @property
    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    @property
    def renode_exit_code(self) -> int | None:
        """The shared Renode's exit code once it has died, else None."""
        return self._proc.poll() if self._proc is not None else None

    def dead_sidecars(self) -> dict[int, int]:
        """{sysid: exit code} of every drone whose physics sidecar has died
        (not counting drones stop_drone() already took out). A drone whose
        sidecar is gone keeps sending MAVLink from frozen sensor values, so
        this is the only place its failure shows."""
        with self._lock:
            stopped = set(self._stopped_drones)
        return {sysid: launcher.physics_exit_code for sysid, launcher in self.launchers.items()
                if sysid not in stopped and launcher.physics_exit_code is not None}

    def pids(self) -> dict:
        """{"renode": pid or None, "physics": {sysid: pid}}."""
        return {
            "renode": self._proc.pid if self._proc is not None else None,
            "physics": {sysid: launcher.physics_pid for sysid, launcher in self.launchers.items()
                        if launcher.physics_pid is not None},
        }

    def monitor(self, commands: list[str], reply_timeout_s: float | None = None) -> list[str]:
        """Run monitor commands on the shared Renode, one connection for all
        of them (so `mach set` applies to the commands after it). Returns
        what each printed, without the echoed command and the prompt.
        Raises OSError if the monitor can't be reached, and TimeoutError
        if a command's echo and the prompt after it don't come back in time.

        The monitor's greeting is not relied on: the first connection gets
        a banner, a prompt and a replay of the startup command's output,
        later connections get nothing until they send something. Each reply
        is found by the command's own echo instead."""
        timeout = self.MONITOR_REPLY_TIMEOUT_S if reply_timeout_s is None else reply_timeout_s
        replies = []
        with socket.create_connection(("127.0.0.1", self.monitor_port), timeout=timeout) as sock:
            sock.settimeout(0.1)
            self._drain_until_quiet(sock)
            for command in commands:
                sock.sendall(command.encode() + b"\n")
                replies.append(self._read_reply(sock, timeout, command))
        return replies

    @staticmethod
    def _drain_until_quiet(sock: socket.socket, quiet_s: float = 0.3, limit_s: float = 2.0) -> None:
        """Discard whatever the monitor sends on connect (possibly nothing)."""
        deadline = time.monotonic() + limit_s
        last = time.monotonic()
        while time.monotonic() < deadline and time.monotonic() - last < quiet_s:
            try:
                if sock.recv(65536):
                    last = time.monotonic()
                else:
                    return
            except socket.timeout:
                pass

    @staticmethod
    def _read_reply(sock: socket.socket, timeout_s: float, command: str) -> str:
        """What the monitor printed for `command`: the text between its echo
        of the command and the next prompt ("(monitor) " or "(droneN) ")."""
        deadline, data = time.monotonic() + timeout_s, b""
        while time.monotonic() < deadline:
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                chunk = None
            if chunk == b"":
                raise ConnectionError(f"the monitor closed the connection while answering {command!r}")
            if chunk:
                data += chunk
            text = _ANSI.sub("", data.decode(errors="replace")).replace("\r", "")
            echo = text.find(command)   # the first: an error reply quotes the command again
            if echo >= 0:
                after = text[echo + len(command):]
                prompt = _PROMPT.search(after)
                if prompt:
                    return after[:prompt.start()].strip()
        raise TimeoutError(f"no reply from the monitor within {timeout_s:.0f} s to {command!r}")

    def _halt_machine(self, sysid: int) -> None:
        """Halt one machine's CPU and disconnect its physics, and CONFIRM the
        halt by reading `cpu IsHalted` back. Raises DroneStopError if any
        reply is an error or the read-back isn't True; OSError/TimeoutError
        if the monitor can't be reached."""
        name = machine_name(sysid)
        commands = [f'mach set "{name}"', "cpu IsHalted true", "cpu IsHalted", "physics Disconnect"]
        replies = self.monitor(commands)
        for command, reply in zip(commands, replies):
            if _MONITOR_ERROR.search(reply):
                raise DroneStopError(f"the monitor refused {command!r}: {' '.join(reply.split())[:200]}")
        if replies[2].strip() != "True":
            raise DroneStopError(f"`cpu IsHalted` on {name} read back {replies[2].strip()!r}, not True")

    def stop_drone(self, sysid: int) -> None:
        """Take one drone out; the others keep running. Its CPU is halted
        and its physics disconnected through the monitor, the halt is read
        back, then its sidecar is stopped. The machine stays halted for the
        rest of the run: a machine un-halted without its physics crashes
        and stalls the others, and `machine Pause` stops every machine, so
        neither is ever used.

        The halt is tried twice. If it still can't be confirmed while the
        shared Renode is running, the sidecar is stopped anyway and
        DroneStopError is raised: that machine may still be executing on
        frozen physics, and the caller must not treat the drone as cleanly
        stopped (services.fleet_mission stops the whole fleet). If Renode
        itself is gone there is nothing to halt, and that is not an error."""
        launcher = self.launchers[sysid]
        with self._lock:
            if sysid in self._stopped_drones:
                return
            self._stopped_drones.add(sysid)
        failure = None
        try:
            for attempt in range(1, self.STOP_DRONE_ATTEMPTS + 1):
                if not self.is_running or self.monitor_port is None:
                    failure = None      # no Renode, nothing left to halt
                    break
                try:
                    self._halt_machine(sysid)
                    failure = None
                    break
                except (OSError, DroneStopError) as exc:   # TimeoutError and ConnectionError are OSErrors
                    failure = f"attempt {attempt}: {exc}"
                    time.sleep(0.5)
            if failure is not None and not self.is_running:
                failure = None          # Renode died while we were asking
        finally:
            launcher.stop_physics()
        if failure is not None:
            raise DroneStopError(
                f"could not confirm that {machine_name(sysid)} (SYSID {sysid}) is halted in the shared Renode "
                f"after {self.STOP_DRONE_ATTEMPTS} attempts ({failure}); its physics sidecar was stopped")

    def stop_all(self) -> None:
        """Kill the one Renode and every sidecar - safe from any thread, at
        any stage, and more than once."""
        with self._lock:
            proc, self._proc = self._proc, None
            log, self._log = self._log, None
        RenodeLauncher._terminate(proc)
        if log is not None:
            log.close()
        for launcher in self.launchers.values():
            launcher.stop()   # attached launchers leave Renode alone and stop their sidecar
