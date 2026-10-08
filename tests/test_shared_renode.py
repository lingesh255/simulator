"""engine.shared_renode: the one-Renode fleet script, without running Renode.

Builds a minimal stand-in for the standalone folder (the real one is not in
the repository) with the same layout RenodeLauncher reads.
"""
import re
import tempfile
import unittest
from pathlib import Path

import socket
import subprocess
import sys
import threading

from engine.renode_launcher import RenodeLauncher, RenodeLauncherError
from engine.shared_renode import (
    MACHINE_QUANTUM_S, MASTER_QUANTUM_S, DroneStopError, SharedRenodeFleet, generate_fleet_script,
)


def make_standalone(root: Path) -> Path:
    s = root / "standalone"
    (s / "renode-bin").mkdir(parents=True)
    (s / "firmware").mkdir()
    (s / "board").mkdir()
    for name in ("renode-bin/renode", "renode-physics", "firmware/sdcard.img", "board/fram.img", "board/flash.img"):
        (s / name).write_bytes(b"")
    (s / "common.resc").write_text(
        "$repo?=@.\n"
        "include $repo/peripherals/A.cs\n"
        "include $repo/peripherals/B.cs\n"
        'mach create "ardupilot"\n'
        "machine LoadPlatformDescription $platform\n"
        'emulation SetGlobalQuantum "0.01"\n')
    (s / "board/Board.repl").write_text(f'fram: X @ spi 0\n    fileName: "{s}/board/fram.img"\n')
    (s / "board/persistent.repl").write_text(f'flash: Y @ none\n    fileName: "{s}/board/flash.img"\n')
    (s / "board/Board.resc").write_text(
        f"$repo?=@{s}\n"
        f"$platform=@{s}/board/Board.repl\n"
        "$app_base=0x08020000\n"
        "include $repo/peripherals/C.cs\n"
        f"include @{s}/common.resc\n"
        "sysbus.adc FeedSample 1 2 -1\n"
        'emulation CreateServerSocketTerminal 5762 "serial" false\n'
        "connector Connect sysbus.uart7Host serial\n"
        'emulation CreateCANHub "can1Hub"\n'
        "connector Connect sysbus.fdcan1 can1Hub\n"
        "connector Connect sysbus.can1Mcast can1Hub\n"
        'emulation CreateCANHub "can2Hub"\n'
        "connector Connect sysbus.fdcan2 can2Hub\n"
        "macro reset\n"
        '"""\n'
        "    cpu VectorTableOffset $vector_base\n"
        '"""\n'
        "runMacro $reset\n"
        "$sdcard?=@none\n"
        'machine SdCardFromFile $sdcard sysbus.sdmmc2 0x10000000 True "sdcard"\n'
        "logLevel 3\n")
    (s / "launch.resc").write_text(
        f"$repo=@{s}\n"
        f"$elf=@{s}/firmware/arducopter\n"
        "$vector_base=0x08020000\n"
        f"$renode_data=@{s}/data\n"
        f"$mcu_svd=@{s}/data/mcu.svd\n"
        f"$sdcard=@{s}/firmware/sdcard.img\n"
        f"include @{s}/board/Board.resc\n"
        f"sysbus LoadBinary @{s}/firmware/flash.img 0x08000000\n"
        f"machine LoadPlatformDescription @{s}/board/persistent.repl\n"
        "start\n")
    return s


class FleetScript(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        root = Path(self._tmp.name)
        self.standalone = make_standalone(root)
        self.work_root = root / "work"
        self.sysids = (1, 2, 4)
        self.launchers = [RenodeLauncher(str(self.standalone), instance=n, latitude_deg=10.0 + n, longitude_deg=20.0,
                                         work_root=str(self.work_root)) for n in self.sysids]
        self.script = generate_fleet_script(self.launchers)
        self.lines = self.script.splitlines()

    def tearDown(self):
        self._tmp.cleanup()

    def count(self, pattern):
        return sum(1 for line in self.lines if re.search(pattern, line))

    def test_one_machine_per_drone_with_its_own_time_source(self):
        for n in self.sysids:
            self.assertEqual(self.count(rf"AddMachine\(Machine\(True\), 'drone{n}'\)"), 1)
            self.assertEqual(self.count(rf'^mach set "drone{n}"$'), 1)
        self.assertEqual(self.count(r"AddMachine\("), len(self.sysids))
        self.assertEqual(self.count(r"mach create"), 0)
        self.assertNotIn("ardupilot", self.script)

    def test_peripheral_sources_are_included_once_before_any_machine(self):
        includes = [line for line in self.lines if line.startswith("include ")]
        self.assertEqual(includes, ["include $repo/peripherals/C.cs", "include $repo/peripherals/A.cs",
                                    "include $repo/peripherals/B.cs"])
        first_machine = next(i for i, line in enumerate(self.lines) if "AddMachine(" in line)
        self.assertLess(max(i for i, line in enumerate(self.lines) if line.startswith("include ")), first_machine)

    def test_ports_follow_the_sysid(self):
        for n in self.sysids:
            self.assertEqual(self.count(rf'^emulation CreateServerSocketTerminal {5762 + n} "serial_d{n}" false$'), 1)
            self.assertEqual(self.count(rf'^physics Connect {9002 + n} "quad" {10.0 + n:.6f} 20\.000000 '), 1)
        self.assertEqual(self.count(r"CreateServerSocketTerminal 5762 "), 0)

    def test_emulation_level_names_are_unique_per_drone(self):
        created = re.findall(r'^emulation Create\w+ (?:\d+ )?"(\w+)"', self.script, re.M)
        self.assertEqual(len(created), len(set(created)), created)
        self.assertEqual(len(created), 4 * len(self.sysids))   # serial, can1Hub, can2Hub, gpsHub
        for n in self.sysids:
            for name in (f"serial_d{n}", f"can1Hub_d{n}", f"can2Hub_d{n}", f"gpsHub_d{n}"):
                self.assertIn(name, created)
            self.assertEqual(self.count(rf"^connector Connect sysbus\.uart7Host serial_d{n}$"), 1)
            self.assertEqual(self.count(rf"^connector Connect sysbus\.can1Mcast can1Hub_d{n}$"), 1)
            self.assertEqual(self.count(rf"^connector Connect sysbus\.gps gpsHub_d{n}$"), 1)
        # the machine-local peripheral names are left alone
        self.assertEqual(self.count(r"sysbus\.fdcan1 can1Hub_d\d$"), len(self.sysids))

    def test_each_drone_uses_its_own_files(self):
        for n in self.sysids:
            work = self.work_root / f"instance-{n}"
            self.assertEqual(self.count(rf"^machine LoadPlatformDescription @{re.escape(str(work / 'Board.repl'))}$"), 1)
            self.assertEqual(self.count(rf"^machine LoadPlatformDescription @{re.escape(str(work / 'persistent.repl'))}$"), 1)
            self.assertEqual(self.count(rf"^machine SdCardFromFile @{re.escape(str(work / 'sdcard.img'))} "), 1)
        self.assertNotIn("$sdcard", self.script)

    def test_quanta_and_a_single_start_at_the_end(self):
        self.assertEqual(self.count(rf'^emulation SetGlobalQuantum "{MACHINE_QUANTUM_S}"$'), 1)
        self.assertEqual(self.count(rf'^emulation SetQuantum "{MASTER_QUANTUM_S}"$'), 1)
        self.assertEqual(self.count(r"SetGlobalSerialExecution"), 0)
        self.assertEqual(self.count(r"^start$"), 1)
        self.assertEqual(self.lines[-1], "start")
        self.assertEqual(self.count(r"^logLevel 3$"), 1)

    def test_mach_create_variant_for_experiments(self):
        script = generate_fleet_script(self.launchers, local_time=False, serial=True, quantum="0.0001")
        self.assertEqual(len(re.findall(r'^mach create "drone\d"$', script, re.M)), len(self.sysids))
        self.assertNotIn("AddMachine", script)
        self.assertNotIn("emulation SetQuantum", script)
        self.assertIn("emulation SetGlobalSerialExecution true", script)


class FleetObject(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        self.standalone = make_standalone(self.root)

    def tearDown(self):
        self._tmp.cleanup()

    def fleet(self, sysids):
        return SharedRenodeFleet(str(self.standalone), [(n, 1.0, 2.0) for n in sysids], work_root=str(self.root / "w"))

    def test_sysids_must_be_unique_and_positive(self):
        for bad in ([1, 1], [0, 1], []):
            with self.assertRaises(RenodeLauncherError):
                self.fleet(bad)

    def test_ports_and_timeout_scale(self):
        fleet = self.fleet([1, 3, 4, 7])
        self.assertEqual(fleet.connection_string(3), "tcp:127.0.0.1:5765")
        self.assertEqual(fleet.launchers[7].physics_port, 9009)
        self.assertEqual(fleet.timeout_scale, 1.75)
        self.assertEqual(self.fleet([2]).timeout_scale, 1.0)

    def test_nothing_running_before_start(self):
        fleet = self.fleet([1, 2])
        self.assertFalse(fleet.is_running)
        self.assertIsNone(fleet.renode_exit_code)
        self.assertEqual(fleet.dead_sidecars(), {})
        self.assertEqual(fleet.pids(), {"renode": None, "physics": {}})
        fleet.stop_all()   # harmless


class FakeMonitor:
    """A stand-in for Renode's telnet monitor: greets with a prompt, echoes
    each command, answers it and prompts again. `halted` is what
    `cpu IsHalted` reads back per machine; `behaviour` picks the fault."""

    def __init__(self, behaviour="good"):
        self.behaviour = behaviour
        self.halted = {}
        self.commands = []
        self.connections = 0
        self._server = socket.socket()
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(("127.0.0.1", 0))
        self._server.listen(4)
        self.port = self._server.getsockname()[1]
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def close(self):
        self._server.close()

    def _serve(self):
        while True:
            try:
                conn, _ = self._server.accept()
            except OSError:
                return
            self.connections += 1
            threading.Thread(target=self._session, args=(conn,), daemon=True).start()

    def _answer(self, machine, command):
        if command.startswith("mach set "):
            return command.split('"')[1], ""
        if self.behaviour == "error" and command == "cpu IsHalted true":
            return machine, "\x1b[;031mThere was an error executing command 'cpu IsHalted true'\x1b[0m\r\nboom"
        if self.behaviour == "error_once" and command == "cpu IsHalted true" and self.connections == 1:
            return machine, "There was an error executing command 'cpu IsHalted true'"
        if command == "cpu IsHalted true":
            if self.behaviour != "ignored":
                self.halted[machine] = True
            return machine, ""
        if command == "cpu IsHalted":
            return machine, "True" if self.halted.get(machine) else "False"
        return machine, ""

    def _session(self, conn):
        machine = "monitor"
        try:
            if self.behaviour == "silent":
                while conn.recv(1024):      # accept, read, never say anything
                    pass
                return
            if self.connections == 1:
                # like the real one: only the first connection is greeted, with a prompt and
                # then a replay of the startup command's output; later ones get nothing
                conn.sendall(b"Renode, version fake\r\n\x1b[31;1m(monitor) \x1b[0m")
                conn.sendall(b"include @/x/fleet.resc\r\nStarting emulation...\r\n\x1b[33;1m(drone2) \x1b[0m")
            buffer = b""
            while True:
                data = conn.recv(4096)
                if not data:
                    return
                buffer += data
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    command = line.decode().strip()
                    self.commands.append(command)
                    machine, reply = self._answer(machine, command)
                    text = command + "\r\n" + (reply + "\r\n" if reply else "") + f"\x1b[33;1m({machine}) \x1b[0m"
                    conn.sendall(text.encode())
        except OSError:
            pass
        finally:
            conn.close()


class StopDrone(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        root = Path(self._tmp.name)
        self.fleet = SharedRenodeFleet(str(make_standalone(root)), [(1, 1.0, 2.0), (2, 1.0, 2.0)],
                                       work_root=str(root / "w"))
        self.fleet.MONITOR_REPLY_TIMEOUT_S = 1.0
        # a live stand-in for the shared Renode, so is_running is True
        self.fleet._proc = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
        self.monitors = []

    def tearDown(self):
        self.fleet._proc.kill()
        self.fleet._proc.wait()
        for monitor in self.monitors:
            monitor.close()
        self._tmp.cleanup()

    def monitor(self, behaviour):
        fake = FakeMonitor(behaviour)
        self.monitors.append(fake)
        self.fleet.monitor_port = fake.port
        return fake

    def test_good_monitor_halts_and_confirms(self):
        fake = self.monitor("good")
        self.fleet.stop_drone(2)
        self.assertEqual(fake.commands, ['mach set "drone2"', "cpu IsHalted true", "cpu IsHalted", "physics Disconnect"])
        self.assertEqual(fake.halted, {"drone2": True})
        self.assertEqual(fake.connections, 1)
        self.fleet.stop_drone(2)          # already stopped: nothing more is sent
        self.assertEqual(fake.connections, 1)

    def test_monitor_replies_are_parsed(self):
        fake = self.monitor("good")
        fake.halted["drone1"] = True
        commands = ['mach set "drone1"', "cpu IsHalted", 'mach set "drone2"', "cpu IsHalted"]
        self.assertEqual(self.fleet.monitor(commands), ["", "True", "", "False"])   # first connection: greeted
        self.assertEqual(self.fleet.monitor(commands), ["", "True", "", "False"])   # second: no greeting

    def test_error_reply_is_reported_after_a_retry(self):
        fake = self.monitor("error")
        with self.assertRaises(DroneStopError) as caught:
            self.fleet.stop_drone(2)
        self.assertIn("drone2 (SYSID 2)", str(caught.exception))
        self.assertIn("monitor refused 'cpu IsHalted true'", str(caught.exception))
        self.assertEqual(fake.connections, 2)      # tried twice

    def test_halt_that_does_not_take_is_reported(self):
        self.monitor("ignored")
        with self.assertRaises(DroneStopError) as caught:
            self.fleet.stop_drone(1)
        self.assertIn("read back 'False', not True", str(caught.exception))

    def test_one_failed_attempt_then_success_is_not_an_error(self):
        fake = self.monitor("error_once")
        self.fleet.stop_drone(2)
        self.assertEqual(fake.connections, 2)
        self.assertEqual(fake.halted, {"drone2": True})

    def test_refused_connection_is_reported(self):
        with socket.socket() as probe:              # a port nobody listens on
            probe.bind(("127.0.0.1", 0))
            self.fleet.monitor_port = probe.getsockname()[1]
        with self.assertRaises(DroneStopError) as caught:
            self.fleet.stop_drone(2)
        self.assertIn("Connection refused", str(caught.exception))

    def test_monitor_that_never_answers_is_reported(self):
        self.monitor("silent")
        with self.assertRaises(DroneStopError) as caught:
            self.fleet.stop_drone(2)
        self.assertIn("no reply from the monitor within 1 s to 'mach set \"drone2\"'", str(caught.exception))

    def test_renode_already_gone_is_not_an_error(self):
        self.fleet._proc.kill()
        self.fleet._proc.wait()
        self.fleet.monitor_port = 1               # would be refused if it were tried
        self.fleet.stop_drone(2)                  # nothing to halt; just the sidecar


class MonitorReadiness(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        root = Path(self._tmp.name)
        self.fleet = SharedRenodeFleet(str(make_standalone(root)), [(1, 1.0, 2.0)], work_root=str(root / "w"))
        self.fleet.MONITOR_READY_TIMEOUT_S = 1.0
        self.fleet.work_dir.mkdir(parents=True)
        self.fleet.log_path.write_text("Renode starting\n")
        self.fleet._proc = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.fleet.monitor_port = self.listener.getsockname()[1]

    def tearDown(self):
        self.listener.close()
        self.fleet._proc.kill()
        self.fleet._proc.wait()
        self._tmp.cleanup()

    def announce(self):
        with open(self.fleet.log_path, "a") as log:
            log.write(f"[INFO] Monitor available in telnet mode on port {self.fleet.monitor_port}\n")

    def test_ready_when_announced_and_listening(self):
        self.listener.listen(1)
        self.announce()
        self.fleet._wait_for_monitor()

    def test_never_announced(self):
        self.listener.listen(1)            # something listens, but Renode never said it is its monitor
        with self.assertRaises(RenodeLauncherError) as caught:
            self.fleet._wait_for_monitor()
        self.assertIn(f"monitor didn't come up on port {self.fleet.monitor_port} within 1 s", str(caught.exception))
        self.assertIn("never announced", str(caught.exception))

    def test_announced_but_not_listening(self):
        self.announce()                    # bound but not listening: connections are refused
        with self.assertRaises(RenodeLauncherError) as caught:
            self.fleet._wait_for_monitor()
        self.assertIn("does not accept connections", str(caught.exception))

    def test_renode_exits_first(self):
        self.fleet._proc.kill()
        self.fleet._proc.wait()
        with self.assertRaises(RenodeLauncherError) as caught:
            self.fleet._wait_for_monitor()
        self.assertIn("exited (code -9) before its monitor came up", str(caught.exception))

    def test_cancel_is_honoured(self):
        def cancelled():
            raise RenodeLauncherError("cancelled")
        with self.assertRaises(RenodeLauncherError) as caught:
            self.fleet._wait_for_monitor(cancelled)
        self.assertEqual(str(caught.exception), "cancelled")


if __name__ == "__main__":
    unittest.main()
