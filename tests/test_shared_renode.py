"""engine.shared_renode: the one-Renode fleet script, without running Renode.

Builds a minimal stand-in for the standalone folder (the real one is not in
the repository) with the same layout RenodeLauncher reads.
"""
import re
import tempfile
import unittest
from pathlib import Path

from engine.renode_launcher import RenodeLauncher, RenodeLauncherError
from engine.shared_renode import MACHINE_QUANTUM_S, MASTER_QUANTUM_S, SharedRenodeFleet, generate_fleet_script


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


if __name__ == "__main__":
    unittest.main()
