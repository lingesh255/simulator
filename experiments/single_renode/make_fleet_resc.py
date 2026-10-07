"""Experiment front end for engine.shared_renode.generate_fleet_script: ONE
Renode script that runs N drones as N machines in one process.

    python experiments/single_renode/make_fleet_resc.py 2 > fleet.resc

The generator itself now lives in engine/shared_renode.py (the app uses it);
this file adds the diagnostics knobs the Task 16/17 experiments needed -
leaving lines out, patched platform copies, the failing `mach create` setup.
The per-drone files (SD copy, FRAM, retargeted .repl) are the ones
RenodeLauncher(instance=N).prepare() makes; run_fleet.py does that first.
"""
from __future__ import annotations

import os
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from engine import shared_renode  # noqa: E402
from engine.renode_launcher import RenodeLauncher  # noqa: E402

STANDALONE = REPO / "pixhawk6c_renode_standalone"

def machine_name(launcher: RenodeLauncher) -> str:
    return shared_renode.machine_name(launcher.instance)


def generate(launchers: list[RenodeLauncher], **options) -> str:
    """engine.shared_renode.generate_fleet_script, plus this experiment's
    SINGLE_RENODE_DROP knob."""
    out = shared_renode.generate_fleet_script(launchers, **options).splitlines()
    # diagnostics only: SINGLE_RENODE_DROP="text1|text2" leaves out every line containing one of them
    drop = [t for t in os.environ.get("SINGLE_RENODE_DROP", "").split("|") if t]
    out = [line for line in out if not any(t in line for t in drop)]
    return "\n".join(out) + "\n"


def patched_platform(launcher: RenodeLauncher, out_dir: Path, drop: list[str], replace: dict[str, str]) -> Path:
    """Diagnostics only (Task 17): a copy of this drone's platform .repl, and
    of the stm32h743_base.repl it pulls in, with the peripherals named in
    `drop` removed (their whole entry) and each `replace` text swapped. The
    originals are only read. Returns the copy to load instead."""
    def strip(text: str) -> str:
        out, skipping = [], False
        for line in text.splitlines():
            head = re.match(r"^(\w+):", line)
            if head:
                skipping = head.group(1) in drop
            if not skipping:
                out.append(line)
        text = "\n".join(out) + "\n"
        for old, new in replace.items():
            text = text.replace(old, new)
        return text

    out_dir.mkdir(parents=True, exist_ok=True)
    board = (launcher.work_dir / launcher.platform_repl.name).read_text()
    base_path = re.search(r'^using "([^"]+)"', board, re.M).group(1)
    base_copy = out_dir / f"base_d{launcher.instance}.repl"
    base_copy.write_text(strip(Path(base_path).read_text()))
    board_copy = out_dir / f"board_d{launcher.instance}.repl"
    board_copy.write_text(strip(board.replace(base_path, str(base_copy))))
    return board_copy


if __name__ == "__main__":
    # Diagnostics knobs (environment): SINGLE_RENODE_SHARED_TIME=1 creates the machines
    # with plain `mach create` (the failing Task 16 setup), SINGLE_RENODE_SERIAL=1 turns
    # serial execution on, SINGLE_RENODE_QUANTUM / SINGLE_RENODE_MASTER_QUANTUM set the
    # quanta, SINGLE_RENODE_INLINE_RESET=1
    # uses no reset macro, SINGLE_RENODE_REPL_DROP="a|b" loads platform copies without
    # those peripherals, SINGLE_RENODE_PATCHED_DIR is where those copies go.
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    fleet = [RenodeLauncher(str(STANDALONE), instance=n) for n in range(1, count + 1)]
    shared = bool(os.environ.get("SINGLE_RENODE_SHARED_TIME"))
    text = generate(fleet, serial=bool(os.environ.get("SINGLE_RENODE_SERIAL")),
                    quantum=os.environ.get("SINGLE_RENODE_QUANTUM", "0.01"),
                    local_time=not shared,
                    master_quantum=os.environ.get("SINGLE_RENODE_MASTER_QUANTUM", "0.1"))
    repl_drop = [t for t in os.environ.get("SINGLE_RENODE_REPL_DROP", "").split("|") if t]
    if repl_drop:
        out_dir = Path(os.environ.get("SINGLE_RENODE_PATCHED_DIR", str(REPO / "experiments/single_renode/out/patched")))
        for launcher in fleet:
            original = launcher.work_dir / launcher.platform_repl.name
            text = text.replace(f"@{original}\n", f"@{patched_platform(launcher, out_dir, repl_drop, {})}\n")
    if os.environ.get("SINGLE_RENODE_INLINE_RESET"):
        text = re.sub(r'macro reset\n"""\n(.*?)"""\nrunMacro \$reset\n', lambda m: m.group(1), text, flags=re.S)
    print(text, end="")
