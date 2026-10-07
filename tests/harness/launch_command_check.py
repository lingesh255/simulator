"""Print the Renode launch command RenodeLauncher builds for instance 0, 1 and 3
(no process is started). Run it on two checkouts and diff the output to show a
launcher change leaves the per-process launch untouched:

    python tests/harness/launch_command_check.py > after.txt
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from engine.renode_launcher import RenodeLauncher  # noqa: E402

for instance in (0, 1, 3):
    launcher = RenodeLauncher(str(REPO / "pixhawk6c_renode_standalone"), instance=instance,
                              latitude_deg=12.5, longitude_deg=80.25)
    print(f"instance {instance}: port {launcher.port} physics {launcher.physics_port} sysid {launcher.sysid} "
          f"log {launcher.renode_log_path}")
    print(launcher._build_launch_command())
