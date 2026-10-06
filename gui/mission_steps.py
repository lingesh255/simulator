"""What the Flight Log table's Mission column shows for a drone's progress text.

`mission_step_label(text)` turns one line of a drone's progress - the fleet's
boot phases and final outcomes, `engine.mavlink_mission.upload_and_fly`'s
progress lines, and the flight controller's STATUSTEXT (with or without its
"[FC] " prefix) - into a short mission step, or None when the line isn't a
step (EKF notes, "Waiting for heartbeat", ...). Separately it rates the line
as trouble: "critical" or "warning", or None. Pure - no Qt - so it can be
unit-tested directly (tests/test_mission_steps.py).
"""
from __future__ import annotations

import re

# Passed through unchanged: the fleet's boot phases (services.fleet_mission).
BOOT_PHASES = ("Booting Renode", "GPS fix - waiting for armable", "Armable - waiting for the fleet")
# Final outcomes (fleet and single-drone paths), also passed through.
FINAL_PREFIXES = ("Completed", "Failed", "Stopped", "Not flown")

# STATUSTEXT that signals trouble, lowercase substrings -> severity. Checked in
# order; anything saying the problem has cleared is not trouble.
_TROUBLE = (
    ("failsafe", "critical"),
    ("crash", "critical"),
    ("error", "critical"),
    ("thrust loss", "critical"),
    ("prearm", "warning"),
    ("ekf variance", "warning"),
    ("glitch", "warning"),
    ("emergency", "warning"),
)

_ARM_RESULT = re.compile(r"\bresult\s*:\s*(-?\d+)")
_MISSION_ITEM = re.compile(r"^Mission:\s*(\d+)\s+(.+?)\s*$")
_REACHED = re.compile(r"^Reached waypoint (\d+/\d+)")
_MISSION_COMMANDS = {"Takeoff": "Takeoff", "Land": "Landing", "WP": "Waypoint"}


def trouble_level(text: str) -> str | None:
    """"critical", "warning", or None for one progress / STATUSTEXT line."""
    lower = text.lower()
    if "cleared" in lower:
        return None
    for needle, level in _TROUBLE:
        if needle in lower:
            return level
    return None


def mission_step_label(text: str, previous: str | None = None) -> tuple[str | None, str | None]:
    """(Mission-cell label, or None if this line isn't a mission step;
    trouble level: "critical" / "warning" / None).

    `previous` is the drone's current label. After landing, the firmware
    resets its mission and announces "Mission: 1 Takeoff" again on disarm -
    that is not a new takeoff, so it maps to None once the drone is
    "Landing"/"Landed"."""
    label, level = _label(text)
    if label == "Takeoff" and previous in ("Landing", "Landed"):
        return None, level
    return label, level


def _label(text: str) -> tuple[str | None, str | None]:
    line = text.strip()
    line = line.removeprefix("[FC]").strip()
    level = trouble_level(line)

    if line in BOOT_PHASES or line.startswith(FINAL_PREFIXES):
        return line, None
    if line.startswith("Connecting to"):
        return "Connecting", level
    if line.startswith("Heartbeat OK"):
        return "Connected", level
    if line.startswith("Mission uploaded and accepted"):
        return "Mission uploaded", level
    if line.startswith("Arm ack:"):
        match = _ARM_RESULT.search(line)
        if match is None:
            return "Arm not acknowledged", "warning"
        result = int(match.group(1))
        return ("Armed", level) if result == 0 else (f"Arm refused (result {result})", "warning")
    if line.startswith("Flying mission"):
        return "Starting mission", level
    if line.startswith("Mission complete"):
        return "Landed", level
    if line.startswith("Mission stopped"):
        return "Stopping", level
    match = _REACHED.match(line)
    if match:
        return f"Reached waypoint {match.group(1)}", level
    match = _MISSION_ITEM.match(line)
    if match:
        item, command = match.groups()
        if command in _MISSION_COMMANDS:
            return _MISSION_COMMANDS[command], level
        return f"{command} (item {item})", level
    return None, level
