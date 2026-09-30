"""One headless flight on a given instance: boot, provision, armable, then fly
home -> ~110 m north at 50 m. Reports EKF failsafes and where it landed.

    python flight_check.py <instance>
"""
import math
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
from pymavlink import mavutil  # noqa: E402

from engine.mavlink_mission import upload_and_fly  # noqa: E402
from engine.renode_launcher import RenodeLauncher  # noqa: E402

instance = int(sys.argv[1])
T0 = time.monotonic()


def log(msg):
    print(f"[{time.monotonic() - T0:6.1f}s] instance {instance}: {msg}", flush=True)


launcher = RenodeLauncher(str(REPO / "pixhawk6c_renode_standalone"), instance=instance)
state = {"pos": None, "failsafes": 0, "alt50": None}
try:
    launcher.start()
    launcher.provision_first_boot_params()
    launcher.wait_until_armable()
    log(f"armable after {time.monotonic() - T0:.0f}s on {launcher.connection_string}")
    home = (launcher.latitude_deg, launcher.longitude_deg)
    dest = (home[0] + 0.001, home[1])
    tf = time.monotonic()

    def on_msg(m):
        if m.get_type() == "GLOBAL_POSITION_INT":
            state["pos"] = (m.lat / 1e7, m.lon / 1e7)
            if m.relative_alt >= 47500 and state["alt50"] is None:
                state["alt50"] = time.monotonic() - tf

    def progress(text):
        if "EKF Failsafe: changed" in text:
            state["failsafes"] += 1
        if text.startswith(("Mission", "Reached", "[FC] EKF Failsafe", "[FC] GPS Glitch")):
            log(text)

    master = mavutil.mavlink_connection(launcher.connection_string)
    try:
        upload_and_fly(master, [home, dest], 50.0, on_message=on_msg, on_progress=progress)
        result = "completed"
    except Exception as exc:  # noqa: BLE001
        result = f"FAILED: {exc}"
    finally:
        master.close()
    dy = (state["pos"][0] - dest[0]) * 111320
    dx = (state["pos"][1] - dest[1]) * 111320 * math.cos(math.radians(dest[0]))
    log(f"RESULT {result}; 50 m at {state['alt50'] and round(state['alt50'], 1)}s; EKF failsafe landings: "
        f"{state['failsafes']}; landed {math.hypot(dx, dy):.1f} m from the destination")
finally:
    launcher.stop()
