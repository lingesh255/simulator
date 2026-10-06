"""Step 4b: fly a 3-point route against scripts/mock_sitl.py with the real
upload_and_fly - it must skip the HOME placeholder, fly the route, and end
by itself on the LAND point ("Mission complete (landed and disarmed).").

    python tests/harness/mock_fly_check.py
"""
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
from pymavlink import mavutil  # noqa: E402

from engine.mavlink_mission import upload_and_fly  # noqa: E402

HOME = (-35.363261, 149.177300)
ROUTE = [HOME, (-35.362361, 149.177300), (-35.362361, 149.178400)]  # ~100 m north, then ~100 m east
mock = subprocess.Popen([sys.executable, str(REPO / "scripts" / "mock_sitl.py"), "--port", "14571",
                         "--home", f"{HOME[0]},{HOME[1]}"], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
t0 = time.monotonic()
last = {"pos": None}


def on_msg(m):
    if m.get_type() == "GLOBAL_POSITION_INT":
        last["pos"] = (m.lat / 1e7, m.lon / 1e7, m.relative_alt / 1000)


try:
    master = mavutil.mavlink_connection("udp:127.0.0.1:14571")
    upload_and_fly(master, ROUTE, 20.0, on_progress=lambda s: print(f"[{time.monotonic() - t0:5.1f}s] {s}"),
                   on_message=on_msg)
    print(f"RETURNED normally; final position {last['pos']} (LAND point {ROUTE[-1]})")
finally:
    mock.kill()
