"""Turn an already-solved PDDL travel plan into a real MAVLink mission and fly it.

    plans/<name>/runs/<timestamp>/problem.pddl + plan.txt   (written by the
        GUI's Mission Planner / services.plan_service when you click
        "Plan Mission" - already retargeted to your picked source/
        destination, and already routed around any restricted area you drew)
        -> ordered [(lat, lon), ...] route        (engine.pddl_planner / pddl_problem)
        -> MISSION_ITEM_INT sequence               (NAV_TAKEOFF, NAV_WAYPOINT x N, NAV_LAND)
        -> uploaded + flown over real MAVLink

By default this loads the *most recent* plan run and flies exactly the route
ENHSP already worked out - no re-solving, no restricted-area recomputation,
so a detour it already planned around a no-fly zone is used precisely as
planned rather than redone.

Two ways to run the result, both using the exact same generated mission bytes:

  --validate (default)   Feed the mission into this repo's own
                          `SimulatedFlightController` - an ArduPilot-shaped
                          firmware model that decodes genuine MAVLink frames,
                          runs the real mission-upload handshake, arms, and
                          flies it. No network, no external SITL - proves the
                          mission is well-formed and flyable.

  --connect CONN_STRING   Upload and fly the same mission against a real
                          ArduPilot/PX4 (SITL or actual hardware) reachable
                          over MAVLink, e.g. --connect udp:127.0.0.1:14550
                          for ArduPilot SITL's default output.

Usage:
    # Fly whatever you last planned in the GUI (auto-picks the newest run):
    python scripts/pddl_to_mavlink.py --plan travell

    # Fly one specific run directory:
    python scripts/pddl_to_mavlink.py --run-dir plans/travell/runs/2026-08-28_105320

    # Against real SITL/hardware instead of the local firmware model:
    python scripts/pddl_to_mavlink.py --plan travell --connect udp:127.0.0.1:14550

    # (Still supported) solve a fresh source/destination instead of loading a run:
    python scripts/pddl_to_mavlink.py --plan travell \\
        --source 13.0827,80.2707 --dest 13.0674,80.2376
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from pymavlink.dialects.v20 import ardupilotmega as mav2  # noqa: E402

from engine.mavlink_interpreter import MODE_AUTO, MODE_GUIDED  # noqa: E402
from engine.mavlink_mission import build_mission_items, upload_and_fly  # noqa: E402
from engine.pddl_planner import (  # noqa: E402
    PlannerError,
    PlanNotFound,
    extract_route,
    parse_plan,
    run_enhsp,
)
from engine.pddl_problem import LatLon as PddlLatLon  # noqa: E402
from engine.pddl_problem import read_locations, retarget_problem  # noqa: E402

PLANS_DIR = REPO_ROOT / "plans"


# ---- Step 1a (default): load a run the GUI already solved -----------------


_CLI_SOLVE_DIR_NAME = "cli"  # where solve_plan() below writes fresh --source/--dest solves


def find_latest_run(plan_name: str) -> Path:
    """The most recently written `plans/<name>/runs/<timestamp>/` directory -
    i.e. whatever you last clicked "Plan Mission" for in the app. Excludes
    this script's own `runs/cli/` scratch dir (see `solve_plan`), which the
    GUI never writes to and would otherwise shadow a genuine run whenever
    `--source`/`--dest` was used more recently than the last GUI plan."""
    runs_dir = PLANS_DIR / plan_name / "runs"
    candidates = (
        [
            d for d in runs_dir.iterdir()
            if d.is_dir() and d.name != _CLI_SOLVE_DIR_NAME and (d / "plan.txt").is_file()
        ]
        if runs_dir.is_dir()
        else []
    )
    if not candidates:
        raise SystemExit(
            f"No plan runs found under {runs_dir}. Click 'Plan Mission' in the app first, "
            f"or pass --run-dir explicitly."
        )
    return max(candidates, key=lambda d: d.stat().st_mtime)


def load_solved_run(run_dir: Path, drone: str = "drone1") -> list[tuple[float, float]]:
    """Load an already-solved plan run straight from disk - no re-solving, so
    whatever restricted-area detour it already worked out is flown exactly as
    planned rather than recomputed."""
    problem_path = run_dir / "problem.pddl"
    plan_path = run_dir / "plan.txt"
    if not problem_path.is_file() or not plan_path.is_file():
        raise SystemExit(f"{run_dir} is missing problem.pddl/plan.txt - not a plan run directory")

    steps = parse_plan(plan_path.read_text(encoding="utf-8"))
    location_names = extract_route(steps, drone)
    if not location_names:
        raise SystemExit(
            f"{plan_path} has no '{drone}' travel legs - that run may have failed to find a plan."
        )

    locations = read_locations(problem_path.read_text(encoding="utf-8"))
    try:
        return [(locations[name].lat, locations[name].lon) for name in location_names]
    except KeyError as exc:
        raise SystemExit(f"Plan references location {exc} with no coordinates.") from exc


# ---- Step 1b (optional): solve a fresh source/destination -----------------


def solve_plan(
    plan_name: str,
    source: tuple[float, float],
    destination: tuple[float, float],
    no_fly_zone: list[tuple[float, float]] | None = None,
    drone: str = "drone1",
) -> list[tuple[float, float]]:
    """Run ENHSP and return the ordered [(lat, lon), ...] route it found."""
    plan_dir = PLANS_DIR / plan_name
    domain_path = plan_dir / "domain.pddl"
    problem_path = plan_dir / "problem.pddl"
    if not domain_path.is_file() or not problem_path.is_file():
        raise SystemExit(f"Plan '{plan_name}' is missing domain.pddl/problem.pddl in {plan_dir}")

    template_text = problem_path.read_text(encoding="utf-8")
    concrete_text = retarget_problem(
        template_text,
        PddlLatLon(lat=source[0], lon=source[1]),
        PddlLatLon(lat=destination[0], lon=destination[1]),
        no_fly_zone=(
            [PddlLatLon(lat=p[0], lon=p[1]) for p in no_fly_zone] if no_fly_zone else None
        ),
    )

    run_dir = plan_dir / "runs" / _CLI_SOLVE_DIR_NAME
    run_dir.mkdir(parents=True, exist_ok=True)
    problem_out = run_dir / "problem.pddl"
    problem_out.write_text(concrete_text, encoding="utf-8")

    try:
        steps, stdout = run_enhsp(domain_path, problem_out)
    except PlanNotFound as exc:
        (run_dir / "plan.txt").write_text(exc.stdout, encoding="utf-8")
        raise SystemExit(f"No plan found: {exc}") from exc
    except PlannerError as exc:
        raise SystemExit(f"Could not run ENHSP: {exc}") from exc
    (run_dir / "plan.txt").write_text(stdout, encoding="utf-8")

    location_names = extract_route(steps, drone)
    if not location_names:
        raise SystemExit(f"ENHSP found a plan but it has no '{drone}' travel legs.")

    locations = read_locations(concrete_text)
    try:
        return [(locations[name].lat, locations[name].lon) for name in location_names]
    except KeyError as exc:
        raise SystemExit(f"Plan references location {exc} with no coordinates.") from exc


# `build_mission_items` (route -> MISSION_ITEM_INT sequence) now lives in
# `engine.mavlink_mission`, shared with `services.mavlink_flight_service` -
# imported above.


# ---- Step 3a: validate locally against this repo's own ArduPilot model ----


def validate_locally(
    waypoints: list[tuple[float, float]],
    altitude_m: float,
    cruise_speed_mps: float = 15.0,
    tick_s: float = 0.2,
    timeout_s: float = 600.0,
    verbose: bool = True,
) -> bool:
    """Run the real MAVLink upload handshake against `SimulatedFlightController`
    and fly the mission to completion. Raises on any protocol/timeout failure."""
    from engine.flight_controller import SimulatedFlightController

    mav = mav2.MAVLink(None, srcSystem=255, srcComponent=190)
    mav.robust_parsing = True
    items = build_mission_items(mav, 1, 1, waypoints, altitude_m)

    fc = SimulatedFlightController(
        sysid=1, home_lat=waypoints[0][0], home_lon=waypoints[0][1],
        cruise_speed_mps=cruise_speed_mps,
    )

    def send(message) -> None:
        fc.send(message.pack(mav))

    send(mav.command_long_encode(
        1, 1, mav2.MAV_CMD_DO_SET_MODE, 0,
        mav2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODE_GUIDED, 0, 0, 0, 0, 0))
    send(mav.command_long_encode(
        1, 1, mav2.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0))
    send(mav.mission_count_encode(1, 1, len(items), mav2.MAV_MISSION_TYPE_MISSION))

    uploaded, elapsed = False, 0.0
    while not uploaded and elapsed < 10.0:
        for message in fc.receive():
            kind = message.get_type()
            if kind in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                send(items[message.seq])
            elif kind == "MISSION_ACK":
                uploaded = True
        fc.update(0.0)
        elapsed += 0.05
    if not uploaded:
        raise RuntimeError("firmware never acknowledged the mission upload")
    if verbose:
        print(f"Mission uploaded and accepted ({len(items)} items).")

    send(mav.command_long_encode(
        1, 1, mav2.MAV_CMD_DO_SET_MODE, 0,
        mav2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODE_AUTO, 0, 0, 0, 0, 0))

    t, next_report = 0.0, 0.0
    while t < timeout_s:
        fc.update(tick_s)
        t += tick_s
        st = fc.state()
        if verbose and t >= next_report:
            print(
                f"[{t:6.1f}s] wp={st.mission_seq}/{len(items) - 1} "
                f"({st.lat:.6f},{st.lon:.6f}) alt={st.relative_alt_m:5.1f}m "
                f"hdg={st.heading_deg:5.1f} gs={st.ground_speed_mps:4.1f}m/s "
                f"batt={st.battery_pct:5.1f}%"
            )
            next_report = t + 5.0
        if st.mission_complete and not st.armed:
            if verbose:
                print(f"[{t:6.1f}s] mission complete - landed={st.landed}")
            return True
    raise TimeoutError(f"mission did not complete within {timeout_s:.0f}s")


# ---- Step 3b: upload + fly against a real MAVLink endpoint -----------------


def fly_on_connection(
    connection_string: str,
    waypoints: list[tuple[float, float]],
    altitude_m: float,
    timeout_s: float = 600.0,
) -> None:
    """Connect and hand off to the shared `upload_and_fly` (engine.mavlink_mission),
    printing progress as it goes."""
    from pymavlink import mavutil

    master = mavutil.mavlink_connection(connection_string)
    print(f"Connecting to {connection_string} ...")
    upload_and_fly(
        master, waypoints, altitude_m,
        heartbeat_timeout_s=30.0, mission_timeout_s=timeout_s, on_progress=print,
    )


# ---- CLI --------------------------------------------------------------------


def _parse_latlon(text: str) -> tuple[float, float]:
    lat_str, lon_str = text.split(",")
    return float(lat_str), float(lon_str)


def _parse_polygon(text: str) -> list[tuple[float, float]]:
    return [_parse_latlon(p) for p in text.split(";") if p]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--plan", default="travell", help="plan folder under plans/ (default: travell)")
    parser.add_argument(
        "--run-dir", type=Path, default=None,
        help="load an already-solved run (plans/<name>/runs/<timestamp>/) instead of "
             "re-solving - default: the most recent run for --plan",
    )
    parser.add_argument(
        "--source", type=_parse_latlon, default=None,
        help="lat,lon - solves a fresh plan instead of loading a run (needs --dest too)",
    )
    parser.add_argument("--dest", type=_parse_latlon, default=None, help="lat,lon")
    parser.add_argument(
        "--no-fly", type=_parse_polygon, default=None,
        help="restricted-area polygon corners as lat,lon;lat,lon;lat,lon;lat,lon "
             "(only used together with --source/--dest)",
    )
    parser.add_argument("--altitude", type=float, default=50.0, help="cruise altitude, metres AGL")
    parser.add_argument("--speed", type=float, default=15.0, help="cruise speed for --validate, m/s")
    parser.add_argument(
        "--connect", default=None,
        help="fly against a real MAVLink endpoint instead of validating locally, "
             "e.g. udp:127.0.0.1:14550",
    )
    parser.add_argument("--dry-run", action="store_true", help="print the route and mission items, then exit")
    args = parser.parse_args()

    if args.source or args.dest:
        if not (args.source and args.dest):
            parser.error("--source and --dest must be given together")
        print(f"Solving plan '{args.plan}': {args.source} -> {args.dest} ...")
        waypoints = solve_plan(args.plan, args.source, args.dest, args.no_fly)
    else:
        run_dir = args.run_dir or find_latest_run(args.plan)
        print(f"Loading already-solved plan run: {run_dir}")
        waypoints = load_solved_run(run_dir)

    print(f"Route ({len(waypoints)} points):")
    for i, (lat, lon) in enumerate(waypoints):
        print(f"  {i}: {lat:.6f}, {lon:.6f}")

    if args.dry_run:
        mav = mav2.MAVLink(None, srcSystem=255, srcComponent=190)
        for item in build_mission_items(mav, 1, 1, waypoints, args.altitude):
            print(f"  seq={item.seq} cmd={item.command} lat={item.x} lon={item.y} alt={item.z}")
        return

    if args.connect:
        fly_on_connection(args.connect, waypoints, args.altitude)
    else:
        validate_locally(waypoints, args.altitude, cruise_speed_mps=args.speed)


if __name__ == "__main__":
    main()
