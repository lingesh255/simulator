"""Turn a two-drone forest-search plan into real MAVLink missions and fly them.

The point-to-point sibling of this tool is `scripts/pddl_to_mavlink.py`; this
one is the area-coverage version and goes through the exact same route ->
MAVLink conversion, just once per drone instead of once for a shared route:

    a marked search area (polygon corners) + a launch point
        -> engine.search_problem.retarget_search_problem
        -> one full-coverage lawnmower route per drone   {drone1: [...], drone2: [...]}
        -> engine.mavlink_mission.build_mission_items      (NAV_TAKEOFF, NAV_WAYPOINT x N, NAV_LAND)
           for each drone
        -> uploaded + flown, one mission per drone

The lawnmower routes come purely from the geometry layer (an equal-*area*
split of the marked shape into disjoint lanes - see `engine.search_problem`),
so the two drones never sweep the same ground. ENHSP is still run by default
as a solvability sanity check on the same coarsened per-lane skeleton the GUI
uses, but it does not decide the flown route; pass --skip-planner to bypass it.

Two ways to run the result, both using the exact same generated mission bytes:

  --validate (default)   Fly every drone's mission against its own in-process
                          `SimulatedFlightController` - the ArduPilot-shaped
                          firmware model that decodes genuine MAVLink frames,
                          runs the real mission-upload handshake, arms, and
                          flies. No network, no external SITL. All drones are
                          stepped on one clock so the per-drone stats lines
                          interleave the way a real swarm's telemetry would.

  --connect CONN[,CONN]   Upload and fly each drone's mission against a real
                          ArduPilot/PX4 (SITL or hardware) over MAVLink - one
                          connection string per drone, e.g.
                          --connect udp:127.0.0.1:14550,udp:127.0.0.1:14560

Usage:
    # Solve + fly a fresh search area (base point, then polygon corners):
    python scripts/pddl_search_to_mavlink.py \\
        --base 13.0827,80.2707 \\
        --area "13.0861,80.2723;13.0861,80.2793;13.0805,80.2793;13.0805,80.2723"

    # Same, split across 3 drones instead of 2:
    python scripts/pddl_search_to_mavlink.py --base 13.0827,80.2707 \\
        --area "..." --drones 3

    # Fly whatever you last planned in the GUI's area-coverage panel:
    python scripts/pddl_search_to_mavlink.py            # newest plans/Search run
    python scripts/pddl_search_to_mavlink.py --run-dir plans/Search/runs/2026-08-28_104341

    # Against real SITL/hardware (one endpoint per drone):
    python scripts/pddl_search_to_mavlink.py --base 13.0827,80.2707 --area "..." \\
        --connect udp:127.0.0.1:14550,udp:127.0.0.1:14560

Loading an already-solved run (--run-dir / the auto-newest default) replays the
*coarsened* skeleton route stored in that run's problem.pddl - the fine
lawnmower sweep is only ever held in memory by the GUI. Pass --base/--area to
regenerate and fly the full-resolution routes.
"""
from __future__ import annotations

import argparse
import re
import sys
import threading
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
    extract_multi_route,
    parse_plan,
    run_enhsp,
)
from engine.pddl_problem import LatLon, read_locations  # noqa: E402
from engine.search_problem import expand_search_drones, retarget_search_problem  # noqa: E402

PLANS_DIR = REPO_ROOT / "plans"
PLAN_NAME = "Search"
_CLI_SOLVE_DIR_NAME = "cli"  # where a fresh --base/--area solve writes its problem.pddl + plan.txt

# Same greedy config plan_service uses for this domain: the default WA* search
# does not return on 4+ separation-coupled lanes, greedy best-first + helpful
# actions solves the coarsened skeleton in seconds. Route optimality is
# irrelevant here - the drones fly the fixed geometry routes regardless.
_ENHSP_ARGS = ["-s", "gbfs", "-h", "hadd", "-ha", "true"]


def _plan_files() -> tuple[Path, Path]:
    """(domain.pddl, SearchProblem.pddl) for the forest-search plan."""
    plan_dir = PLANS_DIR / PLAN_NAME
    domain = plan_dir / "domain.pddl"
    template = plan_dir / "SearchProblem.pddl"
    if not domain.is_file() or not template.is_file():
        raise SystemExit(f"Plan '{PLAN_NAME}' is missing domain.pddl/SearchProblem.pddl in {plan_dir}")
    return domain, template


# ---- Step 1a (default): solve + generate fresh full-resolution routes ------


def solve_search(
    base: tuple[float, float],
    corners: list[tuple[float, float]],
    n_drones: int,
    coverage_width_m: float | None,
    run_planner: bool,
) -> dict[str, list[tuple[float, float]]]:
    """Marked area -> {"drone1": [(lat, lon), ...], "drone2": [...], ...}, one
    full-coverage lawnmower sweep per drone (equal-area disjoint lanes)."""
    domain_path, template_path = _plan_files()
    n_lanes = max(2, n_drones)

    template_text = expand_search_drones(template_path.read_text(encoding="utf-8"), n_lanes)
    concrete_text, fine_routes = retarget_search_problem(
        template_text,
        LatLon(lat=base[0], lon=base[1]),
        [LatLon(lat=p[0], lon=p[1]) for p in corners],
        coverage_width_m=coverage_width_m,
        n_lanes=n_lanes,
    )

    run_dir = PLANS_DIR / PLAN_NAME / "runs" / _CLI_SOLVE_DIR_NAME
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "problem.pddl").write_text(concrete_text, encoding="utf-8")

    if run_planner:
        try:
            steps, stdout = run_enhsp(
                domain_path,
                run_dir / "problem.pddl",
                timeout_s=max(45.0, 15.0 * n_lanes),
                extra_args=_ENHSP_ARGS,
            )
        except PlanNotFound as exc:
            (run_dir / "plan.txt").write_text(exc.stdout, encoding="utf-8")
            raise SystemExit(f"No plan found - the marked area is not coverable as posed: {exc}") from exc
        except PlannerError as exc:
            raise SystemExit(
                f"Could not run ENHSP: {exc}\n(pass --skip-planner to fly the geometry routes anyway)"
            ) from exc
        (run_dir / "plan.txt").write_text(stdout, encoding="utf-8")
        moved = extract_multi_route(steps, list(fine_routes))
        missing = [d for d in fine_routes if d not in moved]
        if missing:
            raise SystemExit("ENHSP found a plan but it never moves: " + ", ".join(missing))
        plan_len = sum(1 for s in steps if s.action in ("move-search", "return-to-base"))
        print(f"ENHSP: plan OK ({len(steps)} actions, {plan_len} flight legs across {n_lanes} lanes).")
    else:
        print("Planner skipped (--skip-planner) - flying the geometry routes directly.")

    return {d: [(p.lat, p.lon) for p in pts] for d, pts in fine_routes.items()}


# ---- Step 1b (optional): replay an already-solved run from disk ------------


def find_latest_run() -> Path:
    """The most recently written plans/Search/runs/<timestamp>/ directory,
    excluding this tool's own runs/cli/ scratch dir."""
    runs_dir = PLANS_DIR / PLAN_NAME / "runs"
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
            f"No solved runs under {runs_dir}. Plan a search area in the app first, "
            f"or pass --base/--area to solve one here."
        )
    return max(candidates, key=lambda d: d.stat().st_mtime)


def load_solved_run(run_dir: Path) -> dict[str, list[tuple[float, float]]]:
    """Replay the coarsened per-lane routes stored in a run's problem.pddl +
    plan.txt. Lower-resolution than a fresh --base/--area solve (the GUI never
    writes the full sweep to disk), but the same MAVLink conversion."""
    problem_path = run_dir / "problem.pddl"
    plan_path = run_dir / "plan.txt"
    if not problem_path.is_file() or not plan_path.is_file():
        raise SystemExit(f"{run_dir} is missing problem.pddl/plan.txt - not a solved run directory")

    problem_text = problem_path.read_text(encoding="utf-8")
    drone_names = sorted(
        set(re.findall(r"\(assigned\s+(drone\d+)\s", problem_text)),
        key=lambda n: int(n[5:]),
    )
    if not drone_names:
        raise SystemExit(f"{problem_path} declares no (assigned droneN ...) facts.")

    steps = parse_plan(plan_path.read_text(encoding="utf-8"))
    routes = extract_multi_route(steps, drone_names)
    missing = [d for d in drone_names if d not in routes]
    if missing:
        raise SystemExit(
            f"{plan_path} has no flight legs for {', '.join(missing)} - that run may have failed."
        )

    locations = read_locations(problem_text)
    try:
        return {
            drone: [(locations[name].lat, locations[name].lon) for name in names]
            for drone, names in routes.items()
        }
    except KeyError as exc:
        raise SystemExit(f"Plan references location {exc} with no coordinates.") from exc


# ---- Step 2: route -> MISSION_ITEM_INT (shared with the travel-plan tool) --
# `build_mission_items` is imported from engine.mavlink_mission unchanged - a
# search lane is just an ordinary [(lat, lon), ...] route as far as it cares.


# ---- Step 3a: validate locally against this repo's own ArduPilot model ----


def _send(fc, mav: "mav2.MAVLink", message) -> None:
    fc.send(message.pack(mav))


def validate_locally(
    routes: dict[str, list[tuple[float, float]]],
    altitude_m: float,
    cruise_speed_mps: float = 15.0,
    tick_s: float = 0.2,
    timeout_s: float = 900.0,
) -> bool:
    """Run the real MAVLink upload handshake for every drone against its own
    `SimulatedFlightController`, then fly them all on one shared clock so the
    stats lines interleave. Raises on any protocol/timeout failure."""
    from engine.flight_controller import SimulatedFlightController

    drones: list[dict] = []
    for name, waypoints in routes.items():
        mav = mav2.MAVLink(None, srcSystem=255, srcComponent=190)
        mav.robust_parsing = True
        items = build_mission_items(mav, 1, 1, waypoints, altitude_m)
        fc = SimulatedFlightController(
            sysid=1, home_lat=waypoints[0][0], home_lon=waypoints[0][1],
            cruise_speed_mps=cruise_speed_mps,
        )
        drones.append({"name": name, "mav": mav, "items": items, "fc": fc, "last": len(items) - 1})

    # --- upload + arm every drone (same handshake as the travel-plan tool) ---
    for d in drones:
        fc, mav, items = d["fc"], d["mav"], d["items"]
        _send(fc, mav, mav.command_long_encode(
            1, 1, mav2.MAV_CMD_DO_SET_MODE, 0,
            mav2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODE_GUIDED, 0, 0, 0, 0, 0))
        _send(fc, mav, mav.command_long_encode(
            1, 1, mav2.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0))
        _send(fc, mav, mav.mission_count_encode(1, 1, len(items), mav2.MAV_MISSION_TYPE_MISSION))

        uploaded, elapsed = False, 0.0
        while not uploaded and elapsed < 10.0:
            for message in fc.receive():
                kind = message.get_type()
                if kind in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                    _send(fc, mav, items[message.seq])
                elif kind == "MISSION_ACK":
                    uploaded = True
            fc.update(0.0)
            elapsed += 0.05
        if not uploaded:
            raise RuntimeError(f"{d['name']}: firmware never acknowledged the mission upload")
        print(f"{d['name']}: mission uploaded and accepted ({len(items)} items).")
        _send(fc, mav, mav.command_long_encode(
            1, 1, mav2.MAV_CMD_DO_SET_MODE, 0,
            mav2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODE_AUTO, 0, 0, 0, 0, 0))

    # --- fly them all on one clock; stats line per drone, same shape as the
    #     travel-plan tool's validate_locally ---
    print("Flying missions.")
    t, next_report = 0.0, 0.0
    done: set[str] = set()
    while t < timeout_s and len(done) < len(drones):
        for d in drones:
            d["fc"].update(tick_s)
        t += tick_s

        if t >= next_report:
            for d in drones:
                st = d["fc"].state()
                print(
                    f"[{t:6.1f}s] {d['name']} wp={st.mission_seq}/{d['last']} "
                    f"({st.lat:.6f},{st.lon:.6f}) alt={st.relative_alt_m:5.1f}m "
                    f"hdg={st.heading_deg:5.1f} gs={st.ground_speed_mps:4.1f}m/s "
                    f"batt={st.battery_pct:5.1f}%"
                )
            next_report = t + 5.0

        for d in drones:
            st = d["fc"].state()
            if d["name"] not in done and st.mission_complete and not st.armed:
                print(f"[{t:6.1f}s] {d['name']} mission complete - landed={st.landed}")
                done.add(d["name"])

    if len(done) < len(drones):
        raise TimeoutError(
            f"missions did not all complete within {timeout_s:.0f}s "
            f"(finished: {', '.join(sorted(done)) or 'none'})"
        )
    print(f"[{t:6.1f}s] all {len(drones)} drones complete.")
    return True


# ---- Step 3b: upload + fly each drone against a real MAVLink endpoint ------


def fly_on_connections(
    connection_strings: list[str],
    routes: dict[str, list[tuple[float, float]]],
    altitude_m: float,
    timeout_s: float = 900.0,
) -> None:
    """One `upload_and_fly` per drone, each on its own thread (it blocks on
    network I/O for the whole flight). A background printer emits the same
    per-drone stats line as the local path, off each vehicle's real
    GLOBAL_POSITION_INT / SYS_STATUS telemetry."""
    from pymavlink import mavutil

    names = list(routes)
    if len(connection_strings) != len(names):
        raise SystemExit(
            f"got {len(connection_strings)} connection string(s) for {len(names)} drones - "
            f"pass one per drone, comma-separated"
        )

    stats: dict[str, dict] = {n: {} for n in names}
    stats_lock = threading.Lock()
    errors: dict[str, BaseException] = {}
    stop = threading.Event()

    def make_on_message(name: str):
        def on_message(message) -> None:
            kind = message.get_type()
            with stats_lock:
                s = stats[name]
                if kind == "GLOBAL_POSITION_INT":
                    s["lat"] = message.lat / 1e7
                    s["lon"] = message.lon / 1e7
                    s["alt"] = message.relative_alt / 1000.0
                    s["hdg"] = (message.hdg / 100.0) % 360.0
                    s["gs"] = (message.vx ** 2 + message.vy ** 2) ** 0.5 / 100.0
                elif kind == "VFR_HUD":
                    s["gs"] = message.groundspeed
                    s["alt"] = message.alt
                    s["hdg"] = message.heading
                elif kind == "SYS_STATUS":
                    s["batt"] = message.battery_remaining
        return on_message

    def fly(name: str, conn: str) -> None:
        try:
            master = mavutil.mavlink_connection(conn)
            upload_and_fly(
                master, routes[name], altitude_m,
                heartbeat_timeout_s=30.0, mission_timeout_s=timeout_s,
                on_progress=lambda msg, n=name: print(f"{n}: {msg}"),
                on_message=make_on_message(name),
                should_abort=stop.is_set,
            )
        except BaseException as exc:  # noqa: BLE001 - surfaced to the main thread below
            errors[name] = exc
            print(f"{name}: FAILED - {exc}")

    def report() -> None:
        t0 = time.monotonic()
        while not stop.wait(5.0):
            t = time.monotonic() - t0
            with stats_lock:
                for name in names:
                    s = stats[name]
                    print(
                        f"[{t:6.1f}s] {name} "
                        f"({s.get('lat', 0.0):.6f},{s.get('lon', 0.0):.6f}) "
                        f"alt={s.get('alt', 0.0):5.1f}m hdg={s.get('hdg', 0.0):5.1f} "
                        f"gs={s.get('gs', 0.0):4.1f}m/s batt={s.get('batt', 0.0):5.1f}%"
                    )

    for name, conn in zip(names, connection_strings):
        print(f"{name}: connecting to {conn} ...")
    workers = [threading.Thread(target=fly, args=(n, c), daemon=True)
               for n, c in zip(names, connection_strings)]
    printer = threading.Thread(target=report, daemon=True)
    for w in workers:
        w.start()
    printer.start()
    try:
        for w in workers:
            w.join()
    finally:
        stop.set()
        printer.join(timeout=1.0)

    if errors:
        raise SystemExit(
            "one or more drones failed:\n"
            + "\n".join(f"  {n}: {e}" for n, e in errors.items())
        )
    print(f"All {len(names)} drones complete.")


# ---- CLI ------------------------------------------------------------------


def _parse_latlon(text: str) -> tuple[float, float]:
    lat_str, lon_str = text.split(",")
    return float(lat_str), float(lon_str)


def _parse_polygon(text: str) -> list[tuple[float, float]]:
    return [_parse_latlon(p) for p in text.split(";") if p.strip()]


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--base", type=_parse_latlon, default=None,
                        help="lat,lon launch point - solves fresh routes (needs --area too)")
    parser.add_argument("--area", type=_parse_polygon, default=None,
                        help="search-area polygon corners in click order, "
                             "lat,lon;lat,lon;lat,lon;... (>= 3)")
    parser.add_argument("--drones", type=int, default=2,
                        help="number of drones / lanes to split the area across (>= 2, default 2)")
    parser.add_argument("--coverage-width", type=float, default=None,
                        help="per-drone sensor swath in metres for sweep spacing "
                             "(default: whatever the PDDL template declares)")
    parser.add_argument("--run-dir", type=Path, default=None,
                        help="replay an already-solved run (plans/Search/runs/<timestamp>/) "
                             "instead of solving - default when no --base/--area given: the newest run")
    parser.add_argument("--altitude", type=float, default=50.0, help="sweep altitude, metres AGL")
    parser.add_argument("--speed", type=float, default=15.0, help="cruise speed for --validate, m/s")
    parser.add_argument("--connect", default=None,
                        help="fly against real MAVLink endpoints instead of validating locally - "
                             "one connection string per drone, comma-separated, "
                             "e.g. udp:127.0.0.1:14550,udp:127.0.0.1:14560")
    parser.add_argument("--skip-planner", action="store_true",
                        help="don't run ENHSP as a solvability check - fly the geometry routes directly")
    parser.add_argument("--dry-run", action="store_true",
                        help="print each drone's route and mission items, then exit")
    args = parser.parse_args()

    if args.base or args.area:
        if not (args.base and args.area):
            parser.error("--base and --area must be given together")
        if len(args.area) < 3:
            parser.error("--area needs at least 3 corners")
        if args.drones < 2:
            parser.error("--drones must be >= 2 (the search domain needs two lanes for mutual separation)")
        print(f"Solving '{PLAN_NAME}': base {args.base}, {len(args.area)}-corner area, {args.drones} drones ...")
        routes = solve_search(
            args.base, args.area, args.drones, args.coverage_width, run_planner=not args.skip_planner
        )
    else:
        run_dir = args.run_dir or find_latest_run()
        print(f"Replaying already-solved run: {run_dir}")
        routes = load_solved_run(run_dir)

    for name, waypoints in routes.items():
        print(f"{name} route ({len(waypoints)} points):")
        for i, (lat, lon) in enumerate(waypoints):
            print(f"  {i:3d}: {lat:.6f}, {lon:.6f}")

    if args.dry_run:
        mav = mav2.MAVLink(None, srcSystem=255, srcComponent=190)
        for name, waypoints in routes.items():
            print(f"{name} mission items:")
            for item in build_mission_items(mav, 1, 1, waypoints, args.altitude):
                print(f"  seq={item.seq} cmd={item.command} lat={item.x} lon={item.y} alt={item.z}")
        return

    if args.connect:
        fly_on_connections([c.strip() for c in args.connect.split(",")], routes, args.altitude)
    else:
        validate_locally(routes, args.altitude, cruise_speed_mps=args.speed)


if __name__ == "__main__":
    main()
