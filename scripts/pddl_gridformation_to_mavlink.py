"""Turn a grid-formation plan into real MAVLink missions and fly them, leader first.

The grid sibling of `scripts/pddl_vformation_to_mavlink.py` (a V of three) and
`scripts/pddl_search_to_mavlink.py` (area-coverage): same route -> MAVLink
conversion, but for the N-drone rectangular grid the `plans/gridformation/`
domain plans (a perfect-square count forms a square, any other the
most-square rectangle that tiles it - see `engine.pddl_problem.grid_dimensions`):

    a picked source/destination
        -> engine.pddl_problem.retarget_grid_problem        (ENHSP plans just
                                                              the leader corridor)
        -> engine.pddl_problem.formation_member_route        (every other cell's
                                                              track, derived
                                                              geometrically from
                                                              its row/column)
        -> engine.mavlink_mission.build_mission_items        (NAV_TAKEOFF,
           for each drone, each at its own slot altitude)
        -> uploaded + flown, all drones off the *same* launch point, one
           drone at a time

The launch sequencing is exactly the V-formation script's, because the
executor behind it is shared: every drone takes off from the one source the
plan was solved for and climbs out to the cell it holds in the airborne grid
before the next one leaves the ground. The leader lifts off first, climbs,
moves `FORMATION_LEAD_FORWARD_M` ahead of the source and holds there; then,
`--launch-stagger` seconds after it settles, the drone nearest the leader
follows, then the next nearest, and so on out to the far corners of the grid
(`grid_drone_names` order - the same one the GUI's grid plan uses). Gating
each takeoff on the previous drone actually reaching its cell, rather than on
a fixed clock, is what keeps two drones off the shared launch point at once.
Once every cell is filled the whole grid is released toward the route
together, each member flying its own parallel track.

Differences from the V script: the drone count is a parameter (default
`MIN_GRID_DRONES`), the leader flies the grid's centre-ish column rather than an
apex, and the real-MAVLink path takes either one connection string per drone
or a single `udp:host:port` base that is expanded to consecutive ports
(14550, 14551, ...) exactly like the GUI's "Fly via real MAVLink" does.

Two ways to run the result, both using the exact same generated mission bytes:

  --validate (default)   Fly every drone against its own in-process
                          `SimulatedFlightController` - the ArduPilot-shaped
                          firmware model that decodes genuine MAVLink frames,
                          runs the real mission-upload handshake, arms, and
                          flies. No network, no external SITL.

  --connect CONN[,CONN,...] | BASE   Upload and fly each drone's mission against
                          a real ArduPilot/PX4 (SITL or hardware) over MAVLink -
                          one connection string per drone, leader first, every
                          vehicle parked on the same launch point; or one
                          `udp:host:port` BASE, expanded to consecutive ports,
                          e.g. --connect udp:127.0.0.1:14550 with 4 drones is
                          14550..14553. `scripts/mock_sitl.py` answers these.

Usage:
    # Solve + fly a fresh source/destination (4 drones = a 2x2 grid):
    python scripts/pddl_gridformation_to_mavlink.py \\
        --source 18.3901547,79.0495640 --dest 18.3876026,79.0816646

    # A 3x3 grid:
    python scripts/pddl_gridformation_to_mavlink.py --drones 9 \\
        --source 18.3901547,79.0495640 --dest 18.3876026,79.0816646

    # Fly whatever you last planned in the GUI's Grid-Formation panel:
    python scripts/pddl_gridformation_to_mavlink.py            # newest plans/gridformation run
    python scripts/pddl_gridformation_to_mavlink.py --run-dir plans/gridformation/runs/2026-08-29_185436

    # Against real SITL/hardware (base port, expanded to one per drone):
    python scripts/pddl_gridformation_to_mavlink.py --drones 4 \\
        --source 18.3901547,79.0495640 --dest 18.3876026,79.0816646 \\
        --connect udp:127.0.0.1:14550
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))  # sibling V-formation script

from pymavlink.dialects.v20 import ardupilotmega as mav2  # noqa: E402

from engine.pddl_planner import (  # noqa: E402
    PlannerError,
    PlanNotFound,
    extract_formation_corridor,
    parse_plan,
    run_enhsp,
)
from engine.pddl_problem import (  # noqa: E402
    GRID_SPACING_M,
    LatLon,
    advance_along_first_leg,
    expand_grid_drones,
    formation_member_route,
    grid_cell_names,
    grid_dimensions,
    read_locations,
    retarget_grid_problem,
)

# The launch sequencing, mission-item printing and both executors are the V
# script's, generalised to take a drone list - one implementation, so the two
# formations cannot drift apart.
from pddl_vformation_to_mavlink import (  # noqa: E402
    FORMATION_LAUNCH_STAGGER_S,
    fly_on_connections,
    format_mission_items,
    validate_locally,
)

PLANS_DIR = REPO_ROOT / "plans"
PLAN_NAME = "gridformation"
_CLI_SOLVE_DIR_NAME = "cli"  # where a fresh --source/--dest solve writes its problem.pddl + plan.txt

# Kept in sync with services.plan_service (MIN_GRID_DRONES,
# FORMATION_LEAD_ALTITUDE_M/FORMATION_WING_ALTITUDE_M, FORMATION_LEAD_FORWARD_M),
# which the GUI's grid plan uses; this script avoids importing that module
# because it pulls in Qt, which a command-line run should not need.
MIN_GRID_DRONES = 4  # smallest shape that reads as a grid rather than a line (a 2x2 square)
LEAD_ALTITUDE_M = 60.0
MEMBER_ALTITUDE_M = 55.0
FORMATION_LEAD_FORWARD_M = 10.0
# `-gro naive`: ENHSP's default grounder runs out of memory on formation
# domains past a dozen-odd drones (see services.plan_service._run_grid_formation).
_ENHSP_EXTRA_ARGS = ["-gro", "naive"]

LEADER = "drone-lead"


def grid_drone_names(drone_count: int) -> list[str]:
    """Every drone name for a `drone_count`-drone grid, leader first then every
    other cell nearest-to-farthest from it (Manhattan distance in grid steps) -
    the launch order, and the same one `services.plan_service._grid_drone_names`
    gives the GUI. Matches the naming `expand_grid_drones` builds into the
    problem file."""
    rows, cols = grid_dimensions(drone_count)
    cell_name = grid_cell_names(rows, cols)
    leader_cell = next(cell for cell, name in cell_name.items() if name == LEADER)

    def manhattan(cell: tuple[int, int]) -> int:
        return abs(cell[0] - leader_cell[0]) + abs(cell[1] - leader_cell[1])

    return [cell_name[cell] for cell in sorted(cell_name, key=manhattan)]


def grid_routes_from_lead(
    lead_route: list[LatLon], drone_count: int
) -> dict[str, list[tuple[float, float]]]:
    """Every drone's track, launch order: the leader's corridor as planned, and
    each other cell's parallel to it at that cell's own row/column offset."""
    rows, cols = grid_dimensions(drone_count)
    cell_name = grid_cell_names(rows, cols)
    leader_cell = next(cell for cell, name in cell_name.items() if name == LEADER)
    routes: dict[str, list[tuple[float, float]]] = {LEADER: [(p.lat, p.lon) for p in lead_route]}
    for (row, col), name in cell_name.items():
        if (row, col) == leader_cell:
            continue
        # `along_m` is "how far behind" (positive = behind, as for a V's wings).
        member = formation_member_route(
            lead_route,
            along_m=row * GRID_SPACING_M,
            cross_m=(col - leader_cell[1]) * GRID_SPACING_M,
        )
        routes[name] = [(p.lat, p.lon) for p in member]
    return {name: routes[name] for name in grid_drone_names(drone_count)}


def grid_slots(
    routes: dict[str, list[tuple[float, float]]], lead_forward_m: float = FORMATION_LEAD_FORWARD_M
) -> dict[str, tuple[float, float]]:
    """Where each drone climbs out to and holds: its own route's first point
    (its cell, at the source) moved `lead_forward_m` along the leader's first
    leg - the leader moves forward of the pad, every member holds its cell
    offset from that spot. Same as the GUI's real-MAVLink grid launch."""
    lead_route = [LatLon(lat=lat, lon=lon) for lat, lon in routes[LEADER]]
    return {
        name: advance_along_first_leg(LatLon(lat=route[0][0], lon=route[0][1]), lead_route, lead_forward_m)
        for name, route in routes.items()
    }


def _plan_files() -> tuple[Path, Path]:
    plan_dir = PLANS_DIR / PLAN_NAME
    domain = plan_dir / "domain.pddl"
    template = plan_dir / "problem.pddl"
    if not domain.is_file() or not template.is_file():
        raise SystemExit(f"Plan '{PLAN_NAME}' is missing domain.pddl/problem.pddl in {plan_dir}")
    return domain, template


# ---- Step 1a (default): load a run the GUI already solved -----------------


def find_latest_run() -> Path:
    """The most recently written `plans/gridformation/runs/<timestamp>/` dir -
    i.e. whatever source/destination you last clicked "Plan Mission" for on
    the Grid-Formation plan in the app. Excludes this script's own
    `runs/cli/` scratch dir."""
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
            f"No plan runs found under {runs_dir}. Plan a grid-formation mission in the app first, "
            f"or pass --run-dir/--source+--dest explicitly."
        )
    return max(candidates, key=lambda d: d.stat().st_mtime)


def count_drones(problem_text: str) -> int:
    """How many drone objects a solved run's `problem.pddl` declares."""
    objects = re.search(r"\(:objects(.*?)\n\s*\)", problem_text, re.S)
    if objects is None:
        raise SystemExit("problem.pddl has no (:objects ...) section")
    # A typed list can name several objects before its "- drone" (e.g.
    # "drone-member drone-r1c0 drone-r1c1 - drone"), so count every name.
    count = 0
    for line in objects.group(1).splitlines():
        names, sep, kind = line.partition(" - ")
        if sep and kind.split(";")[0].strip() == "drone":
            count += len(names.split())
    return count


def load_solved_run(run_dir: Path) -> dict[str, list[tuple[float, float]]]:
    """Replay an already-solved run straight from disk - no re-solving, so the
    leader corridor ENHSP already found is flown exactly as planned, and every
    other drone's track is re-derived from it geometrically. The drone count
    comes from the run's own problem file."""
    problem_path = run_dir / "problem.pddl"
    plan_path = run_dir / "plan.txt"
    if not problem_path.is_file() or not plan_path.is_file():
        raise SystemExit(f"{run_dir} is missing problem.pddl/plan.txt - not a plan run directory")

    steps = parse_plan(plan_path.read_text(encoding="utf-8"))
    corridor = extract_formation_corridor(steps)
    if not corridor:
        raise SystemExit(
            f"{plan_path} has no 'formation-cruise' legs - that run may have failed to find a plan."
        )

    problem_text = problem_path.read_text(encoding="utf-8")
    locations = read_locations(problem_text)
    try:
        lead_route = [locations[name] for name in corridor]
    except KeyError as exc:
        raise SystemExit(f"Plan references location {exc} with no coordinates.") from exc
    drone_count = count_drones(problem_text)
    print(f"Run has {drone_count} drone(s) -> {' x '.join(map(str, grid_dimensions(drone_count)))} grid.")
    return grid_routes_from_lead(lead_route, drone_count)


# ---- Step 1b (optional): solve a fresh source/destination -----------------


def solve_plan(
    source: tuple[float, float], destination: tuple[float, float], drone_count: int
) -> dict[str, list[tuple[float, float]]]:
    """Run ENHSP on the leader's corridor and derive every other track from it."""
    drone_count = max(MIN_GRID_DRONES, drone_count)
    rows, cols = grid_dimensions(drone_count)
    domain_path, template_path = _plan_files()
    template_text = expand_grid_drones(template_path.read_text(encoding="utf-8"), drone_count)
    concrete_text = retarget_grid_problem(
        template_text,
        LatLon(lat=source[0], lon=source[1]),
        LatLon(lat=destination[0], lon=destination[1]),
    )

    run_dir = PLANS_DIR / PLAN_NAME / "runs" / _CLI_SOLVE_DIR_NAME
    run_dir.mkdir(parents=True, exist_ok=True)
    problem_out = run_dir / "problem.pddl"
    problem_out.write_text(concrete_text, encoding="utf-8")

    try:
        steps, stdout = run_enhsp(
            domain_path, problem_out,
            timeout_s=max(30.0, 1.0 * rows * cols), extra_args=_ENHSP_EXTRA_ARGS,
        )
    except PlanNotFound as exc:
        (run_dir / "plan.txt").write_text(exc.stdout, encoding="utf-8")
        raise SystemExit(f"No plan found: {exc}") from exc
    except PlannerError as exc:
        raise SystemExit(f"Could not run ENHSP: {exc}") from exc
    (run_dir / "plan.txt").write_text(stdout, encoding="utf-8")

    corridor = extract_formation_corridor(steps)
    if not corridor:
        raise SystemExit("ENHSP found a plan but it contains no 'formation-cruise' legs.")

    locations = read_locations(concrete_text)
    try:
        lead_route = [locations[name] for name in corridor]
    except KeyError as exc:
        raise SystemExit(f"Plan references location {exc} with no coordinates.") from exc
    return grid_routes_from_lead(lead_route, drone_count)


# ---- CLI --------------------------------------------------------------------


def _parse_latlon(text: str) -> tuple[float, float]:
    lat_str, lon_str = text.split(",")
    return float(lat_str), float(lon_str)


def expand_connections(connect: str, count: int) -> list[str]:
    """`--connect` -> one connection string per drone, leader first. A single
    `udp:host:port` is a base port expanded to consecutive ports (what the
    GUI does and `mock_sitl.py` is launched on); a comma list is used as given."""
    parts = [c.strip() for c in connect.split(",") if c.strip()]
    if len(parts) == 1 and count > 1:
        match = re.match(r"^udp:([^:]+):(\d+)$", parts[0])
        if match is None:
            raise SystemExit(
                f"a single --connect value must be udp:host:port (a base port, expanded to "
                f"{count} consecutive ports), got '{parts[0]}' - or pass {count} comma-separated strings"
            )
        host, base = match.group(1), int(match.group(2))
        return [f"udp:{host}:{base + i}" for i in range(count)]
    return parts


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--run-dir", type=Path, default=None,
        help="load an already-solved run (plans/gridformation/runs/<timestamp>/) instead of "
             "re-solving - default: the most recent run (its own drone count is used)",
    )
    parser.add_argument(
        "--source", type=_parse_latlon, default=None,
        help="lat,lon - solves a fresh plan instead of loading a run (needs --dest too)",
    )
    parser.add_argument("--dest", type=_parse_latlon, default=None, help="lat,lon")
    parser.add_argument(
        "--drones", type=int, default=MIN_GRID_DRONES,
        help=f"drone count for a fresh --source/--dest solve; a perfect square forms a square grid "
             f"(4, 9, 16), anything else the closest rectangle (default and minimum: {MIN_GRID_DRONES})",
    )
    parser.add_argument(
        "--lead-altitude", type=float, default=LEAD_ALTITUDE_M,
        help=f"leader cruise altitude, metres AGL (default: {LEAD_ALTITUDE_M:g})",
    )
    parser.add_argument(
        "--member-altitude", type=float, default=MEMBER_ALTITUDE_M,
        help=f"every other drone's cruise altitude, metres AGL (default: {MEMBER_ALTITUDE_M:g})",
    )
    parser.add_argument(
        "--launch-stagger", type=float, default=FORMATION_LAUNCH_STAGGER_S,
        help=f"seconds to wait after one drone reaches its cell before the next takes off "
             f"from the shared launch point, leader first (default: {FORMATION_LAUNCH_STAGGER_S:g})",
    )
    parser.add_argument("--speed", type=float, default=15.0, help="cruise speed for --validate, m/s")
    parser.add_argument(
        "--connect", default=None,
        help="fly against real MAVLink endpoints instead of validating locally - either one "
             "udp:host:port base (expanded to one consecutive port per drone, leader first) or "
             "one connection string per drone, comma-separated, every vehicle parked on the "
             "same launch point, e.g. udp:127.0.0.1:14550",
    )
    parser.add_argument("--dry-run", action="store_true", help="print each drone's route and mission items, then exit")
    args = parser.parse_args()

    if args.source or args.dest:
        if not (args.source and args.dest):
            parser.error("--source and --dest must be given together")
        if args.drones < MIN_GRID_DRONES:
            parser.error(f"--drones must be at least {MIN_GRID_DRONES} to form a grid")
        print(f"Solving '{PLAN_NAME}' for {args.drones} drone(s): {args.source} -> {args.dest} ...")
        routes = solve_plan(args.source, args.dest, args.drones)
    else:
        run_dir = args.run_dir or find_latest_run()
        print(f"Loading already-solved plan run: {run_dir}")
        routes = load_solved_run(run_dir)

    names = list(routes)
    rows, cols = grid_dimensions(len(names))
    altitudes = {name: (args.lead_altitude if name == LEADER else args.member_altitude) for name in names}
    slots = grid_slots(routes)
    pad = routes[LEADER][0]

    print(f"{rows} x {cols} grid, {len(names)} drones, {GRID_SPACING_M:g} m apart.")
    print(f"Shared launch point (all {len(names)} drones take off here): {pad[0]:.6f}, {pad[1]:.6f}")
    for name in names:
        slot = slots[name]
        print(f"  {name} climbs out to its cell: {slot[0]:.6f}, {slot[1]:.6f} at {altitudes[name]:g}m")

    for name in names:
        waypoints = routes[name]
        print(f"{name} route ({len(waypoints)} points, alt={altitudes[name]:g}m):")
        for i, (lat, lon) in enumerate(waypoints):
            print(f"  {i:3d}: {lat:.6f}, {lon:.6f}")

    # The exact MAVLink mission each drone will fly - named commands, not raw
    # ints - printed up front for every drone regardless of --dry-run, leader
    # first (see format_mission_items).
    for name in names:
        for line in format_mission_items(
            f"{name} climb-out", [pad, slots[name]], altitudes[name],
            final_command=mav2.MAV_CMD_NAV_LOITER_UNLIM,
        ):
            print(line)
        for line in format_mission_items(name, routes[name], altitudes[name]):
            print(line)

    if args.dry_run:
        return

    if args.connect:
        fly_on_connections(
            expand_connections(args.connect, len(names)), routes, altitudes,
            launch_stagger_s=args.launch_stagger, drone_names=names, slots=slots,
        )
    else:
        validate_locally(
            routes, altitudes, launch_stagger_s=args.launch_stagger, cruise_speed_mps=args.speed,
            drone_names=names, slots=slots, label="grid-formation",
        )


if __name__ == "__main__":
    main()
