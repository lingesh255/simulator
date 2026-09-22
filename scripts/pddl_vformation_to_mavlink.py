"""Turn a V-formation plan into real MAVLink missions and fly them, apex first.

The formation sibling of `scripts/pddl_to_mavlink.py` (point-to-point) and
`scripts/pddl_search_to_mavlink.py` (area-coverage): same route -> MAVLink
conversion, but for the fixed three-drone V the `plans/vformation/` domain
plans:

    a picked source/destination
        -> engine.pddl_problem.retarget_formation_problem   (ENHSP plans just
                                                               the apex corridor)
        -> engine.pddl_problem.formation_wing_routes         (both wing tracks,
                                                               derived geometrically)
        -> engine.mavlink_mission.build_mission_items        (NAV_TAKEOFF,
           for each of the 3 drones, each at its own slot altitude)
        -> uploaded + flown, all three off the *same* launch point, one
           drone at a time

All three drones take off from the one source point the plan was solved
for - not from their own offset slots - and each climbs out to the slot it
holds in the airborne V before the next one leaves the ground. The apex
lifts off first, flies to its slot (which is the launch point itself) and
settles at its cruise altitude; only then, plus `--launch-stagger` seconds
of settling time, does the left wing take off from that same point and fly
out to its own slot; then the right wing the same way. Gating each takeoff
on the previous drone actually reaching its slot - rather than on a fixed
clock - is what keeps two drones off the shared launch point at once. Once
every slot is filled the whole V is released toward the route together.

This mirrors `services.thread_backend.ThreadSwarmBackend.start_formation_mission`
(the GUI's in-app formation executor): a fixed per-drone cruise altitude
(apex higher than the wings) instead of one shared terrain-derived altitude,
and a shared launch point with one-at-a-time climb-out instead of arming all
three at once.

Two ways to run the result, both using the exact same generated mission bytes:

  --validate (default)   Fly all three drones against their own in-process
                          `SimulatedFlightController` - the ArduPilot-shaped
                          firmware model that decodes genuine MAVLink frames,
                          runs the real mission-upload handshake, arms, and
                          flies. No network, no external SITL. Every drone is
                          stepped on one shared clock once it has launched, so
                          the per-drone stats lines interleave the way a real
                          formation's telemetry would - and a wing still on
                          the launch point is reported as waiting there.

  --connect CONN[,CONN,CONN]   Upload and fly each drone's mission against a
                          real ArduPilot/PX4 (SITL or hardware) over MAVLink -
                          one connection string per drone, apex first, and
                          every vehicle parked on the same launch point, e.g.
                          --connect udp:127.0.0.1:14550,udp:127.0.0.1:14560,udp:127.0.0.1:14570

Usage:
    # Solve + fly a fresh source/destination:
    python scripts/pddl_vformation_to_mavlink.py \\
        --source 18.3901547,79.0495640 --dest 18.3876026,79.0816646

    # Fly whatever you last planned in the GUI's V-formation panel:
    python scripts/pddl_vformation_to_mavlink.py            # newest plans/vformation run
    python scripts/pddl_vformation_to_mavlink.py --run-dir plans/vformation/runs/2026-08-29_185436

    # Against real SITL/hardware (one endpoint per drone, apex first):
    python scripts/pddl_vformation_to_mavlink.py \\
        --source 18.3901547,79.0495640 --dest 18.3876026,79.0816646 \\
        --connect udp:127.0.0.1:14550,udp:127.0.0.1:14560,udp:127.0.0.1:14570
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from pymavlink.dialects.v20 import ardupilotmega as mav2  # noqa: E402

from engine.mavlink_interpreter import MODE_AUTO, MODE_GUIDED  # noqa: E402
from engine.mavlink_mission import FlightAborted, build_mission_items, upload_and_fly  # noqa: E402
from engine.pddl_planner import (  # noqa: E402
    PlannerError,
    PlanNotFound,
    extract_formation_corridor,
    parse_plan,
    run_enhsp,
)
from engine.pddl_problem import LatLon  # noqa: E402
from engine.pddl_problem import (  # noqa: E402
    formation_wing_routes,
    read_locations,
    retarget_formation_problem,
)

PLANS_DIR = REPO_ROOT / "plans"
PLAN_NAME = "vformation"
_CLI_SOLVE_DIR_NAME = "cli"  # where a fresh --source/--dest solve writes its problem.pddl + plan.txt

# Apex first - also the launch order (see `validate_locally`/`fly_on_connections`).
# Kept in sync with services.plan_service.FORMATION_DRONES/FORMATION_ALTITUDES/
# FORMATION_LAUNCH_STAGGER_S, which the GUI's in-app formation mission uses.
FORMATION_DRONES = ("drone-lead", "drone-left", "drone-right")
FORMATION_BACK_M = 17.320508
FORMATION_SIDE_M = 10.0
FORMATION_ALTITUDES = {"drone-lead": 60.0, "drone-left": 55.0, "drone-right": 55.0}
FORMATION_LAUNCH_STAGGER_S = 6.0
# How close (metres) a drone's altitude must be to its target to count as
# "holding" there - matches services.thread_backend.ALTITUDE_ARRIVAL_M.
ALTITUDE_ARRIVAL_M = 0.5


def formation_launch_point(routes: dict[str, list[tuple[float, float]]]) -> tuple[float, float]:
    """The single point on the ground every drone in the formation takes off
    from: the apex route's source, which is the source the plan was solved
    for. The wings' routes start at their own slots instead (see
    `engine.pddl_problem.formation_wing_routes`, which offsets every wing
    waypoint including the first), and those slots are where each wing flies
    to *after* lifting off this point - they are not where it starts."""
    return routes[FORMATION_DRONES[0]][0]


# ---- MAVLink enum -> readable name, for printing the exact command instead
# of a bare integer (e.g. "MAV_CMD_NAV_TAKEOFF" rather than "22"). ----------


def _enum_name(enum_type: str, value: int) -> str:
    try:
        return mav2.enums[enum_type][value].name
    except KeyError:
        return str(value)


def _cmd_name(command_id: int) -> str:
    return _enum_name("MAV_CMD", command_id)


def _mission_result_name(result: int) -> str:
    return _enum_name("MAV_MISSION_RESULT", result)


def format_mission_items(
    name: str,
    waypoints: list[tuple[float, float]],
    altitude_m: float,
    final_command: int | None = None,
) -> list[str]:
    """The exact MAVLink `MISSION_ITEM_INT` sequence one drone's route turns
    into - same conversion `engine.mavlink_mission.build_mission_items` does
    for upload, but named (MAV_CMD_NAV_TAKEOFF, not 22) and printed up front
    so the whole formation's mission is visible before anything flies.
    `final_command` is passed straight through, so the same call also prints
    the climb-out-to-slot mission (which ends on MAV_CMD_NAV_LOITER_UNLIM
    rather than landing)."""
    mav = mav2.MAVLink(None, srcSystem=255, srcComponent=190)
    lines = [f"{name} mission items ({len(waypoints)} waypoints, alt={altitude_m:g}m):"]
    for item in build_mission_items(mav, 1, 1, waypoints, altitude_m, final_command=final_command):
        # x/y are lat*1e7 / lon*1e7 for a real waypoint (MISSION_ITEM_INT's
        # integer-degree encoding); the takeoff item carries 0/0 (unused for
        # NAV_TAKEOFF - it climbs from wherever the vehicle already is), and
        # dividing that by 1e7 is still just 0.0, so no special-casing needed.
        lines.append(
            f"  seq={item.seq} cmd={_cmd_name(item.command):<20} "
            f"lat={item.x / 1e7:.6f} lon={item.y / 1e7:.6f} alt={item.z:.1f}"
        )
    return lines


def _plan_files() -> tuple[Path, Path]:
    plan_dir = PLANS_DIR / PLAN_NAME
    domain = plan_dir / "domain.pddl"
    template = plan_dir / "problem.pddl"
    if not domain.is_file() or not template.is_file():
        raise SystemExit(f"Plan '{PLAN_NAME}' is missing domain.pddl/problem.pddl in {plan_dir}")
    return domain, template


# ---- Step 1a (default): load a run the GUI already solved -----------------


def find_latest_run() -> Path:
    """The most recently written `plans/vformation/runs/<timestamp>/` dir -
    i.e. whatever source/destination you last clicked "Plan Mission" for on
    the V-formation plan in the app. Excludes this script's own `runs/cli/`
    scratch dir, same as `pddl_to_mavlink.find_latest_run`."""
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
            f"No plan runs found under {runs_dir}. Plan a V-formation mission in the app first, "
            f"or pass --run-dir/--source+--dest explicitly."
        )
    return max(candidates, key=lambda d: d.stat().st_mtime)


def _wing_routes_from_lead(lead_route: list[LatLon]) -> dict[str, list[tuple[float, float]]]:
    left_route, right_route = formation_wing_routes(
        lead_route, back_m=FORMATION_BACK_M, side_m=FORMATION_SIDE_M
    )
    return {
        "drone-lead": [(p.lat, p.lon) for p in lead_route],
        "drone-left": [(p.lat, p.lon) for p in left_route],
        "drone-right": [(p.lat, p.lon) for p in right_route],
    }


def load_solved_run(run_dir: Path) -> dict[str, list[tuple[float, float]]]:
    """Replay an already-solved run straight from disk - no re-solving, so the
    apex corridor ENHSP already found is flown exactly as planned, and both
    wing tracks are re-derived from it geometrically (see module docstring)."""
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

    locations = read_locations(problem_path.read_text(encoding="utf-8"))
    try:
        lead_route = [locations[name] for name in corridor]
    except KeyError as exc:
        raise SystemExit(f"Plan references location {exc} with no coordinates.") from exc
    return _wing_routes_from_lead(lead_route)


# ---- Step 1b (optional): solve a fresh source/destination -----------------


def solve_plan(source: tuple[float, float], destination: tuple[float, float]) -> dict[str, list[tuple[float, float]]]:
    """Run ENHSP on the apex's corridor and derive both wing tracks from it."""
    domain_path, template_path = _plan_files()
    template_text = template_path.read_text(encoding="utf-8")
    concrete_text = retarget_formation_problem(
        template_text,
        LatLon(lat=source[0], lon=source[1]),
        LatLon(lat=destination[0], lon=destination[1]),
    )

    run_dir = PLANS_DIR / PLAN_NAME / "runs" / _CLI_SOLVE_DIR_NAME
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

    corridor = extract_formation_corridor(steps)
    if not corridor:
        raise SystemExit("ENHSP found a plan but it contains no 'formation-cruise' legs.")

    locations = read_locations(concrete_text)
    try:
        lead_route = [locations[name] for name in corridor]
    except KeyError as exc:
        raise SystemExit(f"Plan references location {exc} with no coordinates.") from exc
    return _wing_routes_from_lead(lead_route)


# ---- Step 3a: validate locally against this repo's own ArduPilot model ----


def validate_locally(
    routes: dict[str, list[tuple[float, float]]],
    altitudes: dict[str, float],
    launch_stagger_s: float = FORMATION_LAUNCH_STAGGER_S,
    cruise_speed_mps: float = 15.0,
    tick_s: float = 0.2,
    timeout_s: float = 900.0,
) -> bool:
    """Run the real MAVLink upload handshake for every drone against its own
    `SimulatedFlightController`, all three homed on the *same* launch point
    and leaving it one at a time.

    Each drone's *first* mission is TAKEOFF + a `MAV_CMD_NAV_LOITER_UNLIM`
    item at its own slot in the airborne V - a real "go there and hold
    position" command, not a trick - so it climbs off the shared launch
    point, flies out to its slot and holds there, armed, rather than heading
    straight for its real waypoint the moment it personally finishes
    climbing. For the apex the slot is the launch point itself, so that
    mission is a straight climb.

    Takeoffs are sequenced on arrival, not on a clock: a drone is only
    launched once the drone ahead of it is confirmed settled in its slot,
    plus `launch_stagger_s` seconds of settling time, so the shared launch
    point is never occupied by two of them. Once every slot is filled,
    `release_formation` re-uploads each drone's real route (a second
    MISSION_COUNT/MISSION_ITEM_INT/MISSION_ACK conversation - no fresh ARM or
    mode change needed, it's already armed and in AUTO) and they all head for
    their first real waypoint together. Mirrors
    `ThreadSwarmBackend.start_formation_mission`'s in-app gate, just against
    real (simulated) MAVLink mission uploads instead of internal commands.
    """
    from engine.flight_controller import SimulatedFlightController

    pad = formation_launch_point(routes)

    drones: dict[str, dict] = {}
    for name in FORMATION_DRONES:
        waypoints = routes[name]
        altitude_m = altitudes[name]
        slot = waypoints[0]  # this drone's place in the airborne V
        mav = mav2.MAVLink(None, srcSystem=255, srcComponent=190)
        mav.robust_parsing = True
        # Climb off the shared pad and go take up the slot, then hold there.
        slot_items = build_mission_items(
            mav, 1, 1, [pad, slot], altitude_m, final_command=mav2.MAV_CMD_NAV_LOITER_UNLIM
        )
        real_items = build_mission_items(mav, 1, 1, waypoints, altitude_m)
        # Every drone's firmware is homed on the pad - that is the ground it
        # actually lifts off from, whatever its slot turns out to be.
        fc = SimulatedFlightController(
            sysid=1, home_lat=pad[0], home_lon=pad[1], cruise_speed_mps=cruise_speed_mps,
        )
        drones[name] = {
            "mav": mav, "fc": fc, "altitude_m": altitude_m, "slot": slot,
            "slot_items": slot_items, "real_items": real_items,
            "last": len(real_items) - 1,
            "launched": False,      # sent the takeoff + climb-out-to-slot mission
            "in_slot": False,       # confirmed settled in its slot at cruise altitude
            "released": False,      # sent the real route
        }

    # Only the apex is due now; each following drone is queued by
    # `check_slot_arrivals` once the one ahead of it has reached its slot and
    # the shared launch point is clear again.
    launch_times: dict[str, float] = {FORMATION_DRONES[0]: 0.0}
    launch_pending: list[str] = list(FORMATION_DRONES[1:])

    def _upload(name: str, d: dict, items: list, t: float) -> None:
        """One MISSION_COUNT/MISSION_ITEM_INT.../MISSION_ACK conversation,
        tracing every real MAVLink message as it crosses the wire - `->` for
        what this script sends, `<-` for what the (simulated) flight
        controller sends back - the same conversation
        `engine.mavlink_interpreter.CommandInterpreter` has with a real one,
        just made visible here."""
        mav, fc = d["mav"], d["fc"]

        def send(message, label: str) -> None:
            print(f"[{t:6.1f}s] {name} -> {label}")
            fc.send(message.pack(mav))

        send(
            mav.mission_count_encode(1, 1, len(items), mav2.MAV_MISSION_TYPE_MISSION),
            f"MISSION_COUNT(count={len(items)})",
        )
        uploaded, elapsed = False, 0.0
        while not uploaded and elapsed < 10.0:
            for message in fc.receive():
                kind = message.get_type()
                if kind in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                    print(f"[{t:6.1f}s] {name} <- {kind}(seq={message.seq})")
                    item = items[message.seq]
                    send(
                        item,
                        f"MISSION_ITEM_INT(seq={item.seq}, cmd={_cmd_name(item.command)}, "
                        f"lat={item.x / 1e7:.6f}, lon={item.y / 1e7:.6f}, alt={item.z:.1f})",
                    )
                elif kind == "MISSION_ACK":
                    print(f"[{t:6.1f}s] {name} <- MISSION_ACK(result={_mission_result_name(message.type)})")
                    uploaded = True
            fc.update(0.0)
            elapsed += 0.05
        if not uploaded:
            raise RuntimeError(f"{name}: firmware never acknowledged the mission upload")
        print(f"[{t:6.1f}s] {name}: mission uploaded and accepted ({len(items)} items).")

    def launch(name: str, t: float) -> None:
        """ARM + GUIDED + the climb-out-to-slot mission + AUTO."""
        d = drones[name]
        mav, fc = d["mav"], d["fc"]

        def send(message, label: str) -> None:
            print(f"[{t:6.1f}s] {name} -> {label}")
            fc.send(message.pack(mav))

        send(
            mav.command_long_encode(
                1, 1, mav2.MAV_CMD_DO_SET_MODE, 0,
                mav2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODE_GUIDED, 0, 0, 0, 0, 0),
            f"{_cmd_name(mav2.MAV_CMD_DO_SET_MODE)}(mode=GUIDED)",
        )
        send(
            mav.command_long_encode(1, 1, mav2.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0),
            f"{_cmd_name(mav2.MAV_CMD_COMPONENT_ARM_DISARM)}(arm=1)",
        )
        _upload(name, d, d["slot_items"], t)
        send(
            mav.command_long_encode(
                1, 1, mav2.MAV_CMD_DO_SET_MODE, 0,
                mav2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODE_AUTO, 0, 0, 0, 0, 0),
            f"{_cmd_name(mav2.MAV_CMD_DO_SET_MODE)}(mode=AUTO)",
        )
        d["launched"] = True

    def release_formation(t: float) -> None:
        """Once every drone is confirmed settled in its own slot, re-upload
        each one's real route - no re-arm or mode change needed,
        `SimulatedFlightController` picks up a freshly uploaded mission on its
        very next tick while already armed and in AUTO."""
        launched = [n for n in FORMATION_DRONES if drones[n]["launched"]]
        if len(launched) < len(FORMATION_DRONES):
            return  # a later drone is still waiting on the launch point
        if any(not drones[n]["in_slot"] for n in launched):
            return  # at least one is still climbing out to its slot
        if all(drones[n]["released"] for n in launched):
            return  # already released - nothing new to do
        print(f"[{t:6.1f}s] formation assembled - releasing all {len(launched)} drone(s) together.")
        for name in FORMATION_DRONES:
            d = drones[name]
            _upload(name, d, d["real_items"], t)
            d["released"] = True

    def check_slot_arrivals(t: float) -> None:
        """Mark every drone that has finished climbing out to its slot, and
        release the next one from the launch point once the drone ahead is
        settled there - the shared pad only ever has one drone on it."""
        for name in FORMATION_DRONES:
            d = drones[name]
            if not d["launched"] or d["in_slot"] or d["released"]:
                continue
            st = d["fc"].state()
            # `mission_complete` on the climb-out mission means the LOITER
            # item at the slot was reached; the altitude check is what makes
            # "in slot" mean at the right height as well as the right place.
            if not st.mission_complete:
                continue
            if st.relative_alt_m < d["altitude_m"] - ALTITUDE_ARRIVAL_M:
                continue
            d["in_slot"] = True
            print(
                f"[{t:6.1f}s] {name}: in slot ({d['slot'][0]:.6f},{d['slot'][1]:.6f}) "
                f"at {d['altitude_m']:g}m - launch point clear."
            )
            if launch_pending:
                nxt = launch_pending.pop(0)
                launch_times[nxt] = t + launch_stagger_s
                print(f"[{t:6.1f}s] {nxt}: due to take off from the launch point at t={launch_times[nxt]:.1f}s.")

    print(
        f"Flying V-formation mission - all three drones take off from "
        f"({pad[0]:.6f},{pad[1]:.6f}), apex first, each one leaving {launch_stagger_s:g}s after "
        f"the drone ahead of it has reached its slot;"
    )
    print("each holds its slot (MAV_CMD_NAV_LOITER_UNLIM) until every one of them is up.")
    t, next_report = 0.0, 0.0
    done: set[str] = set()
    while t < timeout_s and len(done) < len(drones):
        for name, at in list(launch_times.items()):
            if not drones[name]["launched"] and t >= at:
                launch(name, t)

        for d in drones.values():
            if d["launched"]:
                d["fc"].update(tick_s)
        t += tick_s

        check_slot_arrivals(t)
        release_formation(t)

        if t >= next_report:
            for name in FORMATION_DRONES:
                d = drones[name]
                if not d["launched"]:
                    due = launch_times.get(name)
                    when = f"t={due:.1f}s" if due is not None else "the drone ahead to reach its slot"
                    print(f"[{t:6.1f}s] {name} - on the launch point, waiting for {when}")
                    continue
                if not d["released"]:
                    st = d["fc"].state()
                    where = "holding in slot" if d["in_slot"] else "climbing out to its slot"
                    print(
                        f"[{t:6.1f}s] {name} - {where} "
                        f"({st.lat:.6f},{st.lon:.6f}) alt={st.relative_alt_m:5.1f}m"
                    )
                    continue
                st = d["fc"].state()
                print(
                    f"[{t:6.1f}s] {name} wp={st.mission_seq}/{d['last']} "
                    f"({st.lat:.6f},{st.lon:.6f}) alt={st.relative_alt_m:5.1f}m "
                    f"hdg={st.heading_deg:5.1f} gs={st.ground_speed_mps:4.1f}m/s "
                    f"batt={st.battery_pct:5.1f}%"
                )
            next_report = t + 5.0

        for name, d in drones.items():
            if not d["released"] or name in done:
                continue
            st = d["fc"].state()
            if st.mission_complete and not st.armed:
                print(f"[{t:6.1f}s] {name} mission complete - landed={st.landed}")
                done.add(name)

    if len(done) < len(drones):
        raise TimeoutError(
            f"missions did not all complete within {timeout_s:.0f}s "
            f"(finished: {', '.join(sorted(done)) or 'none'})"
        )
    print(f"[{t:6.1f}s] all {len(drones)} drones complete - V-formation mission done.")
    return True


# ---- Step 3b: upload + fly each drone against a real MAVLink endpoint ------


def fly_on_connections(
    connection_strings: list[str],
    routes: dict[str, list[tuple[float, float]]],
    altitudes: dict[str, float],
    launch_stagger_s: float = FORMATION_LAUNCH_STAGGER_S,
    timeout_s: float = 900.0,
) -> None:
    """One `upload_and_fly` per drone, each on its own thread (it blocks on
    network I/O for the whole flight). Every vehicle is expected to be parked
    on the same launch point - the source the plan was solved for - and the
    threads take turns: a wing's vehicle isn't even told to connect/arm until
    the drone ahead of it has reported settled in its slot, plus
    `launch_stagger_s` seconds of settling time. Gating on that arrival
    rather than on a blind sleep is what keeps two vehicles off the shared
    launch point.

    Each drone's first mission is the climb-out - TAKEOFF from the launch
    point plus a real `MAV_CMD_NAV_LOITER_UNLIM` item at its own slot in the
    airborne V - not its real route; for the apex the slot is the launch
    point itself, so that mission is a straight climb. Only once every drone
    in the formation is confirmed holding in its own slot
    (`wait_for_formation`) does each one get a second `upload_and_fly` call
    for its actual route, all released together. A background printer emits
    the same per-drone stats line as the local path, off each vehicle's real
    GLOBAL_POSITION_INT/SYS_STATUS telemetry."""
    from pymavlink import mavutil

    names = list(FORMATION_DRONES)
    if len(connection_strings) != len(names):
        raise SystemExit(
            f"got {len(connection_strings)} connection string(s) for {len(names)} drones - "
            f"pass one per drone (apex first), comma-separated"
        )

    pad = formation_launch_point(routes)

    stats: dict[str, dict] = {n: {} for n in names}
    stats_lock = threading.Lock()
    errors: dict[str, BaseException] = {}
    stop = threading.Event()

    climbed: set[str] = set()
    climbed_lock = threading.Lock()

    # One event per drone, set the moment it reports settled in its slot (or
    # gives up). Drone i waits on drone i-1's before it even connects, so the
    # vehicles leave the shared launch point strictly one at a time; the
    # apex's is pre-set because it goes first with nothing to wait for.
    in_slot_events = {name: threading.Event() for name in names}
    in_slot_events[names[0]].set()

    def wait_for_formation(name: str, timeout_s: float = 120.0) -> None:
        with climbed_lock:
            climbed.add(name)
            ready = len(climbed) >= len(names)
        if ready:
            return
        print(f"{name}: holding - waiting for the rest of the formation to climb ...")
        waited = 0.0
        while True:
            with climbed_lock:
                if len(climbed) >= len(names):
                    return
            if stop.is_set():
                raise FlightAborted(f"{name}: aborted while holding for the formation")
            if waited >= timeout_s:
                raise TimeoutError(
                    f"{name}: timed out after {timeout_s:.0f}s waiting for the rest of the "
                    f"formation to reach cruise altitude"
                )
            time.sleep(0.2)
            waited += 0.2

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

    def fly(name: str, conn: str, index: int) -> None:
        try:
            if index > 0:
                ahead = names[index - 1]
                print(f"{name}: on the launch point - waiting for {ahead} to reach its slot ...")
                while not in_slot_events[ahead].wait(0.2):
                    if stop.is_set():
                        raise FlightAborted(f"{name}: aborted while waiting on the launch point")
                waited = 0.0
                while waited < launch_stagger_s:
                    if stop.is_set():
                        raise FlightAborted(f"{name}: aborted while waiting on the launch point")
                    time.sleep(0.2)
                    waited += 0.2
                print(f"{name}: launch point clear - connecting to {conn} ...")

            master = mavutil.mavlink_connection(conn)
            slot = routes[name][0]
            # Take off from the shared launch point and go take up the slot.
            upload_and_fly(
                master, [pad, slot], altitudes[name],
                heartbeat_timeout_s=30.0, mission_timeout_s=timeout_s,
                on_progress=lambda msg, n=name: print(f"{n}: {msg}"),
                on_message=make_on_message(name),
                should_abort=stop.is_set,
                final_command=mav2.MAV_CMD_NAV_LOITER_UNLIM,
            )
            print(
                f"{name}: in slot ({slot[0]:.6f},{slot[1]:.6f}) at {altitudes[name]:g}m "
                f"- launch point clear."
            )
            in_slot_events[name].set()
            wait_for_formation(name)
            print(f"{name}: formation assembled - proceeding to the real route.")
            upload_and_fly(
                master, routes[name], altitudes[name],
                heartbeat_timeout_s=30.0, mission_timeout_s=timeout_s,
                on_progress=lambda msg, n=name: print(f"{n}: {msg}"),
                on_message=make_on_message(name),
                should_abort=stop.is_set,
            )
        except BaseException as exc:  # noqa: BLE001 - surfaced to the main thread below
            errors[name] = exc
            print(f"{name}: FAILED - {exc}")
        finally:
            # Never strand the drone behind this one on the launch point
            # waiting for a slot arrival that can no longer happen.
            in_slot_events[name].set()

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

    print(
        f"All {len(names)} vehicles take off from ({pad[0]:.6f},{pad[1]:.6f}), apex first; "
        f"each waits {launch_stagger_s:g}s after the drone ahead of it reaches its slot."
    )
    workers: list[threading.Thread] = []
    printer = threading.Thread(target=report, daemon=True)
    printer.start()
    # Every worker starts now; the turn-taking is enforced inside `fly`, which
    # holds each drone on the launch point until the one ahead of it reports
    # in slot rather than on a blind sleep that cannot tell whether the launch
    # point is actually clear.
    for i, (name, conn) in enumerate(zip(names, connection_strings)):
        if i == 0:
            print(f"{name}: connecting to {conn} ...")
        w = threading.Thread(target=fly, args=(name, conn, i), daemon=True)
        w.start()
        workers.append(w)

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


# ---- CLI --------------------------------------------------------------------


def _parse_latlon(text: str) -> tuple[float, float]:
    lat_str, lon_str = text.split(",")
    return float(lat_str), float(lon_str)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--run-dir", type=Path, default=None,
        help="load an already-solved run (plans/vformation/runs/<timestamp>/) instead of "
             "re-solving - default: the most recent run",
    )
    parser.add_argument(
        "--source", type=_parse_latlon, default=None,
        help="lat,lon - solves a fresh plan instead of loading a run (needs --dest too)",
    )
    parser.add_argument("--dest", type=_parse_latlon, default=None, help="lat,lon")
    parser.add_argument(
        "--lead-altitude", type=float, default=FORMATION_ALTITUDES["drone-lead"],
        help=f"apex cruise altitude, metres AGL (default: {FORMATION_ALTITUDES['drone-lead']:g})",
    )
    parser.add_argument(
        "--wing-altitude", type=float, default=FORMATION_ALTITUDES["drone-left"],
        help=f"both wings' cruise altitude, metres AGL (default: {FORMATION_ALTITUDES['drone-left']:g})",
    )
    parser.add_argument(
        "--launch-stagger", type=float, default=FORMATION_LAUNCH_STAGGER_S,
        help=f"seconds to wait after one drone reaches its slot before the next takes off "
             f"from the shared launch point, apex first "
             f"(default: {FORMATION_LAUNCH_STAGGER_S:g})",
    )
    parser.add_argument("--speed", type=float, default=15.0, help="cruise speed for --validate, m/s")
    parser.add_argument(
        "--connect", default=None,
        help="fly against real MAVLink endpoints instead of validating locally - "
             "one connection string per drone, apex first, comma-separated, every "
             "vehicle parked on the same launch point, "
             "e.g. udp:127.0.0.1:14550,udp:127.0.0.1:14560,udp:127.0.0.1:14570",
    )
    parser.add_argument("--dry-run", action="store_true", help="print each drone's route and mission items, then exit")
    args = parser.parse_args()

    if args.source or args.dest:
        if not (args.source and args.dest):
            parser.error("--source and --dest must be given together")
        print(f"Solving '{PLAN_NAME}': {args.source} -> {args.dest} ...")
        routes = solve_plan(args.source, args.dest)
    else:
        run_dir = args.run_dir or find_latest_run()
        print(f"Loading already-solved plan run: {run_dir}")
        routes = load_solved_run(run_dir)

    altitudes = {
        "drone-lead": args.lead_altitude,
        "drone-left": args.wing_altitude,
        "drone-right": args.wing_altitude,
    }

    pad = formation_launch_point(routes)
    print(f"Shared launch point (all {len(FORMATION_DRONES)} drones take off here): "
          f"{pad[0]:.6f}, {pad[1]:.6f}")
    for name in FORMATION_DRONES:
        slot = routes[name][0]
        print(f"  {name} climbs out to its slot: {slot[0]:.6f}, {slot[1]:.6f} "
              f"at {altitudes[name]:g}m")

    for name in FORMATION_DRONES:
        waypoints = routes[name]
        print(f"{name} route ({len(waypoints)} points, alt={altitudes[name]:g}m):")
        for i, (lat, lon) in enumerate(waypoints):
            print(f"  {i:3d}: {lat:.6f}, {lon:.6f}")

    # The exact MAVLink mission each drone will fly - named commands, not raw
    # ints - printed up front for all three regardless of --dry-run, apex
    # first (see format_mission_items).
    for name in FORMATION_DRONES:
        for line in format_mission_items(
            f"{name} climb-out", [pad, routes[name][0]], altitudes[name],
            final_command=mav2.MAV_CMD_NAV_LOITER_UNLIM,
        ):
            print(line)
        for line in format_mission_items(name, routes[name], altitudes[name]):
            print(line)

    if args.dry_run:
        return

    if args.connect:
        fly_on_connections(
            [c.strip() for c in args.connect.split(",")], routes, altitudes,
            launch_stagger_s=args.launch_stagger,
        )
    else:
        validate_locally(
            routes, altitudes, launch_stagger_s=args.launch_stagger, cruise_speed_mps=args.speed,
        )


if __name__ == "__main__":
    main()
