"""GUI-facing PDDL mission planning.

    plans/<name>/<problem file> (template)
        -> substitute the picked source/destination (or base + search area)
        -> ENHSP (engine.pddl_planner)
        -> the ordered waypoint route a drone should fly - or, for an
           area-coverage plan, one such route per drone

Runs on a worker thread - `run_enhsp` shells out to Java and blocks - and
reports back through Qt signals, the same shape as `MavlinkSwarmBackend`'s
worker in `services/mavlink_backend.py`.

Three plan *kinds* are supported, told apart by which domain a plan folder's
`domain.pddl` declares (see `plan_kind`) rather than by folder name, so any
future plan sharing one of these domains works with no further wiring:

- point-to-point (`swarm-drone-mission`, e.g. `plans/travell/`): one shared
  route, retargeted from a picked source/destination (+ optional restricted
  area) via `engine.pddl_problem.retarget_problem`. The template's single
  reference drone is then expanded to as many identical drones as the caller
  has checked (`expand_point_to_point_drones`), so the solved plan carries a
  travel sequence per drone and the whole swarm flies the route together.
- area-coverage (`forest-drone-search`, e.g. `plans/Search/`): one route per
  drone, retargeted from a picked base point + a marked search-area polygon
  via `engine.search_problem.retarget_search_problem`.
- formation (`v-formation-drone-mission`, e.g. `plans/vformation/`): picked
  like a point-to-point plan (source + destination, + optional restricted
  area), but ENHSP plans only the apex's corridor
  (`engine.pddl_problem.retarget_formation_problem`) and the two wing routes
  are derived parallel to it - three drones, one per V slot, flown
  concurrently like the area-coverage per-drone routes.
- grid formation (`grid-formation-drone-mission`, e.g. `plans/gridformation/`):
  picked the same way as a V-formation (source + destination, + optional
  restricted area), and likewise ENHSP plans only the leader's corridor
  (`engine.pddl_problem.retarget_grid_problem`); every other drone's route is
  derived parallel to it (`engine.pddl_problem.formation_member_route`), one
  per grid cell - a perfect-square drone count forms a square grid, anything
  else the most-square rectangle that tiles it exactly (see
  `engine.pddl_problem.grid_dimensions`), 10 m apart in every direction, each
  drone coordinating with only the up to 4 neighbors actually next to it
  (front/back/left/right - see `plans/gridformation/domain.pddl`).
"""
from __future__ import annotations

import re
import time
from dataclasses import dataclass
from pathlib import Path

from typing import Optional

from PySide6.QtCore import QObject, QThread, Signal, Slot

from contracts.gui_orchestration import LatLon
from engine.pddl_planner import (
    PlanNotFound,
    PlannerError,
    PlanStep,
    extract_formation_corridor,
    extract_multi_route,
    extract_route,
    run_enhsp,
)
from engine.pddl_problem import LatLon as PddlLatLon
from engine.pddl_problem import (
    GRID_SPACING_M,
    NO_FLY_BUFFER_M,
    expand_formation_drones,
    expand_grid_drones,
    expand_point_to_point_drones,
    formation_member_route,
    formation_wing_routes,
    grid_cell_names,
    grid_dimensions,
    read_locations,
    retarget_formation_problem,
    retarget_grid_problem,
    retarget_problem,
)
from engine.search_problem import expand_search_drones, retarget_search_problem

PLANS_DIR = Path(__file__).resolve().parent.parent / "plans"
PLAN_DRONE_OBJECT = "drone1"  # ENHSP solves the point-to-point route for this reference drone
MIN_AREA_COVERAGE_DRONES = 2  # the Search domain needs >= 2 drones (two lanes, mutual separation)
MIN_GRID_DRONES = 4  # smallest shape that reads as a grid rather than a line (a 2x2 square)


def _area_drone_names(count: int) -> list[str]:
    """`["drone1", ..., "drone{count}"]` - one per search lane, matching what
    `engine.search_problem.expand_search_drones` names them."""
    return [f"drone{i}" for i in range(1, max(MIN_AREA_COVERAGE_DRONES, count) + 1)]

<<<<<<< HEAD
# Drone objects the vformation problem template fixes, apex first. This is
# also the launch order the executor stages takeoff in (see
# `services.thread_backend.ThreadSwarmBackend.start_formation_mission`): the
# lead departs alone first, then the left wing, then the right wing.
FORMATION_DRONES = ("drone-lead", "drone-left", "drone-right")
# The V's shape, in metres: each wing sits this far behind its apex slot and
# this far out to its side - together putting the leader and both wings at
# the corners of an equilateral triangle with 20 m sides (every drone
# exactly 20 m from both of the others, at every point along the route -
# source and destination alike, see `formation_wing_routes`). Must be kept
# in sync with the `slot-along-offset`/`slot-cross-offset` numbers in
=======
# The V's shape, in metres: rank 1 of each wing sits this far behind its
# apex slot and this far out to its side - together putting the leader and
# both rank-1 wings at the corners of an equilateral triangle with 20 m
# sides (every drone exactly 20 m from both of the others, at every point
# along the route - source and destination alike, see
# `formation_wing_routes`). A further rank sits that same 20 m past the
# previous one (`formation_wing_routes` called again with `back_m`/`side_m`
# scaled by the rank number). Must be kept in sync with the
# `slot-along-offset`/`slot-cross-offset` numbers in
>>>>>>> origin/main
# plans/vformation/problem.pddl - they describe the same geometry, but
# aren't read from the PDDL file directly (formation_wing_routes works from
# the apex's actual flown path, not the problem's numeric fluents).
FORMATION_BACK_M = 17.320508
FORMATION_SIDE_M = 10.0

<<<<<<< HEAD
# Per-slot cruise altitude, metres - fixed, not terrain-derived (see
# `_run_formation`/`ThreadSwarmBackend.start_formation_mission`): the lead
# climbs highest so the two wings, ~17 m behind and to either side, hold
# station below and clear of its rotor wash/wake.
FORMATION_LEAD_ALTITUDE_M = 60.0
FORMATION_WING_ALTITUDE_M = 55.0
FORMATION_ALTITUDES = (
    FORMATION_LEAD_ALTITUDE_M, FORMATION_WING_ALTITUDE_M, FORMATION_WING_ALTITUDE_M
)

# Seconds between one drone's launch and the next's, apex first - a real
# gap in flight time (scaled by the backend's own time_scale, same as every
# other duration in the sim), not a PDDL-planned delay: see the module
# docstring and `ThreadSwarmBackend.start_formation_mission`.
FORMATION_LAUNCH_STAGGER_S = 6.0
=======

def _formation_drone_names(ranks: int) -> list[str]:
    """`["drone-lead", "drone-left", "drone-right", "drone-left2", ...]` -
    apex first, then each wing pair rank by rank (nearest the apex first) -
    the same naming `engine.pddl_problem.expand_formation_drones` uses.
    Interleaving by rank (rather than every left rank, then every right)
    means the first checked drones land nearest the apex regardless of how
    many ranks the mission has."""
    names = ["drone-lead"]
    for r in range(1, ranks + 1):
        suffix = "" if r == 1 else str(r)
        names.append(f"drone-left{suffix}")
        names.append(f"drone-right{suffix}")
    return names


def _grid_drone_names(drone_count: int) -> list[str]:
    """Every drone name for a `drone_count`-drone grid formation, leader
    first then every other cell nearest-to-farthest from it (Manhattan
    distance in grid steps) - the same interleaving rationale
    `_formation_drone_names` uses for a V's ranks: so the first checked
    drones land nearest the leader regardless of how large the grid is.
    Matches the naming `engine.pddl_problem.expand_grid_drones`/
    `grid_cell_names` build into the problem file."""
    rows, cols = grid_dimensions(drone_count)
    cell_name = grid_cell_names(rows, cols)
    leader_cell = next(cell for cell, name in cell_name.items() if name == "drone-lead")

    def manhattan(cell: tuple[int, int]) -> int:
        return abs(cell[0] - leader_cell[0]) + abs(cell[1] - leader_cell[1])

    ordered_cells = sorted(cell_name, key=manhattan)
    return [cell_name[cell] for cell in ordered_cells]
>>>>>>> origin/main

# Domain name (as written in `(define (domain NAME) ...)`) -> plan kind.
# Anything not listed here defaults to "point_to_point" - the shape every
# plan folder had before area-coverage existed.
_DOMAIN_KIND = {
    "swarm-drone-mission": "point_to_point",
    "forest-drone-search": "area_coverage",
    "v-formation-drone-mission": "formation",
    "grid-formation-drone-mission": "grid_formation",
}
_DOMAIN_NAME = re.compile(r"\(define\s*\(domain\s+(\S+)\)")


def _find_problem_file(plan_dir: Path) -> Optional[Path]:
    """`problem.pddl` if present (every point-to-point plan so far); failing
    that, whatever single `*problem.pddl` file (case-insensitive) the folder
    has - so a plan can keep a more descriptive name (`SearchProblem.pddl`)
    instead of being forced to rename it."""
    exact = plan_dir / "problem.pddl"
    if exact.is_file():
        return exact
    candidates = [p for p in plan_dir.glob("*.pddl") if p.name.lower().endswith("problem.pddl")]
    return candidates[0] if len(candidates) == 1 else None


def list_plans(plans_dir: Path = PLANS_DIR) -> list[str]:
    """Names of plan folders that have both a domain and a problem file."""
    if not plans_dir.is_dir():
        return []
    return sorted(
        p.name
        for p in plans_dir.iterdir()
        if p.is_dir() and (p / "domain.pddl").is_file() and _find_problem_file(p) is not None
    )


def plan_kind(plan_name: str, plans_dir: Path = PLANS_DIR) -> str:
    """"point_to_point" or "area_coverage", from the plan's own domain.pddl -
    see the module docstring. Defaults to "point_to_point" if the domain
    file is missing or unrecognised, matching how every plan behaved before
    area-coverage existed."""
    domain_path = plans_dir / plan_name / "domain.pddl"
    try:
        text = domain_path.read_text(encoding="utf-8")
    except OSError:
        return "point_to_point"
    match = _DOMAIN_NAME.search(text)
    return _DOMAIN_KIND.get(match.group(1), "point_to_point") if match else "point_to_point"


@dataclass
class PlanRunResult:
    plan_name: str
    steps: list[PlanStep]
    waypoints: list[LatLon]
    location_names: list[str]
    run_dir: Path
    # Set only for an area-coverage plan: each drone object name
    # ("drone1".."droneN", one per checked drone) mapped to its own lane
    # route. `None` for an ordinary point-to-point plan, where `waypoints`
    # above is the (single, shared) route every checked drone flies.
    per_drone_waypoints: Optional[dict[str, list[LatLon]]] = None
    # Set only for a formation plan: each `FORMATION_DRONES` name mapped to
    # its fixed cruise altitude (metres) - see `FORMATION_ALTITUDES`. `None`
    # for every other plan kind, which derives altitude from terrain instead
    # (see `engine.terrain.plan_terrain_profile`).
    per_drone_altitudes: Optional[dict[str, float]] = None


class _Worker(QObject):
    finished = Signal(object)  # PlanRunResult
    failed = Signal(str)

    @Slot(str, object, object, object, int)
    def run(
        self,
        plan_name: str,
        source: object,
        destination: object,
        no_fly_zone: object,
        drone_count: int = 1,
    ) -> None:
        try:
            result = self._run(plan_name, source, destination, no_fly_zone, drone_count)
        except PlanNotFound as exc:
            self._save_stdout(plan_name, exc.stdout)
            self.failed.emit(str(exc))
        except PlannerError as exc:
            self.failed.emit(str(exc))
        except OSError as exc:
            self.failed.emit(f"Could not read/write plan files: {exc}")
        except ValueError as exc:
            self.failed.emit(str(exc))
        else:
            self.finished.emit(result)

    @Slot(str, object, object, int)
    def run_area(
        self, plan_name: str, base: object, corners: object, drone_count: int = 2
    ) -> None:
        try:
            result = self._run_area(plan_name, base, corners, drone_count)
        except PlanNotFound as exc:
            self._save_stdout(plan_name, exc.stdout)
            self.failed.emit(str(exc))
        except PlannerError as exc:
            self.failed.emit(str(exc))
        except (OSError, ValueError) as exc:
            self.failed.emit(f"Could not plan the search area: {exc}")
        else:
            self.finished.emit(result)

    def _save_stdout(self, plan_name: str, stdout: str) -> None:
        """Best-effort: persist ENHSP's raw output for inspection even when it
        found no plan. `_last_run_dir` is set by `_run`/`_run_area` before
        ENHSP is invoked, so it is available here regardless of how the run
        failed."""
        run_dir = getattr(self, "_last_run_dir", None)
        if run_dir is None or not stdout:
            return
        try:
            (run_dir / "plan.txt").write_text(stdout, encoding="utf-8")
        except OSError:
            pass

    def _plan_files(self, plan_name: str) -> tuple[Path, Path]:
        plan_dir = PLANS_DIR / plan_name
        domain_path = plan_dir / "domain.pddl"
        template_path = _find_problem_file(plan_dir)
        if not domain_path.is_file() or template_path is None:
            raise PlannerError(f"Plan '{plan_name}' is missing domain.pddl/problem.pddl")
        return domain_path, template_path

    def _run(
        self,
        plan_name: str,
        source: LatLon,
        destination: LatLon,
        no_fly_zone: Optional[list[LatLon]],
        drone_count: int = 1,
    ) -> PlanRunResult:
        if plan_kind(plan_name) == "formation":
            # Collected on the same clicks as a point-to-point plan, so it
            # arrives through the same `run` slot - it just retargets and
            # extracts differently. `no_fly_zone` is honoured the same way a
            # point-to-point plan honours it (see
            # `retarget_formation_problem`), and `drone_count` picks how many
            # ranks each wing gets (see `_run_formation`), instead of the
            # swarm-wide clone count a point-to-point plan uses it for.
            return self._run_formation(plan_name, source, destination, no_fly_zone, drone_count)

        if plan_kind(plan_name) == "grid_formation":
            # Same click/parameter shape again; `drone_count` picks the
            # grid's rows x columns (see `_run_grid_formation`) instead of
            # either of the other two meanings above.
            return self._run_grid_formation(plan_name, source, destination, no_fly_zone, drone_count)

        domain_path, template_path = self._plan_files(plan_name)
        template_text = template_path.read_text(encoding="utf-8")
        concrete_text = retarget_problem(
            template_text,
            PddlLatLon(lat=source.lat, lon=source.lon),
            PddlLatLon(lat=destination.lat, lon=destination.lon),
            no_fly_zone=(
                [PddlLatLon(lat=p.lat, lon=p.lon) for p in no_fly_zone] if no_fly_zone else None
            ),
        )
        # Grow the single-drone template to one drone per checked drone, so
        # ENHSP plans (and the step list shows) a travel sequence for the
        # whole swarm heading to the same destination together.
        concrete_text = expand_point_to_point_drones(concrete_text, max(1, drone_count))

        run_dir = self._start_run(plan_name, concrete_text)
        steps, stdout = run_enhsp(domain_path, run_dir / "problem.pddl")
        (run_dir / "plan.txt").write_text(stdout, encoding="utf-8")

        location_names = extract_route(steps, PLAN_DRONE_OBJECT)
        if not location_names:
            raise PlanNotFound(
                f"ENHSP found a plan but it contains no '{PLAN_DRONE_OBJECT}' travel legs."
            )
        locations = read_locations(concrete_text)
        try:
            waypoints = [
                LatLon(lat=locations[name].lat, lon=locations[name].lon)
                for name in location_names
            ]
        except KeyError as exc:
            raise PlanNotFound(
                f"Plan references location {exc} with no coordinates in problem.pddl"
            ) from exc

        return PlanRunResult(
            plan_name=plan_name,
            steps=steps,
            waypoints=waypoints,
            location_names=location_names,
            run_dir=run_dir,
        )

    def _run_formation(
        self,
        plan_name: str,
        source: LatLon,
        destination: LatLon,
        no_fly_zone: Optional[list[LatLon]] = None,
        drone_count: int = 3,
    ) -> PlanRunResult:
        """V-formation plan: one apex plus `ranks = (drone_count - 1) // 2`
        drones on each wing (`drone_count` must be odd and >= 3 -
        `expand_formation_drones` raises `ValueError` otherwise, which
        `_Worker.run` reports back as a failed run rather than crashing).

        ENHSP plans only the apex's waypoint corridor from the picked
        source/destination (bent around `no_fly_zone`, if marked and
        actually in the way - see `retarget_formation_problem`); every wing
        drone's own track is then derived parallel to it
        (`formation_wing_routes`, called once per rank with `back_m`/`side_m`
        scaled by the rank number), and the whole swarm flies concurrently
        through `_start_area_coverage_mission`'s per-drone-route path - one
        drone per slot, in `_formation_drone_names` order."""
        ranks = max(1, (drone_count - 1) // 2)
        domain_path, template_path = self._plan_files(plan_name)
        template_text = template_path.read_text(encoding="utf-8")
        template_text = expand_formation_drones(template_text, max(3, drone_count))
        concrete_text = retarget_formation_problem(
            template_text,
            PddlLatLon(lat=source.lat, lon=source.lon),
            PddlLatLon(lat=destination.lat, lon=destination.lon),
            no_fly_zone=(
                [PddlLatLon(lat=p.lat, lon=p.lon) for p in no_fly_zone] if no_fly_zone else None
            ),
            # The apex's corridor is planned against the buffered zone, but
            # the outermost wing drones ride `FORMATION_SIDE_M * ranks` out
            # to either side of it - pad the buffer by that much so the
            # whole V, not just the apex's centreline, clears the marked
            # area.
            no_fly_buffer_m=NO_FLY_BUFFER_M + FORMATION_SIDE_M * ranks,
        )

        run_dir = self._start_run(plan_name, concrete_text)
        steps, stdout = run_enhsp(domain_path, run_dir / "problem.pddl")
        (run_dir / "plan.txt").write_text(stdout, encoding="utf-8")

        corridor = extract_formation_corridor(steps)
        if not corridor:
            raise PlanNotFound(
                "ENHSP found a plan but it contains no 'formation-cruise' legs."
            )
        locations = read_locations(concrete_text)
        try:
            lead_route = [
                PddlLatLon(lat=locations[name].lat, lon=locations[name].lon)
                for name in corridor
            ]
        except KeyError as exc:
            raise PlanNotFound(
                f"Plan references location {exc} with no coordinates in problem.pddl"
            ) from exc

        def _as_gui(route: list[PddlLatLon]) -> list[LatLon]:
            return [LatLon(lat=p.lat, lon=p.lon) for p in route]

        routes = [lead_route]
        for r in range(1, ranks + 1):
            left_route, right_route = formation_wing_routes(
                lead_route, back_m=FORMATION_BACK_M * r, side_m=FORMATION_SIDE_M * r
            )
            routes.append(left_route)
            routes.append(right_route)
        per_drone_waypoints = dict(zip(_formation_drone_names(ranks), (_as_gui(r) for r in routes)))

        return PlanRunResult(
            plan_name=plan_name,
            steps=steps,
            waypoints=_as_gui(lead_route),
            location_names=corridor,
            run_dir=run_dir,
            per_drone_waypoints=per_drone_waypoints,
        )

    def _run_grid_formation(
        self,
        plan_name: str,
        source: LatLon,
        destination: LatLon,
        no_fly_zone: Optional[list[LatLon]] = None,
        drone_count: int = MIN_GRID_DRONES,
    ) -> PlanRunResult:
        """Grid-formation plan: `drone_count` drones locked into a
        rectangular grid (a perfect square forms a square, e.g. 4/9/16;
        anything else the most-square rectangle that tiles it exactly, e.g.
        8 -> 2x4, 12 -> 3x4 - see `grid_dimensions`). `drone_count` must be
        >= 2 (`expand_grid_drones` raises `ValueError` otherwise, which
        `_Worker.run` reports back as a failed run rather than crashing) -
        `MainWindow` additionally requires `MIN_GRID_DRONES` before it will
        even attempt a run, so a grid always reads as 2-D rather than a bare
        line.

        ENHSP plans only the leader's waypoint corridor from the picked
        source/destination (bent around `no_fly_zone`, if marked and
        actually in the way - see `retarget_grid_problem`); every other
        drone's own track is then derived parallel to it
        (`formation_member_route`, one call per grid cell with that cell's
        own along/cross offset - unlike a V's two symmetric wings, a grid's
        members don't come in matched +-side_m pairs), and the whole swarm
        flies concurrently through `_start_area_coverage_mission`'s
        per-drone-route path - one drone per cell, nearest-to-the-leader
        first (`_grid_drone_names` order)."""
        rows, cols = grid_dimensions(max(MIN_GRID_DRONES, drone_count))
        leader_col = (cols - 1) // 2
        domain_path, template_path = self._plan_files(plan_name)
        template_text = template_path.read_text(encoding="utf-8")
        template_text = expand_grid_drones(template_text, max(MIN_GRID_DRONES, drone_count))
        concrete_text = retarget_grid_problem(
            template_text,
            PddlLatLon(lat=source.lat, lon=source.lon),
            PddlLatLon(lat=destination.lat, lon=destination.lon),
            no_fly_zone=(
                [PddlLatLon(lat=p.lat, lon=p.lon) for p in no_fly_zone] if no_fly_zone else None
            ),
            # The leader's corridor is planned against the buffered zone,
            # but the grid's outermost columns ride this far out to either
            # side of it - pad the buffer by that much so the whole grid,
            # not just the leader's centreline, clears the marked area.
            no_fly_buffer_m=NO_FLY_BUFFER_M + GRID_SPACING_M * max(leader_col, cols - 1 - leader_col),
        )

        run_dir = self._start_run(plan_name, concrete_text)
        # ENHSP's default ("internal") grounder was found in practice to
        # blow up - a multi-minute, multi-gigabyte `OutOfMemoryError` inside
        # its own combination-building, well before search even starts -
        # on a formation-style domain (this one, and plans/vformation's, are
        # both affected) once the drone count climbs past a dozen-odd,
        # regardless of how much JVM heap it's given. Its "naive" grounder
        # (`-gro naive`) does not share that blowup - a 36-drone (6x6) grid
        # grounds and solves in ~2s with the JVM's ordinary default heap -
        # so it's used here rather than trying to out-resource the default
        # grounder's own growth.
        steps, stdout = run_enhsp(
            domain_path,
            run_dir / "problem.pddl",
            timeout_s=max(30.0, 1.0 * rows * cols),
            extra_args=["-gro", "naive"],
        )
        (run_dir / "plan.txt").write_text(stdout, encoding="utf-8")

        corridor = extract_formation_corridor(steps)
        if not corridor:
            raise PlanNotFound(
                "ENHSP found a plan but it contains no 'formation-cruise' legs."
            )
        locations = read_locations(concrete_text)
        try:
            lead_route = [
                PddlLatLon(lat=locations[name].lat, lon=locations[name].lon)
                for name in corridor
            ]
        except KeyError as exc:
            raise PlanNotFound(
                f"Plan references location {exc} with no coordinates in problem.pddl"
            ) from exc

        def _as_gui(route: list[PddlLatLon]) -> list[LatLon]:
            return [LatLon(lat=p.lat, lon=p.lon) for p in route]

<<<<<<< HEAD
        per_drone_waypoints = dict(
            zip(FORMATION_DRONES, (_as_gui(lead_route), _as_gui(left_route), _as_gui(right_route)))
        )
        per_drone_altitudes = dict(zip(FORMATION_DRONES, FORMATION_ALTITUDES))
=======
        cell_name = grid_cell_names(rows, cols)
        leader_cell = next(cell for cell, name in cell_name.items() if name == "drone-lead")
        routes: dict[str, list[LatLon]] = {"drone-lead": _as_gui(lead_route)}
        for (row, col), name in cell_name.items():
            if (row, col) == leader_cell:
                continue
            # `along_m` is "how far behind" (positive = behind - the same
            # sign `FORMATION_BACK_M` uses for a V's wings), the opposite
            # sign from the PDDL problem's own `slot-along-offset` fact
            # (negative = behind, see expand_grid_drones) - that fact is
            # descriptive bookkeeping only, not fed into this call directly.
            member_route = formation_member_route(
                lead_route,
                along_m=row * GRID_SPACING_M,
                cross_m=(col - leader_cell[1]) * GRID_SPACING_M,
            )
            routes[name] = _as_gui(member_route)
        per_drone_waypoints = {name: routes[name] for name in _grid_drone_names(rows * cols)}
>>>>>>> origin/main

        return PlanRunResult(
            plan_name=plan_name,
            steps=steps,
            waypoints=_as_gui(lead_route),
            location_names=corridor,
            run_dir=run_dir,
            per_drone_waypoints=per_drone_waypoints,
            per_drone_altitudes=per_drone_altitudes,
        )

    def _run_area(
        self, plan_name: str, base: LatLon, corners: list[LatLon], drone_count: int = 2
    ) -> PlanRunResult:
        n_lanes = max(MIN_AREA_COVERAGE_DRONES, drone_count)
        domain_path, template_path = self._plan_files(plan_name)
        template_text = template_path.read_text(encoding="utf-8")
        # Grow the 2-drone template to one drone/lane per checked drone, then
        # generate that many equal-area lawnmower lanes for the marked area.
        template_text = expand_search_drones(template_text, n_lanes)
        concrete_text, fine_routes = retarget_search_problem(
            template_text,
            PddlLatLon(lat=base.lat, lon=base.lon),
            [PddlLatLon(lat=p.lat, lon=p.lon) for p in corners],
            n_lanes=n_lanes,
        )

        run_dir = self._start_run(plan_name, concrete_text)
        # ENHSP reasons over a coarsened per-lane skeleton (see
        # retarget_search_problem); with greedy best-first + helpful actions
        # that solves in a few seconds even for 5-6 separation-coupled lanes,
        # where the default WA* search does not return at all. Plan
        # optimality does not matter here - the drones fly the fixed
        # `fine_routes`, not whatever order ENHSP sequences the hops in.
        steps, stdout = run_enhsp(
            domain_path,
            run_dir / "problem.pddl",
            timeout_s=max(45.0, 15.0 * n_lanes),
            extra_args=["-s", "gbfs", "-h", "hadd", "-ha", "true"],
        )
        (run_dir / "plan.txt").write_text(stdout, encoding="utf-8")

        # ENHSP's plan must move every drone; the coarse `multi_route` is just
        # that sanity check. What the drones actually FLY is the full
        # lawnmower `fine_routes` the geometry layer produced.
        drone_names = _area_drone_names(n_lanes)
        moved = extract_multi_route(steps, drone_names)
        missing = [d for d in drone_names if d not in moved]
        if missing:
            raise PlanNotFound(
                "ENHSP found a plan but it never moves: " + ", ".join(missing)
            )
        per_drone_waypoints = {
            drone: [LatLon(lat=p.lat, lon=p.lon) for p in fine_routes[drone]]
            for drone in drone_names
        }

        # `waypoints`/`location_names` carry the first drone's route, for
        # whatever display/back-compat code only looks at "the" route (e.g.
        # a status line's distance figure) - actual execution always reads
        # `per_drone_waypoints`.
        first_drone = drone_names[0]
        return PlanRunResult(
            plan_name=plan_name,
            steps=steps,
            waypoints=per_drone_waypoints[first_drone],
            location_names=moved[first_drone],
            run_dir=run_dir,
            per_drone_waypoints=per_drone_waypoints,
        )

    def _start_run(self, plan_name: str, concrete_text: str) -> Path:
        run_dir = PLANS_DIR / plan_name / "runs" / time.strftime("%Y-%m-%d_%H%M%S")
        run_dir.mkdir(parents=True, exist_ok=True)
        self._last_run_dir = run_dir
        (run_dir / "problem.pddl").write_text(concrete_text, encoding="utf-8")
        return run_dir


class PlanService(QObject):
    """GUI-facing handle: call `run_async`/`run_area_async`, get `finished`/`failed`."""

    finished = Signal(object)  # PlanRunResult
    failed = Signal(str)

    _run_requested = Signal(str, object, object, object, int)
    _run_area_requested = Signal(str, object, object, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._thread = QThread(self)
        self._worker = _Worker()
        self._worker.moveToThread(self._thread)
        self._worker.finished.connect(self.finished)
        self._worker.failed.connect(self.failed)
        self._run_requested.connect(self._worker.run)
        self._run_area_requested.connect(self._worker.run_area)
        self._thread.start()

    def run_async(
        self,
        plan_name: str,
        source: LatLon,
        destination: LatLon,
        no_fly_zone: Optional[list[LatLon]] = None,
        drone_count: int = 1,
    ) -> None:
        self._run_requested.emit(plan_name, source, destination, no_fly_zone, drone_count)

    def run_area_async(
        self,
        plan_name: str,
        base: LatLon,
        corners: list[LatLon],
        drone_count: int = 2,
    ) -> None:
        self._run_area_requested.emit(plan_name, base, corners, drone_count)

    def shutdown(self) -> None:
        self._thread.quit()
        self._thread.wait(2000)
