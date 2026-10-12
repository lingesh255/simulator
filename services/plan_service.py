"""GUI-facing PDDL mission planning.

    plans/<name>/<problem file> (template)
        -> substitute the picked source/destination (or base + search area)
        -> ENHSP (engine.pddl_planner)
        -> the ordered waypoint route a drone should fly - or, for an
           area-coverage plan, one such route per drone

Runs on a worker thread - `run_enhsp` shells out to Java and blocks - and
reports back through Qt signals, the same shape as `MavlinkSwarmBackend`'s
worker in `services/mavlink_backend.py`.

Nine plan *kinds* are supported, told apart by which domain a plan folder's
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
- 3-D grid / cube (`cube-grid-formation-drone-mission`, e.g.
  `plans/3dgridformation/`): the grid formation extended into the vertical.
  Picked the same way, and again only the leader's corridor is planned
  (`engine.pddl_problem.retarget_cube_problem`), but the swarm forms a block
  of stacked horizontal layers - 8 drones a full 2x2x2 cube, any larger count
  the smallest block that holds it with a possibly partial top layer (see
  `engine.pddl_problem.cube_dimensions`). Each drone coordinates with the up
  to 24 drones adjacent to it in three dimensions (8 in its own layer, 8
  above, 8 below - not the two directly overhead and underneath). This is the
  one plan whose drones do NOT all fly the same altitude: its layers are
  separated vertically, so it also returns a per-drone altitude offset (see
  `PlanRunResult.per_drone_altitude_offset_m`).
- figure-8 (`figure8-formation-drone-mission`, e.g. `plans/figure8formation/`):
  picked differently from every other plan - four clicks: the BASE, then
  the 8's LEFT end, the MIDDLE point where its lobes cross, and its RIGHT
  end. The drones fly as a train - one behind another, 10 m apart, each
  talking only to the drone in front of and behind it - from the base to
  the middle of the 8, round it `laps` times, and back to the base (see
  `engine.figure8_problem`). ENHSP plans the leader's trip through the key
  waypoints; every drone flies the same dense path started one spacing
  further back. `plan_figure8_mission` says when the layout can't take the
  checked number of drones.
- star show (`star-formation-drone-mission`, e.g. `plans/starformation/`): a
  drone show round one picked SOURCE - no destination. The drones take off
  from pads there, form a circle, and every lap form a star, a circle, spin
  the circle, form a star, spin the star and form a circle again; then land
  back on their pads (see `engine.star_problem`). ENHSP plans the show's
  sequence of shapes; every drone flies its own timed track, so each shape
  change and spin starts and ends together.
- planets (`planets-formation-drone-mission`, e.g. `plans/planets/`): a
  solar-system show round one picked SOURCE (the sun) - no destination. Drone
  k flies its own circular orbit, 50 m further out than the one inside it
  (up to 8 drones, mercury to neptune), neighbouring orbits turning in
  opposite directions; a lap is one turn of the outermost drone, and every
  lap ends with all drones lined up again, then they land on their pads (see
  `engine.planets_problem`). ENHSP plans enter-orbits, the laps and the
  landing; every drone flies its own timed track, like the star show.
- road inspection (`road-inspection-drone-mission`, e.g. `plans/Inspect/`):
  source + destination clicked ON A ROAD. The road's exact path between
  them comes from an OSRM routing server (`engine.road_inspection`); one
  drone follows it, every curve, to the destination, flies the same road
  back and lands at the source. No road route (roads not connected, a click
  off the road, no connection to the routing service) fails the plan with
  the reason, before anything flies.
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
    CUBE_LAYER_SPACING_M,
    CUBE_SPACING_M,
    GRID_SPACING_M,
    NO_FLY_BUFFER_M,
    cube_cell_names,
    cube_dimensions,
    expand_cube_drones,
    expand_formation_drones,
    expand_grid_drones,
    expand_point_to_point_drones,
    formation_member_route,
    formation_wing_routes,
    grid_cell_names,
    grid_dimensions,
    read_locations,
    retarget_cube_problem,
    retarget_formation_problem,
    retarget_grid_problem,
    retarget_problem,
)
from engine.figure8_problem import (
    MIN_FIGURE8_DRONES,
    Figure8ShapeError,
    TrainRoute,
    expand_figure8_drones,
    figure8_corridor,
    figure8_drone_names,
    figure8_train_routes,
    plan_figure8_mission,
    retarget_figure8_problem,
)
from engine.road_inspection import (
    fetch_road_route,
    inspection_road_names,
    retarget_inspection_problem,
)
from engine.planets_problem import (
    expand_planet_drones,
    plan_planets_show,
    planets_drone_count_error,
    retarget_planets_problem,
)
from engine.search_problem import expand_search_drones, retarget_search_problem
from engine.star_problem import (
    MIN_STAR_DRONES,
    expand_star_drones,
    plan_star_show,
    retarget_star_show_problem,
    star_drone_count_error,
)

PLANS_DIR = Path(__file__).resolve().parent.parent / "plans"
PLAN_DRONE_OBJECT = "drone1"  # ENHSP solves the point-to-point route for this reference drone
MIN_AREA_COVERAGE_DRONES = 2  # the Search domain needs >= 2 drones (two lanes, mutual separation)
MIN_GRID_DRONES = 4  # smallest shape that reads as a grid rather than a line (a 2x2 square)
MIN_CUBE_DRONES = 8  # smallest shape that reads as a cube rather than a plane (a 2x2x2 block)


def _area_drone_names(count: int) -> list[str]:
    """`["drone1", ..., "drone{count}"]` - one per search lane, matching what
    `engine.search_problem.expand_search_drones` names them."""
    return [f"drone{i}" for i in range(1, max(MIN_AREA_COVERAGE_DRONES, count) + 1)]

# The V's shape, in metres: rank 1 of each wing sits this far behind its
# apex slot and this far out to its side - together putting the leader and
# both rank-1 wings at the corners of an equilateral triangle with 20 m
# sides (every drone exactly 20 m from both of the others, at every point
# along the route - source and destination alike, see
# `formation_wing_routes`). A further rank sits that same 20 m past the
# previous one (`formation_wing_routes` called again with `back_m`/`side_m`
# scaled by the rank number). Must be kept in sync with the
# `slot-along-offset`/`slot-cross-offset` numbers in
# plans/vformation/problem.pddl - they describe the same geometry, but
# aren't read from the PDDL file directly (formation_wing_routes works from
# the apex's actual flown path, not the problem's numeric fluents).
FORMATION_BACK_M = 17.320508
FORMATION_SIDE_M = 10.0


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


def _cube_drone_names(drone_count: int) -> list[str]:
    """Every drone name for a `drone_count`-drone 3-D grid ("cube")
    formation, leader first then every other filled cell nearest-to-farthest
    from it by 3-D Manhattan distance (layer + row + column steps) - the same
    rationale `_grid_drone_names` uses in two dimensions, so the first drones
    the operator checks land nearest the leader however large the block is.
    Matches the naming `engine.pddl_problem.expand_cube_drones`/
    `cube_cell_names` build into the problem file."""
    layers, rows, cols = cube_dimensions(drone_count)
    cell_name = cube_cell_names(layers, rows, cols, drone_count)
    leader_cell = next(cell for cell, name in cell_name.items() if name == "drone-lead")

    def manhattan(cell: tuple[int, int, int]) -> int:
        return (
            abs(cell[0] - leader_cell[0])
            + abs(cell[1] - leader_cell[1])
            + abs(cell[2] - leader_cell[2])
        )

    ordered_cells = sorted(cell_name, key=manhattan)
    return [cell_name[cell] for cell in ordered_cells]

# Domain name (as written in `(define (domain NAME) ...)`) -> plan kind.
# Anything not listed here defaults to "point_to_point" - the shape every
# plan folder had before area-coverage existed.
_DOMAIN_KIND = {
    "swarm-drone-mission": "point_to_point",
    "forest-drone-search": "area_coverage",
    "v-formation-drone-mission": "formation",
    "grid-formation-drone-mission": "grid_formation",
    "cube-grid-formation-drone-mission": "cube_formation",
    "figure8-formation-drone-mission": "figure8_formation",
    "star-formation-drone-mission": "star_formation",
    "planets-formation-drone-mission": "planets_formation",
    "road-inspection-drone-mission": "road_inspection",
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
    # Set only for a 3-D grid ("cube") plan: metres to add to each drone's
    # own cruise altitude, keyed the same way `per_drone_waypoints` is. A
    # cube separates its layers VERTICALLY, so unlike every other plan its
    # drones cannot all fly the same terrain-derived altitude - they would
    # occupy the same points in space. `None` for every other plan kind,
    # where the drones are separated laterally and one shared altitude is
    # correct. See `services.thread_backend.start_mission_paths`.
    per_drone_altitude_offset_m: Optional[dict[str, float]] = None
    # Set only for a figure-8 plan: each drone's route, keyed the same way
    # `per_drone_waypoints` is, structured as approach / loop x laps / depart
    # so the flight layer can repeat the lap on board.
    # `per_drone_waypoints` holds the same routes written out in full.
    train_routes: Optional[dict[str, TrainRoute]] = None
    laps: int = 1


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

    @Slot(str, object, object, object, object, int, int)
    def run_figure8(
        self,
        plan_name: str,
        base: object,
        left: object,
        crossing: object,
        right: object,
        drone_count: int,
        laps: int = 1,
    ) -> None:
        try:
            result = self._run_figure8(plan_name, base, left, crossing, right, drone_count, laps)
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

    @Slot(str, object, int, int)
    def run_show(self, plan_name: str, source: object, drone_count: int, laps: int = 1) -> None:
        try:
            if plan_kind(plan_name) == "planets_formation":
                result = self._run_planets_show(plan_name, source, drone_count, laps)
            else:
                result = self._run_star_show(plan_name, source, drone_count, laps)
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

        if plan_kind(plan_name) == "road_inspection":
            # Source + destination clicked the same way, but the route is the
            # real road between them, not a straight corridor.
            return self._run_road_inspection(plan_name, source, destination)

        if plan_kind(plan_name) == "cube_formation":
            # Same click/parameter shape again; `drone_count` picks the
            # block's layers x rows x columns (see `_run_cube_formation`).
            return self._run_cube_formation(plan_name, source, destination, no_fly_zone, drone_count)

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

        return PlanRunResult(
            plan_name=plan_name,
            steps=steps,
            waypoints=_as_gui(lead_route),
            location_names=corridor,
            run_dir=run_dir,
            per_drone_waypoints=per_drone_waypoints,
        )

    def _run_cube_formation(
        self,
        plan_name: str,
        source: LatLon,
        destination: LatLon,
        no_fly_zone: Optional[list[LatLon]] = None,
        drone_count: int = MIN_CUBE_DRONES,
    ) -> PlanRunResult:
        """3-D grid ("cube") plan: `drone_count` drones locked into a
        rectangular block of stacked horizontal layers (8 -> a full 2x2x2
        cube, 27 -> 3x3x3, 64 -> 4x4x4; any other count fills the smallest
        block that holds it, leaving the top layer partially filled - see
        `cube_dimensions`). `drone_count` must be >= 2 (`expand_cube_drones`
        raises `ValueError` otherwise, which `_Worker.run` reports back as a
        failed run rather than crashing) - `MainWindow` additionally requires
        `MIN_CUBE_DRONES` before it will attempt a run, so a cube always
        reads as a solid block rather than a flat plane.

        As with the V and the 2-D grid, ENHSP plans only the leader's
        waypoint corridor (bent around `no_fly_zone` if one is marked and
        actually in the way - see `retarget_cube_problem`); every other
        drone's track is derived parallel to it by `formation_member_route`,
        one call per filled cell.

        What is new here is the VERTICAL axis. A cube's layers are separated
        by height, not by lateral offset, so each drone also carries an
        altitude offset - `layer * CUBE_LAYER_SPACING_M` above the bottom
        layer - returned in `PlanRunResult.per_drone_altitude_offset_m` and
        added to the terrain-derived cruise altitude at execution time. Two
        drones in the same (row, column) but different layers fly the *same*
        lateral track, so without that offset they would occupy the same
        points in space; it is what actually keeps the layers apart.
        """
        layers, rows, cols = cube_dimensions(max(MIN_CUBE_DRONES, drone_count))
        leader_col = (cols - 1) // 2
        domain_path, template_path = self._plan_files(plan_name)
        template_text = template_path.read_text(encoding="utf-8")
        template_text = expand_cube_drones(template_text, max(MIN_CUBE_DRONES, drone_count))
        concrete_text = retarget_cube_problem(
            template_text,
            PddlLatLon(lat=source.lat, lon=source.lon),
            PddlLatLon(lat=destination.lat, lon=destination.lon),
            no_fly_zone=(
                [PddlLatLon(lat=p.lat, lon=p.lon) for p in no_fly_zone] if no_fly_zone else None
            ),
            # The leader's corridor is planned against the buffered zone, but
            # the block's outermost columns ride this far out to either side
            # of it - pad the buffer by that much so the whole cube, not just
            # the leader's centreline, clears the marked area. Only the
            # lateral reach matters: the zone is a ground polygon, so the
            # layers stacked above the leader do not widen the footprint.
            no_fly_buffer_m=NO_FLY_BUFFER_M
            + CUBE_SPACING_M * max(leader_col, cols - 1 - leader_col),
        )

        run_dir = self._start_run(plan_name, concrete_text)
        # `-gro naive` for the same reason plans/gridformation/ needs it: the
        # default grounder blows up on a formation-style domain once the
        # drone count climbs past a dozen-odd. The timeout scales with the
        # cell count, and more generously than the 2-D grid's, because a cube
        # adds a launch phase per layer on top of the per-drone one.
        steps, stdout = run_enhsp(
            domain_path,
            run_dir / "problem.pddl",
            timeout_s=max(45.0, 1.5 * layers * rows * cols),
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

        filled = max(MIN_CUBE_DRONES, drone_count)
        cell_name = cube_cell_names(layers, rows, cols, filled)
        leader_cell = next(cell for cell, name in cell_name.items() if name == "drone-lead")
        routes: dict[str, list[LatLon]] = {"drone-lead": _as_gui(lead_route)}
        altitude_offsets: dict[str, float] = {"drone-lead": 0.0}
        for (layer, row, col), name in cell_name.items():
            if (layer, row, col) == leader_cell:
                continue
            # `along_m` is "how far behind" (positive = behind), the opposite
            # sign from the PDDL problem's own `slot-along-offset` fact
            # (negative = behind, see expand_cube_drones) - that fact is
            # descriptive bookkeeping only, not fed into this call directly.
            member_route = formation_member_route(
                lead_route,
                along_m=row * CUBE_SPACING_M,
                cross_m=(col - leader_cell[2]) * CUBE_SPACING_M,
            )
            routes[name] = _as_gui(member_route)
            # The third dimension: layer 0 flies the plan's own cruise
            # altitude, every layer above it that much higher again.
            altitude_offsets[name] = (layer - leader_cell[0]) * CUBE_LAYER_SPACING_M

        ordered = _cube_drone_names(filled)
        per_drone_waypoints = {name: routes[name] for name in ordered}
        per_drone_altitude_offset_m = {name: altitude_offsets[name] for name in ordered}

        return PlanRunResult(
            plan_name=plan_name,
            steps=steps,
            waypoints=_as_gui(lead_route),
            location_names=corridor,
            run_dir=run_dir,
            per_drone_waypoints=per_drone_waypoints,
            per_drone_altitude_offset_m=per_drone_altitude_offset_m,
        )

    def _run_figure8(
        self,
        plan_name: str,
        base: LatLon,
        left: LatLon,
        crossing: LatLon,
        right: LatLon,
        drone_count: int = MIN_FIGURE8_DRONES,
        laps: int = 1,
    ) -> PlanRunResult:
        """Figure-8 plan: `drone_count` drones, as a train
        `FIGURE8_SPACING_M` apart, flying from `base` to the middle of the 8
        built through the other three clicked points, round it `laps` times,
        and back to `base` (see `engine.figure8_problem`).

        `plan_figure8_mission` validates the layout first and raises
        `ValueError` with the same operator-facing message `MainWindow`
        shows before it even gets here, so an over-full layout fails cleanly
        rather than as a bare "no plan" from ENHSP.

        ENHSP plans the leader's trip through the key waypoints; as with the
        area-coverage plan, that coarse corridor is the sanity check and the
        drones actually fly the dense `figure8_train_routes`."""
        def _pddl(p: LatLon) -> PddlLatLon:
            return PddlLatLon(lat=p.lat, lon=p.lon)

        def _gui(p: PddlLatLon) -> LatLon:
            return LatLon(lat=p.lat, lon=p.lon)

        mission = plan_figure8_mission(
            _pddl(base), _pddl(left), _pddl(crossing), _pddl(right), drone_count, laps
        )

        domain_path, template_path = self._plan_files(plan_name)
        template_text = template_path.read_text(encoding="utf-8")
        concrete_text = retarget_figure8_problem(
            expand_figure8_drones(template_text, drone_count), mission
        )

        run_dir = self._start_run(plan_name, concrete_text)
        # `-gro naive` for the same reason the grid plans need it - the
        # default grounder blows up on a formation-style domain as the drone
        # count climbs.
        steps, stdout = run_enhsp(
            domain_path,
            run_dir / "problem.pddl",
            timeout_s=max(30.0, 1.0 * drone_count * laps),
            extra_args=["-gro", "naive"],
        )
        (run_dir / "plan.txt").write_text(stdout, encoding="utf-8")

        # The leader's moves, in order: out to the middle, every leg of
        # every lap, back to the base.
        visited = [
            step.args[2]
            for step in sorted(steps, key=lambda s: s.index)
            if step.action in ("fly-to-pattern", "formation-cruise", "return-to-base")
            and len(step.args) >= 3
        ]
        if visited != figure8_corridor(mission):
            raise PlanNotFound(
                "ENHSP's plan does not fly base -> figure-8 -> base in order: "
                + " -> ".join(visited or ["(no moves)"])
            )

        routes = figure8_train_routes(mission)
        train_routes = {
            name: TrainRoute(
                start=_gui(r.start),
                approach=[_gui(p) for p in r.approach],
                loop=[_gui(p) for p in r.loop],
                laps=r.laps,
                depart=[_gui(p) for p in r.depart],
                shared_from_m=r.shared_from_m,
                slot_m=r.slot_m,
            )
            for name, r in routes.items()
        }
        per_drone_waypoints = {name: r.expanded() for name, r in train_routes.items()}
        return PlanRunResult(
            plan_name=plan_name,
            steps=steps,
            waypoints=per_drone_waypoints["drone-lead"],
            location_names=["base", *visited],
            run_dir=run_dir,
            per_drone_waypoints=per_drone_waypoints,
            train_routes=train_routes,
            laps=laps,
        )

    def _run_star_show(
        self,
        plan_name: str,
        source: LatLon,
        drone_count: int = MIN_STAR_DRONES,
        laps: int = 1,
    ) -> PlanRunResult:
        """Star show round `source`, `laps` laps - see `engine.star_problem`.

        ENHSP plans the show's sequence (form circle, then per lap: star,
        circle, spin circle, star, spin star, circle; then land) and the
        result is checked against exactly that; `plan_star_show` gives
        every drone its own timed track."""
        problem = star_drone_count_error(drone_count)
        if problem:
            raise ValueError(problem)
        source_pddl = PddlLatLon(lat=source.lat, lon=source.lon)
        show = plan_star_show(source_pddl, drone_count, laps)

        domain_path, template_path = self._plan_files(plan_name)
        template_text = template_path.read_text(encoding="utf-8")
        concrete_text = retarget_star_show_problem(
            expand_star_drones(template_text, drone_count), source_pddl, laps
        )
        run_dir = self._start_run(plan_name, concrete_text)
        # `-gro naive` for the same reason the grid plans need it.
        steps, stdout = run_enhsp(
            domain_path,
            run_dir / "problem.pddl",
            timeout_s=max(30.0, 1.0 * drone_count + 2.0 * laps),
            extra_args=["-gro", "naive"],
        )
        (run_dir / "plan.txt").write_text(stdout, encoding="utf-8")

        lap = ["form-star", "form-circle-in-lap", "spin-circle", "form-star-again", "spin-star", "close-lap"]
        expected = ["form-circle", *(lap * laps), "land-show"]
        sequence = [
            step.action for step in sorted(steps, key=lambda s: s.index)
            if step.action in ("form-circle", "land-show", *lap)
        ]
        if sequence != expected:
            raise PlanNotFound(
                "ENHSP's plan does not fly the show in order: "
                + " -> ".join(sequence or ["(no formation changes)"])
            )

        def _gui(p: PddlLatLon) -> LatLon:
            return LatLon(lat=p.lat, lon=p.lon)

        train_routes = {
            name: TrainRoute(
                start=_gui(r.start),
                approach=[_gui(p) for p in r.approach],
                loop=[_gui(p) for p in r.loop],
                laps=r.laps,
                depart=[_gui(p) for p in r.depart],
                schedule=r.schedule,
            )
            for name, r in show.routes.items()
        }
        per_drone_waypoints = {name: r.expanded() for name, r in train_routes.items()}
        return PlanRunResult(
            plan_name=plan_name,
            steps=steps,
            waypoints=[_gui(p) for p in show.circle],
            location_names=["source"],
            run_dir=run_dir,
            per_drone_waypoints=per_drone_waypoints,
            train_routes=train_routes,
            laps=laps,
        )

    def _run_planets_show(
        self, plan_name: str, source: LatLon, drone_count: int, laps: int = 1
    ) -> PlanRunResult:
        """Planets show round `source`, `laps` laps of the outermost orbit -
        see `engine.planets_problem`.

        ENHSP plans enter-orbits, one fly-lap per lap and land-planets, and
        the result is checked against exactly that; `plan_planets_show`
        gives every drone its own timed track."""
        problem = planets_drone_count_error(drone_count)
        if problem:
            raise ValueError(problem)
        source_pddl = PddlLatLon(lat=source.lat, lon=source.lon)
        show = plan_planets_show(source_pddl, drone_count, laps)

        domain_path, template_path = self._plan_files(plan_name)
        template_text = template_path.read_text(encoding="utf-8")
        concrete_text = retarget_planets_problem(
            expand_planet_drones(template_text, drone_count), source_pddl, laps
        )
        run_dir = self._start_run(plan_name, concrete_text)
        # `-gro naive` for the same reason the grid plans need it.
        steps, stdout = run_enhsp(
            domain_path,
            run_dir / "problem.pddl",
            timeout_s=max(30.0, 1.0 * drone_count + 2.0 * laps),
            extra_args=["-gro", "naive"],
        )
        (run_dir / "plan.txt").write_text(stdout, encoding="utf-8")

        expected = ["enter-orbits", *(["fly-lap"] * laps), "land-planets"]
        sequence = [
            step.action for step in sorted(steps, key=lambda s: s.index)
            if step.action in expected
        ]
        if sequence != expected:
            raise PlanNotFound(
                "ENHSP's plan does not fly the planets show in order: "
                + " -> ".join(sequence or ["(no orbit actions)"])
            )

        def _gui(p: PddlLatLon) -> LatLon:
            return LatLon(lat=p.lat, lon=p.lon)

        train_routes = {
            name: TrainRoute(
                start=_gui(r.start),
                approach=[_gui(p) for p in r.approach],
                loop=[_gui(p) for p in r.loop],
                laps=r.laps,
                depart=[_gui(p) for p in r.depart],
                schedule=r.schedule,
            )
            for name, r in show.routes.items()
        }
        per_drone_waypoints = {name: r.expanded() for name, r in train_routes.items()}
        return PlanRunResult(
            plan_name=plan_name,
            steps=steps,
            waypoints=[_gui(p) for p in show.preview_shape()],
            location_names=["source"],
            run_dir=run_dir,
            per_drone_waypoints=per_drone_waypoints,
            train_routes=train_routes,
            laps=laps,
        )

    def _run_road_inspection(
        self, plan_name: str, source: LatLon, destination: LatLon
    ) -> PlanRunResult:
        """Road inspection: one drone follows the road from `source` to
        `destination` - its exact path from the routing service - then flies
        the same road back and lands at the source (see
        `engine.road_inspection`).

        `fetch_road_route` raises `RoadRouteError` (a ValueError) with the
        operator-facing reason when there is no road route; `_Worker.run`
        reports that as a failed plan - the pop-up - before anything flies.
        ENHSP plans the inspection over the road's segments; the drone flies
        every point of the road's geometry, as one smooth mission."""
        route = fetch_road_route(
            PddlLatLon(lat=source.lat, lon=source.lon),
            PddlLatLon(lat=destination.lat, lon=destination.lon),
        )
        domain_path, template_path = self._plan_files(plan_name)
        concrete_text = retarget_inspection_problem(template_path.read_text(encoding="utf-8"), route)

        run_dir = self._start_run(plan_name, concrete_text)
        steps, stdout = run_enhsp(domain_path, run_dir / "problem.pddl", timeout_s=60.0)
        (run_dir / "plan.txt").write_text(stdout, encoding="utf-8")

        road = inspection_road_names(route)
        ordered = sorted(steps, key=lambda s: s.index)
        outbound = [s.args[2] for s in ordered if s.action == "inspect-segment" and len(s.args) >= 3]
        inbound = [s.args[2] for s in ordered if s.action == "return-along-road" and len(s.args) >= 3]
        landed = any(s.action == "land-at-source" for s in ordered)
        if outbound != road[1:] or inbound != list(reversed(road[:-1])) or not landed:
            raise PlanNotFound(
                "ENHSP's plan does not inspect the road to the destination and come "
                "back along it: out " + " -> ".join(outbound or ["-"])
                + "; back " + " -> ".join(inbound or ["-"])
            )

        path = [LatLon(lat=p.lat, lon=p.lon) for p in route.out_and_back()]
        inspection = TrainRoute(
            start=path[0], approach=path[1:-1], loop=[], laps=1, depart=[path[-1]]
        )
        return PlanRunResult(
            plan_name=plan_name,
            steps=steps,
            waypoints=path,
            location_names=["source", *outbound, *inbound],
            run_dir=run_dir,
            per_drone_waypoints={"drone1": path},
            train_routes={"drone1": inspection},
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
    """GUI-facing handle: call `run_async`/`run_area_async`/`run_figure8_async`,
    get `finished`/`failed`."""

    finished = Signal(object)  # PlanRunResult
    failed = Signal(str)

    _run_requested = Signal(str, object, object, object, int)
    _run_area_requested = Signal(str, object, object, int)
    _run_figure8_requested = Signal(str, object, object, object, object, int, int)
    _run_show_requested = Signal(str, object, int, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._thread = QThread(self)
        self._worker = _Worker()
        self._worker.moveToThread(self._thread)
        self._worker.finished.connect(self.finished)
        self._worker.failed.connect(self.failed)
        self._run_requested.connect(self._worker.run)
        self._run_area_requested.connect(self._worker.run_area)
        self._run_figure8_requested.connect(self._worker.run_figure8)
        self._run_show_requested.connect(self._worker.run_show)
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

    def run_show_async(
        self, plan_name: str, source: LatLon, drone_count: int = MIN_STAR_DRONES, laps: int = 1
    ) -> None:
        self._run_show_requested.emit(plan_name, source, drone_count, laps)

    def run_figure8_async(
        self,
        plan_name: str,
        base: LatLon,
        left: LatLon,
        crossing: LatLon,
        right: LatLon,
        drone_count: int = MIN_FIGURE8_DRONES,
        laps: int = 1,
    ) -> None:
        self._run_figure8_requested.emit(plan_name, base, left, crossing, right, drone_count, laps)

    def shutdown(self) -> None:
        self._thread.quit()
        self._thread.wait(2000)
