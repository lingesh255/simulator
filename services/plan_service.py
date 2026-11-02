"""GUI-facing PDDL mission planning.

    Swarm-Planning/Plans/<name>/<problem file> (template)
        -> substitute the picked source/destination (or base + search area)
        -> ENHSP (engine.pddl_planner)
        -> the ordered waypoint route a drone should fly - or, for an
           area-coverage plan, one such route per drone

Runs on a worker thread - `run_enhsp` shells out to Java and blocks - and
reports back through Qt signals, the same shape as `MavlinkSwarmBackend`'s
worker in `services/mavlink_backend.py`.

Thirteen plan *kinds* are supported, told apart by which domain a plan folder's
`domain.pddl` declares (see `plan_kind`) rather than by folder name, so any
future plan sharing one of these domains works with no further wiring:

- point-to-point (`swarm-drone-mission`, e.g. `Swarm-Planning/Plans/travell/`): one shared
  route, retargeted from a picked source/destination (+ optional restricted
  area) via `engine.pddl_problem.retarget_problem`. The template's single
  reference drone is then expanded to as many identical drones as the caller
  has checked (`expand_point_to_point_drones`), so the solved plan carries a
  travel sequence per drone and the whole swarm flies the route together.
- area-coverage (`forest-drone-search`, e.g. `Swarm-Planning/Plans/Search/`): one route per
  drone, retargeted from a picked base point + a marked search-area polygon
  via `engine.search_problem.retarget_search_problem`.
- formation (`v-formation-drone-mission`, e.g. `Swarm-Planning/Plans/vformation/`): picked
  like a point-to-point plan (source + destination, + optional restricted
  area), but ENHSP plans only the apex's corridor
  (`engine.pddl_problem.retarget_formation_problem`) and the two wing routes
  are derived parallel to it - three drones, one per V slot, flown
  concurrently like the area-coverage per-drone routes.
- grid formation (`grid-formation-drone-mission`, e.g. `Swarm-Planning/Plans/gridformation/`):
  picked the same way as a V-formation (source + destination, + optional
  restricted area), and likewise ENHSP plans only the leader's corridor
  (`engine.pddl_problem.retarget_grid_problem`); every other drone's route is
  derived parallel to it (`engine.pddl_problem.formation_member_route`), one
  per grid cell - a perfect-square drone count forms a square grid, anything
  else the most-square rectangle that tiles it exactly (see
  `engine.pddl_problem.grid_dimensions`), 10 m apart in every direction, each
  drone coordinating with only the up to 4 neighbors actually next to it
  (front/back/left/right - see `Swarm-Planning/Plans/gridformation/domain.pddl`).
- 3-D grid / cube (`cube-grid-formation-drone-mission`, e.g.
  `Swarm-Planning/Plans/3dgridformation/`): the grid formation extended into the vertical.
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
- parabola (`parabola-formation-drone-mission`, e.g. `Swarm-Planning/Plans/parabolaformation/`):
  picked like a V-formation (source + destination, + optional restricted
  area). Exactly three drones fly side by side on one line across the
  route, `PARABOLA_SPACING_M` apart - the middle drone (the vertex) at the
  plan's 50 m cruise altitude and the two arm drones `PARABOLA_ARM_RISE_M`
  higher - so from behind they form an upward-opening parabola. Every drone
  keeps a link with both of the others (`parabola-neighbor`/`link-up` in
  the domain). ENHSP plans the vertex's corridor; the arms' tracks are
  derived parallel to it and their extra height is carried, like a cube's
  layers, in `PlanRunResult.per_drone_altitude_offset_m`.
- figure-8 (`figure8-formation-drone-mission`, e.g. `Swarm-Planning/Plans/figure8formation/`):
  picked differently from every other plan - four clicks: the BASE, then
  the 8's LEFT end, the MIDDLE point where its lobes cross, and its RIGHT
  end. The drones fly as a train - one behind another, 10 m apart, each
  talking only to the drone in front of and behind it - from the base to
  the middle of the 8, round it `laps` times, and back to the base (see
  `engine.figure8_problem`). ENHSP plans the leader's trip through the key
  waypoints; every drone flies the same dense path started one spacing
  further back. `plan_figure8_mission` says when the layout can't take the
  checked number of drones.
- star show (`star-formation-drone-mission`, e.g. `Swarm-Planning/Plans/starformation/`): a
  picked SOURCE and DESTINATION. The drones take off from pads at the
  source and ONE BY ONE fly to the destination, each taking its place on
  the show's circle; then every lap form a star, a circle, spin the circle,
  form a star, spin the star and form a circle again; then ONE BY ONE fly
  back and land on their pads (see `engine.star_problem`). ENHSP plans the
  drones' turns and the show's sequence of shapes; every drone flies its
  own timed track, so each shape change and spin starts and ends together.
- planets (`planets-formation-drone-mission`, e.g. `Swarm-Planning/Plans/planets/`): a
  picked SOURCE and DESTINATION (the sun). The drones take off from pads at
  the source and ONE BY ONE fly to the destination, each to the start of
  its own circular orbit, 50 m further out than the one inside it (up to 8
  drones, mercury to neptune), neighbouring orbits turning in opposite
  directions. Every drone turns at the same angular speed - its speed in
  proportion to its radius - so the drones turning each way stay in one
  straight line out from the sun; a lap is one turn, and every lap ends
  with all drones lined up again. Then ONE BY ONE they fly back and land on
  their pads (see `engine.planets_problem`). ENHSP plans the drones' turns,
  the laps and the landing; every drone flies its own timed track, like the
  star show.
- road inspection (`road-inspection-drone-mission`, e.g. `Swarm-Planning/Plans/Inspect/`):
  source + destination clicked ON A ROAD. The road's exact path between
  them comes from an OSRM routing server (`engine.road_inspection`); one
  drone follows it, every curve, to the destination, flies the same road
  back and lands at the source. No road route (roads not connected, a click
  off the road, no connection to the routing service) fails the plan with
  the reason, before anything flies.
- battery-aware (`battery-aware-drone-mission`, e.g. `Swarm-Planning/Plans/BatteryAware/`):
  a picked SOURCE, DESTINATION and `CHARGING_STATION_COUNT` charging
  stations for ONE drone (the first checked). If its current charge covers
  the trip it flies straight there; otherwise ENHSP's optimal search picks
  the shortest chain of hops through the stations that the battery allows.
  The drone lands at every station on that chain, charges only what the
  next hop needs plus a 10% margin, and takes off again
  (`services.thread_backend.ThreadSwarmBackend.start_charging_mission`).
- Formula-4 (`formula4-drone-mission`, e.g. `Swarm-Planning/Plans/Formula-4/`): a picked
  SOURCE and DESTINATION; four drones in two lines of two fly there while
  the front and back drone of each lane keep swapping places - the front one
  slowing and dipping to 50 m, the back one speeding up and passing over at
  60 m. Each drone flies its own closed-form formula of one shared clock
  (`engine.formula4_problem`), which keeps every two drones >= 11.6 m apart
  without any of them knowing where the others are.
- Formation-Circle (`circle-formation-drone-mission`, e.g.
  `Swarm-Planning/Plans/Formation-Circle/`): a picked SOURCE and DESTINATION; four drones
  on a vertical circle across the route (bottom 40 m, left and right 50 m,
  top 60 m) that keeps turning as it flies, every drone moving bottom ->
  left -> top -> right - a spiral. Each drone flies its own closed-form
  formula of one shared clock (`engine.circle_formation_problem`); the
  circle turns rigidly, so neighbours stay 14.1 m apart without talking.
"""
from __future__ import annotations

import math
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
    haversine_m,
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
from engine.battery_mission import (
    best_station_route,
    check_plan_charges,
    hop_estimate,
    plan_stops,
    retarget_battery_problem,
)
from engine.circle_formation_problem import (
    DRONE_NAMES as CIRCLE_DRONE_NAMES,
    expected_actions as circle_expected_actions,
    plan_actions as circle_plan_actions,
    plan_circle_formation,
    retarget_circle_problem,
)
from engine.formula4_problem import (
    DRONE_NAMES as FORMULA4_DRONE_NAMES,
    MID_ALT_M as FORMULA4_MID_ALT_M,
    expected_actions as formula4_expected_actions,
    plan_formula4,
    plan_swaps as formula4_plan_swaps,
    retarget_formula4_problem,
)
from engine.road_inspection import (
    fetch_road_route,
    inspection_road_names,
    retarget_inspection_problem,
)
from engine.planets_problem import (
    expand_planet_drones,
    plan_planets_show,
    planets_distance_error,
    planets_drone_count_error,
    retarget_planets_problem,
)
from engine.train_keeping import SCHEDULE_UNIT_MPS
from engine.search_problem import expand_search_drones, retarget_search_problem
from engine.terrain import plan_terrain_profile
from engine.star_problem import (
    MIN_STAR_DRONES,
    expand_star_drones,
    plan_star_show,
    retarget_star_show_problem,
    star_drone_count_error,
    star_show_distance_error,
)

# Swarm-Planning/Plans/<plan>/        each plan's domain + problem files
# Swarm-Planning/Results/ENHSP/<plan>/runs/<timestamp>/   what each run of
#     the ENHSP planner produced: the concrete problem.pddl and plan.txt
SWARM_PLANNING_DIR = Path(__file__).resolve().parent.parent / "Swarm-Planning"
PLANS_DIR = SWARM_PLANNING_DIR / "Plans"
RESULTS_DIR = SWARM_PLANNING_DIR / "Results" / "ENHSP"
PLAN_DRONE_OBJECT = "drone1"  # ENHSP solves the point-to-point route for this reference drone
MIN_AREA_COVERAGE_DRONES = 2  # the Search domain needs >= 2 drones (two lanes, mutual separation)
MIN_GRID_DRONES = 4  # smallest shape that reads as a grid rather than a line (a 2x2 square)
MIN_CUBE_DRONES = 8  # smallest shape that reads as a cube rather than a plane (a 2x2x2 block)
PARABOLA_DRONES = 3  # a vertex and one arm either side

# The parabola's shape, in metres: each arm sits this far to the side of the
# vertex, level with it along the route, and this much higher - with the
# flat 50 m cruise altitude (engine.terrain.FLAT_CRUISE_ALT_M) that is the
# vertex at 50 m and both arms at 60 m. Must be kept in sync with the
# slot-cross-offset / target-altitude / arm-rise / link-distance numbers in
# Swarm-Planning/Plans/parabolaformation/problem.pddl - they describe the same geometry.
PARABOLA_SPACING_M = 10.0
PARABOLA_ARM_RISE_M = 10.0
PARABOLA_DRONE_NAMES = ("drone-vertex", "drone-left", "drone-right")
# Longest leg of a parabola drone's flown route: the corridor is split into
# legs no longer than this, so every drone has intermediate waypoints (the
# drone threads only fly a route as a timed formation when it has some) and
# the three are checked against each other at least this often.
PARABOLA_LEG_M = 200.0


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
# Swarm-Planning/Plans/vformation/problem.pddl - they describe the same geometry, but
# aren't read from the PDDL file directly (formation_wing_routes works from
# the apex's actual flown path, not the problem's numeric fluents).
FORMATION_BACK_M = 17.320508
FORMATION_SIDE_M = 10.0
FORMULA4_DRONES = 4  # Formula-4: two lines of two


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
    "parabola-formation-drone-mission": "parabola_formation",
    "figure8-formation-drone-mission": "figure8_formation",
    "star-formation-drone-mission": "star_formation",
    "planets-formation-drone-mission": "planets_formation",
    "road-inspection-drone-mission": "road_inspection",
    "battery-aware-drone-mission": "battery_aware",
    "formula4-drone-mission": "formula4_formation",
    "circle-formation-drone-mission": "circle_formation",
    "task-allocation": "task_allocation",
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
    task_initial_drone: Optional[str] = None
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

    @Slot(str, object, object, object, object)
    def run_battery(
        self, plan_name: str, source: object, destination: object, stations: object, drone: object
    ) -> None:
        try:
            result = self._run_battery_aware(plan_name, source, destination, stations, drone)
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

    @Slot(str, object, object, int, int)
    def run_show(
        self, plan_name: str, source: object, destination: object, drone_count: int, laps: int = 1
    ) -> None:
        try:
            if plan_kind(plan_name) == "planets_formation":
                result = self._run_planets_show(plan_name, source, destination, drone_count, laps)
            else:
                result = self._run_star_show(plan_name, source, destination, drone_count, laps)
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
        if plan_kind(plan_name) == "task_allocation":
            return self._run_task_allocation(plan_name, source, destination, drone_count)

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

        if plan_kind(plan_name) == "parabola_formation":
            # Same click shape again; always exactly three drones.
            return self._run_parabola_formation(plan_name, source, destination, no_fly_zone)

        if plan_kind(plan_name) == "formula4_formation":
            # Same clicks, no restricted area; always exactly four drones.
            return self._run_formula4(plan_name, source, destination)

        if plan_kind(plan_name) == "circle_formation":
            # Same clicks, no restricted area; always exactly four drones.
            return self._run_circle_formation(plan_name, source, destination)

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

    def _run_task_allocation(self, plan_name, source, destination, drone_count, drones=None):
        if drone_count != 4:
            raise ValueError("Task-Allocation needs exactly four checked drones.")
        if haversine_m(source, destination) < 30:
            raise ValueError("Task-Allocation source and destination must be at least 30 metres apart.")
        domain, template = self._plan_files(plan_name)
        concrete = template.read_text(encoding="utf-8")
        for name, point in (("source", source), ("destination", destination)):
            for axis, value in (("latitude", point.lat), ("longitude", point.lon)):
                concrete = re.sub(
                    rf"\(=\s*\({axis}\s+{name}\)\s+[^)]+\)",
                    f"(= ({axis} {name}) {value:.7f})", concrete,
                )
        if drones is not None:
            from services.local_flight import plan_route_flights
            route, _ = plan_terrain_profile([source, destination])
            flights = plan_route_flights(drones, route)
            if not any(f.feasible for f in flights):
                raise ValueError("None of the four drones has enough battery for this route.")
            for i, flight in enumerate(flights, 1):
                if not flight.feasible:
                    concrete = concrete.replace(f"(capable drone{i})", "")
        run_dir = self._start_run(plan_name, concrete)
        steps, stdout = run_enhsp(domain, run_dir / "problem.pddl")
        (run_dir / "plan.txt").write_text(stdout, encoding="utf-8")
        starts = [s for s in steps if s.action == "start-mission"]
        arrivals = [s for s in steps if s.action == "travel-to-destination"]
        if len(starts) != 1 or len(arrivals) != 1:
            raise PlanNotFound("Task-Allocation must launch one initial drone and reach the destination.")
        return PlanRunResult(
            plan_name=plan_name, steps=steps, waypoints=[source, destination],
            location_names=["source", "destination"], run_dir=run_dir,
            task_initial_drone=starts[0].args[0],
        )

    @Slot(str, object, object, object)
    def run_task(self, plan_name, source, destination, drones):
        try:
            result = self._run_task_allocation(plan_name, source, destination, len(drones), drones)
        except (PlannerError, PlanNotFound, ValueError, OSError) as exc:
            self.failed.emit(str(exc))
        else:
            self.finished.emit(result)

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
        # on a formation-style domain (this one, and Swarm-Planning/Plans/vformation's, are
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
        # `-gro naive` for the same reason Swarm-Planning/Plans/gridformation/ needs it: the
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

        # Split every corridor leg into equal pieces of at most
        # PARABOLA_LEG_M, as the parabola does, so the timetable below has
        # frequent checkpoints keeping the layers locked together.
        dense_route = [lead_route[0]]
        for a, b in zip(lead_route, lead_route[1:]):
            pieces = max(1, math.ceil(haversine_m(a, b) / PARABOLA_LEG_M))
            for k in range(1, pieces + 1):
                f = k / pieces
                dense_route.append(PddlLatLon(lat=a.lat + (b.lat - a.lat) * f, lon=a.lon + (b.lon - a.lon) * f))
        lead_route = dense_route

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

        def _cumulative_m(route: list[LatLon]) -> list[float]:
            out = [0.0]
            for a, b in zip(route, route[1:]):
                out.append(out[-1] + haversine_m(
                    PddlLatLon(lat=a.lat, lon=a.lon), PddlLatLon(lat=b.lat, lon=b.lon)
                ))
            return out

        # One shared timetable (schedule mode, as for the parabola): every
        # drone is due at its matching waypoint at the same clock time, and
        # the clock only starts once all have climbed. Flown independently,
        # the bottom layer - 10 m less to climb - would set off first and run
        # ahead, so a drone would no longer be directly beneath the one
        # stacked above it on the same lateral track.
        clock = [m / SCHEDULE_UNIT_MPS for m in _cumulative_m(per_drone_waypoints["drone-lead"])]
        train_routes = {}
        for name, route in per_drone_waypoints.items():
            train_routes[name] = TrainRoute(
                start=route[0],
                approach=list(route[1:-1]),
                loop=[],
                laps=1,
                depart=[route[-1]],
                schedule=tuple(zip(clock, _cumulative_m(route))),
            )

        return PlanRunResult(
            plan_name=plan_name,
            steps=steps,
            waypoints=_as_gui(lead_route),
            location_names=corridor,
            run_dir=run_dir,
            per_drone_waypoints=per_drone_waypoints,
            per_drone_altitude_offset_m=per_drone_altitude_offset_m,
            train_routes=train_routes,
        )

    def _run_parabola_formation(
        self,
        plan_name: str,
        source: LatLon,
        destination: LatLon,
        no_fly_zone: Optional[list[LatLon]] = None,
    ) -> PlanRunResult:
        """Parabola plan: three drones abreast on one line across the route -
        the vertex in the middle at the plan's cruise altitude, an arm drone
        `PARABOLA_SPACING_M` to either side and `PARABOLA_ARM_RISE_M` higher.

        ENHSP plans the vertex's corridor (bent around `no_fly_zone` if one
        is marked and actually in the way) and sets up every drone-to-drone
        link before the shape is allowed to form; the arms' tracks are then
        derived parallel to the corridor by `formation_member_route`, with no
        offset along the route so all three stay on one line. Like the
        cube's layers, the arms' extra height is returned in
        `per_drone_altitude_offset_m` and added to the cruise altitude when
        the drones fly.

        The three also fly one shared timetable (`train_routes`, schedule
        mode - see `engine.train_keeping`): each drone is due at every
        corridor waypoint at the same clock time, and the clock only starts
        once all three have climbed. Flown independently the vertex, with
        10 m less to climb, would set off first and run ~25 m ahead of the
        arms instead of staying on their line.
        """
        domain_path, template_path = self._plan_files(plan_name)
        template_text = template_path.read_text(encoding="utf-8")
        # The grid's retarget does exactly what is needed - relocate and
        # re-cost the corridor, detour round the zone - with no V-specific
        # slot-direction bookkeeping.
        concrete_text = retarget_grid_problem(
            template_text,
            PddlLatLon(lat=source.lat, lon=source.lon),
            PddlLatLon(lat=destination.lat, lon=destination.lon),
            no_fly_zone=(
                [PddlLatLon(lat=p.lat, lon=p.lon) for p in no_fly_zone] if no_fly_zone else None
            ),
            # The arms ride PARABOLA_SPACING_M out to either side of the
            # vertex's centreline - pad the buffer so they clear it too.
            no_fly_buffer_m=NO_FLY_BUFFER_M + PARABOLA_SPACING_M,
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
            vertex_route = [
                PddlLatLon(lat=locations[name].lat, lon=locations[name].lon)
                for name in corridor
            ]
        except KeyError as exc:
            raise PlanNotFound(
                f"Plan references location {exc} with no coordinates in problem.pddl"
            ) from exc

        def _as_gui(route: list[PddlLatLon]) -> list[LatLon]:
            return [LatLon(lat=p.lat, lon=p.lon) for p in route]

        # Split every corridor leg into equal pieces of at most
        # PARABOLA_LEG_M. The extra points are on the straight leg, so the
        # arms (derived below) still run exactly parallel to it.
        dense_route = [vertex_route[0]]
        for a, b in zip(vertex_route, vertex_route[1:]):
            pieces = max(1, math.ceil(haversine_m(a, b) / PARABOLA_LEG_M))
            for k in range(1, pieces + 1):
                f = k / pieces
                dense_route.append(PddlLatLon(lat=a.lat + (b.lat - a.lat) * f, lon=a.lon + (b.lon - a.lon) * f))
        vertex_route = dense_route

        vertex, left, right = PARABOLA_DRONE_NAMES
        per_drone_waypoints = {
            vertex: _as_gui(vertex_route),
            # `cross_m` is metres to the LEFT of the direction of travel.
            left: _as_gui(formation_member_route(vertex_route, along_m=0.0, cross_m=PARABOLA_SPACING_M)),
            right: _as_gui(formation_member_route(vertex_route, along_m=0.0, cross_m=-PARABOLA_SPACING_M)),
        }
        per_drone_altitude_offset_m = {
            vertex: 0.0,
            left: PARABOLA_ARM_RISE_M,
            right: PARABOLA_ARM_RISE_M,
        }

        def _cumulative_m(route: list[LatLon]) -> list[float]:
            out = [0.0]
            for a, b in zip(route, route[1:]):
                out.append(out[-1] + haversine_m(
                    PddlLatLon(lat=a.lat, lon=a.lon), PddlLatLon(lat=b.lat, lon=b.lon)
                ))
            return out

        # Clock time at each waypoint is the vertex's own distance there (in
        # nominal seconds); every drone is due at its own matching waypoint
        # then, so on a bend the outer arm simply flies a little faster.
        clock = [m / SCHEDULE_UNIT_MPS for m in _cumulative_m(per_drone_waypoints[vertex])]
        train_routes = {}
        for name, route in per_drone_waypoints.items():
            train_routes[name] = TrainRoute(
                start=route[0],
                approach=list(route[1:-1]),
                loop=[],
                laps=1,
                depart=[route[-1]],
                schedule=tuple(zip(clock, _cumulative_m(route))),
            )

        return PlanRunResult(
            plan_name=plan_name,
            steps=steps,
            waypoints=_as_gui(vertex_route),
            location_names=corridor,
            run_dir=run_dir,
            per_drone_waypoints=per_drone_waypoints,
            per_drone_altitude_offset_m=per_drone_altitude_offset_m,
            train_routes=train_routes,
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
        """Figure-8 plan: `drone_count` drones flying from `base` to the
        middle of the 8 built through the other three clicked points, spread
        evenly round it for `laps` laps, and back to `base` (see
        `engine.figure8_problem`).

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
                cruise_mps=r.cruise_mps,
                loop_alt_offsets=r.loop_alt_offsets,
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
        destination: LatLon,
        drone_count: int = MIN_STAR_DRONES,
        laps: int = 1,
    ) -> PlanRunResult:
        """Star show at `destination`, flown from and back to `source`,
        `laps` laps - see `engine.star_problem`.

        ENHSP plans the drones flying out one by one, the show's sequence
        (form circle, then per lap: star, circle, spin circle, star, spin
        star, circle; end the show) and the drones flying back one by one,
        and the result is checked against exactly that - in the turns
        `plan_star_show` gave the drones, along with their timed tracks."""
        problem = star_drone_count_error(drone_count)
        if problem:
            raise ValueError(problem)
        if destination is None:
            raise ValueError("A star show needs a destination - click the source, then the destination.")
        source_pddl = PddlLatLon(lat=source.lat, lon=source.lon)
        destination_pddl = PddlLatLon(lat=destination.lat, lon=destination.lon)
        show = plan_star_show(source_pddl, destination_pddl, drone_count, laps)

        domain_path, template_path = self._plan_files(plan_name)
        template_text = template_path.read_text(encoding="utf-8")
        concrete_text = retarget_star_show_problem(
            expand_star_drones(template_text, drone_count), source_pddl, destination_pddl, laps, show
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
        expected = [
            *(f"fly-to-formation {name}" for name in show.arrival_order),
            "form-circle", *(lap * laps), "end-show",
            *(f"return-to-source {name}" for name in show.return_order),
            "complete-show",
        ]
        sequence = []
        for step in sorted(steps, key=lambda s: s.index):
            if step.action in ("fly-to-formation", "return-to-source"):
                sequence.append(f"{step.action} {step.args[0]}")
            elif step.action in ("form-circle", "end-show", "complete-show", *lap):
                sequence.append(step.action)
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
            location_names=["source", "destination"],
            run_dir=run_dir,
            per_drone_waypoints=per_drone_waypoints,
            train_routes=train_routes,
            laps=laps,
        )

    def _run_planets_show(
        self, plan_name: str, source: LatLon, destination: LatLon, drone_count: int, laps: int = 1
    ) -> PlanRunResult:
        """Planets show round `destination` (the sun), flown from and back
        to `source`, `laps` laps - see `engine.planets_problem`.

        ENHSP plans the drones flying out one by one, enter-orbits, one
        fly-lap per lap, end-show and the drones flying back one by one, and
        the result is checked against exactly that - in the turns
        `plan_planets_show` gave the drones, along with their timed tracks."""
        problem = planets_drone_count_error(drone_count)
        if problem:
            raise ValueError(problem)
        if destination is None:
            raise ValueError("The planets show needs a destination - click the source, then the destination (the sun).")
        source_pddl = PddlLatLon(lat=source.lat, lon=source.lon)
        destination_pddl = PddlLatLon(lat=destination.lat, lon=destination.lon)
        show = plan_planets_show(source_pddl, destination_pddl, drone_count, laps)

        domain_path, template_path = self._plan_files(plan_name)
        template_text = template_path.read_text(encoding="utf-8")
        concrete_text = retarget_planets_problem(
            expand_planet_drones(template_text, drone_count), source_pddl, destination_pddl, laps, show
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

        expected = [
            *(f"fly-to-orbit {name}" for name in show.arrival_order),
            "enter-orbits", *(["fly-lap"] * laps), "end-show",
            *(f"return-to-source {name}" for name in show.return_order),
            "complete-planets",
        ]
        sequence = []
        for step in sorted(steps, key=lambda s: s.index):
            if step.action in ("fly-to-orbit", "return-to-source"):
                sequence.append(f"{step.action} {step.args[0]}")
            elif step.action in ("enter-orbits", "fly-lap", "end-show", "complete-planets"):
                sequence.append(step.action)
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
            location_names=["source", "destination"],
            run_dir=run_dir,
            per_drone_waypoints=per_drone_waypoints,
            train_routes=train_routes,
            laps=laps,
        )

    def _run_formula4(self, plan_name: str, source: LatLon, destination: LatLon) -> PlanRunResult:
        """Formula-4: four drones from `source` to `destination`, swapping
        places all the way, each on its own formula of the shared clock - see
        `engine.formula4_problem`.

        ENHSP checks the formula's numbers keep every pair >= min-separation
        apart (check-formula) and plans each drone on its own; the result
        must be exactly: take off, its N swaps alternating (front drones slow
        first), arrive, land - for every drone."""
        mission = plan_formula4(
            PddlLatLon(lat=source.lat, lon=source.lon),
            PddlLatLon(lat=destination.lat, lon=destination.lon),
        )
        domain_path, template_path = self._plan_files(plan_name)
        concrete_text = retarget_formula4_problem(template_path.read_text(encoding="utf-8"), mission)
        run_dir = self._start_run(plan_name, concrete_text)
        steps, stdout = run_enhsp(domain_path, run_dir / "problem.pddl", timeout_s=60.0)
        (run_dir / "plan.txt").write_text(stdout, encoding="utf-8")

        if not any(s.action == "check-formula" for s in steps):
            raise PlanNotFound("ENHSP's plan never checks the formula is collision-free.")
        flown = formula4_plan_swaps(steps)
        for name in FORMULA4_DRONE_NAMES:
            if flown[name] != formula4_expected_actions(name, mission.swaps):
                raise PlanNotFound(
                    f"ENHSP's plan for {name} is not its {mission.swaps} swaps from its "
                    f"source slot to its destination slot: " + " -> ".join(flown[name] or ["-"])
                )

        def _gui(p: PddlLatLon) -> LatLon:
            return LatLon(lat=p.lat, lon=p.lon)

        train_routes = {
            name: TrainRoute(
                start=_gui(r.start),
                approach=[],
                loop=[_gui(p) for p in r.loop],
                laps=1,
                depart=[_gui(p) for p in r.depart],
                schedule=r.schedule,
                loop_alt_offsets=r.loop_alt_offsets,
            )
            for name, r in mission.routes.items()
        }
        # The tracks' heights are offsets from Z0: lift each drone's cruise
        # altitude from the terrain profile's to exactly Z0.
        _points, altitudes = plan_terrain_profile([source, destination])
        lift = FORMULA4_MID_ALT_M - max(altitudes)
        return PlanRunResult(
            plan_name=plan_name,
            steps=steps,
            waypoints=[source, destination],
            location_names=["source", "destination"],
            run_dir=run_dir,
            per_drone_waypoints={name: r.expanded() for name, r in train_routes.items()},
            per_drone_altitude_offset_m={name: lift for name in train_routes},
            train_routes=train_routes,
        )

    def _run_circle_formation(self, plan_name: str, source: LatLon, destination: LatLon) -> PlanRunResult:
        """Formation-Circle: four drones on a turning vertical circle, flying
        a spiral from `source` to `destination`, each on its own formula of
        the shared clock - see `engine.circle_formation_problem`.

        ENHSP checks the formula's numbers keep every pair >= min-separation
        apart (check-formula) and plans each drone on its own; the result
        must be exactly: take off, join the circle at its starting place,
        one turn-quarter per quarter turn round bottom -> left -> top ->
        right, leave the circle, land - for every drone."""
        mission = plan_circle_formation(
            PddlLatLon(lat=source.lat, lon=source.lon),
            PddlLatLon(lat=destination.lat, lon=destination.lon),
        )
        domain_path, template_path = self._plan_files(plan_name)
        concrete_text = retarget_circle_problem(template_path.read_text(encoding="utf-8"), mission)
        run_dir = self._start_run(plan_name, concrete_text)
        steps, stdout = run_enhsp(domain_path, run_dir / "problem.pddl", timeout_s=60.0)
        (run_dir / "plan.txt").write_text(stdout, encoding="utf-8")

        if not any(s.action == "check-formula" for s in steps):
            raise PlanNotFound("ENHSP's plan never checks the formula is collision-free.")
        flown = circle_plan_actions(steps)
        for name in CIRCLE_DRONE_NAMES:
            if flown[name] != circle_expected_actions(name, mission.quarter_turns):
                raise PlanNotFound(
                    f"ENHSP's plan for {name} is not its {mission.quarter_turns} quarter turns "
                    f"round the circle from its pad to its destination pad: "
                    + " -> ".join(a[0] for a in flown[name][:12]) + " ..."
                )

        def _gui(p: PddlLatLon) -> LatLon:
            return LatLon(lat=p.lat, lon=p.lon)

        train_routes = {
            name: TrainRoute(
                start=_gui(r.start),
                approach=[],
                loop=[_gui(p) for p in r.loop],
                laps=1,
                depart=[_gui(p) for p in r.depart],
                schedule=r.schedule,
                loop_alt_offsets=r.loop_alt_offsets,
            )
            for name, r in mission.routes.items()
        }
        # Each drone takes off to its own height on the circle (40/50/60/50 m);
        # its track's heights are offsets from that.
        _points, altitudes = plan_terrain_profile([source, destination])
        return PlanRunResult(
            plan_name=plan_name,
            steps=steps,
            waypoints=[source, destination],
            location_names=["source", "destination"],
            run_dir=run_dir,
            per_drone_waypoints={name: r.expanded() for name, r in train_routes.items()},
            per_drone_altitude_offset_m={
                name: mission.start_alt_m[name] - max(altitudes) for name in train_routes
            },
            train_routes=train_routes,
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

    def _run_battery_aware(
        self,
        plan_name: str,
        source: LatLon,
        destination: LatLon,
        stations: list[LatLon],
        drone,
    ) -> PlanRunResult:
        """Battery-aware: `drone` from `source` to `destination`, through as
        few metres of the `stations` as its battery allows - none if it can
        fly straight there (see `engine.battery_mission`).

        `best_station_route` raises `BatteryRouteError` (a ValueError) with
        the reason when no chain of hops is flyable, before ENHSP runs.
        ENHSP's optimal search (metric: metres flown) must come back with a
        route exactly as short as that check's, landing and charging at
        every station on it."""
        pddl = [PddlLatLon(lat=p.lat, lon=p.lon) for p in (source, destination, *stations)]
        src, dst, pads = pddl[0], pddl[1], pddl[2:]
        expected = best_station_route(drone, src, dst, pads)
        domain_path, template_path = self._plan_files(plan_name)
        concrete_text = retarget_battery_problem(
            template_path.read_text(encoding="utf-8"), drone, src, dst, pads
        )

        run_dir = self._start_run(plan_name, concrete_text)
        steps, stdout = run_enhsp(
            domain_path, run_dir / "problem.pddl", timeout_s=60.0,
            extra_args=["-planner", "opt-hrmax"],
        )
        (run_dir / "plan.txt").write_text(stdout, encoding="utf-8")

        stops = plan_stops(steps)
        points = read_locations(concrete_text)

        def route_cost(route: list[str]) -> float:
            # The plan's metric: metres flown, plus 1 per station landed at.
            return len(route) - 2 + sum(
                hop_estimate(drone, points[a], points[b]).distance_m
                for a, b in zip(route, route[1:])
            )

        if not stops or stops[0] != "source" or stops[-1] != "destination":
            raise PlanNotFound(
                "ENHSP's plan does not fly the drone from the source to the destination: "
                + " -> ".join(stops or ["-"])
            )
        if route_cost(stops) > route_cost(expected) + 0.5:
            raise PlanNotFound(
                "ENHSP's route is not the shortest one the battery allows: "
                + " -> ".join(stops) + " instead of " + " -> ".join(expected)
            )
        check_plan_charges(steps, stops)
        if not any(s.action == "complete-mission" for s in steps):
            raise PlanNotFound("ENHSP's plan never lands the drone at the destination.")

        waypoints = [LatLon(lat=points[name].lat, lon=points[name].lon) for name in stops]
        return PlanRunResult(
            plan_name=plan_name,
            steps=steps,
            waypoints=waypoints,
            location_names=stops,
            run_dir=run_dir,
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
        run_dir = RESULTS_DIR / plan_name / "runs" / time.strftime("%Y-%m-%d_%H%M%S")
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
    _run_show_requested = Signal(str, object, object, int, int)
    _run_battery_requested = Signal(str, object, object, object, object)
    _run_task_requested = Signal(str, object, object, object)

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
        self._run_battery_requested.connect(self._worker.run_battery)
        self._run_task_requested.connect(self._worker.run_task)
        self._thread.start()

    def run_task_async(self, plan_name, source, destination, drones):
        self._run_task_requested.emit(plan_name, source, destination, list(drones))

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
        self,
        plan_name: str,
        source: LatLon,
        drone_count: int = MIN_STAR_DRONES,
        laps: int = 1,
        destination: Optional[LatLon] = None,
    ) -> None:
        """A drone show (the star show or the planets) flown from `source`
        to `destination`."""
        self._run_show_requested.emit(plan_name, source, destination, drone_count, laps)

    def run_battery_async(
        self,
        plan_name: str,
        source: LatLon,
        destination: LatLon,
        stations: list[LatLon],
        drone,
    ) -> None:
        """A battery-aware mission for one `drone` (a DroneConfig): straight
        there if its battery allows, otherwise via the `stations`."""
        self._run_battery_requested.emit(plan_name, source, destination, list(stations), drone)

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
