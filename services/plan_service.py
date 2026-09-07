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
  like a point-to-point plan (source + destination), but ENHSP plans only
  the apex's corridor (`engine.pddl_problem.retarget_formation_problem`) and
  the two wing routes are derived parallel to it - three drones, one per V
  slot, flown concurrently like the area-coverage per-drone routes.
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
    expand_point_to_point_drones,
    formation_wing_routes,
    read_locations,
    retarget_formation_problem,
    retarget_problem,
)
from engine.search_problem import expand_search_drones, retarget_search_problem

PLANS_DIR = Path(__file__).resolve().parent.parent / "plans"
PLAN_DRONE_OBJECT = "drone1"  # ENHSP solves the point-to-point route for this reference drone
MIN_AREA_COVERAGE_DRONES = 2  # the Search domain needs >= 2 drones (two lanes, mutual separation)


def _area_drone_names(count: int) -> list[str]:
    """`["drone1", ..., "drone{count}"]` - one per search lane, matching what
    `engine.search_problem.expand_search_drones` names them."""
    return [f"drone{i}" for i in range(1, max(MIN_AREA_COVERAGE_DRONES, count) + 1)]

# Drone objects the vformation problem template fixes, apex first.
FORMATION_DRONES = ("drone-lead", "drone-left", "drone-right")
# The V's shape, in metres: each wing sits this far behind its apex slot and
# this far out to its side - together putting the leader and both wings at
# the corners of an equilateral triangle with 20 m sides (every drone
# exactly 20 m from both of the others, at every point along the route -
# source and destination alike, see `formation_wing_routes`). Must be kept
# in sync with the `slot-along-offset`/`slot-cross-offset` numbers in
# plans/vformation/problem.pddl - they describe the same geometry, but
# aren't read from the PDDL file directly (formation_wing_routes works from
# the apex's actual flown path, not the problem's numeric fluents).
FORMATION_BACK_M = 17.320508
FORMATION_SIDE_M = 10.0

# Domain name (as written in `(define (domain NAME) ...)`) -> plan kind.
# Anything not listed here defaults to "point_to_point" - the shape every
# plan folder had before area-coverage existed.
_DOMAIN_KIND = {
    "swarm-drone-mission": "point_to_point",
    "forest-drone-search": "area_coverage",
    "v-formation-drone-mission": "formation",
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
            # Collected on the same two clicks as a point-to-point plan, so
            # it arrives through the same `run` slot - it just retargets and
            # extracts differently. `no_fly_zone` is not modelled by the
            # formation domain and is ignored, and its drone count is fixed
            # by the template (apex + two wings).
            return self._run_formation(plan_name, source, destination)

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
        self, plan_name: str, source: LatLon, destination: LatLon
    ) -> PlanRunResult:
        """V-formation plan: ENHSP plans the apex's waypoint corridor from
        the picked source/destination; the two wing tracks are then derived
        parallel to it (`formation_wing_routes`), and all three fly
        concurrently through `_start_area_coverage_mission`'s per-drone-route
        path - one drone per slot, apex/left/right in `FORMATION_DRONES`
        order."""
        domain_path, template_path = self._plan_files(plan_name)
        template_text = template_path.read_text(encoding="utf-8")
        concrete_text = retarget_formation_problem(
            template_text,
            PddlLatLon(lat=source.lat, lon=source.lon),
            PddlLatLon(lat=destination.lat, lon=destination.lon),
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

        left_route, right_route = formation_wing_routes(
            lead_route, back_m=FORMATION_BACK_M, side_m=FORMATION_SIDE_M
        )

        def _as_gui(route: list[PddlLatLon]) -> list[LatLon]:
            return [LatLon(lat=p.lat, lon=p.lon) for p in route]

        per_drone_waypoints = dict(
            zip(FORMATION_DRONES, (_as_gui(lead_route), _as_gui(left_route), _as_gui(right_route)))
        )

        return PlanRunResult(
            plan_name=plan_name,
            steps=steps,
            waypoints=_as_gui(lead_route),
            location_names=corridor,
            run_dir=run_dir,
            per_drone_waypoints=per_drone_waypoints,
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
