"""GUI-facing PDDL mission planning.

    plans/<name>/<problem file> (template)
        -> substitute the picked source/destination (or base + search area)
        -> ENHSP (engine.pddl_planner)
        -> the ordered waypoint route a drone should fly - or, for an
           area-coverage plan, one such route per drone

Runs on a worker thread - `run_enhsp` shells out to Java and blocks - and
reports back through Qt signals, the same shape as `MavlinkSwarmBackend`'s
worker in `services/mavlink_backend.py`.

Two plan *kinds* are supported, told apart by which domain a plan folder's
`domain.pddl` declares (see `plan_kind`) rather than by folder name, so any
future plan sharing one of these two domains works with no further wiring:

- point-to-point (`swarm-drone-mission`, e.g. `plans/travell/`): one shared
  route, retargeted from a picked source/destination (+ optional restricted
  area) via `engine.pddl_problem.retarget_problem`.
- area-coverage (`forest-drone-search`, e.g. `plans/Search/`): one route per
  drone, retargeted from a picked base point + a marked search-area polygon
  via `engine.search_problem.retarget_search_problem`.
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
    extract_multi_route,
    extract_route,
    run_enhsp,
)
from engine.pddl_problem import LatLon as PddlLatLon
from engine.pddl_problem import read_locations, retarget_problem
from engine.search_problem import retarget_search_problem

PLANS_DIR = Path(__file__).resolve().parent.parent / "plans"
PLAN_DRONE_OBJECT = "drone1"  # ENHSP solves the point-to-point route for this reference drone
AREA_COVERAGE_DRONES = ("drone1", "drone2")  # names the Search domain's problem template fixes

# Domain name (as written in `(define (domain NAME) ...)`) -> plan kind.
# Anything not listed here defaults to "point_to_point" - the shape every
# plan folder had before area-coverage existed.
_DOMAIN_KIND = {
    "swarm-drone-mission": "point_to_point",
    "forest-drone-search": "area_coverage",
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
    # Set only for an area-coverage plan: each drone object name (see
    # AREA_COVERAGE_DRONES) mapped to its own route. `None` for an ordinary
    # point-to-point plan, where `waypoints` above is the (single, shared)
    # route every checked drone flies.
    per_drone_waypoints: Optional[dict[str, list[LatLon]]] = None


class _Worker(QObject):
    finished = Signal(object)  # PlanRunResult
    failed = Signal(str)

    @Slot(str, object, object, object)
    def run(self, plan_name: str, source: object, destination: object, no_fly_zone: object) -> None:
        try:
            result = self._run(plan_name, source, destination, no_fly_zone)
        except PlanNotFound as exc:
            self._save_stdout(plan_name, exc.stdout)
            self.failed.emit(str(exc))
        except PlannerError as exc:
            self.failed.emit(str(exc))
        except OSError as exc:
            self.failed.emit(f"Could not read/write plan files: {exc}")
        else:
            self.finished.emit(result)

    @Slot(str, object, object)
    def run_area(self, plan_name: str, base: object, corners: object) -> None:
        try:
            result = self._run_area(plan_name, base, corners)
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
    ) -> PlanRunResult:
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

    def _run_area(
        self, plan_name: str, base: LatLon, corners: list[LatLon]
    ) -> PlanRunResult:
        domain_path, template_path = self._plan_files(plan_name)
        template_text = template_path.read_text(encoding="utf-8")
        concrete_text = retarget_search_problem(
            template_text,
            PddlLatLon(lat=base.lat, lon=base.lon),
            [PddlLatLon(lat=p.lat, lon=p.lon) for p in corners],
        )

        run_dir = self._start_run(plan_name, concrete_text)
        steps, stdout = run_enhsp(domain_path, run_dir / "problem.pddl")
        (run_dir / "plan.txt").write_text(stdout, encoding="utf-8")

        multi_route = extract_multi_route(steps, list(AREA_COVERAGE_DRONES))
        if not multi_route:
            raise PlanNotFound(
                f"ENHSP found a plan but it contains no drone movement legs."
            )
        locations = read_locations(concrete_text)
        try:
            per_drone_waypoints = {
                drone: [LatLon(lat=locations[name].lat, lon=locations[name].lon) for name in route]
                for drone, route in multi_route.items()
            }
        except KeyError as exc:
            raise PlanNotFound(
                f"Plan references location {exc} with no coordinates in problem.pddl"
            ) from exc

        # `waypoints`/`location_names` carry the first drone's route, for
        # whatever display/back-compat code only looks at "the" route (e.g.
        # a status line's distance figure) - actual execution always reads
        # `per_drone_waypoints`.
        first_drone = next(iter(per_drone_waypoints))
        return PlanRunResult(
            plan_name=plan_name,
            steps=steps,
            waypoints=per_drone_waypoints[first_drone],
            location_names=multi_route[first_drone],
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

    _run_requested = Signal(str, object, object, object)
    _run_area_requested = Signal(str, object, object)

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
    ) -> None:
        self._run_requested.emit(plan_name, source, destination, no_fly_zone)

    def run_area_async(self, plan_name: str, base: LatLon, corners: list[LatLon]) -> None:
        self._run_area_requested.emit(plan_name, base, corners)

    def shutdown(self) -> None:
        self._thread.quit()
        self._thread.wait(2000)
