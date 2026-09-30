"""GUI-facing: fly one mission per drone on a fleet of Renode instances.

    [(DroneConfig, route), ...]
        -> kill_all_renode_processes() once
        -> RenodeLauncher(instance=sysid) per drone, spawned at its route's
           start, booted until every one is armable (any failure stops all)
        -> one MavlinkFlightService per drone, each flying its own route
        -> every drone's telemetry merged into one SwarmTelemetryBatch
        -> a per-drone outcome once every flight has ended, then every
           instance stopped (a fleet is never reused - the next mission
           boots a fresh one)

Boot runs on this service's own worker thread (Python threads inside it, one
per instance), so the GUI thread never blocks on Renode.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

from PySide6.QtCore import QObject, QThread, QTimer, Signal, Slot

from contracts.gui_orchestration import DroneConfig, LatLon, SwarmTelemetryBatch
from engine.renode_launcher import RenodeLauncher, kill_all_renode_processes
from services.mavlink_flight_service import MavlinkFlightService

# Highest sysid (= Renode instance) a fleet may use. Instance N listens on
# MAVLink 5762+N and physics 9002+N; 32 keeps both ranges clear of other
# well-known local ports (VNC's 5900, X11's 6000).
MAX_FLEET_SYSID = 32

# Seconds between starting one instance's boot and the next (0 = all at once).
BOOT_STAGGER_S = 0.0

# A plan with one shared route (travell) is flown by every drone, each one
# shifted this far further east than the last, spawn point included, so no
# two drones ever share a spot.
SHARED_ROUTE_SPACING_M = 18.0
_METRES_PER_DEG_LAT = 111_320.0


def offset_route(route: list[LatLon], east_m: float) -> list[LatLon]:
    """`route` moved `east_m` metres east (local flat-earth approximation,
    accurate at the tens of metres used here)."""
    return [
        LatLon(lat=p.lat, lon=p.lon + east_m / (_METRES_PER_DEG_LAT * math.cos(math.radians(p.lat))))
        for p in route
    ]


@dataclass
class DroneOutcome:
    sysid: int
    name: str
    state: str = "booting"          # booting / flying / completed / failed / stopped
    reason: str = ""


@dataclass
class _FleetPlan:
    assignments: list = field(default_factory=list)   # [(DroneConfig, [LatLon, ...])]
    standalone_dir: str = ""


class _BootWorker(QObject):
    ready = Signal(object)    # {sysid: connection_string}
    failed = Signal(str)
    progress = Signal(str)

    def __init__(self):
        super().__init__()
        self._launchers: dict[int, RenodeLauncher] = {}
        self._lock = threading.Lock()
        self._aborted = threading.Event()
        self._cancelled = threading.Event()   # stopped by the user, not by a failure

    @Slot(object)
    def boot(self, plan: _FleetPlan) -> None:
        self._aborted.clear()
        self._cancelled.clear()
        killed = kill_all_renode_processes()
        if killed:
            self.progress.emit(f"Cleared {len(killed)} stray Renode process(es) before the fleet launch.")
        errors: dict[int, str] = {}
        connections: dict[int, str] = {}
        threads = []

        def boot_one(drone: DroneConfig, route: list[LatLon]) -> None:
            try:
                launcher = RenodeLauncher(
                    plan.standalone_dir, instance=drone.sysid,
                    latitude_deg=route[0].lat, longitude_deg=route[0].lon,
                )
                with self._lock:
                    if self._aborted.is_set():
                        return
                    self._launchers[drone.sysid] = launcher
                t = time.monotonic()
                self.progress.emit(f"{drone.name} (SYSID {drone.sysid}): booting Renode instance {drone.sysid} ...")
                # The fleet's abort cancels this launch at any stage, even
                # before it has spawned anything for stop() to kill.
                connection = launcher.start(cancel=self._aborted)
                for step in (launcher.provision_first_boot_params, launcher.wait_until_armable):
                    if self._aborted.is_set():
                        launcher.stop()
                        return
                    step()
                if self._aborted.is_set():
                    launcher.stop()
                    return
                connections[drone.sysid] = connection
                self.progress.emit(
                    f"{drone.name} (SYSID {drone.sysid}): armable on {connection} after {time.monotonic() - t:.0f}s"
                )
            except Exception as exc:  # noqa: BLE001 - any boot failure fails the fleet
                with self._lock:
                    # Only the first failure is the cause; the others it
                    # stops then fail with "renode was stopped".
                    if not self._aborted.is_set():
                        errors[drone.sysid] = f"{drone.name} (SYSID {drone.sysid}): {exc}"
                # Don't leave the others booting for minutes towards a fleet
                # that can no longer fly.
                self.stop_all()

        for i, (drone, route) in enumerate(plan.assignments):
            if i and BOOT_STAGGER_S:
                time.sleep(BOOT_STAGGER_S)
            thread = threading.Thread(target=boot_one, args=(drone, route), name=f"boot-{drone.sysid}", daemon=True)
            thread.start()
            threads.append(thread)
        for thread in threads:
            thread.join()

        if errors or self._aborted.is_set():
            self.stop_all()
            if self._cancelled.is_set():
                self.failed.emit("fleet launch stopped by the user")
            else:
                others = len(plan.assignments) - 1
                self.failed.emit(
                    next(iter(errors.values()), "fleet launch aborted")
                    + (f" - the other {others} instance(s) were stopped" if others else "")
                )
            return
        self.ready.emit(connections)

    def exited_instances(self) -> dict[int, int]:
        """{sysid: exit code} of every instance whose Renode has died."""
        with self._lock:
            launchers = dict(self._launchers)
        return {
            sysid: launcher._proc.returncode
            for sysid, launcher in launchers.items()
            if launcher._proc is not None and launcher._proc.poll() is not None
        }

    def stop_one(self, sysid: int) -> None:
        """Stop one instance (its physics sidecar too) - the rest keep going."""
        with self._lock:
            launcher = self._launchers.pop(sysid, None)
        if launcher is not None:
            launcher.stop()

    def stop_all(self, cancelled: bool = False) -> None:
        """Stop every instance - safe from any thread, and while booting.
        `cancelled` marks it as the user's Stop rather than a failure."""
        if cancelled:
            self._cancelled.set()
        with self._lock:
            self._aborted.set()
            launchers = list(self._launchers.values())
            self._launchers.clear()
        for launcher in launchers:
            launcher.stop()


class FleetMission(QObject):
    """GUI-facing handle: `start(assignments, standalone_dir)`, then
    `batch_ready` (merged telemetry), `progress`, `boot_failed` (exactly once,
    no mission started) or `finished` (list[DroneOutcome], fleet_ok) once every
    drone's flight has ended. `stop()` stops every drone and instance."""

    batch_ready = Signal(object)
    progress = Signal(str)
    boot_failed = Signal(str)
    flying = Signal()
    finished = Signal(object, bool)

    _boot_requested = Signal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._thread = QThread(self)
        self._worker = _BootWorker()
        self._worker.moveToThread(self._thread)
        self._worker.ready.connect(self._on_booted)
        self._worker.failed.connect(self._on_boot_failed)
        self._worker.progress.connect(self.progress)
        self._boot_requested.connect(self._worker.boot)
        self._thread.start()
        self._plan: _FleetPlan | None = None
        self._outcomes: dict[int, DroneOutcome] = {}
        self._flights: dict[int, MavlinkFlightService] = {}
        self._latest: dict[int, object] = {}
        self._tick = 0
        self._started_at = 0.0
        self._stopping = False
        self._active = False
        self._ended: set[int] = set()
        # A drone's instance dying mid-flight only shows as a MAVLink link
        # going quiet, which upload_and_fly gives up on after minutes - watch
        # the processes themselves instead.
        self._watchdog = QTimer(self)
        self._watchdog.setInterval(1000)
        self._watchdog.timeout.connect(self._check_instances)

    @property
    def active(self) -> bool:
        """Booting or flying."""
        return self._active

    @staticmethod
    def validate(drones: list[DroneConfig]) -> str | None:
        """Why this set of drones can't form a fleet, or None."""
        by_sysid: dict[int, list[str]] = {}
        for drone in drones:
            by_sysid.setdefault(drone.sysid, []).append(drone.name)
        shared = {s: names for s, names in by_sysid.items() if len(names) > 1}
        if shared:
            detail = "; ".join(f"SYSID {s}: {', '.join(n)}" for s, n in shared.items())
            return f"Each drone in a fleet needs its own SYSID (it is also its Renode instance) - {detail}."
        outside = [f"{d.name} (SYSID {d.sysid})" for d in drones if not 1 <= d.sysid <= MAX_FLEET_SYSID]
        if outside:
            return (f"Fleet SYSIDs must be between 1 and {MAX_FLEET_SYSID} (the supported Renode instance "
                    f"range): {', '.join(outside)}.")
        return None

    def start(self, assignments: list[tuple[DroneConfig, list[LatLon]]], standalone_dir: str) -> None:
        self._plan = _FleetPlan(assignments=list(assignments), standalone_dir=standalone_dir)
        self._outcomes = {d.sysid: DroneOutcome(d.sysid, d.name) for d, _ in assignments}
        self._latest.clear()
        self._ended = set()
        self._stopping = False
        self._active = True
        self._started_at = time.monotonic()
        self.progress.emit(
            f"Booting a fleet of {len(assignments)} Renode instance(s) "
            f"({'all at once' if not BOOT_STAGGER_S else f'{BOOT_STAGGER_S:.0f}s apart'}) - "
            "the missions start once every drone is armable ..."
        )
        self._boot_requested.emit(self._plan)

    # ---- boot ----

    def _on_boot_failed(self, message: str) -> None:
        if not self._active:
            return
        self._active = False
        for outcome in self._outcomes.values():
            outcome.state, outcome.reason = "failed", "fleet did not boot"
        self.boot_failed.emit(message)

    def _on_booted(self, connections: dict) -> None:
        if not self._active or self._stopping:
            self._worker.stop_all()
            return
        self.progress.emit(
            f"Fleet ready after {time.monotonic() - self._started_at:.0f}s - flying "
            f"{len(connections)} mission(s)."
        )
        self.flying.emit()
        for drone, route in self._plan.assignments:
            service = MavlinkFlightService(self)
            sysid = drone.sysid
            service.batch_ready.connect(lambda batch, s=sysid: self._on_drone_batch(s, batch))
            service.progress.connect(lambda text, d=drone: self.progress.emit(f"[{d.name}] {text}"))
            service.finished.connect(lambda s=sysid: self._on_drone_ended(s, None))
            service.failed.connect(lambda text, s=sysid: self._on_drone_ended(s, text))
            self._flights[sysid] = service
            self._outcomes[sysid].state = "flying"
            service.run_async(route, connections[sysid], drone.cruise_altitude_m, sysid)
        self._watchdog.start()

    # ---- flight ----

    def _on_drone_batch(self, sysid: int, batch) -> None:
        if not self._active or not batch.drones:
            return
        self._latest[sysid] = batch.drones[0]
        self._tick += 1
        self.batch_ready.emit(SwarmTelemetryBatch(
            tick=self._tick, timestamp=time.monotonic() - self._started_at,
            drones=[self._latest[s] for s in sorted(self._latest)],
        ))

    def _check_instances(self) -> None:
        for sysid, code in self._worker.exited_instances().items():
            outcome = self._outcomes.get(sysid)
            if outcome is None or outcome.state != "flying":
                continue
            self._mark(sysid, "failed", f"its Renode instance exited mid-flight (exit code {code})")
            self._worker.stop_one(sysid)   # its physics sidecar too
            self._flights[sysid].abort()   # its flight then reports finished -> _on_drone_ended

    def _mark(self, sysid: int, state: str, reason: str = "") -> None:
        outcome = self._outcomes[sysid]
        outcome.state, outcome.reason = state, reason
        self.progress.emit(f"{outcome.name} (SYSID {sysid}): {state}" + (f" - {reason}" if reason else ""))

    def _on_drone_ended(self, sysid: int, error: str | None) -> None:
        outcome = self._outcomes.get(sysid)
        if outcome is None or sysid in self._ended:
            return
        self._ended.add(sysid)
        if outcome.state == "flying":   # not already failed by the watchdog
            if error is not None:
                self._mark(sysid, "failed", error)
            elif self._stopping:
                self._mark(sysid, "stopped", "stopped by the user")
            else:
                self._mark(sysid, "completed")
        if self._active and self._ended >= set(self._flights):
            self._wrap_up()

    def _wrap_up(self) -> None:
        self._active = False
        self._watchdog.stop()
        self._worker.stop_all()
        for service in self._flights.values():
            service.shutdown()
            service.deleteLater()
        self._flights.clear()
        outcomes = [self._outcomes[s] for s in sorted(self._outcomes)]
        self.finished.emit(outcomes, all(o.state == "completed" for o in outcomes))

    # ---- stop / shutdown ----

    def stop(self) -> None:
        """Stop every drone's flight and every instance (also mid-boot)."""
        if not self._active:
            self._worker.stop_all()
            return
        self._stopping = True
        if not self._flights:
            # Still booting: the boot worker reports the cancelled launch.
            self._worker.stop_all(cancelled=True)
            return
        for service in self._flights.values():
            service.abort()
        # Each flight's worker notices the abort within ~1s and reports
        # finished; _wrap_up then stops every instance.

    def shutdown(self) -> None:
        """Synchronous - for closeEvent."""
        self._stopping = True
        self._watchdog.stop()
        for service in self._flights.values():
            service.shutdown()
        self._worker.stop_all()
        self._thread.quit()
        # Booting threads notice their instance being stopped within a few
        # seconds (their MAVLink/physics waits poll the process).
        self._thread.wait(15000)
