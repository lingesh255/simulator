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

The fleet's emulation is one of two backends, chosen per mission
(`FleetMission.start(..., shared_renode=...)`): one Renode process per drone
as above (the default), or every drone as a machine in ONE Renode process
(engine.shared_renode - about a quarter of the memory for four drones, a
little slower). Everything from the per-drone MavlinkFlightService down is
the same for both.

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
from engine.shared_renode import DroneStopError, SharedRenodeFleet
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
    shared_renode: bool = False   # every drone a machine in ONE Renode process


class _BootWorker(QObject):
    ready = Signal(object)    # {sysid: connection_string}
    failed = Signal(str)
    progress = Signal(str)
    phase = Signal(int, str)  # (sysid, short boot phase) - see FleetMission.drone_phase
    stop_failed = Signal(int, str)  # (sysid, why) - a shared-Renode drone could not be confirmed halted

    def __init__(self):
        super().__init__()
        self._launchers: dict[int, RenodeLauncher] = {}
        self._shared: SharedRenodeFleet | None = None   # the shared-Renode backend, when used
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
        if plan.shared_renode:
            self._boot_shared(plan)
            return
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
                self.phase.emit(drone.sysid, "Booting Renode")
                # The fleet's abort cancels this launch at any stage, even
                # before it has spawned anything for stop() to kill.
                connection = launcher.start(cancel=self._aborted)
                self.phase.emit(drone.sysid, "GPS fix - waiting for armable")
                for step in (launcher.provision_first_boot_params, launcher.wait_until_armable):
                    if self._aborted.is_set():
                        launcher.stop()
                        return
                    step()
                if self._aborted.is_set():
                    launcher.stop()
                    return
                connections[drone.sysid] = connection
                self.phase.emit(drone.sysid, "Armable - waiting for the fleet")
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

    def _boot_shared(self, plan: _FleetPlan) -> None:
        """The shared-Renode backend's boot: same signals as the per-process
        one. SharedRenodeFleet.start() already stops everything and names
        the drone if any of them fails to boot."""
        names = {drone.sysid: drone.name for drone, _route in plan.assignments}
        t = time.monotonic()
        try:
            fleet = SharedRenodeFleet(
                plan.standalone_dir,
                [(drone.sysid, route[0].lat, route[0].lon) for drone, route in plan.assignments],
            )
            with self._lock:
                if self._aborted.is_set():
                    raise RuntimeError("fleet launch aborted")
                self._shared = fleet
            self.progress.emit(
                "One shared Renode: " + ", ".join(
                    f"{drone.name} (SYSID {drone.sysid}) is machine drone{drone.sysid}"
                    for drone, _route in plan.assignments) + " ...")

            def on_phase(sysid: int, text: str) -> None:
                self.phase.emit(sysid, text)
                if text.startswith("Armable"):
                    self.progress.emit(
                        f"{names[sysid]} (SYSID {sysid}): armable on {fleet.connection_string(sysid)} "
                        f"after {time.monotonic() - t:.0f}s")

            connections = fleet.start(cancel=self._aborted, on_phase=on_phase)
        except Exception as exc:  # noqa: BLE001 - any boot failure fails the fleet
            self.stop_all()
            if self._cancelled.is_set():
                self.failed.emit("fleet launch stopped by the user")
            else:
                self.failed.emit(f"{exc} - the shared Renode and every physics sidecar were stopped")
            return
        if self._aborted.is_set():
            self.stop_all()
            self.failed.emit("fleet launch stopped by the user" if self._cancelled.is_set() else "fleet launch aborted")
            return
        pids = fleet.pids()
        self.progress.emit(f"Shared Renode pid {pids['renode']}, physics sidecars {pids['physics']}.")
        self.ready.emit(connections)

    def dead_drones(self) -> dict[int, str]:
        """{sysid: why} for every drone whose emulation has died under it.
        Per-process fleet: its own Renode exited. Shared Renode: the one
        Renode exited (every drone), or that drone's physics sidecar did -
        its MAVLink link stays up on frozen sensor values, so only the
        process shows it."""
        with self._lock:
            launchers = dict(self._launchers)
            shared = self._shared
        if shared is not None:
            code = shared.renode_exit_code
            if code is not None:
                return {sysid: f"the shared Renode exited mid-flight (exit code {code})" for sysid in shared.launchers}
            return {sysid: f"its physics sidecar exited mid-flight (exit code {sidecar_code})"
                    for sysid, sidecar_code in shared.dead_sidecars().items()}
        return {
            sysid: f"its Renode instance exited mid-flight (exit code {launcher._proc.returncode})"
            for sysid, launcher in launchers.items()
            if launcher._proc is not None and launcher._proc.poll() is not None
        }

    def stop_one(self, sysid: int) -> None:
        """Stop one drone's emulation (its physics sidecar too) - the rest
        keep going. In a shared Renode that halts its machine."""
        with self._lock:
            launcher = self._launchers.pop(sysid, None)
            shared = self._shared
        if launcher is not None:
            launcher.stop()
        if shared is not None:
            # Off this (GUI) thread: it talks to Renode's monitor for a few seconds.
            def halt() -> None:
                try:
                    shared.stop_drone(sysid)
                except DroneStopError as exc:
                    self.stop_failed.emit(sysid, str(exc))
            threading.Thread(target=halt, name=f"stop-drone-{sysid}", daemon=True).start()

    def stop_all(self, cancelled: bool = False) -> None:
        """Stop every instance - safe from any thread, and while booting.
        `cancelled` marks it as the user's Stop rather than a failure."""
        if cancelled:
            self._cancelled.set()
        with self._lock:
            self._aborted.set()
            launchers = list(self._launchers.values())
            self._launchers.clear()
            shared, self._shared = self._shared, None
        for launcher in launchers:
            launcher.stop()
        if shared is not None:
            shared.stop_all()


class FleetMission(QObject):
    """GUI-facing handle: `start(assignments, standalone_dir)`, then
    `batch_ready` (merged telemetry), `progress`, `boot_failed` (exactly once,
    no mission started) or `finished` (list[DroneOutcome], fleet_ok) once every
    drone's flight has ended. `stop()` stops every drone and instance."""

    batch_ready = Signal(object)
    progress = Signal(str)
    # (sysid, short text) for each drone's current phase: boot steps, each
    # line of its flight's progress ("[FC] " prefix stripped), and its final
    # outcome ("Completed" / "Failed - <reason>" / "Stopped"). Additive to
    # `progress`, whose texts are unchanged.
    drone_phase = Signal(int, str)
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
        self._worker.phase.connect(self.drone_phase)
        self._worker.stop_failed.connect(self._on_stop_failed)
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

    def start(self, assignments: list[tuple[DroneConfig, list[LatLon]]], standalone_dir: str,
              shared_renode: bool = False) -> None:
        """`shared_renode` boots every drone as a machine in ONE Renode
        process instead of one Renode per drone."""
        self._plan = _FleetPlan(assignments=list(assignments), standalone_dir=standalone_dir,
                                shared_renode=shared_renode)
        self._outcomes = {d.sysid: DroneOutcome(d.sysid, d.name) for d, _ in assignments}
        self._latest.clear()
        self._ended = set()
        self._stopping = False
        self._active = True
        self._started_at = time.monotonic()
        if shared_renode:
            self.progress.emit(
                f"Booting a fleet of {len(assignments)} drones in one shared Renode (low memory) - "
                "the missions start once every drone is armable ..."
            )
        else:
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
            self.drone_phase.emit(outcome.sysid, f"Not flown - fleet did not boot: {message}")
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
            service.progress.connect(lambda text, s=sysid: self.drone_phase.emit(s, text.removeprefix("[FC] ")))
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
        for sysid, reason in self._worker.dead_drones().items():
            outcome = self._outcomes.get(sysid)
            if outcome is None or outcome.state != "flying":
                continue
            self._mark(sysid, "failed", reason)
            self._worker.stop_one(sysid)   # its physics sidecar too; in a shared Renode, halt its machine
            self._flights[sysid].abort()   # its flight then reports finished -> _on_drone_ended

    def _on_stop_failed(self, sysid: int, why: str) -> None:
        """A failed drone's machine could not be confirmed halted in the
        shared Renode. It may still be executing on frozen physics next to
        the drones that are flying, which nothing has been tested against -
        so the fleet is not left running on a guess: say so loudly and stop
        every drone that is still flying."""
        if not self._active:
            return
        outcome = self._outcomes.get(sysid)
        name = outcome.name if outcome is not None else f"SYSID {sysid}"
        self.progress.emit(f"ERROR: {name} (SYSID {sysid}) COULD NOT BE STOPPED - {why}")
        if outcome is not None:
            outcome.reason = f"{outcome.reason}; and it could not be halted" if outcome.reason else "could not be halted"
            self.drone_phase.emit(sysid, f"Failed - {outcome.reason}")
        flying = [s for s, o in self._outcomes.items() if o.state == "flying"]
        if flying:
            self.progress.emit(
                f"Stopping the other {len(flying)} drone(s): the fleet is not flown on with a machine "
                "that may still be running.")
        for other in flying:
            self._mark(other, "failed", f"fleet stopped because {name} (SYSID {sysid}) could not be halted")
            self._flights[other].abort()   # its flight then reports finished -> _on_drone_ended

    def _mark(self, sysid: int, state: str, reason: str = "") -> None:
        outcome = self._outcomes[sysid]
        outcome.state, outcome.reason = state, reason
        self.progress.emit(f"{outcome.name} (SYSID {sysid}): {state}" + (f" - {reason}" if reason else ""))
        self.drone_phase.emit(sysid, {"completed": "Completed", "stopped": "Stopped"}.get(state, f"Failed - {reason}"))

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
