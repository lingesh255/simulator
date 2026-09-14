"""GUI-facing: start the standalone Renode + Pixhawk6C/6X package
(engine.renode_launcher.RenodeLauncher) on its own worker thread and report
back the resulting MAVLink connection string, so the GUI thread never blocks
on the subprocess launch or the up-to-60s wait for the emulated boot to
reach a live MAVLink port.

Mirrors services.mavlink_flight_service.MavlinkFlightService's own
worker/QThread shape for consistency with the rest of this codebase.
"""
from __future__ import annotations

from PySide6.QtCore import QObject, QThread, Signal, Slot

from engine.renode_launcher import RenodeLauncher, RenodeLauncherError


class _Worker(QObject):
    ready = Signal(str)      # emits the connection string on success
    progress = Signal(str)
    failed = Signal(str)

    def __init__(self):
        super().__init__()
        self._launcher: RenodeLauncher | None = None

    @Slot(str, int, float, float)
    def run(self, standalone_dir: str, port: int, latitude_deg: float, longitude_deg: float) -> None:
        try:
            self.progress.emit(f"Starting Renode from {standalone_dir} ...")
            self._launcher = RenodeLauncher(
                standalone_dir, port=port, latitude_deg=latitude_deg, longitude_deg=longitude_deg,
            )
            connection_string = self._launcher.start()
            self.progress.emit(f"Renode ready on {connection_string} - provisioning first-boot params ...")
            # A separate, short-lived connection, closed before this
            # returns - see RenodeLauncher.start()'s docstring for why this
            # isn't folded into that same connection.
            self._launcher.provision_first_boot_params()
            self.progress.emit("Params provisioned - waiting for the vehicle to become armable ...")
            # Also a separate, short-lived connection - see
            # RenodeLauncher.wait_until_armable()'s own docstring. Without
            # this, a caller that arms and switches to AUTO immediately
            # after provisioning can hit real, confirmed PreArm failures
            # (EKF/GPS/yaw alignment not yet converged) that a single
            # early arm attempt has no way to recover from.
            self._launcher.wait_until_armable()
            self.progress.emit(f"Renode ready on {connection_string}")
            self.ready.emit(connection_string)
        except (RenodeLauncherError, TimeoutError, OSError) as exc:
            self.failed.emit(str(exc))

    def stop(self) -> None:
        if self._launcher is not None:
            self._launcher.stop()
            self._launcher = None


class RenodeLaunchService(QObject):
    """GUI-facing handle: call `start_async(standalone_dir, port)`, get
    `ready(str)` (the connection string to hand to MavlinkFlightService),
    `progress(str)`, or `failed(str)`."""

    ready = Signal(str)
    progress = Signal(str)
    failed = Signal(str)

    _start_requested = Signal(str, int, float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._thread = QThread(self)
        self._worker = _Worker()
        self._worker.moveToThread(self._thread)
        self._worker.ready.connect(self.ready)
        self._worker.progress.connect(self.progress)
        self._worker.failed.connect(self.failed)
        self._start_requested.connect(self._worker.run)
        self._thread.start()
        # A launch already in flight, tracked here (GUI thread) - not on
        # the worker. Found and fixed during a verification pass: calling
        # start_async() a second time while a launch is still running (a
        # real double-click reproduced it) queued a second run() call on
        # the same _Worker, which built a brand new RenodeLauncher and
        # overwrote self._launcher on the worker - orphaning the first
        # instance's real Renode/physics subprocesses (no reference left to
        # stop() them through) and crashing the second call with a real
        # AttributeError ('_proc' was still None on the object whichever
        # invocation's stack frame was actually running deeper down).
        # Rejected here, before ever emitting the queued signal, rather
        # than inside the worker after the damage is done.
        self._busy = False
        self.ready.connect(self._on_finished)
        self.failed.connect(self._on_finished)

    def _on_finished(self, *_args) -> None:
        self._busy = False

    def start_async(
        self,
        standalone_dir: str,
        port: int = 5762,
        latitude_deg: float | None = None,
        longitude_deg: float | None = None,
    ) -> None:
        """`latitude_deg`/`longitude_deg`, if given, spawn the vehicle
        there instead of RenodeLauncher's own confirmed Canberra default
        (e.g. to match a real mission's clicked Start point) - see
        RenodeLauncher.__init__'s own docstring/comment for why only
        lat/lon are parametrized, not altitude/heading. Resolved to
        concrete floats here, before emitting, rather than passed through
        as Optional[float]: Qt signal parameter types have to be
        concrete, and resolving on this (GUI) thread rather than the
        worker thread keeps RenodeLauncher's own default-resolution logic
        as the single source of truth either way (None here always means
        the same thing None there would).

        Callers requesting a DIFFERENT location than whatever instance is
        currently running or was last requested should call `stop()`
        first - this method does not do that itself (it may be invoked
        for the very first launch of a session, with nothing to stop).
        RenodeLauncher.start()'s own `_kill_any_stale_processes()` is a
        real, working defensive cleanup, but a confirmed real test
        (renode_firmware_guide.md §4.9.1) found relying on it ALONE, with
        no prior synchronous stop(), left a race - the still-alive
        previous instance (or its not-yet-released listening socket) was
        still there when the new one tried to bind/connect, producing a
        connection that accepted but never actually spoke MAVLink (an
        infinite "EOF on TCP socket" loop). MainWindow._restart_renode_
        after_mission() already calls stop() first for exactly this
        reason; any other caller requesting a location change must do
        the same."""
        if self._busy:
            self.failed.emit(
                "A Renode launch is already in progress - wait for it to "
                "finish or fail before launching again."
            )
            return
        self._busy = True
        if not self._thread.isRunning():
            # stop() (e.g. to force a genuinely fresh Renode process
            # between missions - see MainWindow._restart_renode_after_
            # mission()) fully stops this QThread's own event loop, not
            # just the Renode subprocess - _start_requested below would
            # otherwise queue up and never actually be delivered, silently
            # hanging forever instead of relaunching. QThread supports
            # being started again after it has fully stopped, so this
            # makes start_async() safe to call again after stop(), the
            # same way starting a fresh Renode process itself always is.
            self._thread.start()
        resolved_lat = RenodeLauncher.PHYSICS_LATITUDE_DEG if latitude_deg is None else latitude_deg
        resolved_lon = RenodeLauncher.PHYSICS_LONGITUDE_DEG if longitude_deg is None else longitude_deg
        self._start_requested.emit(standalone_dir, port, resolved_lat, resolved_lon)

    def stop(self) -> None:
        """Synchronous - safe to call from closeEvent during app shutdown.

        Pre-existing bug found and fixed during a verification pass (not
        introduced tonight - confirmed via git history, present before any
        of tonight's changes): this used to only call `self._worker.stop()`
        (tears down the Renode/physics subprocesses) but never actually
        stopped the QThread itself. On a real app close this produced a
        genuine crash - "QThread: Destroyed while thread '' is still
        running" followed by SIGABRT - reproduced directly via the GUI's
        own close path. Mirrors the already-correct pattern in
        `services.mavlink_flight_service.MavlinkFlightService.shutdown()`.
        """
        self._worker.stop()
        self._thread.quit()
        self._thread.wait(2000)
