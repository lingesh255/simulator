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

    @Slot(str, int)
    def run(self, standalone_dir: str, port: int) -> None:
        try:
            self.progress.emit(f"Starting Renode from {standalone_dir} ...")
            self._launcher = RenodeLauncher(standalone_dir, port=port)
            connection_string = self._launcher.start()
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

    _start_requested = Signal(str, int)

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

    def start_async(self, standalone_dir: str, port: int = 5762) -> None:
        self._start_requested.emit(standalone_dir, port)

    def stop(self) -> None:
        """Synchronous - safe to call from closeEvent during app shutdown."""
        self._worker.stop()
