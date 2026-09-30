"""Entry point for Module 1 - GUI Application (SRS §3)."""
import signal
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from PySide6 import QtLocation, QtPositioning  # noqa: F401  (registers QML plugins)
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from gui.main_window import MainWindow
from gui.theme import build_stylesheet, theme_manager
from services.storage import load_app_settings


def _install_signal_handlers(app: QApplication, window: MainWindow) -> QTimer:
    """Route Ctrl+C / `kill` through `window.close()`, so MainWindow.closeEvent
    stops Renode and its physics sidecar instead of orphaning them.

    Python only runs signal handlers between bytecodes and `app.exec()` sits
    in C++, so the returned timer wakes the interpreter every 200 ms to let a
    pending signal through. The caller must keep a reference to it.
    """
    closing = False

    def request_close(signum, _frame) -> None:
        nonlocal closing
        if closing:
            return
        closing = True
        print(f"[main] {signal.Signals(signum).name} received - closing the window.", flush=True)
        window.close()

    signal.signal(signal.SIGINT, request_close)
    signal.signal(signal.SIGTERM, request_close)
    pump = QTimer(app)
    pump.timeout.connect(lambda: None)
    pump.start(200)
    return pump


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName("Drone Swarm Simulator - GUI")

    # Apply the persisted theme before the window is built, so there is no
    # flash of the wrong theme on startup.
    theme_manager.set_theme(load_app_settings().theme)
    app.setStyleSheet(build_stylesheet(theme_manager.palette()))
    theme_manager.theme_changed.connect(
        lambda _name: app.setStyleSheet(build_stylesheet(theme_manager.palette()))
    )

    window = MainWindow()
    window.show()
    signal_pump = _install_signal_handlers(app, window)  # noqa: F841  (kept alive for app.exec())
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
