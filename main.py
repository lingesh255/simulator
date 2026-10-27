"""Entry point for Module 1 - GUI Application (SRS §3)."""
import os
import signal
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from PySide6 import QtLocation, QtPositioning  # noqa: F401  (registers QML plugins)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import QApplication

from gui.main_window import MainWindow
from gui.theme import build_stylesheet, theme_manager
from services.storage import load_app_settings


def _use_software_quick_without_gpu() -> None:
    """Render the map's Qt Quick scene in software when there is no GPU.

    Under WSL there is no /dev/dri, so Qt Quick's OpenGL path runs on Mesa's
    CPU renderer (llvmpipe) and every repaint of the window that hosts the map
    freezes the GUI thread for 0.5-2 s. Popups then open and close late: a
    combo-box click is processed after its popup has already timed out, and
    the list lingers. Qt Quick's software renderer has no such freezes. An
    explicit QT_QUICK_BACKEND from the user always wins.
    """
    if os.environ.get("QT_QUICK_BACKEND") or os.environ.get("QSG_RHI_BACKEND"):
        return
    if "WSL_DISTRO_NAME" in os.environ and not Path("/dev/dri").exists():
        os.environ["QT_QUICK_BACKEND"] = "software"


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
    _use_software_quick_without_gpu()  # must run before any Qt Quick item is created
    app = QApplication(sys.argv)
    app.setApplicationName("Drone Swarm Simulator - GUI")
    # Drop the fade/scroll animation on combo-box and menu popups: the
    # animated close draws the popup in a temporary extra window.
    for effect in (Qt.UI_AnimateCombo, Qt.UI_FadeMenu, Qt.UI_AnimateMenu):
        app.setEffectEnabled(effect, False)

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
