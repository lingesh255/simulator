"""Entry point for Module 1 - GUI Application (SRS §3)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from PySide6 import QtLocation, QtPositioning  # noqa: F401  (registers QML plugins)
from PySide6.QtWidgets import QApplication

from gui.main_window import MainWindow
from gui.theme import build_stylesheet, theme_manager
from services.storage import load_app_settings


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
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
