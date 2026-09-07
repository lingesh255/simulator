"""Flight Log panel: a plain, scrolling text log of where each drone is as
it travels (timestamped lat / lon / altitude / status per telemetry
batch), plus mission-planner events and the solved PDDL action sequence.

Replaces the old Global State Matrix + Drone Inspector: same telemetry,
shown as log lines instead of a table.
"""
from __future__ import annotations

import time

from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from contracts.gui_orchestration import DroneTelemetry

MAX_LOG_LINES = 5000


class FlightLogPanel(QWidget):
    """Read-only running log of drone movement and planner activity."""

    def __init__(self, parent=None):
        super().__init__(parent)

        self.view = QPlainTextEdit()
        self.view.setReadOnly(True)
        self.view.setMaximumBlockCount(MAX_LOG_LINES)
        self.view.setFont(QFont("Consolas", 9))
        self.view.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.view.setPlaceholderText(
            "Drone travel log - one timestamped line per drone per telemetry "
            "update (lat / lon / altitude / status) once a flight is running."
        )

        self.clear_btn = QPushButton("Clear")
        self.clear_btn.clicked.connect(self.clear_log)

        header_row = QHBoxLayout()
        header_row.addStretch(1)
        header_row.addWidget(self.clear_btn)

        box = QGroupBox("Flight Log  (lat / lon as drones travel)")
        box_layout = QVBoxLayout(box)
        box_layout.addLayout(header_row)
        box_layout.addWidget(self.view, stretch=1)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(box)

    # ---- Writing ----

    @staticmethod
    def _stamp() -> str:
        return time.strftime("%H:%M:%S")

    def log_line(self, text: str) -> None:
        """Append a raw line exactly as given (no timestamp) - used for the
        continuation lines of a multi-line entry such as a plan's steps."""
        self.view.appendPlainText(text)

    def log_event(self, text: str) -> None:
        """Append a timestamped one-off event (run started/stopped, plan
        ready, ...)."""
        self.view.appendPlainText(f"[{self._stamp()}] === {text}")

    def log_batch(self, drones: list[DroneTelemetry]) -> None:
        """Append one line per drone in this telemetry batch."""
        stamp = self._stamp()
        for drone in sorted(drones, key=lambda d: d.sysid):
            status = getattr(drone.status, "value", drone.status)
            self.view.appendPlainText(
                f"[{stamp}] SYSID {drone.sysid:<4} "
                f"lat={drone.lat:10.5f}  lon={drone.lon:10.5f}  "
                f"alt={drone.altitude_m:7.1f} m  "
                f"bat={drone.battery_pct:3.0f}%  {status}"
            )

    def clear_log(self) -> None:
        self.view.clear()
