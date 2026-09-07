"""Simple artificial-horizon indicator for the Individual Drone Inspector
(SRS §3.2.3), driven by roll/pitch decoded from the drone's MAVLink ATTITUDE
telemetry.
"""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QBrush, QColor, QPainter, QPen
from PySide6.QtWidgets import QWidget

from gui.theme import theme_manager


class ArtificialHorizon(QWidget):
    """QPainter-drawn, so it does not pick up the app's QSS stylesheet -
    colors are pulled directly from `gui.theme`'s active palette instead,
    and it repaints on `theme_manager.theme_changed` so toggling the theme
    updates it immediately."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(160, 160)
        self._roll_deg = 0.0
        self._pitch_deg = 0.0
        theme_manager.theme_changed.connect(lambda _name: self.update())

    def set_attitude(self, roll_deg: float, pitch_deg: float) -> None:
        self._roll_deg = roll_deg
        self._pitch_deg = pitch_deg
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt override
        palette = theme_manager.palette()

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(palette.panel_bg))
        side = min(self.width(), self.height())
        painter.setViewport((self.width() - side) // 2, (self.height() - side) // 2, side, side)
        painter.setWindow(-100, -100, 200, 200)

        painter.setClipRect(-100, -100, 200, 200)
        painter.save()
        painter.rotate(-self._roll_deg)
        pitch_offset = max(-80, min(80, self._pitch_deg * 2))
        painter.translate(0, pitch_offset)

        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(palette.horizon_sky)))
        painter.drawRect(-200, -400, 400, 400)
        painter.setBrush(QBrush(QColor(palette.horizon_ground)))
        painter.drawRect(-200, 0, 400, 400)
        painter.setPen(QPen(QColor(palette.horizon_line), 2))
        painter.drawLine(-200, 0, 200, 0)
        painter.restore()

        painter.setClipping(False)
        painter.setPen(QPen(QColor(palette.accent), 3))
        painter.drawLine(-30, 0, -10, 0)
        painter.drawLine(10, 0, 30, 0)
        painter.drawLine(0, -8, 0, 8)

        painter.setPen(QPen(QColor(palette.horizon_bezel), 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(-99, -99, 198, 198)
