"""Central theme definitions and the app-wide ThemeManager (SRS-adjacent
styling pass): single source of truth for colors used by the PySide6 widget
tree (via generated QSS), the hand-painted artificial horizon (QPainter,
does not see QSS), and the QML map view (via a bridge object exposed to the
QML engine's context).

Panels connect to `theme_manager.theme_changed` to update anything that
isn't covered by the global QSS cascade (per-widget `setStyleSheet()` calls,
QPainter-drawn widgets, QML bindings) live, without an app restart.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from PySide6.QtCore import Property, QObject, Signal

from contracts.gui_orchestration import AppSettings
from services.storage import save_app_settings

ThemeName = Literal["light", "dark"]

DEFAULT_THEME: ThemeName = "dark"  # a live-telemetry dashboard reads better on dark by default


@dataclass(frozen=True)
class Palette:
    """One theme's full color set. Every color the GUI needs comes from
    here - nothing should be hardcoded in a panel, the artificial horizon,
    or the QML map chrome.
    """

    name: ThemeName

    # Base surfaces / text
    window_bg: str
    panel_bg: str
    text_primary: str
    text_secondary: str
    border: str

    # Accent: buttons, selected rows/items, the artificial horizon's pitch
    # ladder + aircraft symbol, map path lines.
    accent: str
    accent_text: str  # text/foreground color to use *on* an accent-colored surface

    # Semantic status colors - meaning is shared with fault_injection.py's
    # and telemetry_dashboard.py's existing red/green/amber usage, and with
    # the map's fault/in-flight marker colors. Kept close to the original
    # hardcoded values so existing meaning ("red = critical", "green =
    # nominal", "amber = caution/restricted") carries over unchanged.
    nominal: str
    warning: str
    critical: str
    fault_row_bg: str  # table-row highlight for an active fault - see telemetry_dashboard.py

    # Artificial horizon (gui/artificial_horizon.py) - QPainter, not QSS.
    horizon_sky: str
    horizon_ground: str
    horizon_line: str
    horizon_bezel: str


LIGHT = Palette(
    name="light",
    window_bg="#f2f3f5",
    panel_bg="#ffffff",
    text_primary="#202124",
    text_secondary="#5f6368",
    border="#d0d3d8",
    accent="#c8790d",
    accent_text="#1a1a1a",
    nominal="#2ecc71",
    warning="#f39c12",
    critical="#e74c3c",
    fault_row_bg="#f8d7da",
    horizon_sky="#3d85c6",
    horizon_ground="#7f4a26",
    horizon_line="#ffffff",
    horizon_bezel="#202124",
)

DARK = Palette(
    name="dark",
    window_bg="#1e1f22",
    panel_bg="#2b2d31",
    text_primary="#e8e6e3",
    text_secondary="#9aa0a6",
    border="#44464b",
    accent="#f1c40f",
    accent_text="#1a1a1a",
    nominal="#2ecc71",
    warning="#f39c12",
    critical="#ff6b5b",
    fault_row_bg="#5c2020",
    horizon_sky="#25415a",
    horizon_ground="#513017",
    horizon_line="#e8e6e3",
    horizon_bezel="#9aa0a6",
)

_PALETTES: dict[ThemeName, Palette] = {"light": LIGHT, "dark": DARK}


class ThemeManager(QObject):
    """App-wide theme state. One instance (`theme_manager`, below) is shared
    by the whole GUI; panels connect to `theme_changed` to restyle anything
    the global QSS cascade doesn't reach on its own.
    """

    theme_changed = Signal(str)  # new theme name ("light" | "dark")

    def __init__(self, initial: ThemeName = DEFAULT_THEME) -> None:
        super().__init__()
        self._current: ThemeName = initial

    def current_theme(self) -> ThemeName:
        return self._current

    def palette(self) -> Palette:
        return _PALETTES[self._current]

    def set_theme(self, name: ThemeName) -> None:
        if name not in _PALETTES or name == self._current:
            return
        self._current = name
        save_app_settings(AppSettings(theme=name))
        self.theme_changed.emit(name)


theme_manager = ThemeManager()


class ThemeBridge(QObject):
    """Exposes the active palette's map-relevant colors to QML (gui/map_viewer.py
    sets one of these as a context property; qml/Map.qml binds to its
    properties instead of hardcoding colors). Each property's NOTIFY signal
    fires on `theme_manager.theme_changed`, so QML bindings update live.
    """

    colorsChanged = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        theme_manager.theme_changed.connect(lambda _name: self.colorsChanged.emit())

    def _get(self, field: str) -> str:
        return getattr(theme_manager.palette(), field)

    accent = Property(str, lambda self: self._get("accent"), notify=colorsChanged)
    nominal = Property(str, lambda self: self._get("nominal"), notify=colorsChanged)
    warning = Property(str, lambda self: self._get("warning"), notify=colorsChanged)
    critical = Property(str, lambda self: self._get("critical"), notify=colorsChanged)
    panelBg = Property(str, lambda self: self._get("panel_bg"), notify=colorsChanged)
    borderColor = Property(str, lambda self: self._get("border"), notify=colorsChanged)


def current_theme() -> ThemeName:
    return theme_manager.current_theme()


def current_palette() -> Palette:
    return theme_manager.palette()


def set_theme(name: ThemeName) -> None:
    theme_manager.set_theme(name)


def build_stylesheet(palette: Palette) -> str:
    """Qt stylesheet applied once at the QApplication level (see main.py) so
    it cascades to every standard PySide6 widget automatically."""
    return f"""
    QWidget {{
        background-color: {palette.window_bg};
        color: {palette.text_primary};
        selection-background-color: {palette.accent};
        selection-color: {palette.accent_text};
    }}

    QMainWindow, QDialog {{
        background-color: {palette.window_bg};
    }}

    QGroupBox {{
        background-color: {palette.panel_bg};
        border: 1px solid {palette.border};
        border-radius: 4px;
        margin-top: 10px;
        padding-top: 6px;
        font-weight: bold;
    }}
    QGroupBox::title {{
        subcontrol-origin: margin;
        left: 8px;
        padding: 0 4px;
        color: {palette.text_secondary};
    }}

    QListWidget, QTableWidget, QPlainTextEdit, QLineEdit,
    QComboBox, QSpinBox, QDoubleSpinBox {{
        background-color: {palette.panel_bg};
        color: {palette.text_primary};
        border: 1px solid {palette.border};
        border-radius: 3px;
        selection-background-color: {palette.accent};
        selection-color: {palette.accent_text};
    }}
    QHeaderView::section {{
        background-color: {palette.panel_bg};
        color: {palette.text_secondary};
        border: none;
        border-bottom: 1px solid {palette.border};
        padding: 4px;
    }}

    QPushButton {{
        background-color: {palette.panel_bg};
        color: {palette.text_primary};
        border: 1px solid {palette.border};
        border-radius: 4px;
        padding: 4px 10px;
    }}
    QPushButton:hover {{
        border-color: {palette.accent};
    }}
    QPushButton:pressed {{
        background-color: {palette.accent};
        color: {palette.accent_text};
    }}
    QPushButton:checked {{
        background-color: {palette.accent};
        color: {palette.accent_text};
        border-color: {palette.accent};
    }}
    QPushButton:disabled {{
        color: {palette.text_secondary};
    }}

    QToolBar {{
        background-color: {palette.panel_bg};
        border-bottom: 1px solid {palette.border};
        spacing: 4px;
    }}
    QToolButton:checked {{
        background-color: {palette.accent};
        color: {palette.accent_text};
        border-radius: 3px;
    }}

    QStatusBar {{
        background-color: {palette.panel_bg};
        border-top: 1px solid {palette.border};
    }}

    QDockWidget {{
        color: {palette.text_primary};
        titlebar-close-icon: none;
    }}
    QDockWidget::title {{
        background-color: {palette.panel_bg};
        border-bottom: 1px solid {palette.border};
        padding: 4px;
    }}

    QSplitter::handle {{
        background-color: {palette.border};
    }}

    QCheckBox, QLabel {{
        background: transparent;
    }}
    """
