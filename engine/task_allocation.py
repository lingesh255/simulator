"""Runtime state for the four-drone Task-Allocation mission."""
from dataclasses import dataclass, field
from typing import Optional

from contracts.gui_orchestration import FaultType, LatLon


@dataclass
class TaskAllocation:
    source: LatLon
    destination: LatLon
    active: int
    standby: list[int]
    eligible: set[int]
    pending: Optional[int] = None
    pending_since: float = 0.0
    generation: int = 0
    data: dict = field(default_factory=dict)
    retired: set[int] = field(default_factory=set)
    faults: dict[int, tuple[FaultType, Optional[float]]] = field(default_factory=dict)
    done: bool = False
    winner: Optional[int] = None
    message: str = ""
