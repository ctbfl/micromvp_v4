from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple


Point = Tuple[float, float]


@dataclass(slots=True)
class CarState:
    car_id: int
    tag_id: int
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    l_speed: float = 0.0
    r_speed: float = 0.0
    path: List[Point] = field(default_factory=list)


@dataclass(slots=True)
class WorldState:
    cars: List[CarState]
    targets: Dict[int, Point] = field(default_factory=dict)
