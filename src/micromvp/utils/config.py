from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple


RGB_PATH_COLORS = [
    (0, 0, 0),
    (1, 0, 103),
    (213, 255, 0),
    (255, 0, 86),
    (158, 0, 142),
    (14, 76, 161),
    (255, 229, 2),
    (0, 95, 57),
    (0, 255, 0),
    (149, 0, 58),
]


@dataclass(slots=True)
class Boundary:
    left: float
    right: float
    top: float
    bottom: float

    @property
    def width(self) -> float:
        return self.right - self.left

    @property
    def height(self) -> float:
        return self.bottom - self.top


@dataclass(slots=True)
class AppConfig:
    sim: bool = True
    v_max: float = 1.0
    sim_speed: float = 100.0
    wheel_base: float = 30.0
    tag_ratio: float = 1.0
    car_heading_offset_deg: float = 90.0
    target_points_per_sec: float = 6.0

    container_size: Tuple[int, int] = (1420, 780)
    painter_size: Tuple[int, int] = (1280, 720)
    spacer: int = 8

    zmq_endpoint: str = "tcp://localhost:5556"

    # list of (car_id, tag_id)
    car_info: List[Tuple[int, int]] = field(default_factory=lambda: [(i, i) for i in range(1, 6)])

    def boundary(self) -> Boundary:
        margin = 2 * self.wheel_base
        left = margin
        right = self.painter_size[0] - margin
        top = margin
        bottom = self.painter_size[1] - margin
        return Boundary(left=left, right=right, top=top, bottom=bottom)
