from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from micromvp.core.models import Pose, Point




@dataclass(slots=True)
class CarFeedback:
    tag_id: int
    pose: Pose


@dataclass(slots=True)
class CarCommand:
    car_id: int
    left: float
    right: float
    target: Optional[Point] = None


@dataclass(slots=True)
class FeedbackFrame:
    cars: Dict[int, Pose]


@dataclass(slots=True)
class CommandFrame:
    cars: List[CarCommand]
