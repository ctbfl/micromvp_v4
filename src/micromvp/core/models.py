from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple


Point = Tuple[float, float]


@dataclass(slots=True)
class Pose:
    """A position with orientation in world coordinates."""
    x: float
    y: float
    theta: float  # radians, always required

    def to_point(self) -> Point:
        """Convert to position-only Point."""
        return (self.x, self.y)

    @staticmethod
    def from_point(point: Point, theta: float = 0.0) -> Pose:
        """Create Pose from Point with given orientation."""
        return Pose(x=point[0], y=point[1], theta=theta)


# Type alias for semantic clarity in path planning
Waypoint = Pose


@dataclass(slots=True)
class CarState:
    car_id: int
    tag_id: int
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    l_speed: float = 0.0
    r_speed: float = 0.0
    path: List[Waypoint] = field(default_factory=list)

    @property
    def pose(self) -> Pose:
        """Current pose as a Pose object."""
        return Pose(x=self.x, y=self.y, theta=self.theta)


@dataclass(slots=True)
class WorldState:
    cars: List[CarState]
    targets: Dict[int, Waypoint] = field(default_factory=dict)


def points_to_waypoints(path: List[Point], default_theta: float = 0.0) -> List[Waypoint]:
    """Convert position-only path to waypoint path with default orientations."""
    return [Waypoint(x=p[0], y=p[1], theta=default_theta) for p in path]
