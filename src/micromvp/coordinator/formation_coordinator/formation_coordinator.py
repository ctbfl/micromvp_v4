"""
Formation Coordinator - Multi-robot formation following patterns.

This coordinator manages multiple robots following a formation pattern:
- Supports circle and figure-8 patterns
- Configurable number of active cars (filters from environment)
- Moving target points along the pattern
- Speed controls for both target point movement and car speed

GUI Controls:
- Pattern selection (circle/figure-8)
- Number of active cars
- Point speed (target movement speed)
- Car speed (maximum robot speed)
"""
from __future__ import annotations

import math
import random
import time
from typing import Dict, List, Any, Optional, Tuple

from micromvp.controller.base import Controller
from micromvp.controller.target_follow_controller import TargetFollowController
from micromvp.coordinator.base import Coordinator
from micromvp.core.models import (
    Action,
    CarState,
    RobotObservation,
    WorkspaceConfig,
)
from micromvp.utils.config import Boundary


Point = Tuple[float, float]


class FormationCoordinator(Coordinator):
    """
    Coordinator for multi-robot formation control.

    Manages robots following a pattern with configurable:
    - Pattern shape (circle or figure-8)
    - Pattern sizes (circle_scale, figure8_scale)
    - Boundary margin for pattern placement
    - Number of active cars
    - Target point movement speed
    - Robot movement speed

    Usage:
        # Basic usage with default pattern sizes
        coordinator = FormationCoordinator(ws_config, controllers, env)

        # Custom pattern sizes (larger patterns for more car space)
        coordinator = FormationCoordinator(
            ws_config, controllers, env,
            circle_scale=0.8,      # 80% of boundary size (2x default)
            figure8_scale=1.05,    # 105% of boundary size (3x default)
            boundary_margin=0.1,   # 10% margin (smaller margin = larger effective area)
        )

        coordinator.set_pattern("circle")
        coordinator.set_active_count(5)
        coordinator.set_point_speed(0.5)
        coordinator.set_car_speed(0.3)

        # Adjust pattern sizes at runtime
        coordinator.set_circle_scale(0.6)
        coordinator.set_figure8_scale(0.8)

        # Register GUI callbacks
        gui.register_callback("set_pattern", coordinator.on_pattern_change)
        gui.register_callback("set_car_count", coordinator.on_car_count_change)
        gui.register_callback("set_point_speed", coordinator.set_point_speed)
        gui.register_callback("set_car_speed", coordinator.set_car_speed)
    """

    def __init__(
        self,
        ws_config: WorkspaceConfig,
        controllers: Dict[int, Controller],
        env: Any = None,  # SimEnv for position reset
        initial_pattern: str = "circle",
        initial_active_count: Optional[int] = None,
        initial_point_speed: float = 100.0,
        initial_car_speed: float = 0.3,
        circle_scale: float = 0.4,
        figure8_scale: float = 0.35,
        boundary_margin: float = 0.15,
        spawn_near_target: bool = False,
    ) -> None:
        """
        Initialize formation coordinator.

        Args:
            ws_config: Workspace configuration
            controllers: Dict mapping robot_id to Controller (should be TargetFollowController)
            env: Environment reference for position reset (SimEnv)
            initial_pattern: Starting pattern ("circle" or "figure8")
            initial_active_count: Number of active cars (default: all)
            initial_point_speed: Target movement speed in workspace units/second (default: 100.0)
            initial_car_speed: Initial robot movement speed [0, 1] (default: 0.3)
            circle_scale: Scale factor for circle radius relative to boundary (default: 0.4)
            figure8_scale: Scale factor for figure-8 size relative to boundary (default: 0.35)
            boundary_margin: Margin ratio for pattern boundary (default: 0.15)
            spawn_near_target: If True, spawn cars near their target points when
                               reinitializing (e.g., on car count change). Default: False.
        """
        super().__init__(ws_config, controllers)

        self._env = env
        self._all_car_ids = list(controllers.keys())
        self._max_cars = len(self._all_car_ids)

        # Active count (how many cars are visible/controlled)
        self._active_count = initial_active_count or self._max_cars

        # Pattern configuration
        self._circle_scale = circle_scale
        self._figure8_scale = figure8_scale
        self._boundary_margin = boundary_margin

        # Pattern state
        self._pattern_type = initial_pattern
        self._pattern_path: List[Point] = []
        self._pattern_length = 0.0  # Total arc length
        self._cumulative_arc_lengths: List[float] = []  # Arc length at each point

        # Target positions as arc length along the path (in real units)
        # This ensures uniform speed regardless of point density
        self._target_arc_positions: Dict[int, float] = {}

        # Speed controls
        self._point_speed = initial_point_speed  # Workspace units per second
        self._car_speed = initial_car_speed

        # Spawn behavior - whether to place cars near targets on reinitialization
        self._spawn_near_target = spawn_near_target

        # Timing for target movement
        self._last_update_time: Optional[float] = None

        # Initialize
        self._generate_pattern()
        self._distribute_targets()
        self._initialize_car_positions()

    @property
    def active_count(self) -> int:
        """Get the number of active cars."""
        return self._active_count

    @property
    def pattern_type(self) -> str:
        """Get the current pattern type."""
        return self._pattern_type

    @property
    def point_speed(self) -> float:
        """Get the target point movement speed."""
        return self._point_speed

    @property
    def car_speed(self) -> float:
        """Get the robot movement speed."""
        return self._car_speed

    @property
    def circle_scale(self) -> float:
        """Get the circle pattern scale factor."""
        return self._circle_scale

    @property
    def figure8_scale(self) -> float:
        """Get the figure-8 pattern scale factor."""
        return self._figure8_scale

    @property
    def boundary_margin(self) -> float:
        """Get the boundary margin ratio."""
        return self._boundary_margin

    @property
    def spawn_near_target(self) -> bool:
        """Get whether cars spawn near their targets on reinitialization."""
        return self._spawn_near_target

    def set_spawn_near_target(self, enabled: bool) -> None:
        """
        Set whether cars spawn near their target points on reinitialization.

        When enabled, cars are placed within a small area (wheel_base x wheel_base)
        around their target point with random orientation.
        When disabled, cars are placed randomly across the workspace.

        Args:
            enabled: True to spawn near targets, False for random placement
        """
        self._spawn_near_target = bool(enabled)

    def set_circle_scale(self, scale: float, regenerate: bool = True) -> None:
        """
        Set the circle pattern scale factor.

        Args:
            scale: Scale factor relative to boundary size (e.g., 0.4 = 40% of boundary)
            regenerate: Whether to regenerate the pattern immediately (default: True)
        """
        self._circle_scale = max(0.1, scale)
        if regenerate and self._pattern_type == "circle":
            self._generate_pattern()
            self._distribute_targets()

    def set_figure8_scale(self, scale: float, regenerate: bool = True) -> None:
        """
        Set the figure-8 pattern scale factor.

        Args:
            scale: Scale factor relative to boundary size (e.g., 0.35 = 35% of boundary)
            regenerate: Whether to regenerate the pattern immediately (default: True)
        """
        self._figure8_scale = max(0.1, scale)
        if regenerate and self._pattern_type == "figure8":
            self._generate_pattern()
            self._distribute_targets()

    def set_boundary_margin(self, margin: float, regenerate: bool = True) -> None:
        """
        Set the boundary margin ratio.

        Args:
            margin: Margin ratio (e.g., 0.15 = 15% margin on each side)
            regenerate: Whether to regenerate the pattern immediately (default: True)
        """
        self._boundary_margin = max(0.0, min(0.4, margin))
        if regenerate:
            self._generate_pattern()
            self._distribute_targets()

    def _get_active_car_ids(self) -> List[int]:
        """Get the list of active car IDs (first N cars)."""
        return self._all_car_ids[: self._active_count]

    def _generate_pattern(self) -> None:
        """Generate the pattern path based on current pattern type."""
        bound = Boundary.from_workspace_config(self._ws_config, margin_ratio=self._boundary_margin)

        if self._pattern_type == "circle":
            self._pattern_path = self._generate_circle_path(bound)
        elif self._pattern_type == "figure8":
            self._pattern_path = self._generate_figure8_path(bound)
        else:
            self._pattern_path = self._generate_circle_path(bound)

        # Calculate cumulative arc lengths for uniform speed parameterization
        self._cumulative_arc_lengths, self._pattern_length = self._compute_cumulative_arc_lengths(self._pattern_path)

    def _generate_circle_path(self, bound: Boundary, num_points: int = 100) -> List[Point]:
        """Generate a circular path."""
        radius = min(bound.width, bound.height) * self._circle_scale
        center = bound.center
        path = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            path.append((x, y))
        return path

    def _generate_figure8_path(self, bound: Boundary, num_points: int = 100) -> List[Point]:
        """Generate a figure-8 (lemniscate) path."""
        center = bound.center
        # Scale to fit in boundary
        scale = min(bound.width, bound.height) * self._figure8_scale
        path = []
        for i in range(num_points):
            t = 2 * math.pi * i / num_points
            # Lemniscate of Bernoulli parametric equations
            denom = 1 + math.sin(t) ** 2
            x = center[0] + scale * math.cos(t) / denom
            y = center[1] + scale * math.sin(t) * math.cos(t) / denom
            path.append((x, y))
        return path

    def _compute_cumulative_arc_lengths(self, path: List[Point]) -> Tuple[List[float], float]:
        """
        Compute cumulative arc lengths for each point in the path.

        Returns:
            Tuple of (cumulative_lengths, total_length)
            - cumulative_lengths[i] = arc length from start to point i
            - total_length = total arc length of closed path
        """
        if len(path) < 2:
            return [0.0], 0.0

        cumulative = [0.0]  # First point is at arc length 0
        total = 0.0

        for i in range(len(path)):
            p1 = path[i]
            p2 = path[(i + 1) % len(path)]
            segment_length = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
            total += segment_length
            if i < len(path) - 1:
                cumulative.append(total)

        return cumulative, total

    def _distribute_targets(self) -> None:
        """Distribute target positions equally along the pattern by arc length."""
        active_ids = self._get_active_car_ids()
        n = len(active_ids)
        if n == 0:
            self._target_arc_positions = {}
            return

        # Distribute equally by arc length (real distance along path)
        for i, car_id in enumerate(active_ids):
            self._target_arc_positions[car_id] = (i * self._pattern_length / n) % self._pattern_length

    def _get_target_point(self, car_id: int) -> Optional[Point]:
        """Get the current target point for a car by interpolating on path using arc length."""
        if car_id not in self._target_arc_positions:
            return None

        arc_pos = self._target_arc_positions[car_id]
        path = self._pattern_path
        if not path or not self._cumulative_arc_lengths:
            return None

        return self._arc_length_to_point(arc_pos)

    def _arc_length_to_point(self, arc_length: float) -> Point:
        """
        Convert an arc length position to a point on the path.

        Uses binary search to find the segment, then linear interpolation.

        Args:
            arc_length: Distance along the path from the start

        Returns:
            Interpolated (x, y) point on the path
        """
        path = self._pattern_path
        cumulative = self._cumulative_arc_lengths

        # Wrap arc_length to [0, total_length)
        arc_length = arc_length % self._pattern_length

        # Binary search to find the segment containing this arc length
        # cumulative[i] is the arc length at the start of segment i
        left, right = 0, len(cumulative) - 1
        while left < right:
            mid = (left + right + 1) // 2
            if cumulative[mid] <= arc_length:
                left = mid
            else:
                right = mid - 1

        idx = left
        next_idx = (idx + 1) % len(path)

        # Calculate segment length
        p1 = path[idx]
        p2 = path[next_idx]
        segment_length = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

        # Calculate fraction within segment
        if segment_length > 1e-9:
            frac = (arc_length - cumulative[idx]) / segment_length
        else:
            frac = 0.0

        # Clamp fraction to [0, 1]
        frac = max(0.0, min(1.0, frac))

        # Linear interpolation
        x = p1[0] + frac * (p2[0] - p1[0])
        y = p1[1] + frac * (p2[1] - p1[1])
        return (x, y)

    def _advance_targets(self, dt: float) -> None:
        """
        Advance all target positions along the pattern by real distance.

        The point_speed is in real units per second, so targets move at
        uniform speed regardless of path point density.
        """
        if not self._pattern_path or dt <= 0 or self._pattern_length <= 0:
            return

        # Distance to advance = speed (units/sec) * time (sec)
        distance_to_advance = self._point_speed * dt

        for car_id in self._target_arc_positions:
            self._target_arc_positions[car_id] = (
                self._target_arc_positions[car_id] + distance_to_advance
            ) % self._pattern_length

    def _initialize_car_positions(self) -> None:
        """
        Initialize car positions.

        Behavior depends on `spawn_near_target` flag:
        - False (default): Random placement across workspace, collision-free
        - True: Place cars near their target points within wheel_base x wheel_base area
        """
        if self._env is None:
            return

        active_ids = self._get_active_car_ids()
        if not active_ids:
            return

        if self._spawn_near_target:
            self._initialize_cars_near_targets(active_ids)
        else:
            self._initialize_cars_random(active_ids)

    def _initialize_cars_random(self, active_ids: List[int]) -> None:
        """Initialize car positions randomly but collision-free across the workspace."""
        car_size = max(self._ws_config.car_width, self._ws_config.car_height)
        min_distance = car_size * 1.5  # Minimum distance between cars

        # Bounds for placement
        margin = car_size
        x_min, x_max = margin, self._ws_config.width - margin
        y_min, y_max = margin, self._ws_config.height - margin

        positions: List[Tuple[float, float]] = []
        max_attempts = 100

        for car_id in active_ids:
            placed = False
            for _ in range(max_attempts):
                x = random.uniform(x_min, x_max)
                y = random.uniform(y_min, y_max)

                # Check collision with existing positions
                collision = False
                for px, py in positions:
                    dist = math.sqrt((x - px) ** 2 + (y - py) ** 2)
                    if dist < min_distance:
                        collision = True
                        break

                if not collision:
                    positions.append((x, y))
                    theta = random.uniform(0, 360)
                    # Set pose in environment
                    if hasattr(self._env, "set_pose"):
                        self._env.set_pose(car_id, x, y, theta)
                    placed = True
                    break

            if not placed:
                # Fallback: place anyway
                x = random.uniform(x_min, x_max)
                y = random.uniform(y_min, y_max)
                theta = random.uniform(0, 360)
                positions.append((x, y))
                if hasattr(self._env, "set_pose"):
                    self._env.set_pose(car_id, x, y, theta)

    def _initialize_cars_near_targets(self, active_ids: List[int]) -> None:
        """
        Initialize car positions near their target points.

        Each car is placed within a wheel_base x wheel_base area centered on its
        target point, with random orientation. This allows cars to start following
        their targets smoothly without needing to travel far.
        """
        wheel_base = self._ws_config.wheel_base
        spawn_radius = wheel_base / 2.0  # Half of wheel_base in each direction

        for car_id in active_ids:
            target = self._get_target_point(car_id)
            if target is None:
                # No target, skip this car
                continue

            target_x, target_y = target

            # Random offset within wheel_base x wheel_base area
            offset_x = random.uniform(-spawn_radius, spawn_radius)
            offset_y = random.uniform(-spawn_radius, spawn_radius)

            x = target_x + offset_x
            y = target_y + offset_y

            # Clamp to workspace bounds
            margin = max(self._ws_config.car_width, self._ws_config.car_height) / 2
            x = max(margin, min(self._ws_config.width - margin, x))
            y = max(margin, min(self._ws_config.height - margin, y))

            # Random orientation
            theta = random.uniform(0, 360)

            # Set pose in environment
            if hasattr(self._env, "set_pose"):
                self._env.set_pose(car_id, x, y, theta)

    def process(
        self, observations: Dict[int, RobotObservation]
    ) -> Dict[int, Action]:
        """
        Process observations and return actions.

        1. Advance target positions based on elapsed time
        2. Update controller targets
        3. Filter observations to active cars only
        4. Collect actions from controllers

        Args:
            observations: Dict mapping robot_id to RobotObservation

        Returns:
            Dict mapping robot_id to Action
        """
        # Advance targets based on time
        now = time.time()
        if self._last_update_time is not None:
            dt = now - self._last_update_time
            self._advance_targets(dt)
        self._last_update_time = now

        # Get active car IDs
        active_ids = self._get_active_car_ids()

        # Update controller targets and speeds
        for car_id in active_ids:
            controller = self._controllers.get(car_id)
            if controller is None:
                continue

            target = self._get_target_point(car_id)
            if target and isinstance(controller, TargetFollowController):
                controller.set_target(target)
                controller.set_max_speed(self._car_speed)

        # Process only active cars
        actions: Dict[int, Action] = {}
        for car_id in active_ids:
            controller = self._controllers.get(car_id)
            if controller is None:
                continue

            if car_id in observations:
                actions[car_id] = controller.step(observations[car_id])
            else:
                actions[car_id] = Action.stop()

        # Stop inactive cars
        for car_id in self._all_car_ids:
            if car_id not in active_ids:
                actions[car_id] = Action.stop()

        return actions

    def gather_car_state(self) -> List[CarState]:
        """
        Gather car states for active cars only.

        Returns:
            List of CarState objects for active cars
        """
        active_ids = self._get_active_car_ids()
        states = []
        for car_id in active_ids:
            controller = self._controllers.get(car_id)
            if controller is not None:
                states.append(controller.car_state)
        return states

    def get_additional_drawings(self) -> List[Dict[str, Any]]:
        """
        Get additional drawings for GUI.

        Draws:
        - Pattern path (always visible)
        - Target points for each active car
        - Lines from each car to its target

        Returns:
            List of drawing commands
        """
        drawings = []

        # Draw the pattern path
        if self._pattern_path and len(self._pattern_path) >= 2:
            # Close the path by adding first point at end
            closed_path = list(self._pattern_path) + [self._pattern_path[0]]
            drawings.append({
                "uuid": "pattern_path",
                "type": "path",
                "points": closed_path,
                "color": "#4444FF",  # Blue pattern
                "width": 2,
            })

        # Draw target points and lines for active cars
        active_ids = self._get_active_car_ids()
        for car_id in active_ids:
            target = self._get_target_point(car_id)
            if target is None:
                continue

            controller = self._controllers.get(car_id)
            if controller is None:
                continue

            state = controller.car_state

            # Draw target point
            drawings.append({
                "uuid": f"target_{car_id}",
                "type": "point",
                "position": target,
                "radius": 6,
                "color": "#FF4444",  # Red target
                "fill": "#FF4444",
            })

            # Draw line from car to target
            drawings.append({
                "uuid": f"target_line_{car_id}",
                "type": "line",
                "start": (state.x, state.y),
                "end": target,
                "color": "#FF444480",  # Semi-transparent red
                "width": 1,
            })

        return drawings

    # -------------------------------------------------------------------------
    # GUI Callback Methods
    # -------------------------------------------------------------------------

    def set_pattern(self, pattern: str) -> None:
        """
        Change the pattern type.

        Does NOT reset car positions, only redistributes targets.

        Args:
            pattern: "circle" or "figure8"
        """
        if pattern not in ("circle", "figure8"):
            return

        self._pattern_type = pattern
        self._generate_pattern()
        self._distribute_targets()

    def on_pattern_change(self, pattern: str) -> None:
        """GUI callback for pattern selection."""
        self.set_pattern(pattern)

    def set_active_count(self, count: int) -> None:
        """
        Change the number of active cars.

        Reinitializes car positions when count changes.

        Args:
            count: Number of cars to activate (1 to max)
        """
        new_count = max(1, min(count, self._max_cars))
        if new_count == self._active_count:
            return

        self._active_count = new_count
        self._distribute_targets()
        self._initialize_car_positions()

    def on_car_count_change(self, count: int) -> None:
        """GUI callback for car count slider."""
        self.set_active_count(count)

    def set_point_speed(self, speed: float) -> None:
        """
        Set target point movement speed in real units per second.

        The speed is the actual distance traveled per second along the path,
        ensuring uniform movement regardless of path shape or point density.

        Args:
            speed: Speed in workspace units per second (e.g., pixels/sec)
        """
        self._point_speed = max(0.0, float(speed))

    def set_car_speed(self, speed: float) -> None:
        """
        Set robot movement speed.

        Args:
            speed: Maximum wheel speed [0, 1]
        """
        self._car_speed = max(0.0, min(1.0, float(speed)))

        # Update all controllers
        for controller in self._controllers.values():
            if isinstance(controller, TargetFollowController):
                controller.set_max_speed(self._car_speed)

    def reset(self) -> None:
        """Reset coordinator and reinitialize positions."""
        super().reset()
        self._last_update_time = None
        self._distribute_targets()
        self._initialize_car_positions()
