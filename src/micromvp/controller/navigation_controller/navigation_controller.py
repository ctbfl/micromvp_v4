"""
Navigation Controller - Path following with in-place rotation support.

This controller extends path following with:
1. Pure pursuit path following (same as FollowPathController)
2. In-place rotation to align with target orientation (new)

The rotation function slowly rotates the robot in-place until it aligns
with the target orientation (within +-5 degrees) and holds stable for 0.5 seconds.
"""
from __future__ import annotations

import math
import time
from typing import List, Optional, Tuple
from enum import Enum, auto

from micromvp.controller.base import Controller
from micromvp.core.models import (
    Action,
    RobotObservation,
    WorkspaceConfig,
)


Point = Tuple[float, float]


class NavigationState(Enum):
    """Navigation controller states."""
    IDLE = auto()
    FOLLOWING = auto()
    FINISHED_PATH = auto()
    ROTATING = auto()
    ROTATION_STABLE = auto()
    ROTATION_DONE = auto()


class NavigationController(Controller):
    """
    Navigation controller with path following and in-place rotation.

    This controller follows paths using pure pursuit and can rotate
    in-place to align with a target orientation.

    Status labels:
    - "IDLE": No task assigned
    - "FOLLOWING": Actively following path
    - "FINISHED": Reached end of path
    - "ROTATING": Rotating in-place to target orientation
    - "ROTATION_STABLE": Aligned and holding stable
    - "ROTATION_DONE": Rotation completed successfully

    Usage:
        controller = NavigationController(robot_id, ws_config)
        controller.set_path([(x1,y1), (x2,y2), ...])  # Follow path
        controller.rotate_to(45.0)  # Rotate to 45 degrees
        action = controller.step(observation)
    """

    # Rotation parameters
    ROTATION_TOLERANCE_DEG = 2.5  # +-5 degrees tolerance
    ROTATION_STABLE_TIME = 0.5   # 0.5 seconds stable requirement
    ROTATION_SPEED_MAX = 0.25    # Maximum rotation speed
    ROTATION_SPEED_MIN = 0.15    # Minimum rotation speed (must overcome motor deadzone)

    def __init__(
        self,
        robot_id: int,
        ws_config: WorkspaceConfig,
        lookahead_distance: Optional[float] = None,
        max_speed: float = 0.3,
        goal_tolerance: Optional[float] = None,
    ) -> None:
        """
        Initialize navigation controller.

        Args:
            robot_id: The ID of the robot this controller manages
            ws_config: Workspace configuration from environment
            lookahead_distance: Distance to look ahead on path (default: 1.5 * car_size)
            max_speed: Maximum normalized wheel speed [0, 1] (default: 0.3)
            goal_tolerance: Distance to consider goal reached (default: 0.5 * car_size)
        """
        super().__init__(robot_id, ws_config)

        # Pure pursuit parameters
        car_size = max(ws_config.car_width, ws_config.car_height)
        self._lookahead_distance = lookahead_distance or (1.5 * car_size)
        self._max_speed = min(1.0, max(0.0, max_speed))
        self._goal_tolerance = goal_tolerance or (0.5 * car_size)

        # Path state
        self._path: List[Point] = []
        self._path_index: int = 0
        self._prev_timestamp: Optional[float] = None

        # Rotation state
        self._target_theta: Optional[float] = None
        self._rotation_stable_start: Optional[float] = None
        self._nav_state = NavigationState.IDLE

        # Callback for rotation completion
        self._on_rotation_done_callback: Optional[callable] = None

    @property
    def path(self) -> List[Point]:
        """Get the current path."""
        return self._path.copy()

    @property
    def path_index(self) -> int:
        """Get the current path index (progress along path)."""
        return self._path_index

    @property
    def lookahead_distance(self) -> float:
        """Get the lookahead distance."""
        return self._lookahead_distance

    @property
    def max_speed(self) -> float:
        """Get the maximum speed."""
        return self._max_speed

    @property
    def is_following_path(self) -> bool:
        """Check if currently following a path."""
        return self._nav_state == NavigationState.FOLLOWING

    @property
    def is_rotating(self) -> bool:
        """Check if currently rotating."""
        return self._nav_state in (NavigationState.ROTATING, NavigationState.ROTATION_STABLE)

    @property
    def is_busy(self) -> bool:
        """Check if controller is busy with any task."""
        return self._nav_state not in (NavigationState.IDLE, NavigationState.FINISHED_PATH, NavigationState.ROTATION_DONE)

    def set_path(self, path: List[Point]) -> None:
        """
        Set a new path to follow.

        Args:
            path: List of (x, y) points defining the path
        """
        self._path = list(path)
        self._path_index = 0
        self._target_theta = None
        self._rotation_stable_start = None

        if self._path:
            self._nav_state = NavigationState.FOLLOWING
            self._car_state.status_label = "FOLLOWING"
            self._car_state.metadata["path"] = self._path
            self._car_state.metadata["path_index"] = self._path_index
        else:
            self._nav_state = NavigationState.IDLE
            self._car_state.status_label = "IDLE"
            self._car_state.metadata.pop("path", None)
            self._car_state.metadata.pop("path_index", None)

    def clear_path(self) -> None:
        """Clear the current path and stop."""
        self._path = []
        self._path_index = 0
        self._nav_state = NavigationState.IDLE
        self._car_state.status_label = "IDLE"
        self._car_state.metadata.pop("path", None)
        self._car_state.metadata.pop("path_index", None)
        self._car_state.metadata.pop("target_point", None)

    def rotate_to(
        self,
        target_theta: float,
        on_done: Optional[callable] = None
    ) -> None:
        """
        Start in-place rotation to target orientation.

        The robot will slowly rotate until it aligns with the target
        (within +-5 degrees) and holds stable for 0.5 seconds.

        Args:
            target_theta: Target orientation in degrees [0, 360)
            on_done: Optional callback when rotation completes
        """
        # Normalize to [0, 360)
        self._target_theta = target_theta % 360.0
        self._rotation_stable_start = None
        self._on_rotation_done_callback = on_done

        # Clear any existing path
        self._path = []
        self._path_index = 0

        self._nav_state = NavigationState.ROTATING
        self._car_state.status_label = "ROTATING"
        self._car_state.metadata["target_theta"] = self._target_theta
        self._car_state.metadata.pop("path", None)
        self._car_state.metadata.pop("path_index", None)
        self._car_state.metadata.pop("target_point", None)

    def cancel_rotation(self) -> None:
        """Cancel current rotation and go to idle."""
        self._target_theta = None
        self._rotation_stable_start = None
        self._on_rotation_done_callback = None
        self._nav_state = NavigationState.IDLE
        self._car_state.status_label = "IDLE"
        self._car_state.metadata.pop("target_theta", None)

    def step(self, observation: RobotObservation) -> Action:
        """
        Process one control step.

        Args:
            observation: Current observation from environment

        Returns:
            Action based on current task (path following or rotation)
        """
        self.update(observation)
        return self.calculate_action()

    def update(self, observation: RobotObservation) -> None:
        """
        Update internal CarState from observation.

        Args:
            observation: Current observation from environment
        """
        # Store for velocity estimation
        prev_x = self._car_state.x
        prev_y = self._car_state.y
        prev_theta = self._car_state.theta
        prev_time = self._prev_timestamp

        # Update position from observation
        self._car_state.x = observation.x
        self._car_state.y = observation.y
        self._car_state.theta = observation.theta

        # Estimate velocities if we have previous data
        if prev_time is not None and observation.timestamp > prev_time:
            dt = observation.timestamp - prev_time
            if dt > 0:
                dx = observation.x - prev_x
                dy = observation.y - prev_y
                distance = math.sqrt(dx * dx + dy * dy)
                self._car_state.linear_velocity = distance / dt

                dtheta = observation.theta - prev_theta
                if dtheta > 180:
                    dtheta -= 360
                elif dtheta < -180:
                    dtheta += 360
                self._car_state.angular_velocity = dtheta / dt

        self._prev_timestamp = observation.timestamp
        self._last_observation = observation

    def calculate_action(self) -> Action:
        """
        Calculate action based on current state.

        Returns:
            Action with wheel speeds
        """
        if self._nav_state == NavigationState.ROTATING:
            return self._calculate_rotation_action()
        elif self._nav_state == NavigationState.ROTATION_STABLE:
            return self._calculate_rotation_action()
        elif self._nav_state == NavigationState.FOLLOWING:
            return self._calculate_path_action()
        else:
            return Action.stop()

    def _calculate_rotation_action(self) -> Action:
        """
        Calculate action for in-place rotation.

        Returns:
            Action with wheel speeds for rotation
        """
        if self._target_theta is None:
            self._nav_state = NavigationState.IDLE
            self._car_state.status_label = "IDLE"
            return Action.stop()

        current_theta = self._car_state.theta

        # Calculate angle error
        error = self._target_theta - current_theta

        # Normalize to [-180, 180]
        while error > 180:
            error -= 360
        while error < -180:
            error += 360

        # Check if within tolerance
        if abs(error) <= self.ROTATION_TOLERANCE_DEG:
            # Within tolerance - check stability
            current_time = time.time()

            if self._rotation_stable_start is None:
                # Just entered tolerance zone
                self._rotation_stable_start = current_time
                self._nav_state = NavigationState.ROTATION_STABLE
                self._car_state.status_label = "ROTATION_STABLE"

            elif current_time - self._rotation_stable_start >= self.ROTATION_STABLE_TIME:
                # Stable for required duration - rotation complete
                self._nav_state = NavigationState.ROTATION_DONE
                self._car_state.status_label = "ROTATION_DONE"
                self._car_state.metadata.pop("target_theta", None)

                # Call completion callback if set
                if self._on_rotation_done_callback:
                    callback = self._on_rotation_done_callback
                    self._on_rotation_done_callback = None
                    callback()

                return Action.stop()

            # Still waiting for stability time - hold position
            return Action.stop()

        else:
            # Outside tolerance - need to rotate
            self._rotation_stable_start = None
            self._nav_state = NavigationState.ROTATING
            self._car_state.status_label = "ROTATING"

            # Determine rotation direction and speed
            # Positive error means turn counter-clockwise (left wheel back, right wheel forward)
            # Negative error means turn clockwise (left wheel forward, right wheel back)

            # Scale speed based on error magnitude
            # Use higher speed for large errors, but always maintain minimum speed
            # to overcome motor deadzone/friction
            abs_error = abs(error)
            if abs_error > 45.0:
                # Large error: use max speed
                speed = self.ROTATION_SPEED_MAX
            elif abs_error > 15.0:
                # Medium error: interpolate between min and max
                t = (abs_error - 15.0) / 30.0  # 0 at 15°, 1 at 45°
                speed = self.ROTATION_SPEED_MIN + t * (self.ROTATION_SPEED_MAX - self.ROTATION_SPEED_MIN)
            else:
                # Small error (5-15°): use minimum speed to ensure movement
                speed = self.ROTATION_SPEED_MIN

            if error > 0:
                # Turn counter-clockwise (positive rotation)
                return Action(left_speed=-speed, right_speed=speed)
            else:
                # Turn clockwise (negative rotation)
                return Action(left_speed=speed, right_speed=-speed)

    def _calculate_path_action(self) -> Action:
        """
        Calculate action using pure pursuit algorithm.

        Returns:
            Action with wheel speeds to follow the path
        """
        # No path -> stop
        if not self._path:
            self._nav_state = NavigationState.IDLE
            self._car_state.status_label = "IDLE"
            return Action.stop()

        # Get current position
        robot_x = self._car_state.x
        robot_y = self._car_state.y
        robot_theta = self._car_state.theta

        # Check if we've reached the end of the path
        final_point = self._path[-1]
        dist_to_goal = math.sqrt(
            (robot_x - final_point[0]) ** 2 + (robot_y - final_point[1]) ** 2
        )
        if dist_to_goal < self._goal_tolerance:
            self._nav_state = NavigationState.FINISHED_PATH
            self._car_state.status_label = "FINISHED"
            self._car_state.metadata.pop("target_point", None)
            return Action.stop()

        # Find target point using pure pursuit
        target_point = self._find_target_point(robot_x, robot_y)
        if target_point is None:
            target_point = final_point

        self._car_state.metadata["target_point"] = target_point
        self._car_state.metadata["path_index"] = self._path_index
        self._car_state.status_label = "FOLLOWING"

        # Calculate steering using pure pursuit geometry
        return self._calculate_pursuit_action(
            robot_x, robot_y, robot_theta, target_point
        )

    def _find_target_point(self, robot_x: float, robot_y: float) -> Optional[Point]:
        """
        Find the target point on the path using pure pursuit.

        Args:
            robot_x: Robot x position
            robot_y: Robot y position

        Returns:
            Target point (x, y) or None if path is exhausted
        """
        if not self._path:
            return None

        # Start searching from current index (never go backward)
        best_idx = self._path_index
        best_dist = float("inf")

        # First, find the closest point from current index onward
        for i in range(self._path_index, len(self._path)):
            px, py = self._path[i]
            dist = math.sqrt((robot_x - px) ** 2 + (robot_y - py) ** 2)
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        # Update path index (only advance, never go back)
        self._path_index = max(self._path_index, best_idx)

        # Now find the target point at lookahead distance
        for i in range(self._path_index, len(self._path)):
            px, py = self._path[i]
            dist = math.sqrt((robot_x - px) ** 2 + (robot_y - py) ** 2)
            if dist >= self._lookahead_distance:
                self._path_index = i
                return (px, py)

        # If no point is far enough, return the last point
        if self._path:
            return self._path[-1]
        return None

    def _calculate_pursuit_action(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        target: Point,
    ) -> Action:
        """
        Calculate wheel speeds using pure pursuit geometry.

        Args:
            robot_x: Robot x position
            robot_y: Robot y position
            robot_theta: Robot orientation (degrees, 0 = +X)
            target: Target point (x, y)

        Returns:
            Action with wheel speeds
        """
        # Calculate angle to target
        dx = target[0] - robot_x
        dy = target[1] - robot_y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < 1e-6:
            return Action.stop()

        # Angle to target in degrees (0 = +X direction)
        angle_to_target = math.degrees(math.atan2(dy, dx))

        # Calculate heading error
        heading_error = angle_to_target - robot_theta

        # Normalize to [-180, 180]
        while heading_error > 180:
            heading_error -= 360
        while heading_error < -180:
            heading_error += 360

        # Pure pursuit curvature
        alpha_rad = math.radians(heading_error)
        curvature = 2.0 * math.sin(alpha_rad) / max(distance, self._lookahead_distance)

        # Convert curvature to differential drive wheel speeds
        wheel_base = self._ws_config.wheel_base

        # Base forward speed (scaled by heading error)
        turn_factor = max(0.3, math.cos(alpha_rad))
        base_speed = self._max_speed * turn_factor

        # Calculate wheel speeds
        diff = curvature * wheel_base / 2.0
        left_speed = base_speed * (1.0 - diff)
        right_speed = base_speed * (1.0 + diff)

        # Normalize to keep within [-max_speed, max_speed]
        max_wheel = max(abs(left_speed), abs(right_speed))
        if max_wheel > self._max_speed:
            scale = self._max_speed / max_wheel
            left_speed *= scale
            right_speed *= scale

        # Clamp to valid range
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))

        return Action(left_speed=left_speed, right_speed=right_speed)

    def reset(self) -> None:
        """Reset controller to initial state."""
        super().reset()
        self._path = []
        self._path_index = 0
        self._prev_timestamp = None
        self._target_theta = None
        self._rotation_stable_start = None
        self._on_rotation_done_callback = None
        self._nav_state = NavigationState.IDLE
        self._car_state.metadata.pop("path", None)
        self._car_state.metadata.pop("path_index", None)
        self._car_state.metadata.pop("target_point", None)
        self._car_state.metadata.pop("target_theta", None)

    def set_speed(self, speed: float) -> None:
        """
        Set max speed for path following.

        Args:
            speed: New maximum speed [0, 1]
        """
        self._max_speed = max(0.0, min(1.0, speed))
