"""
Follow Path Controller - Pure pursuit path following for differential drive robots.

This controller receives a path (list of points) and uses the pure pursuit algorithm
to follow it. The algorithm:
1. Find the closest point on the path
2. Look ahead from that point to find a target point
3. Calculate wheel speeds to steer toward the target point
4. Only advance forward along the path (never go backward)

The controller limits speed to a configurable max (default 0.3) to ensure safe operation.
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

from micromvp.controller.base import Controller
from micromvp.core.models import (
    Action,
    RobotObservation,
    WorkspaceConfig,
)


Point = Tuple[float, float]


class FollowPathController(Controller):
    """
    Pure pursuit path following controller.

    This controller follows a user-drawn path using the pure pursuit algorithm,
    which is well-suited for differential drive robots.

    Status labels:
    - "IDLE": No path assigned
    - "FOLLOWING": Actively following path
    - "FINISHED": Reached end of path

    Usage:
        controller = FollowPathController(robot_id, ws_config)
        controller.set_path([(x1,y1), (x2,y2), ...])  # Set path to follow
        action = controller.step(observation)  # Get wheel commands
    """

    def __init__(
        self,
        robot_id: int,
        ws_config: WorkspaceConfig,
        lookahead_distance: Optional[float] = None,
        max_speed: float = 0.3,
        goal_tolerance: Optional[float] = None,
    ) -> None:
        """
        Initialize follow path controller.

        Args:
            robot_id: The ID of the robot this controller manages
            ws_config: Workspace configuration from environment
            lookahead_distance: Distance to look ahead on path (default: 1.5 * car_height)
            max_speed: Maximum normalized wheel speed [0, 1] (default: 0.3)
            goal_tolerance: Distance to consider goal reached (default: 0.5 * car_height)
        """
        super().__init__(robot_id, ws_config)

        # Pure pursuit parameters
        car_size = max(ws_config.car_width, ws_config.car_height)
        self._lookahead_distance = lookahead_distance or (1.5 * car_size)
        self._max_speed = min(1.0, max(0.0, max_speed))
        self._goal_tolerance = goal_tolerance or (0.5 * car_size)

        # Path state
        self._path: List[Point] = []
        self._path_index: int = 0  # Current progress along path (never goes backward)
        self._prev_timestamp: Optional[float] = None

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

    def set_path(self, path: List[Point]) -> None:
        """
        Set a new path to follow.

        Args:
            path: List of (x, y) points defining the path
        """
        self._path = list(path)
        self._path_index = 0
        if self._path:
            self._car_state.status_label = "FOLLOWING"
            self._car_state.metadata["path"] = self._path
            self._car_state.metadata["path_index"] = self._path_index
        else:
            self._car_state.status_label = "IDLE"
            self._car_state.metadata.pop("path", None)
            self._car_state.metadata.pop("path_index", None)

    def clear_path(self) -> None:
        """Clear the current path and stop."""
        self._path = []
        self._path_index = 0
        self._car_state.status_label = "IDLE"
        self._car_state.metadata.pop("path", None)
        self._car_state.metadata.pop("path_index", None)
        self._car_state.metadata.pop("target_point", None)

    def step(self, observation: RobotObservation) -> Action:
        """
        Process one control step.

        Args:
            observation: Current observation from environment

        Returns:
            Action based on path following
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
                # Linear velocity (distance / time)
                dx = observation.x - prev_x
                dy = observation.y - prev_y
                distance = math.sqrt(dx * dx + dy * dy)
                self._car_state.linear_velocity = distance / dt

                # Angular velocity (angle change / time)
                dtheta = observation.theta - prev_theta
                # Handle wraparound
                if dtheta > 180:
                    dtheta -= 360
                elif dtheta < -180:
                    dtheta += 360
                self._car_state.angular_velocity = dtheta / dt

        self._prev_timestamp = observation.timestamp
        self._last_observation = observation

    def calculate_action(self) -> Action:
        """
        Calculate action using pure pursuit algorithm.

        Returns:
            Action with wheel speeds to follow the path
        """
        # No path -> stop
        if not self._path:
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
            self._car_state.status_label = "FINISHED"
            self._car_state.metadata.pop("target_point", None)
            return Action.stop()

        # Find target point using pure pursuit
        target_point = self._find_target_point(robot_x, robot_y)
        if target_point is None:
            # Path exhausted but not at goal - try to reach final point
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

        The target point is found by:
        1. Starting from current path_index (never going backward)
        2. Finding the first point that is at least lookahead_distance away
        3. Updating path_index to track progress

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
                # Update path index to this point
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

        For a differential drive robot:
        - Calculate angle to target point
        - Convert to wheel speeds using curvature

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

        # Pure pursuit curvature: kappa = 2 * sin(alpha) / L
        # where alpha is heading error and L is lookahead distance
        alpha_rad = math.radians(heading_error)
        curvature = 2.0 * math.sin(alpha_rad) / max(distance, self._lookahead_distance)

        # Convert curvature to differential drive wheel speeds
        # v = (v_r + v_l) / 2
        # omega = (v_r - v_l) / wheel_base
        # kappa = omega / v
        #
        # Solving: v_r = v * (1 + kappa * wheel_base / 2)
        #          v_l = v * (1 - kappa * wheel_base / 2)

        wheel_base = self._ws_config.wheel_base

        # Base forward speed (scaled by heading error - slow down for sharp turns)
        # Use cosine of heading error to reduce speed for large turns
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
        self._car_state.metadata.pop("path", None)
        self._car_state.metadata.pop("path_index", None)
        self._car_state.metadata.pop("target_point", None)

    def set_speed(self, speed: float) -> None:
        """
        Internal method to set max speed.

        Args:
            speed: New maximum speed [0, 1]
        """
        self._max_speed = max(0.0, min(1.0, speed))