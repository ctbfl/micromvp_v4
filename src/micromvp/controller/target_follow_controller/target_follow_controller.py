"""
Target Follow Controller - Follow a single moving target point.

This controller is designed for formation control where the coordinator
sets and updates a target point that the robot should follow.
Uses pure pursuit with in-place rotation for large heading errors.

Algorithm:
1. Coordinator provides a goal point each timestep (long-term goal)
2. If goal distance > look_ahead_distance: compute short-term goal on the line
   between car and goal at look_ahead_distance from the car
3. If goal distance <= look_ahead_distance: use goal directly as short-term goal
4. If heading error to short-term goal > 45°: rotate in place until < 45°
5. Otherwise: use pure pursuit to follow the short-term goal
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

from micromvp.controller.base import Controller
from micromvp.core.models import (
    Action,
    RobotObservation,
    WorkspaceConfig,
)


Point = Tuple[float, float]

# Threshold for in-place rotation (degrees)
IN_PLACE_ROTATION_THRESHOLD = 45.0

# Default window size for velocity smoothing
DEFAULT_VELOCITY_WINDOW_SIZE = 10


@dataclass
class PositionSample:
    """A single position/orientation sample with timestamp."""
    x: float
    y: float
    theta: float
    timestamp: float


class TargetFollowController(Controller):
    """
    Controller that follows a target point set by the coordinator.

    Uses pure pursuit with automatic short-term goal computation:
    - Long-term goal (from coordinator) is used when distance > look_ahead_distance
    - Short-term goal is computed on the line to the long-term goal
    - In-place rotation when heading error exceeds 45 degrees

    Status labels:
    - "IDLE": No target assigned
    - "ROTATING": Rotating in place to align with target
    - "FOLLOWING": Actively following target with pure pursuit
    - "ARRIVED": Within tolerance of target

    Usage:
        controller = TargetFollowController(robot_id, ws_config)
        controller.set_target((x, y))  # Set target point
        controller.set_max_speed(0.3)  # Set speed limit
        action = controller.step(observation)
    """

    def __init__(
        self,
        robot_id: int,
        ws_config: WorkspaceConfig,
        max_speed: float = 0.3,
        lookahead_distance: Optional[float] = None,
        arrival_tolerance: Optional[float] = None,
        rotation_speed: float = 0.5,
        velocity_window_size: int = DEFAULT_VELOCITY_WINDOW_SIZE,
    ) -> None:
        """
        Initialize target follow controller.

        Args:
            robot_id: The ID of the robot this controller manages
            ws_config: Workspace configuration from environment
            max_speed: Maximum normalized wheel speed [0, 1] (default: 0.3)
            lookahead_distance: Distance for pure pursuit and short-term goal (default: 1.5 * car_size)
            arrival_tolerance: Distance to consider arrived (default: 0.3 * car_size)
            rotation_speed: Speed for in-place rotation [0, 1] (default: 0.5)
            velocity_window_size: Number of frames for velocity smoothing (default: 10)
        """
        super().__init__(robot_id, ws_config)

        car_size = max(ws_config.car_width, ws_config.car_height)
        self._max_speed = min(1.0, max(0.0, max_speed))
        self._lookahead_distance = lookahead_distance or (1.5 * car_size)
        self._arrival_tolerance = arrival_tolerance or (0.3 * car_size)
        self._rotation_speed = min(1.0, max(0.0, rotation_speed))

        # Target state
        self._target: Optional[Point] = None  # Long-term goal from coordinator
        self._short_term_goal: Optional[Point] = None  # Computed pure pursuit point

        # Velocity estimation with sliding window
        self._velocity_window_size = max(2, velocity_window_size)
        self._position_history: Deque[PositionSample] = deque(maxlen=self._velocity_window_size)

    @property
    def target(self) -> Optional[Point]:
        """Get the current long-term target point (from coordinator)."""
        return self._target

    @property
    def short_term_goal(self) -> Optional[Point]:
        """Get the computed short-term goal (pure pursuit point)."""
        return self._short_term_goal

    @property
    def max_speed(self) -> float:
        """Get the maximum speed."""
        return self._max_speed

    @property
    def lookahead_distance(self) -> float:
        """Get the lookahead distance."""
        return self._lookahead_distance

    def set_target(self, target: Point) -> None:
        """
        Set the target point to follow (long-term goal from coordinator).

        The short-term goal will be computed automatically in calculate_action():
        - If target is within lookahead_distance: use target directly
        - If target is farther: compute point on line at lookahead_distance

        Args:
            target: (x, y) target position
        """
        self._target = target
        self._car_state.metadata["target_point"] = target
        if self._car_state.status_label == "IDLE":
            self._car_state.status_label = "FOLLOWING"

    def clear_target(self) -> None:
        """Clear the target and stop."""
        self._target = None
        self._short_term_goal = None
        self._car_state.status_label = "IDLE"
        self._car_state.metadata.pop("target_point", None)
        self._car_state.metadata.pop("short_term_goal", None)

    def set_max_speed(self, speed: float) -> None:
        """
        Set the maximum speed.

        Args:
            speed: Maximum normalized wheel speed [0, 1]
        """
        self._max_speed = min(1.0, max(0.0, speed))

    def set_lookahead_distance(self, distance: float) -> None:
        """
        Set the lookahead distance.

        Args:
            distance: Lookahead distance for pure pursuit
        """
        self._lookahead_distance = max(0.01, distance)

    def step(self, observation: RobotObservation) -> Action:
        """
        Process one control step.

        Args:
            observation: Current observation from environment

        Returns:
            Action to reach the target
        """
        self.update(observation)
        return self.calculate_action()

    def update(self, observation: RobotObservation) -> None:
        """
        Update internal CarState from observation.

        Uses a sliding window of position samples to calculate smoothed velocity.
        Velocity is calculated as total distance / total time over the window.

        Args:
            observation: Current observation from environment
        """
        # Update position from observation
        self._car_state.x = observation.x
        self._car_state.y = observation.y
        self._car_state.theta = observation.theta

        # Add current sample to history
        current_sample = PositionSample(
            x=observation.x,
            y=observation.y,
            theta=observation.theta,
            timestamp=observation.timestamp,
        )
        self._position_history.append(current_sample)

        # Calculate smoothed velocity over the window
        self._calculate_smoothed_velocity()

        self._last_observation = observation

    def _calculate_smoothed_velocity(self) -> None:
        """
        Calculate smoothed linear and angular velocity over the position history window.

        Uses total distance traveled / total time elapsed for more stable estimates.
        """
        if len(self._position_history) < 2:
            # Not enough samples yet
            self._car_state.linear_velocity = 0.0
            self._car_state.angular_velocity = 0.0
            return

        # Get oldest and newest samples
        oldest = self._position_history[0]
        newest = self._position_history[-1]

        # Calculate total time elapsed
        dt = newest.timestamp - oldest.timestamp
        if dt <= 0:
            self._car_state.linear_velocity = 0.0
            self._car_state.angular_velocity = 0.0
            return

        # Calculate total distance traveled (sum of segment distances)
        total_distance = 0.0
        total_angle_change = 0.0

        for i in range(1, len(self._position_history)):
            prev = self._position_history[i - 1]
            curr = self._position_history[i]

            # Linear distance
            dx = curr.x - prev.x
            dy = curr.y - prev.y
            total_distance += math.sqrt(dx * dx + dy * dy)

            # Angular change (handle wrap-around)
            dtheta = curr.theta - prev.theta
            if dtheta > 180:
                dtheta -= 360
            elif dtheta < -180:
                dtheta += 360
            total_angle_change += dtheta

        # Calculate velocities (units per second)
        self._car_state.linear_velocity = total_distance / dt
        self._car_state.angular_velocity = total_angle_change / dt  # degrees per second

    def _compute_short_term_goal(
        self, robot_x: float, robot_y: float, target_x: float, target_y: float
    ) -> Tuple[Point, float]:
        """
        Compute the short-term goal (pure pursuit point) based on distance to target.

        If target is within lookahead_distance, use it directly.
        Otherwise, compute a point on the line from robot to target at lookahead_distance.

        Args:
            robot_x: Robot x position
            robot_y: Robot y position
            target_x: Long-term target x position
            target_y: Long-term target y position

        Returns:
            Tuple of (short_term_goal, distance_to_target)
        """
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance <= self._lookahead_distance or distance < 1e-6:
            # Target is close enough, use it directly as short-term goal
            return (target_x, target_y), distance
        else:
            # Target is far, compute point on line at lookahead_distance
            # Normalize direction and scale to lookahead_distance
            unit_x = dx / distance
            unit_y = dy / distance
            short_term_x = robot_x + unit_x * self._lookahead_distance
            short_term_y = robot_y + unit_y * self._lookahead_distance
            return (short_term_x, short_term_y), distance

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-180, 180] degrees."""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def _rotate_in_place(self, heading_error: float) -> Action:
        """
        Generate action to rotate in place toward the target heading.

        Args:
            heading_error: Heading error in degrees (positive = turn left)

        Returns:
            Action for in-place rotation
        """
        # Rotate in place: opposite wheel speeds
        if heading_error > 0:
            # Turn left (counter-clockwise)
            return Action(left_speed=-self._rotation_speed, right_speed=self._rotation_speed)
        else:
            # Turn right (clockwise)
            return Action(left_speed=self._rotation_speed, right_speed=-self._rotation_speed)

    def calculate_action(self) -> Action:
        """
        Calculate action to move toward target using pure pursuit with in-place rotation.

        Algorithm:
        1. Compute short-term goal based on distance to long-term goal (from coordinator)
        2. Calculate heading error to short-term goal
        3. If heading error > 45°: rotate in place until aligned
        4. Otherwise: use pure pursuit to follow short-term goal

        Returns:
            Action with wheel speeds
        """
        # No target -> stop
        if self._target is None:
            self._car_state.status_label = "IDLE"
            self._short_term_goal = None
            return Action.stop()

        robot_x = self._car_state.x
        robot_y = self._car_state.y
        robot_theta = self._car_state.theta

        # Compute short-term goal
        self._short_term_goal, distance_to_target = self._compute_short_term_goal(
            robot_x, robot_y, self._target[0], self._target[1]
        )
        self._car_state.metadata["short_term_goal"] = self._short_term_goal

        # Check if arrived at long-term goal
        if distance_to_target < self._arrival_tolerance:
            self._car_state.status_label = "ARRIVED"
            self._short_term_goal = None
            self._car_state.metadata.pop("short_term_goal", None)
            return Action.stop()

        if distance_to_target < 1e-6:
            return Action.stop()

        # Calculate angle to short-term goal (degrees, 0 = +X direction)
        stg_dx = self._short_term_goal[0] - robot_x
        stg_dy = self._short_term_goal[1] - robot_y
        angle_to_goal = math.degrees(math.atan2(stg_dy, stg_dx))

        # Calculate heading error
        heading_error = self._normalize_angle(angle_to_goal - robot_theta)

        # Check if we need in-place rotation (heading error > 45 degrees)
        if abs(heading_error) > IN_PLACE_ROTATION_THRESHOLD:
            self._car_state.status_label = "ROTATING"
            return self._rotate_in_place(heading_error)

        # Normal pure pursuit
        self._car_state.status_label = "FOLLOWING"

        # Convert to radians for math
        alpha_rad = math.radians(heading_error)

        # Pure pursuit curvature: kappa = 2 * sin(alpha) / L
        # where alpha is heading error and L is lookahead distance
        # Use the distance to short-term goal for L
        stg_distance = math.sqrt(stg_dx * stg_dx + stg_dy * stg_dy)
        curvature = 2.0 * math.sin(alpha_rad) / max(stg_distance, 0.01)

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
        self._target = None
        self._short_term_goal = None
        self._position_history.clear()
        self._car_state.metadata.pop("target_point", None)
        self._car_state.metadata.pop("short_term_goal", None)
