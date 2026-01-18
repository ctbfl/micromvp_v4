"""
WASD Controller - Simple keyboard-driven wheel control.

This controller translates keyboard WASD inputs to wheel speeds:
- W: Forward (left=1.0, right=1.0)
- S: Backward (left=-1.0, right=-1.0)
- A: Turn left in place (left=-1.0, right=1.0)
- D: Turn right in place (left=1.0, right=-1.0)
- No keys: Stop (left=0.0, right=0.0)

Supports multiple keys for combined motion:
- W+A: Forward left arc
- W+D: Forward right arc
- S+A: Backward left arc
- S+D: Backward right arc
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Set

from micromvp.controller.base import Controller
from micromvp.core.models import (
    Action,
    RobotObservation,
    WorkspaceConfig,
)


@dataclass
class WASDInput:
    """
    WASD keyboard input state.

    Supports multiple simultaneous key presses for combined motion.
    """

    w: bool = False  # Forward
    a: bool = False  # Turn left
    s: bool = False  # Backward
    d: bool = False  # Turn right

    @classmethod
    def from_keys(cls, keys: Set[str]) -> "WASDInput":
        """
        Create WASDInput from a set of pressed key names.

        Args:
            keys: Set of lowercase key names (e.g., {"w", "a"})

        Returns:
            WASDInput with corresponding flags set
        """
        return cls(
            w="w" in keys,
            a="a" in keys,
            s="s" in keys,
            d="d" in keys,
        )

    def is_idle(self) -> bool:
        """Check if no keys are pressed."""
        return not (self.w or self.a or self.s or self.d)


class WASDController(Controller):
    """
    WASD keyboard controller for manual robot control.

    This controller maintains the robot's CarState and converts
    WASD keyboard inputs into wheel speed actions.

    Status labels:
    - "IDLE": No input, stopped
    - "FORWARD": W pressed
    - "BACKWARD": S pressed
    - "TURN_LEFT": A pressed
    - "TURN_RIGHT": D pressed
    - "FORWARD_LEFT": W+A pressed
    - "FORWARD_RIGHT": W+D pressed
    - "BACKWARD_LEFT": S+A pressed
    - "BACKWARD_RIGHT": S+D pressed
    """

    def __init__(
        self,
        robot_id: int,
        ws_config: WorkspaceConfig,
        max_speed: float = 1.0,
    ) -> None:
        """
        Initialize WASD controller.

        Args:
            robot_id: The ID of the robot this controller manages
            ws_config: Workspace configuration from environment
            max_speed: Maximum speed multiplier [0.0, 1.0] (default: 1.0)
        """
        super().__init__(robot_id, ws_config)
        self._wasd_input = WASDInput()
        self._prev_timestamp: Optional[float] = None
        self._max_speed = max(0.0, min(1.0, max_speed))

    def step(self, observation: RobotObservation) -> Action:
        """
        Process one control step.

        Updates internal state from observation and computes action
        based on current WASD input.

        Args:
            observation: Current observation from environment

        Returns:
            Action based on WASD input
        """
        self.update(observation)
        return self.calculate_action()

    def update(self, observation: RobotObservation) -> None:
        """
        Update internal CarState from observation.

        Updates position, orientation, and estimates velocities.

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
                distance = (dx * dx + dy * dy) ** 0.5
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

    @property
    def max_speed(self) -> float:
        """Get current maximum speed multiplier."""
        return self._max_speed

    @max_speed.setter
    def max_speed(self, value: float) -> None:
        """Set maximum speed multiplier [0.0, 1.0]."""
        self._max_speed = max(0.0, min(1.0, value))

    def calculate_action(self) -> Action:
        """
        Calculate action based on current WASD input.

        Handles single keys and key combinations:
        - W: (1, 1) forward
        - S: (-1, -1) backward
        - A: (-1, 1) turn left
        - D: (1, -1) turn right
        - W+A: (0, 1) forward-left arc
        - W+D: (1, 0) forward-right arc
        - S+A: (-1, 0) backward-left arc
        - S+D: (0, -1) backward-right arc

        Returns:
            Action with normalized wheel speeds
        """
        wasd = self._wasd_input

        # No input -> stop
        if wasd.is_idle():
            self._car_state.status_label = "IDLE"
            return Action.stop()

        # Initialize speeds
        left_speed = 0.0
        right_speed = 0.0

        # Forward/backward component
        if wasd.w and not wasd.s:
            left_speed += 1.0
            right_speed += 1.0
        elif wasd.s and not wasd.w:
            left_speed -= 1.0
            right_speed -= 1.0

        # Left/right component
        if wasd.a and not wasd.d:
            left_speed -= 1.0
            right_speed += 1.0
        elif wasd.d and not wasd.a:
            left_speed += 1.0
            right_speed -= 1.0

        # Clamp to [-1, 1] and scale by max_speed
        left_speed = max(-1.0, min(1.0, left_speed)) * self._max_speed
        right_speed = max(-1.0, min(1.0, right_speed)) * self._max_speed

        # Update status label
        self._update_status_label(wasd)

        return Action(left_speed=left_speed, right_speed=right_speed)

    def _update_status_label(self, wasd: WASDInput) -> None:
        """Update status label based on WASD input."""
        if wasd.w and wasd.a:
            self._car_state.status_label = "FORWARD_LEFT"
        elif wasd.w and wasd.d:
            self._car_state.status_label = "FORWARD_RIGHT"
        elif wasd.s and wasd.a:
            self._car_state.status_label = "BACKWARD_LEFT"
        elif wasd.s and wasd.d:
            self._car_state.status_label = "BACKWARD_RIGHT"
        elif wasd.w:
            self._car_state.status_label = "FORWARD"
        elif wasd.s:
            self._car_state.status_label = "BACKWARD"
        elif wasd.a:
            self._car_state.status_label = "TURN_LEFT"
        elif wasd.d:
            self._car_state.status_label = "TURN_RIGHT"
        else:
            self._car_state.status_label = "IDLE"

    def set_wasd(self, wasd: WASDInput) -> None:
        """
        Set WASD input state.

        Called by coordinator when keyboard state changes.

        Args:
            wasd: Current WASD input state
        """
        self._wasd_input = wasd

    def set_keys(self, keys: Set[str]) -> None:
        """
        Set WASD input from key set.

        Convenience method for coordinator.

        Args:
            keys: Set of lowercase key names (e.g., {"w", "a"})
        """
        self._wasd_input = WASDInput.from_keys(keys)

    def clear_input(self) -> None:
        """Clear all WASD input (stop)."""
        self._wasd_input = WASDInput()

    def reset(self) -> None:
        """Reset controller to initial state."""
        super().reset()
        self._wasd_input = WASDInput()
        self._prev_timestamp = None
