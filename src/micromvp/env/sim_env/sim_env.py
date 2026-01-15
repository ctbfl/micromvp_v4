"""
Simulation Environment - In-memory physics simulation for MicroMVP.

SimEnv provides a simulated world for testing without hardware.
It maintains robot states internally and advances physics based on actions.

Key features:
- Provides its own WorkspaceConfig
- Simulates differential-drive kinematics
- Thread-safe action application
- Configurable simulation speed
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from micromvp.core.models import Action, RobotObservation, WorkspaceConfig
from micromvp.env.base import Environment

from .ddr import simulate_step


@dataclass
class SimConfig:
    """
    Configuration for simulation environment.

    This defines both the workspace parameters and simulation behavior.
    All spatial units are in "workspace units" (typically pixels or cm).
    """
    # Workspace dimensions
    width: float = 640.0
    height: float = 480.0

    # Car geometry (all cars assumed identical)
    car_width: float = 36.0
    car_height: float = 52.0
    offset_w: float = 18.0  # Center offset from left edge
    offset_h: float = 26.0  # Center offset from bottom edge

    # Dynamics
    wheel_base: float = 30.0       # Distance between wheels
    max_wheel_speed: float = 100.0  # Max wheel speed (units/second)

    # Simulation parameters
    frequency: float = 20.0        # Target update frequency (Hz)
    speed_scale: float = 1.0       # Global speed multiplier (0=pause, 1=normal)
    max_dt: float = 0.1            # Maximum time step to prevent large jumps

    # Initial robot poses: list of (id, x, y, theta_deg)
    initial_poses: List[Tuple[int, float, float, float]] = field(default_factory=list)


class SimEnv(Environment):
    """
    In-memory simulation environment.

    Simulates differential-drive kinematics for multiple robots.
    Provides WorkspaceConfig and handles physics updates internally.

    Usage:
        config = SimConfig(
            width=800, height=600,
            initial_poses=[(1, 100, 100, 0), (2, 200, 200, 90)]
        )
        env = SimEnv(config)

        # Main loop
        while running:
            observations = env.observe()
            actions = compute_actions(observations)
            env.apply_actions(actions)
    """

    def __init__(self, config: SimConfig) -> None:
        """
        Initialize simulation environment.

        Args:
            config: SimConfig with workspace and simulation parameters
        """
        self._config = config

        # Build WorkspaceConfig from SimConfig
        self._workspace_config = WorkspaceConfig(
            width=config.width,
            height=config.height,
            car_width=config.car_width,
            car_height=config.car_height,
            offset_w=config.offset_w,
            offset_h=config.offset_h,
            wheel_base=config.wheel_base,
            max_wheel_speed=config.max_wheel_speed,
            frequency=config.frequency,
            car_id_list=[pose[0] for pose in config.initial_poses],
        )

        # Robot state: {robot_id: (x, y, theta_deg)}
        self._poses: Dict[int, Tuple[float, float, float]] = {}
        for robot_id, x, y, theta in config.initial_poses:
            self._poses[robot_id] = (x, y, theta)

        # Pending actions: {robot_id: Action}
        self._actions: Dict[int, Action] = {}

        # Timing
        self._last_time: Optional[float] = None
        self._speed_scale = config.speed_scale

    @property
    def workspace_config(self) -> WorkspaceConfig:
        """Get the workspace configuration for this environment."""
        return self._workspace_config

    def observe(self) -> Dict[int, RobotObservation]:
        """
        Advance simulation and return current observations.

        Physics are advanced based on elapsed wall-clock time since last observe().
        On first call, no physics update occurs (just returns initial state).

        Returns:
            Dict mapping robot_id to RobotObservation
        """
        # Advance physics
        now = time.perf_counter()

        if self._last_time is not None:
            dt = now - self._last_time
            # Clamp dt to prevent large jumps (e.g., after pause or debug)
            dt = min(dt, self._config.max_dt)
            dt = max(dt, 0.0)
            self._advance_physics(dt)

        self._last_time = now

        # Build observations
        timestamp = time.time()
        observations: Dict[int, RobotObservation] = {}

        for robot_id, (x, y, theta) in self._poses.items():
            observations[robot_id] = RobotObservation(
                robot_id=robot_id,
                x=x,
                y=y,
                theta=theta,
                timestamp=timestamp,
            )

        return observations

    def _advance_physics(self, dt: float) -> None:
        """
        Advance physics simulation by dt seconds.

        Args:
            dt: Time step in seconds
        """
        if dt <= 0.0 or self._speed_scale <= 0.0:
            return

        effective_dt = dt * self._speed_scale

        for robot_id, (x, y, theta) in list(self._poses.items()):
            action = self._actions.get(robot_id, Action.stop())

            # Convert normalized [-1, 1] action to actual wheel speeds
            v_left = action.left_speed * self._config.max_wheel_speed
            v_right = action.right_speed * self._config.max_wheel_speed

            # Simulate kinematics
            new_x, new_y, new_theta = simulate_step(
                x, y, theta,
                v_left, v_right,
                self._config.wheel_base,
                effective_dt,
            )

            # Optional: clamp to workspace bounds
            new_x = max(0, min(new_x, self._config.width))
            new_y = max(0, min(new_y, self._config.height))

            self._poses[robot_id] = (new_x, new_y, new_theta)

    def apply_actions(self, actions: Dict[int, Action]) -> None:
        """
        Store actions to be applied on next physics update.

        Args:
            actions: Dict mapping robot_id to Action.
                     Robots not in the dict maintain their previous action.
        """
        for robot_id, action in actions.items():
            if robot_id in self._poses:
                self._actions[robot_id] = action

    def step(self, actions: Dict[int, Action])-> Dict[int, RobotObservation]:
        """
        Convenience method to advance simulation and get observations.

        This combines observe() and apply_actions() for a single step.
        The actions must be set prior to calling this method.

        Returns:
            Dict mapping robot_id to RobotObservation
        """
        self.apply_actions(actions)
        return self.observe()

    def set_speed_scale(self, scale: float) -> None:
        """
        Set simulation speed scale.

        Args:
            scale: Speed multiplier (0.0=paused, 1.0=normal, 2.0=2x speed, etc.)
        """
        self._speed_scale = max(0.0, float(scale))

    def get_speed_scale(self) -> float:
        """Get current speed scale."""
        return self._speed_scale

    # -------------------------------------------------------------------------
    # Utility methods for testing and initialization
    # -------------------------------------------------------------------------

    def set_pose(self, robot_id: int, x: float, y: float, theta: float) -> None:
        """
        Directly set a robot's pose (for testing/reset).

        Args:
            robot_id: Robot ID
            x, y: Position in workspace units
            theta: Heading in degrees [0, 360)
        """
        if robot_id in self._poses:
            self._poses[robot_id] = (x, y, theta % 360.0)

    def get_pose(self, robot_id: int) -> Optional[Tuple[float, float, float]]:
        """
        Get a robot's current pose.

        Returns:
            (x, y, theta) tuple or None if robot not found
        """
        return self._poses.get(robot_id)

    def add_robot(self, robot_id: int, x: float, y: float, theta: float) -> None:
        """
        Add a new robot to the simulation.

        Args:
            robot_id: Unique robot ID
            x, y: Initial position
            theta: Initial heading in degrees
        """
        self._poses[robot_id] = (x, y, theta % 360.0)
        # Update workspace config's car list
        if robot_id not in self._workspace_config.car_id_list:
            self._workspace_config.car_id_list.append(robot_id)

    def remove_robot(self, robot_id: int) -> bool:
        """
        Remove a robot from the simulation.

        Returns:
            True if robot was removed, False if not found
        """
        if robot_id in self._poses:
            del self._poses[robot_id]
            self._actions.pop(robot_id, None)
            if robot_id in self._workspace_config.car_id_list:
                self._workspace_config.car_id_list.remove(robot_id)
            return True
        return False

    @property
    def robot_ids(self) -> List[int]:
        """Get list of robot IDs in simulation."""
        return list(self._poses.keys())

    def reset_time(self) -> None:
        """Reset the internal timer (useful after pausing)."""
        self._last_time = None

    def close(self) -> None:
        """Clean up (no-op for simulation)."""
        pass
