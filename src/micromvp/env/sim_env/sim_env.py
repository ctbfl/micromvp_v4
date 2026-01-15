"""
Simulation environment - In-memory physics simulation.

SimEnvironment provides a simulated world for testing without hardware.
It maintains robot states internally and advances physics based on actions.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from micromvp.core.ddr import simulate_step
from micromvp.core.models import Action, RobotObservation, Pose
from micromvp.env.base import Environment


@dataclass
class SimConfig:
    """Configuration for simulation environment."""
    sim_speed: float = 100.0      # Speed multiplier for simulation
    wheel_base: float = 30.0      # Distance between wheels (pixels)
    max_dt: float = 0.05          # Maximum time step (prevents jumps)


class SimEnvironment(Environment):
    """
    In-memory simulation environment.

    Simulates differential-drive kinematics for multiple robots.
    Time advances based on wall-clock time, scaled by sim_speed.
    """

    def __init__(
        self,
        config: SimConfig,
        initial_poses: Dict[int, Tuple[float, float, float]],
    ) -> None:
        """
        Initialize simulation with robot poses.

        Args:
            config: Simulation configuration
            initial_poses: Dict mapping robot_id to (x, y, theta) initial pose
        """
        self._config = config
        self._poses: Dict[int, Tuple[float, float, float]] = dict(initial_poses)
        self._actions: Dict[int, Action] = {}
        self._last_time = time.perf_counter()
        self._speed_scale = 1.0

    def observe(self) -> Dict[int, RobotObservation]:
        """
        Get observations after advancing simulation.

        Advances physics based on elapsed time and current actions,
        then returns observations for all robots.
        """
        # Advance simulation
        now = time.perf_counter()
        dt = now - self._last_time
        self._last_time = now

        # Clamp dt to prevent large jumps
        dt = max(0.0, min(dt, self._config.max_dt))

        # Update each robot's pose
        for robot_id, pose in list(self._poses.items()):
            action = self._actions.get(robot_id, Action.stop())
            x, y, theta = pose

            # Apply speed scaling
            scale = self._config.sim_speed * self._speed_scale
            v_l = action.left_speed * scale
            v_r = action.right_speed * scale

            # Simulate kinematics
            new_x, new_y, new_theta = simulate_step(
                x, y, theta,
                v_l, v_r,
                self._config.wheel_base,
                dt,
            )
            self._poses[robot_id] = (new_x, new_y, new_theta)

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
                valid=True,
                confidence=1.0,
            )

        return observations

    def apply_actions(self, actions: Dict[int, Action]) -> None:
        """
        Store actions to be applied on next observe().

        Args:
            actions: Dict mapping robot_id to Action
        """
        for robot_id, action in actions.items():
            self._actions[robot_id] = action

    def set_speed_scale(self, scale: float) -> None:
        """Set speed scale (0.0 = paused, 1.0 = normal)."""
        self._speed_scale = max(0.0, float(scale))

    def set_pose(self, robot_id: int, x: float, y: float, theta: float) -> None:
        """Directly set a robot's pose (for testing/initialization)."""
        self._poses[robot_id] = (x, y, theta)

    def get_pose(self, robot_id: int) -> Optional[Tuple[float, float, float]]:
        """Get a robot's current pose."""
        return self._poses.get(robot_id)

    @property
    def robot_ids(self) -> list[int]:
        """Get list of robot IDs in simulation."""
        return list(self._poses.keys())
