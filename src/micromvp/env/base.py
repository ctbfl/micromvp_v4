"""
Environment base class - World adapter for observation and action execution.

The Environment is the interface between the control system and the world.
It provides two core functions:
1. Observe: Get robot observations from the world
2. Apply Actions: Execute robot actions in the world

Key principles:
- Env does NOT import or know about Car/Controller
- Env does NOT contain any control logic
- Env is a passive executor and state provider
- Env only communicates via Observation/Action data structures
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Dict

from micromvp.core.models import Action, RobotObservation


class Environment(ABC):
    """
    Abstract base class for robot environments.

    Implementations:
    - SimEnvironment: In-memory physics simulation
    - RealEnvironment: Real hardware with vision + WiFi communication
    """

    @abstractmethod
    def observe(self) -> Dict[int, RobotObservation]:
        """
        Get current observations for all robots.

        Returns:
            Dict mapping robot_id to RobotObservation.
            Not all robots may have valid observations (e.g., tracking lost).
        """
        raise NotImplementedError

    @abstractmethod
    def apply_actions(self, actions: Dict[int, Action]) -> None:
        """
        Apply actions to robots.

        Args:
            actions: Dict mapping robot_id to Action.
                     Robots not in the dict will maintain previous action.
        """
        raise NotImplementedError

    def step(self, actions: Dict[int, Action]) -> Dict[int, RobotObservation]:
        """
        Convenience method: apply actions then observe.

        This is equivalent to calling apply_actions() then observe().
        For SimEnvironment, this advances simulation time.
        For RealEnvironment, this waits for next frame.

        Args:
            actions: Dict mapping robot_id to Action.

        Returns:
            Dict mapping robot_id to RobotObservation.
        """
        self.apply_actions(actions)
        return self.observe()

    def set_speed_scale(self, scale: float) -> None:
        """
        Set global speed scale (0.0 to 1.0+).

        This allows pausing (scale=0) or slowing down the system.
        Default implementation does nothing.
        """
        pass

    def close(self) -> None:
        """
        Clean up resources.

        Default implementation does nothing.
        """
        pass
