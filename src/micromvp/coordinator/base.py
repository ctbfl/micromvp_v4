"""
Coordinator base class - Bridge between Environment, Controllers, and GUI.

The Coordinator orchestrates the entire control system:
1. Receives observations from Environment and distributes to Controllers
2. Collects actions from Controllers and returns to Environment
3. Registers callbacks with GUI for user interaction
4. Gathers car states for GUI rendering
5. Provides additional drawings for GUI visualization

Key principles:
- Coordinator is the central hub for all communication
- Controllers are managed by Coordinator, not Environment
- GUI interactions are handled via registered callbacks
- High-level decision making (task assignment, collision avoidance) happens here
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Dict, List, Any, Optional

from micromvp.core.models import (
    Action,
    CarState,
    RobotObservation,
    WorkspaceConfig,
)
from micromvp.controller.base import Controller


class Coordinator(ABC):
    """
    Abstract base class for coordinators.

    The coordinator is the bridge between environment, controllers, and GUI.
    It handles:
    - Observation distribution to controllers
    - Action collection from controllers
    - GUI callback registration
    - Car state gathering for rendering
    - Optional additional drawings for visualization

    Implementations:
    - KeyboardCoordinator: Simple keyboard control with car selection
    - FormationCoordinator: Multi-robot formation control
    """

    def __init__(
        self,
        ws_config: WorkspaceConfig,
        controllers: Dict[int, Controller],
    ) -> None:
        """
        Initialize coordinator.

        Args:
            ws_config: Workspace configuration from environment
            controllers: Dict mapping robot_id to Controller instance
        """
        self._ws_config = ws_config
        self._controllers = controllers

    @property
    def ws_config(self) -> WorkspaceConfig:
        """Get the workspace configuration."""
        return self._ws_config

    @property
    def controllers(self) -> Dict[int, Controller]:
        """Get the controllers dict."""
        return self._controllers

    @abstractmethod
    def process(
        self, observations: Dict[int, RobotObservation]
    ) -> Dict[int, Action]:
        """
        Main processing loop - distribute observations, collect actions.

        This is called each control cycle. Should:
        1. Perform any high-level coordination logic
        2. Distribute observations to each controller
        3. Collect actions from each controller
        4. Return actions for environment to execute

        Args:
            observations: Dict mapping robot_id to RobotObservation

        Returns:
            Dict mapping robot_id to Action
        """
        raise NotImplementedError

    def gather_car_state(self) -> List[CarState]:
        """
        Gather car states from all controllers for GUI rendering.

        Returns:
            List of CarState objects from all controllers
        """
        return [ctrl.car_state for ctrl in self._controllers.values()]

    def get_additional_drawings(self) -> List[Dict[str, Any]]:
        """
        Get additional drawing commands for GUI.

        Override this to provide custom visualizations like:
        - Target points
        - Paths
        - Formation shapes
        - Debug info

        Returns:
            List of drawing commands in GUI format.
            Each dict should have at minimum:
            - "uuid": str - Unique ID for retained mode drawing
            - "type": str - Drawing type (line, circle, polygon, etc.)
            See gui/readme.md for full format specification.
        """
        return []

    def get_controller(self, robot_id: int) -> Optional[Controller]:
        """
        Get controller for a specific robot.

        Args:
            robot_id: Robot ID

        Returns:
            Controller instance or None if not found
        """
        return self._controllers.get(robot_id)

    def reset(self) -> None:
        """
        Reset all controllers to initial state.

        Default implementation calls reset() on each controller.
        Subclasses may override for additional cleanup.
        """
        for controller in self._controllers.values():
            controller.reset()

    def close(self) -> None:
        """
        Clean up resources.

        Default implementation does nothing.
        Subclasses may override for cleanup.
        """
        pass
