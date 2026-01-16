"""
Controller base class - Per-robot control logic.

The Controller is responsible for managing a single robot's behavior:
1. Track robot state based on observations (CarState)
2. Maintain a state machine (status_label)
3. Calculate actions based on current state and task requirements
4. Expose API for coordinator to control (direct action, target point, path following)

Key principles:
- Each robot has its own controller instance
- Controller maintains internal CarState from observations
- Controller does NOT communicate directly with Environment
- Controller receives commands from Coordinator, outputs Actions
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from micromvp.core.models import (
    Action,
    CarState,
    RobotObservation,
    WorkspaceConfig,
)


class Controller(ABC):
    """
    Abstract base class for robot controllers.

    Each robot has its own controller instance that:
    - Tracks the robot's state via CarState
    - Implements control algorithms to generate Actions
    - Provides API for coordinator commands

    Implementations:
    - WASDController: Simple keyboard-driven wheel control
    - PathFollowController: Pure pursuit path following
    """

    def __init__(self, robot_id: int, ws_config: WorkspaceConfig) -> None:
        """
        Initialize controller.

        Args:
            robot_id: The ID of the robot this controller manages
            ws_config: Workspace configuration from environment
        """
        self._robot_id = robot_id
        self._ws_config = ws_config
        self._car_state = CarState(car_id=robot_id)
        self._last_observation: Optional[RobotObservation] = None

    @property
    def robot_id(self) -> int:
        """Get the robot ID this controller manages."""
        return self._robot_id

    @property
    def car_state(self) -> CarState:
        """Get the current car state snapshot."""
        return self._car_state

    @property
    def ws_config(self) -> WorkspaceConfig:
        """Get the workspace configuration."""
        return self._ws_config

    @abstractmethod
    def step(self, observation: RobotObservation) -> Action:
        """
        Process one control step.

        This is the main entry point called each control cycle.
        Should update internal state and compute the action.

        Args:
            observation: Current observation from environment

        Returns:
            Action to apply to the robot
        """
        raise NotImplementedError

    @abstractmethod
    def update(self, observation: RobotObservation) -> None:
        """
        Update internal state from observation.

        Called internally by step() to update CarState.
        Implementations should:
        - Update position/orientation from observation
        - Estimate velocities from observation history

        Args:
            observation: Current observation from environment
        """
        raise NotImplementedError

    @abstractmethod
    def calculate_action(self) -> Action:
        """
        Calculate action based on current state.

        Called internally by step() after update().
        Implementations should compute action based on:
        - Current CarState
        - Current task/status_label

        Returns:
            Action to apply
        """
        raise NotImplementedError

    def set_direct_action(self, action: Action) -> None:
        """
        Set a direct action override.

        This allows coordinator to directly control wheel speeds,
        bypassing the controller's normal calculation.

        Default implementation stores action in metadata.
        Subclasses may override for custom behavior.

        Args:
            action: Direct action to apply
        """
        self._car_state.metadata["direct_action"] = action
        self._car_state.status_label = "DIRECT"

    def clear_direct_action(self) -> None:
        """Clear direct action override and return to normal control."""
        self._car_state.metadata.pop("direct_action", None)
        if self._car_state.status_label == "DIRECT":
            self._car_state.status_label = "IDLE"

    def stop(self) -> None:
        """Command the robot to stop."""
        self._car_state.status_label = "STOP"

    def reset(self) -> None:
        """
        Reset controller to initial state.

        Default implementation resets status to IDLE.
        Subclasses may override for additional cleanup.
        """
        self._car_state.status_label = "IDLE"
        self._car_state.metadata.clear()
        self._last_observation = None
