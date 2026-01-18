"""
Follow Path Coordinator - Lets robots follow user-drawn paths.

This coordinator allows users to:
1. Select a robot by clicking on it
2. Draw a path on the canvas for the selected robot to follow
3. The robot will follow the path using pure pursuit

GUI callbacks:
- on_car_click: Select the clicked robot as active
- on_curve_drawn: Send the drawn path to the active robot's controller
"""
from __future__ import annotations

from typing import Dict, List, Set, Any, Optional, Tuple

from micromvp.controller.base import Controller
from micromvp.controller.follow_path_controller import PurePursuitFollowPathController, StanleyFollowPathController, PurePursuit_PD_FollowPathController
from micromvp.coordinator.base import Coordinator

# Type alias for path-following controllers
FollowPathController = (PurePursuitFollowPathController, StanleyFollowPathController, PurePursuit_PD_FollowPathController)
from micromvp.core.models import (
    Action,
    CarState,
    RobotObservation,
    WorkspaceConfig,
)


Point = Tuple[float, float]


class FollowPathCoordinator(Coordinator):
    """
    Coordinator for path-following robots.

    Allows user to select a robot and draw a path for it to follow.
    Works with FollowPathController instances.

    Usage:
        # Create controllers
        controllers = {
            robot_id: FollowPathController(robot_id, ws_config)
            for robot_id in ws_config.car_id_list
        }

        # Create coordinator
        coordinator = FollowPathCoordinator(ws_config, controllers)

        # Register GUI callbacks
        gui.register_callback("on_car_click", coordinator.on_car_click)
        gui.register_callback("on_curve_drawn", coordinator.on_curve_drawn)

        # Main loop
        while running:
            observations = env.observe()
            actions = coordinator.process(observations)
            env.apply_actions(actions)
    """

    def __init__(
        self,
        ws_config: WorkspaceConfig,
        controllers: Dict[int, Controller],
    ) -> None:
        """
        Initialize follow path coordinator.

        Args:
            ws_config: Workspace configuration from environment
            controllers: Dict mapping robot_id to Controller instance.
                         Should be FollowPathController instances for full functionality.
        """
        super().__init__(ws_config, controllers)

        # Currently selected robot
        self._active_robot_id: Optional[int] = None
        if ws_config.car_id_list:
            self._active_robot_id = ws_config.car_id_list[0]

        # Currently pressed keys (for keyboard shortcuts)
        self._pressed_keys: Set[str] = set()

    @property
    def active_robot_id(self) -> Optional[int]:
        """Get the currently active (selected) robot ID."""
        return self._active_robot_id

    def process(
        self, observations: Dict[int, RobotObservation]
    ) -> Dict[int, Action]:
        """
        Process observations and return actions.

        Distributes observations to all controllers and collects their actions.

        Args:
            observations: Dict mapping robot_id to RobotObservation

        Returns:
            Dict mapping robot_id to Action
        """
        actions: Dict[int, Action] = {}

        for robot_id, controller in self._controllers.items():
            if robot_id in observations:
                actions[robot_id] = controller.step(observations[robot_id])
            else:
                # No observation for this robot, output stop action
                actions[robot_id] = Action.stop()

        return actions

    def gather_car_state(self) -> List[CarState]:
        """
        Gather car states with active robot metadata.

        Adds "is_active" flag to active robot's metadata for GUI highlighting.

        Returns:
            List of CarState objects
        """
        states = []
        for controller in self._controllers.values():
            state = controller.car_state
            # Mark active robot in metadata
            state.metadata["is_active"] = (
                controller.robot_id == self._active_robot_id
            )
            states.append(state)
        return states

    def get_additional_drawings(self) -> List[Dict[str, Any]]:
        """
        Get additional drawings for GUI.

        Draws:
        - Selection indicator around active robot
        - Path for each robot with assigned path
        - Target point for each robot following a path

        Returns:
            List of drawing commands
        """
        drawings = []

        # Draw selection indicator for active robot
        if self._active_robot_id is not None:
            controller = self._controllers.get(self._active_robot_id)
            if controller is not None:
                state = controller.car_state
                drawings.append({
                    "uuid": f"selection_indicator_{self._active_robot_id}",
                    "type": "circle",
                    "center": (state.x, state.y),
                    "radius": max(self._ws_config.car_width, self._ws_config.car_height) * 0.8,
                    "color": "green",
                    "width": 2,
                })

        # Draw paths and target points for all controllers
        for robot_id, controller in self._controllers.items():
            if isinstance(controller, FollowPathController):
                state = controller.car_state

                # Draw the path if it exists
                path = state.metadata.get("path")
                if path and len(path) >= 2:
                    # Choose color based on active state
                    if robot_id == self._active_robot_id:
                        path_color = "#00FF00"  # Green for active
                    else:
                        path_color = "#888888"  # Gray for inactive

                    drawings.append({
                        "uuid": f"path_{robot_id}",
                        "type": "path",
                        "points": path,
                        "color": path_color,
                        "width": 2,
                    })

                # Draw the target point if following
                target = state.metadata.get("target_point")
                if target and state.status_label == "FOLLOWING":
                    drawings.append({
                        "uuid": f"target_{robot_id}",
                        "type": "point",
                        "position": target,
                        "radius": 5,
                        "color": "#FF0000",  # Red target point
                    })

                    # Draw line from robot to target
                    drawings.append({
                        "uuid": f"target_line_{robot_id}",
                        "type": "line",
                        "start": (state.x, state.y),
                        "end": target,
                        "color": "#FF000080",  # Semi-transparent red
                        "width": 1,
                    })

        return drawings

    # -------------------------------------------------------------------------
    # GUI Callback Methods
    # -------------------------------------------------------------------------

    def on_car_click(self, robot_id: int) -> None:
        """
        Handle car click event from GUI.

        Selects the clicked robot as active.

        Args:
            robot_id: ID of the clicked robot
        """
        if robot_id in self._controllers:
            self._active_robot_id = robot_id

    def on_curve_drawn(self, points: List[Tuple[float, float]]) -> None:
        """
        Handle curve drawn event from GUI.

        Sends the drawn path to the active robot's controller.

        Args:
            points: List of (x, y) points defining the drawn curve
        """
        if self._active_robot_id is None:
            return

        controller = self._controllers.get(self._active_robot_id)
        if controller is None:
            return

        if isinstance(controller, FollowPathController):
            # Set the path for the controller to follow
            controller.set_path(points)

    def on_key_press(self, key: str) -> None:
        """
        Handle key press event from GUI.

        Supports keyboard shortcuts:
        - 1-9: Select robot by index
        - Tab: Cycle to next robot
        - Space: Stop all robots
        - C: Clear path of active robot

        Args:
            key: Key name (lowercase)
        """
        key = key.lower()
        self._pressed_keys.add(key)

        # Handle robot selection via number keys
        if key.isdigit() and key != "0":
            index = int(key) - 1  # 1-based to 0-based
            self._select_robot_by_index(index)

        # Handle Tab for cycling through robots
        elif key == "tab":
            self._cycle_active_robot()

        # Handle Space for emergency stop
        elif key == "space":
            self._emergency_stop()

        # Handle C for clearing path of active robot
        elif key == "c":
            self._clear_active_path()

    def on_key_release(self, key: str) -> None:
        """
        Handle key release event from GUI.

        Args:
            key: Key name (lowercase)
        """
        key = key.lower()
        self._pressed_keys.discard(key)

    # -------------------------------------------------------------------------
    # Robot Selection Methods
    # -------------------------------------------------------------------------

    def select_robot(self, robot_id: int) -> bool:
        """
        Select a robot as active.

        Args:
            robot_id: ID of robot to select

        Returns:
            True if robot was found and selected, False otherwise
        """
        if robot_id in self._controllers:
            self._active_robot_id = robot_id
            return True
        return False

    def _select_robot_by_index(self, index: int) -> None:
        """Select robot by index in car_id_list."""
        car_ids = self._ws_config.car_id_list
        if 0 <= index < len(car_ids):
            self.select_robot(car_ids[index])

    def _cycle_active_robot(self) -> None:
        """Cycle to the next robot in the list."""
        car_ids = self._ws_config.car_id_list
        if not car_ids:
            return

        if self._active_robot_id is None:
            self._active_robot_id = car_ids[0]
        else:
            try:
                current_index = car_ids.index(self._active_robot_id)
                next_index = (current_index + 1) % len(car_ids)
                self.select_robot(car_ids[next_index])
            except ValueError:
                # Current active not in list, select first
                self.select_robot(car_ids[0])

    def _emergency_stop(self) -> None:
        """Stop all robots immediately."""
        for controller in self._controllers.values():
            if isinstance(controller, FollowPathController):
                controller.clear_path()
            controller.stop()

    def _clear_active_path(self) -> None:
        """Clear the path of the active robot."""
        if self._active_robot_id is None:
            return

        controller = self._controllers.get(self._active_robot_id)
        if controller is not None and isinstance(controller, FollowPathController):
            controller.clear_path()

    def set_all_speeds(self, speed: float) -> None:
        """
        Set speed for all controllers.

        Args:
            speed: Speed value in range [0, 1]
        """
        for controller in self._controllers.values():
            if hasattr(controller, "set_speed"):
                controller.set_speed(speed)

    def reset(self) -> None:
        """Reset coordinator and all controllers."""
        self._pressed_keys.clear()
        super().reset()
        # Re-select first robot if available
        if self._ws_config.car_id_list:
            self._active_robot_id = self._ws_config.car_id_list[0]
