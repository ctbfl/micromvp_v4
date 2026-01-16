"""
Keyboard Coordinator - Manual keyboard control for multiple robots.

This coordinator allows users to:
1. Select a robot using number keys (1-9) or clicking on canvas
2. Control the selected robot using WASD keys
3. All other robots remain stationary

Keyboard controls:
- 1-9: Select robot by index in car_id_list
- W/A/S/D: Control selected robot movement
- Space: Stop all robots
- Tab: Cycle to next robot

GUI callbacks to register:
- on_key_press: Called when a key is pressed
- on_key_release: Called when a key is released
- on_car_click: Called when user clicks on a car in canvas
"""
from __future__ import annotations

from typing import Dict, List, Set, Any, Optional

from micromvp.controller.base import Controller
from micromvp.controller.WASD_controller import WASDController, WASDInput
from micromvp.coordinator.base import Coordinator
from micromvp.core.models import (
    Action,
    CarState,
    RobotObservation,
    WorkspaceConfig,
)


class KeyboardCoordinator(Coordinator):
    """
    Keyboard-based manual control coordinator.

    Allows user to select and control robots using keyboard.
    Works with WASDController instances.

    Usage:
        # Create controllers
        controllers = {
            robot_id: WASDController(robot_id, ws_config)
            for robot_id in ws_config.car_id_list
        }

        # Create coordinator
        coordinator = KeyboardCoordinator(ws_config, controllers)

        # Register GUI callbacks
        gui.register_callback("on_key_press", coordinator.on_key_press)
        gui.register_callback("on_key_release", coordinator.on_key_release)
        gui.register_callback("on_car_click", coordinator.on_car_click)

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
        Initialize keyboard coordinator.

        Args:
            ws_config: Workspace configuration from environment
            controllers: Dict mapping robot_id to Controller instance.
                         Should be WASDController instances for full functionality.
        """
        super().__init__(ws_config, controllers)

        # Currently selected robot
        self._active_robot_id: Optional[int] = None
        if ws_config.car_id_list:
            self._active_robot_id = ws_config.car_id_list[0]

        # Currently pressed keys (lowercase)
        self._pressed_keys: Set[str] = set()

    @property
    def active_robot_id(self) -> Optional[int]:
        """Get the currently active (selected) robot ID."""
        return self._active_robot_id

    @property
    def pressed_keys(self) -> Set[str]:
        """Get the set of currently pressed keys."""
        return self._pressed_keys.copy()

    def process(
        self, observations: Dict[int, RobotObservation]
    ) -> Dict[int, Action]:
        """
        Process observations and return actions.

        1. Update WASD input for active robot
        2. Clear WASD input for inactive robots
        3. Distribute observations to all controllers
        4. Collect and return actions

        Args:
            observations: Dict mapping robot_id to RobotObservation

        Returns:
            Dict mapping robot_id to Action
        """
        # Update WASD input for controllers
        wasd_input = WASDInput.from_keys(self._pressed_keys)

        for robot_id, controller in self._controllers.items():
            if isinstance(controller, WASDController):
                if robot_id == self._active_robot_id:
                    # Active robot gets the WASD input
                    controller.set_wasd(wasd_input)
                else:
                    # Inactive robots are cleared
                    controller.clear_input()

        # Process observations and collect actions
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

        Draws a selection indicator around the active robot.

        Returns:
            List of drawing commands
        """
        if self._active_robot_id is None:
            return []

        controller = self._controllers.get(self._active_robot_id)
        if controller is None:
            return []

        state = controller.car_state
        # Draw selection circle around active robot
        return [
            {
                "uuid": f"selection_indicator_{self._active_robot_id}",
                "type": "circle",
                "center": (state.x, state.y),
                "radius": max(self._ws_config.car_width, self._ws_config.car_height) * 0.8,
                "color": "green",  # Green
                "width": 2,
                "fill_color": None,
            }
        ]

    # -------------------------------------------------------------------------
    # GUI Callback Methods
    # -------------------------------------------------------------------------

    def on_key_press(self, key: str) -> None:
        """
        Handle key press event from GUI.

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

    def on_key_release(self, key: str) -> None:
        """
        Handle key release event from GUI.

        Args:
            key: Key name (lowercase)
        """
        key = key.lower()
        self._pressed_keys.discard(key)

    def on_car_click(self, robot_id: int) -> None:
        """
        Handle car click event from GUI.

        Selects the clicked robot as active.

        Args:
            robot_id: ID of the clicked robot
        """
        if robot_id in self._controllers:
            self._active_robot_id = robot_id

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
            # Clear input on previously active robot
            if self._active_robot_id is not None:
                prev_controller = self._controllers.get(self._active_robot_id)
                if isinstance(prev_controller, WASDController):
                    prev_controller.clear_input()

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
        self._pressed_keys.clear()
        for controller in self._controllers.values():
            if isinstance(controller, WASDController):
                controller.clear_input()
            controller.stop()

    def reset(self) -> None:
        """Reset coordinator and all controllers."""
        self._pressed_keys.clear()
        super().reset()
        # Re-select first robot if available
        if self._ws_config.car_id_list:
            self._active_robot_id = self._ws_config.car_id_list[0]
