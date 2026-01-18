"""
Test path-following control in simulation environment.

This example demonstrates:
- Simulation environment with multiple robots
- Path following using Pure Pursuit / Pure Pursuit + PD / Stanley controllers
- GUI controls for switching controllers and adjusting speed

Controls:
- 1-4: Select robot by number
- Tab: Cycle to next robot
- Click car: Select robot
- Draw curve: Robot follows the drawn path
- C: Clear path of selected robot
- Space: Emergency stop all robots
- Escape: Quit

GUI Controls:
- Controller dropdown: Switch between Pure Pursuit / Pure Pursuit + PD / Stanley
- Speed slider: Adjust robot movement speed
"""
import threading
import time
import argparse
from typing import Dict, Optional

from micromvp.gui import MVPWindow
from micromvp.env import SimEnv, SimConfig
from micromvp.controller import (
    PurePursuitFollowPathController,
    StanleyFollowPathController,
    PurePursuit_PD_FollowPathController,
)
from micromvp.controller.base import Controller
from micromvp.coordinator import FollowPathCoordinator
from micromvp.core.models import WorkspaceConfig


# ===== Controller name normalization (single source of truth) =====
CTRL_PP = "Pure Pursuit"
CTRL_PP_PD = "Pure Pursuit + PD"
CTRL_STANLEY = "Stanley"

CONTROLLER_OPTIONS = [CTRL_PP, CTRL_PP_PD, CTRL_STANLEY]


def normalize_controller_name(name: str) -> str:
    """
    Normalize any user/legacy naming into one of:
    - Pure Pursuit
    - Pure Pursuit + PD
    - Stanley
    """
    if name is None:
        return CTRL_PP

    s = name.strip().lower()

    # common legacy spellings
    if s in {"pure_pursuit", "pure pursuit", "pp"}:
        return CTRL_PP
    if s in {"pure_pursuit_pd", "pure pursuit + pd", "pure pursuit+pd", "pp+pd", "pp_pd"}:
        return CTRL_PP_PD
    if s in {"stanley", "stanley_controller"}:
        return CTRL_STANLEY

    # if user already passed the friendly names (case-insensitive)
    if s == CTRL_PP.lower():
        return CTRL_PP
    if s == CTRL_PP_PD.lower():
        return CTRL_PP_PD
    if s == CTRL_STANLEY.lower():
        return CTRL_STANLEY

    raise ValueError(f"Unknown controller type: {name}. Options: {CONTROLLER_OPTIONS}")


def create_controllers(
    ws_config: WorkspaceConfig,
    controller_name: str,
    max_speed: float,
    lookahead: Optional[float],
) -> Dict[int, Controller]:
    """Create path-following controllers for all robots."""
    controller_name = normalize_controller_name(controller_name)

    controllers: Dict[int, Controller] = {}
    for robot_id in ws_config.car_id_list:
        if controller_name == CTRL_STANLEY:
            controllers[robot_id] = StanleyFollowPathController(
                robot_id,
                ws_config,
                lookahead_distance=lookahead,
                max_speed=max_speed,
            )
        elif controller_name == CTRL_PP:
            controllers[robot_id] = PurePursuitFollowPathController(
                robot_id,
                ws_config,
                lookahead_distance=lookahead,
                max_speed=max_speed,
            )
        elif controller_name == CTRL_PP_PD:
            controllers[robot_id] = PurePursuit_PD_FollowPathController(
                robot_id,
                ws_config,
                lookahead_distance=lookahead,
                max_speed=max_speed,
            )
        else:
            # should never happen due to normalize_controller_name
            raise ValueError(f"Unknown controller type: {controller_name}")

    return controllers


def main():
    parser = argparse.ArgumentParser(description="Simulation path following test")
    parser.add_argument(
        "--robots", type=int, default=2,
        help="Number of robots (default: 2)"
    )
    parser.add_argument(
        "--max-speed", type=float, default=0.4,
        help="Maximum robot speed [0-1] (default: 0.4)"
    )
    parser.add_argument(
        "--lookahead", type=float, default=None,
        help="Lookahead distance (default: 1.5 * car_size)"
    )
    parser.add_argument(
        "--controller", type=str, default=CTRL_PP,
        choices=CONTROLLER_OPTIONS,
        help=f"Initial controller type (default: {CTRL_PP})"
    )
    args = parser.parse_args()

    # Create initial poses for robots
    num_robots = max(1, min(args.robots, 9))
    initial_poses = []
    positions = [
        (150, 150, 45),
        (650, 150, 135),
        (150, 450, -45),
        (650, 450, -135),
        (400, 300, 0),
        (250, 300, 90),
        (550, 300, -90),
        (400, 150, 180),
        (400, 450, 0),
    ]
    for i in range(num_robots):
        x, y, theta = positions[i % len(positions)]
        initial_poses.append((i + 1, x, y, theta))

    print(f"Configuring {num_robots} robot(s) in simulation")

    # Setup simulation environment
    sim_config = SimConfig(
        width=800,
        height=600,
        car_width=36,
        car_height=52,
        offset_w=18,
        offset_h=26,
        wheel_base=30,
        max_wheel_speed=100,
        frequency=20,
        initial_poses=initial_poses,
    )
    env = SimEnv(sim_config)
    ws_config = env.workspace_config

    # Track current settings (friendly names only)
    current_controller_name = normalize_controller_name(args.controller)
    current_speed = args.max_speed

    # Create initial controllers
    controllers = create_controllers(
        ws_config, current_controller_name, current_speed, args.lookahead
    )

    # Create coordinator
    coordinator = FollowPathCoordinator(ws_config, controllers)

    # Setup GUI with controls
    gui_config = {
        "canvas": {
            "click_canvas_callback": True,
            "draw_curve_callback": True,
        },
        "control_panel": [
            {"type": "label", "text": "=== Path Following ==="},
            {"type": "label", "text": "Click robot to select"},
            {"type": "label", "text": "Draw curve to set path"},
            {"type": "label", "text": ""},
            {
                "type": "options",
                "label": "Select Controller",
                "options": CONTROLLER_OPTIONS,
                "default": current_controller_name,
                "callback_name": "switch_controller",
            },
            {
                "type": "dynamic_label",
                "title": "Current Controller",
                "widget_name": "controller_display",
                "default": current_controller_name,
            },
            {"type": "label", "text": ""},
            {
                "type": "continuous_slider",
                "label": "Robot Speed",
                "range": [0.0, 1.0],
                "default": current_speed,
                "callback_name": "set_robot_speed",
            },
            {"type": "label", 
             "text": """Keyboard:
    1-9: Select robot
    Tab: Next robot
     C: Clear path
    Space: Stop all
    Escape: Quit"""
             },
            {"type": "label", "text": f"Num Robots: {num_robots}"},
        ],
    }
    gui = MVPWindow(gui_config, ws_config)

    # Track running state
    running = True
    controller_lock = threading.Lock()

    def switch_controller(controller_name: str):
        """Callback to switch controller type."""
        nonlocal coordinator, controllers, current_controller_name

        new_name = normalize_controller_name(controller_name)
        if new_name == current_controller_name:
            return

        with controller_lock:
            current_controller_name = new_name

            controllers = create_controllers(
                ws_config, current_controller_name, current_speed, args.lookahead
            )
            coordinator = FollowPathCoordinator(ws_config, controllers)

            gui.update_widget_text("controller_display", current_controller_name)
            print(f"Switched to {current_controller_name} controller")

    def set_robot_speed(speed: float):
        """Callback to set robot speed."""
        nonlocal current_speed
        current_speed = speed
        with controller_lock:
            coordinator.set_all_speeds(speed)

    # Register callbacks
    def on_key_press(key: str):
        nonlocal running
        if key == "escape":
            running = False
            gui.close_window()
        else:
            with controller_lock:
                coordinator.on_key_press(key)

    def on_key_release(key: str):
        with controller_lock:
            coordinator.on_key_release(key)

    def on_car_click(robot_id: int):
        with controller_lock:
            coordinator.on_car_click(robot_id)

    def on_curve_drawn(points):
        with controller_lock:
            coordinator.on_curve_drawn(points)

    gui.register_callback("on_key_press", on_key_press)
    gui.register_callback("on_key_release", on_key_release)
    gui.register_callback("on_car_click", on_car_click)
    gui.register_callback("on_curve_drawn", on_curve_drawn)
    gui.register_callback("set_robot_speed", set_robot_speed)
    gui.register_callback("switch_controller", switch_controller)

    # Logic loop
    def logic_loop():
        while running:
            start_time = time.time()

            observations = env.observe()

            with controller_lock:
                actions = coordinator.process(observations)

            env.apply_actions(actions)

            with controller_lock:
                car_states = {s.car_id: s for s in coordinator.gather_car_state()}
                drawings = coordinator.get_additional_drawings()

            gui.update(car_states, drawings)

            elapsed = time.time() - start_time
            sleep_time = max(0, (1 / ws_config.frequency) - elapsed)
            time.sleep(sleep_time)

    logic_thread = threading.Thread(target=logic_loop, daemon=True)
    logic_thread.start()

    # Print instructions
    print("\n" + "=" * 50)
    print("Simulation Path Following")
    print("=" * 50)
    print("Controls:")
    print("  1-9       : Select robot")
    print("  Tab       : Cycle to next robot")
    print("  Click car : Select robot")
    print("  Draw curve: Set path for selected robot")
    print("  C         : Clear path of selected robot")
    print("  Space     : Emergency stop all")
    print("  Escape    : Quit")
    print("")
    print("GUI Controls:")
    print("  Dropdown  : Switch between Pure Pursuit / Pure Pursuit + PD / Stanley")
    print("  Slider    : Adjust robot speed")
    print("=" * 50)
    print(f"\nInitial controller: {current_controller_name}")
    print(f"Initial speed: {args.max_speed}")
    print("\nDraw a path for the selected robot to follow!")

    gui.run()

    running = False
    env.close()
    print("Done.")


if __name__ == "__main__":
    main()
