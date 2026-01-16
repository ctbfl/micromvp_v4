"""
Test keyboard control of cars in simulation environment.

Controls:
- 1-4: Select robot by number
- Tab: Cycle to next robot
- W/A/S/D: Move selected robot (forward/left/backward/right)
- W+A/W+D: Arc turns (forward-left/forward-right)
- S+A/S+D: Reverse arc turns
- Space: Emergency stop all robots
- Click car: Select robot
- Escape: Quit
"""
import threading
import time

from micromvp.gui import MVPWindow
from micromvp.env import SimEnv, SimConfig
from micromvp.controller import WASDController
from micromvp.coordinator import KeyboardCoordinator


def main():
    # Setup simulation environment with 4 cars
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
        initial_poses=[
            (1, 150, 150, 0),    # Car 1: bottom-left, facing right
            (2, 650, 150, 180),  # Car 2: bottom-right, facing left
            (3, 150, 450, 90),   # Car 3: top-left, facing up
            (4, 650, 450, 270),  # Car 4: top-right, facing down
        ],
    )
    env = SimEnv(sim_config)
    ws_config = env.workspace_config

    # Create controllers for each car
    controllers = {
        robot_id: WASDController(robot_id, ws_config)
        for robot_id in ws_config.car_id_list
    }

    # Create keyboard coordinator
    coordinator = KeyboardCoordinator(ws_config, controllers)

    # Setup GUI
    gui_config = {
        "canvas": {
            "click_canvas_callback": True,
        },
        "control_panel": [
            {"type": "label", "text": "=== Keyboard Control ==="},
            {"type": "label", "text": "1-4: Select car"},
            {"type": "label", "text": "WASD: Move car"},
            {"type": "label", "text": "Tab: Next car"},
            {"type": "label", "text": "Space: Stop all"},
        ],
    }
    gui = MVPWindow(gui_config, ws_config)

    # Track running state
    running = True

    # Register keyboard callbacks
    def on_key_press(key: str):
        nonlocal running
        if key == "escape":
            running = False
            gui.close_window()
        else:
            coordinator.on_key_press(key)

    gui.register_callback("on_key_press", on_key_press)
    gui.register_callback("on_key_release", coordinator.on_key_release)
    gui.register_callback("on_car_click", coordinator.on_car_click)

    # Logic loop
    def logic_loop():
        while running:
            start_time = time.time()

            # Get observations and process
            observations = env.observe()
            actions = coordinator.process(observations)
            env.apply_actions(actions)

            # Update GUI
            car_states = {s.car_id: s for s in coordinator.gather_car_state()}
            drawings = coordinator.get_additional_drawings()
            gui.update(car_states, drawings)

            # Maintain frequency
            elapsed = time.time() - start_time
            sleep_time = max(0, (1 / ws_config.frequency) - elapsed)
            time.sleep(sleep_time)

    # Start logic thread
    logic_thread = threading.Thread(target=logic_loop, daemon=True)
    logic_thread.start()

    # Run GUI (blocks until window closed)
    print("Keyboard Control Test")
    print("=" * 40)
    print("Controls:")
    print("  1-4    : Select car")
    print("  Tab    : Cycle to next car")
    print("  W/A/S/D: Move selected car")
    print("  Space  : Emergency stop")
    print("  Escape : Quit")
    print("  Click  : Select car")
    print("=" * 40)

    gui.run()

    # Cleanup
    running = False
    env.close()
    print("Done.")


if __name__ == "__main__":
    main()
