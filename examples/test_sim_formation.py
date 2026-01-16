"""
Test multi-robot formation control in simulation.

This example demonstrates:
- Multiple robots following a pattern (circle or figure-8)
- Dynamic number of active cars
- Adjustable target point speed and car speed
- Random collision-free initialization

GUI Controls:
- Pattern: Select circle or figure-8
- Car Count: Number of active robots (1 to max)
- Point Speed: How fast targets move along pattern
- Car Speed: Maximum robot movement speed
- Escape: Quit

The robots will follow equally-spaced target points that move
along the selected pattern path.
"""
import threading
import time
import argparse

from micromvp.gui import MVPWindow
from micromvp.env import SimEnv, SimConfig
from micromvp.controller import TargetFollowController
from micromvp.coordinator import FormationCoordinator


def main():
    parser = argparse.ArgumentParser(description="Simulation formation control test")
    parser.add_argument(
        "--cars", type=int, default=8,
        help="Maximum number of cars in simulation (default: 8)"
    )
    parser.add_argument(
        "--width", type=float, default=1280,
        help="Workspace width (default: 1280)"
    )
    parser.add_argument(
        "--height", type=float, default=720,
        help="Workspace height (default: 720)"
    )
    parser.add_argument(
        "--pattern", type=str, default="circle",
        choices=["circle", "figure8"],
        help="Initial pattern (default: circle)"
    )
    parser.add_argument(
        "--active", type=int, default=None,
        help="Initial active car count (default: all)"
    )
    parser.add_argument(
        "--point-speed", type=float, default=50.0,
        help="Initial target point speed in units/second (default: 50.0)"
    )
    parser.add_argument(
        "--car-speed", type=float, default=0.6,
        help="Initial car speed (default: 0.6)"
    )
    parser.add_argument(
        "--circle-scale", type=float, default=0.7,
        help="Circle pattern scale factor relative to boundary (default: 0.7)"
    )
    parser.add_argument(
        "--figure8-scale", type=float, default=1,
        help="Figure-8 pattern scale factor relative to boundary (default: 1)"
    )
    parser.add_argument(
        "--boundary-margin", type=float, default=0.15,
        help="Boundary margin ratio for patterns (default: 0.15)"
    )
    parser.add_argument(
        "--spawn-near-target", action="store_true", default=False,
        help="Spawn cars near their target points when reinitializing (default: False)"
    )
    args = parser.parse_args()

    # Create initial poses for all cars (will be randomized by coordinator)
    # Just need placeholder positions
    initial_poses = [
        (i + 1, args.width / 2, args.height / 2, 0)
        for i in range(args.cars)
    ]

    # Setup simulation environment
    sim_config = SimConfig(
        width=args.width,
        height=args.height,
        initial_poses=initial_poses,
        frequency=30.0,
    )
    env = SimEnv(sim_config)
    ws_config = env.workspace_config

    print(f"Simulation configured with {args.cars} robots")
    print(f"Workspace: {args.width} x {args.height}")
    print(f"Pattern scales - Circle: {args.circle_scale}, Figure-8: {args.figure8_scale}")
    print(f"Boundary margin: {args.boundary_margin}")

    # Create target-following controllers for each robot
    controllers = {
        robot_id: TargetFollowController(
            robot_id,
            ws_config,
            max_speed=args.car_speed,
        )
        for robot_id in ws_config.car_id_list
    }

    # Create formation coordinator
    coordinator = FormationCoordinator(
        ws_config,
        controllers,
        env=env,  # Pass env for position reset
        initial_pattern=args.pattern,
        initial_active_count=args.active,
        initial_point_speed=args.point_speed,
        initial_car_speed=args.car_speed,
        circle_scale=args.circle_scale,
        figure8_scale=args.figure8_scale,
        boundary_margin=args.boundary_margin,
        spawn_near_target=args.spawn_near_target,
    )

    # Setup GUI with control panel
    gui_config = {
        "canvas": {
            "click_canvas_callback": False,
            "draw_curve_callback": False,
        },
        "control_panel": [
            {"type": "label", "text": "=== Formation Control ==="},
            {"type": "label", "text": ""},
            {
                "type": "options",
                "label": "Pattern",
                "options": ["circle", "figure8"],
                "callback_name": "set_pattern",
                "default": args.pattern,
            },
            {"type": "label", "text": ""},
            {
                "type": "discrete_slider",
                "label": "Car Count",
                "tiers": list(range(1, args.cars + 1)),
                "callback_name": "set_car_count",
                "default_idx": (args.active or args.cars) - 1,
            },
            {"type": "label", "text": ""},
            {
                "type": "toggle",
                "label": "Spawn Near Target",
                "callback_name": "set_spawn_near_target",
                "default": args.spawn_near_target,
            },
            {"type": "label", "text": ""},
            {
                "type": "continuous_slider",
                "label": "Point Speed (units/s)",
                "range": [0.0, 100.0],
                "callback_name": "set_point_speed",
                "default": args.point_speed,
            },
            {"type": "label", "text": ""},
            {
                "type": "continuous_slider",
                "label": "Car Speed",
                "range": [0.1, 1.0],
                "callback_name": "set_car_speed",
                "default": args.car_speed,
            },
            {"type": "label", "text": ""},
            {"type": "label", "text": "Press Escape to quit"},
        ],
    }
    gui = MVPWindow(gui_config, ws_config)

    # Track running state
    running = True

    # Register callbacks
    def on_key_press(key: str):
        nonlocal running
        if key == "escape":
            running = False
            gui.close_window()

    gui.register_callback("on_key_press", on_key_press)
    gui.register_callback("set_pattern", coordinator.on_pattern_change)
    gui.register_callback("set_car_count", coordinator.on_car_count_change)
    gui.register_callback("set_spawn_near_target", coordinator.set_spawn_near_target)
    gui.register_callback("set_point_speed", coordinator.set_point_speed)
    gui.register_callback("set_car_speed", coordinator.set_car_speed)

    # Logic loop
    def logic_loop():
        while running:
            start_time = time.time()

            # Get observations and process
            observations = env.observe()

            if observations:
                actions = coordinator.process(observations)
                env.apply_actions(actions)

                # Update GUI (only active cars)
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

    # Print instructions
    print("\n" + "=" * 50)
    print("Formation Control Simulation")
    print("=" * 50)
    print("Controls:")
    print("  Pattern dropdown : Switch between circle and figure-8")
    print("  Car Count slider : Adjust number of active robots")
    print("  Point Speed slider: Adjust target movement speed")
    print("  Car Speed slider  : Adjust robot movement speed")
    print("  Escape           : Quit")
    print("=" * 50)
    print(f"\nStarting with {coordinator.active_count} robots in {coordinator.pattern_type} pattern")

    # Run GUI (blocks until closed)
    gui.run()

    # Cleanup
    running = False
    print("\nDone.")


if __name__ == "__main__":
    main()
