"""
Test path-following control of real robots.

This example uses:
- ArUco camera observation (with warmup)
- Serial action sending to robots (by robot ID)
- User-drawn path following via pure pursuit

Controls:
- 1-4: Select robot by number
- Tab: Cycle to next robot
- Click car: Select robot
- Draw curve: Robot follows the drawn path
- C: Clear path of selected robot
- Space: Emergency stop all robots
- Escape: Quit

Prerequisites:
- Camera calibration file at configured path
- Workspace markers (5x5 ArUco) placed on table
- Robot(s) with 4x4 ArUco markers
"""
import threading
import time
import argparse

from micromvp.gui import MVPWindow
from micromvp.env import RealPushEnv, RealPushConfig
from micromvp.controller import PurePursuitFollowPathController
from micromvp.coordinator import FollowPathCoordinator
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication


def main():
    parser = argparse.ArgumentParser(description="Real robot path following test")
    parser.add_argument(
        "--robots", type=str, default="1",
        help="Robot IDs as 'id,id,...' (e.g., '1,2,3')"
    )
    parser.add_argument(
        "--camera", type=int, default=0,
        help="Camera device index (default: 0)"
    )
    parser.add_argument(
        "--warmup", type=int, default=30,
        help="Warmup frames for workspace pose (0=dynamic mode, default: 30)"
    )
    parser.add_argument(
        "--no-preview", action="store_true", default=False,
        help="Disable camera preview window"
    )
    parser.add_argument(
        "--calib", type=str,
        default="/home/omen/junshan/micromvp_push/micromvp_v4/src/micromvp/env/real_push_env/camera.yaml",
        help="Camera calibration file path"
    )
    parser.add_argument(
        "--max-speed", type=float, default=0.3,
        help="Maximum robot speed [0-1] (default: 0.3)"
    )
    parser.add_argument(
        "--lookahead", type=float, default=None,
        help="Lookahead distance for pure pursuit (default: 1.5 * car_size)"
    )
    args = parser.parse_args()

    # Parse robot IDs
    robot_ids = []
    for entry in args.robots.split(","):
        entry = entry.strip()
        if entry:
            robot_ids.append(int(entry))

    if not robot_ids:
        print("No robots specified. Use --robots 'id,id,...'")
        return

    print(f"Configuring {len(robot_ids)} robot(s):")
    for rid in robot_ids:
        print(f"  Robot {rid}")

    # Setup real environment
    config = RealPushConfig(
        robot_ids=robot_ids,
        camera_device=args.camera,
        calibration_file=args.calib,
        warmup_frames=args.warmup,
        no_preview=args.no_preview,
    )
    env = RealPushEnv(config)

    # Start environment (this runs warmup if configured)
    print("\nStarting environment, wait workspace settle...")

    if not env.start(wait_for_ready=True, timeout=10.0):
        print("Failed to start environment (or camera issue)")
        return

    ws_config = env.workspace_config
    print("Environment ready! Starting controllers...")

    # Create path-following controllers for each robot
    controllers = {
        robot_id: PurePursuitFollowPathController(
            robot_id,
            ws_config,
            lookahead_distance=args.lookahead,
            max_speed=args.max_speed,
        )
        for robot_id in ws_config.car_id_list
    }

    # Create follow path coordinator
    coordinator = FollowPathCoordinator(ws_config, controllers)

    # Setup GUI with curve drawing enabled
    gui_config = {
        "canvas": {
            "click_canvas_callback": True,
            "draw_curve_callback": True,  # Enable curve drawing
        },
        "control_panel": [
            {"type": "label", "text": "=== Path Following ==="},
            {"type": "label", "text": "Click robot to select"},
            {"type": "label", "text": "Draw curve to set path"},
            {"type": "label", "text": ""},
            {
                "type": "continuous_slider",
                "label": "Robot Speed",
                "range": [0.0, 1.0],
                "default": args.max_speed,
                "callback_name": "set_robot_speed",
            },
            {"type": "label", "text": ""},
            {"type": "label", "text": "Keyboard:"},
            {"type": "label", "text": "  1-9: Select robot"},
            {"type": "label", "text": "  Tab: Next robot"},
            {"type": "label", "text": "  C: Clear path"},
            {"type": "label", "text": "  Space: Stop all"},
            {"type": "label", "text": ""},
            {"type": "label", "text": f"Robots: {len(robot_ids)}"},
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
        else:
            coordinator.on_key_press(key)

    gui.register_callback("on_key_press", on_key_press)
    gui.register_callback("on_key_release", coordinator.on_key_release)
    gui.register_callback("on_car_click", coordinator.on_car_click)
    gui.register_callback("on_curve_drawn", coordinator.on_curve_drawn)
    gui.register_callback("set_robot_speed", coordinator.set_all_speeds)

    # Shared state between threads
    latest_observations = {}
    obs_lock = threading.Lock()

    # Logic loop
    def logic_loop():
        while running:
            start_time = time.time()

            # Get observations and process
            observations = env.observe()

            # Only process if we have observations
            if observations:
                actions = coordinator.process(observations)
                env.apply_actions(actions)

                # Update shared state for GUI thread
                with obs_lock:
                    latest_observations.update(observations)

            # Maintain frequency
            elapsed = time.time() - start_time
            sleep_time = max(0, (1 / ws_config.frequency) - elapsed)
            time.sleep(sleep_time)

    # Start logic thread
    logic_thread = threading.Thread(target=logic_loop, daemon=True)
    logic_thread.start()

    # Print instructions
    print("\n" + "=" * 50)
    print("Real Robot Path Following")
    print("=" * 50)
    print("Controls:")
    print("  1-4       : Select robot")
    print("  Tab       : Cycle to next robot")
    print("  Click car : Select robot")
    print("  Draw curve: Set path for selected robot")
    print("  C         : Clear path of selected robot")
    print("  Space     : Emergency stop all")
    print("  Escape    : Quit")
    print("=" * 50)
    print("\nWaiting for robots to be detected...")

    # QTimer callback runs in Qt main thread
    def main_thread_update():
        """Called by QTimer in the Qt main thread."""
        if not running:
            return

        # Render CV preview (safe: we're in main thread now!)
        env.render()

        # Update Qt GUI
        with obs_lock:
            obs = latest_observations.copy() if latest_observations else {}

        if obs:
            car_states = {s.car_id: s for s in coordinator.gather_car_state()}
            drawings = coordinator.get_additional_drawings()
            gui.update(car_states, drawings)

    # Create timer AFTER gui is created (so QApplication exists)
    render_timer = QTimer()
    render_timer.timeout.connect(main_thread_update)
    render_timer.start(33)  # ~30 FPS

    print("\nDraw a path for the selected robot to follow!")

    # Run GUI (blocks until closed)
    gui.run()

    # Cleanup
    running = False
    render_timer.stop()
    print("\nStopping...")
    env.close()
    print("Done.")


if __name__ == "__main__":
    main()
