"""
Test navigation control with web server API and obstacle avoidance.

This example demonstrates:
- NavigationController with path following and in-place rotation
- NavigationCoordinator with web server API
- RVG-based path planning with obstacle avoidance
- GUI controls for speed and rotation

Features:
- Draw paths on canvas for robot to follow
- Web server API on port 8080:
  - POST /setup_obstacle - Set obstacles
  - POST /goto - Navigate to position with orientation
  - POST /follow_path - Follow a waypoint sequence
  - GET /status - Get current robot status
- Speed slider to adjust movement speed
- Rotation input to rotate robot to target angle
- GUI commands take priority over web server

Controls:
- Draw curve: Robot follows the drawn path
- Speed slider: Adjust robot movement speed
- Rotation input: Enter angle (0-359.99) and press Enter to rotate
- C: Clear current task
- Space: Emergency stop
- Escape: Quit

Prerequisites:
- Camera calibration file at configured path
- Workspace markers (5x5 ArUco) placed on table
- Robot(s) with 4x4 ArUco markers
- (Optional) RVG library for obstacle avoidance path planning

Web API Examples:
    # Set obstacles (rectangles defined by 4 counter-clockwise points)
    curl -X POST http://localhost:8080/setup_obstacle \\
        -H "Content-Type: application/json" \\
        -d '{"obstacles": [[[100,100], [200,100], [200,200], [100,200]]]}'

    # Navigate to position with final orientation
    curl -X POST http://localhost:8080/goto \\
        -H "Content-Type: application/json" \\
        -d '{"x": 300, "y": 200, "theta": 45}'

    # Follow a path of waypoints
    curl -X POST http://localhost:8080/follow_path \\
        -H "Content-Type: application/json" \\
        -d '{"path": [[100,100], [200,150], [300,100]]}'

    # Get robot status
    curl http://localhost:8080/status
"""
import threading
import time
import argparse

from micromvp.gui import MVPWindow
from micromvp.env import RealPushEnv, RealPushConfig
from micromvp.controller import NavigationController
from micromvp.coordinator import NavigationCoordinator
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication


def main():
    parser = argparse.ArgumentParser(description="Real robot navigation test with web API")
    parser.add_argument(
        "--robot", type=int, default=1,
        help="Robot ID to control (default: 1)"
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
    parser.add_argument(
        "--port", type=int, default=8080,
        help="Web server port (default: 8080)"
    )
    args = parser.parse_args()

    robot_id = args.robot
    print(f"Configuring robot {robot_id}")
    print(f"Web server will run on port {args.port}")

    # Setup real environment
    config = RealPushConfig(
        robot_ids=[robot_id],
        camera_device=args.camera,
        calibration_file=args.calib,
        warmup_frames=args.warmup,
        no_preview=args.no_preview,
    )
    env = RealPushEnv(config)

    # Start environment
    print("\nStarting environment, wait workspace settle...")

    if not env.start(wait_for_ready=True, timeout=10.0):
        print("Failed to start environment (or camera issue)")
        return

    ws_config = env.workspace_config
    print("Environment ready! Starting controller...")

    # Create navigation controller
    controllers = {
        robot_id: NavigationController(
            robot_id,
            ws_config,
            lookahead_distance=args.lookahead,
            max_speed=args.max_speed,
        )
    }

    # Create navigation coordinator with web server
    coordinator = NavigationCoordinator(
        ws_config,
        controllers,
        active_robot_id=robot_id,
        webserver_port=args.port,
    )

    # Setup GUI with controls
    gui_config = {
        "canvas": {
            "click_canvas_callback": True,
            "draw_curve_callback": True,
        },
        "control_panel": [
            {"type": "label", "text": "=== Navigation ==="},
            {"type": "label", "text": "Draw path to follow"},
            {"type": "label", "text": ""},
            {
                "type": "continuous_slider",
                "label": "Robot Speed",
                "range": [0.0, 1.0],
                "default": args.max_speed,
                "callback_name": "set_robot_speed",
            },
            {"type": "label", "text": ""},
            {
                "type": "input",
                "label": "Rotate to:",
                "placeholder": "0-359.99",
                "callback_name": "set_rotation_target",
            },
            {
                "type": "dynamic_label", 
                "title": "AP Status",
                "widget_name": "ap_status_display",
                "default": "N/A"
            },
            {"type": "label", "text": ""},
            {"type": "label", "text": "=== Web API ==="},
            {"type": "label", "text": f"Port: {args.port}"},
            {"type": "label", "text": "POST /setup_obstacle"},
            {"type": "label", "text": "POST /goto"},
            {"type": "label", "text": "POST /follow_path"},
            {"type": "label", "text": "GET /status"},
            {"type": "label", "text": ""},
            {"type": "label", "text": "=== Keyboard ==="},
            {"type": "label", "text": "  C: Clear task"},
            {"type": "label", "text": "  Space: Stop"},
            {"type": "label", "text": "  Escape: Quit"},
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
    gui.register_callback("set_robot_speed", coordinator.on_speed_change)
    gui.register_callback("set_rotation_target", coordinator.on_rotation_input)
    gui.register_callback("on_canvas_click", coordinator.on_canvas_click)

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
    print("\n" + "=" * 60)
    print("Real Robot Navigation with Web API")
    print("=" * 60)
    print(f"\nWeb Server running at http://localhost:{args.port}")
    print("\nAPI Endpoints:")
    print("  POST /setup_obstacle - Set obstacle geometry")
    print("  POST /goto           - Navigate to position with orientation")
    print("  POST /follow_path    - Follow a waypoint sequence")
    print("  GET  /status         - Get robot status")
    print("\nGUI Controls:")
    print("  Draw curve: Robot follows the path")
    print("  Speed slider: Adjust movement speed")
    print("  Rotation input: Enter angle and press Enter")
    print("  C: Clear current task")
    print("  Space: Emergency stop")
    print("  Escape: Quit")
    print("\nNOTE: GUI commands take priority over web API commands")
    print("=" * 60)
    print("\nWaiting for robot to be detected...")

    # QTimer callback runs in Qt main thread
    def main_thread_update():
        """Called by QTimer in the Qt main thread."""
        if not running:
            return

        # Render CV preview
        env.render()

        # Update Qt GUI
        with obs_lock:
            obs = latest_observations.copy() if latest_observations else {}

        if obs:
            micromvp_ap_status = env.get_micromvp_ap_status()
            gui.update_widget_text("ap_status_display", micromvp_ap_status)
            car_states = {s.car_id: s for s in coordinator.gather_car_state()}
            drawings = coordinator.get_additional_drawings()
            gui.update(car_states, drawings)

    # Create timer
    render_timer = QTimer()
    render_timer.timeout.connect(main_thread_update)
    render_timer.start(33)  # ~30 FPS

    print("\nDraw a path or use the web API to control the robot!")

    # Run GUI (blocks until closed)
    gui.run()

    # Cleanup
    running = False
    render_timer.stop()
    print("\nStopping...")
    coordinator.close()
    env.close()
    print("Done.")


if __name__ == "__main__":
    main()
