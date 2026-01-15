#!/usr/bin/env python3
"""
Example 1: Circle Loop with Multiple Cars

This example demonstrates how to use the MicroMVP API to:
1. Create a simulation environment with multiple cars
2. Generate a circle path pattern
3. Have cars first move to their starting positions (goto)
4. Then follow the circle path continuously (follow_path)

The example shows both headless mode (no GUI) and GUI mode.

Usage:
    # Run with GUI (default)
    python examples/1_run_circle_loop.py

    # Run headless (no GUI, prints to console)
    python examples/1_run_circle_loop.py --headless

    # Specify number of cars
    python examples/1_run_circle_loop.py --cars 3
"""
from __future__ import annotations

import argparse
import math
import time
from typing import Dict, List, Tuple

# Core imports
from micromvp.core.coordinator import Coordinator, CoordinatorConfig
from micromvp.core.models import CarConfig, Point
from micromvp.core.patterns import circle_pattern
from micromvp.core.planner import random_arrangement

# Environment imports
from micromvp.env.sim_env import SimConfig, SimEnvironment

# UI imports (optional)
from micromvp.utils.config import AppConfig, Boundary


def create_environment(
    num_cars: int,
    boundary: Boundary,
    wheel_base: float,
) -> Tuple[SimEnvironment, List[CarConfig]]:
    """
    Create simulation environment with random initial car positions.

    Args:
        num_cars: Number of cars to create
        boundary: World boundary
        wheel_base: Car wheel base (pixels)

    Returns:
        Tuple of (environment, car_configs)
    """
    # Generate random starting positions
    starts = random_arrangement(num_cars, boundary, wheel_base * 2)

    # Create initial poses dict: {tag_id: (x, y, theta)}
    initial_poses: Dict[int, Tuple[float, float, float]] = {}
    for i, (x, y, theta) in enumerate(starts):
        tag_id = i + 1  # tag_id starts from 1
        initial_poses[tag_id] = (x, y, theta)

    # Create simulation config
    sim_config = SimConfig(
        sim_speed=100.0,      # Simulation speed multiplier
        wheel_base=wheel_base,
        max_dt=0.05,          # Maximum time step
    )

    # Create environment
    env = SimEnvironment(config=sim_config, initial_poses=initial_poses)

    # Create car configs
    car_configs = [
        CarConfig.from_wheel_base(
            robot_id=i + 1,
            tag_id=i + 1,
            wheel_base=wheel_base,
            v_max=1.0,
        )
        for i in range(num_cars)
    ]

    return env, car_configs


def generate_circle_paths(
    num_cars: int,
    boundary: Boundary,
) -> List[List[Point]]:
    """
    Generate circle paths for multiple cars.

    Each car gets a path starting from a different point on the circle,
    evenly distributed around the circumference.

    Args:
        num_cars: Number of cars
        boundary: World boundary

    Returns:
        List of paths (one per car)
    """
    return circle_pattern(num_cars, boundary)


def get_path_start_points(paths: List[List[Point]]) -> List[Point]:
    """Extract the starting point of each path."""
    return [path[0] for path in paths]


def run_headless(
    env: SimEnvironment,
    car_configs: List[CarConfig],
    paths: List[List[Point]],
    duration: float = 30.0,
) -> None:
    """
    Run the simulation without GUI (headless mode).

    This demonstrates direct use of Coordinator API.

    Args:
        env: Simulation environment
        car_configs: Car configurations
        paths: Paths for each car
        duration: How long to run (seconds)
    """
    print("=" * 60)
    print("Running Circle Loop Example (Headless Mode)")
    print("=" * 60)

    # Create coordinator
    coordinator_config = CoordinatorConfig(
        control_hz=100.0,
        collision_avoidance=True,
    )
    coordinator = Coordinator(
        environment=env,
        car_configs=car_configs,
        config=coordinator_config,
    )

    # Get start points from paths
    start_points = get_path_start_points(paths)

    # =========================================================
    # Phase 1: Move cars to their starting positions (smooth)
    # =========================================================
    print("\n--- Phase 1: Moving to start positions (smooth curve) ---")

    # Command each car to go to its starting position with correct heading
    for i, car in enumerate(coordinator.cars):
        car.prepare_for_path(paths[i], tolerance_pos=15.0)
        start_x, start_y = start_points[i]
        print(f"  Car {car.robot_id}: smooth goto ({start_x:.0f}, {start_y:.0f})")

    # Start the coordinator background thread
    coordinator.start()

    # Wait for all cars to reach their start positions
    print("\n  Waiting for all cars to reach start positions...")
    start_time = time.time()
    while not coordinator.all_tasks_done():
        # Print progress every second
        elapsed = time.time() - start_time
        if int(elapsed) % 2 == 0:
            snapshot = coordinator.snapshot()
            for car_state in snapshot.cars:
                print(f"    Car {car_state.car_id}: "
                      f"pos=({car_state.x:.0f}, {car_state.y:.0f}), "
                      f"state={car_state.task_state.name}")

        time.sleep(0.5)

        # Timeout protection
        if elapsed > 30.0:
            print("  Timeout waiting for cars to reach start!")
            break

    print(f"\n  All cars at start positions! (took {time.time() - start_time:.1f}s)")

    # =========================================================
    # Phase 2: Follow circle path
    # =========================================================
    print("\n--- Phase 2: Following circle path ---")

    # Set paths for all cars (looping)
    coordinator.set_paths(paths, loop=True)
    print("  Paths set. Cars now following circle...")

    # Run for specified duration
    print(f"\n  Running for {duration} seconds...")
    start_time = time.time()
    last_print = 0

    while (time.time() - start_time) < duration:
        elapsed = time.time() - start_time

        # Print status every 3 seconds
        if int(elapsed) - last_print >= 3:
            last_print = int(elapsed)
            snapshot = coordinator.snapshot()
            print(f"\n  [t={elapsed:.0f}s]")
            for car_state in snapshot.cars:
                speed = (car_state.l_speed + car_state.r_speed) / 2
                print(f"    Car {car_state.car_id}: "
                      f"({car_state.x:.0f}, {car_state.y:.0f}), "
                      f"theta={math.degrees(car_state.theta):.0f}°, "
                      f"speed={speed:.2f}")

        time.sleep(0.1)

    # Cleanup
    coordinator.stop()
    print("\n" + "=" * 60)
    print("Simulation complete!")
    print("=" * 60)


def run_with_gui(
    env: SimEnvironment,
    car_configs: List[CarConfig],
    paths: List[List[Point]],
    app_config: AppConfig,
) -> None:
    """
    Run the simulation with PyQt6 GUI.

    Args:
        env: Simulation environment
        car_configs: Car configurations
        paths: Paths for each car
        app_config: Application config for GUI
    """
    from micromvp.ui.qt_app import run_app

    print("=" * 60)
    print("Running Circle Loop Example (GUI Mode)")
    print("=" * 60)
    print("\nControls:")
    print("  - Speed slider: Adjust car speed (0-200%)")
    print("  - Run/Stop: Start/pause motion")
    print("  - Clear: Clear all paths")
    print("  - Pattern: Apply circle or figure-8 pattern")
    print("  - Click on canvas: Add waypoints to selected car")
    print()

    # Create coordinator
    coordinator_config = CoordinatorConfig(
        control_hz=100.0,
        collision_avoidance=True,
    )
    coordinator = Coordinator(
        environment=env,
        car_configs=car_configs,
        config=coordinator_config,
    )

    # Get start points from paths
    start_points = get_path_start_points(paths)

    # Command cars to go to start positions first (smooth curve approach)
    print("Phase 1: Commanding cars to move to start positions (smooth curve)...")
    for i, car in enumerate(coordinator.cars):
        car.prepare_for_path(paths[i], tolerance_pos=15.0)
        start_x, start_y = start_points[i]
        print(f"  Car {car.robot_id}: smooth goto ({start_x:.0f}, {start_y:.0f})")

    # Start coordinator BEFORE GUI so user sees the move-to-start phase
    coordinator.start()

    print("Starting GUI (cars moving to start positions)...\n")

    # Run GUI (blocking) - pass paths for initial display
    # The GUI will show cars moving to start, then we need to set paths after
    try:
        exit_code = run_app(
            config=app_config,
            coordinator=coordinator,
            initial_paths=paths,
            goto_then_follow=True,  # Enable goto-then-follow mode
        )
    finally:
        coordinator.stop()
        env.close()

    return exit_code


def main() -> int:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Circle Loop Example - Multiple cars following a circle path",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without GUI (console output only)",
    )
    parser.add_argument(
        "--cars",
        type=int,
        default=5,
        help="Number of cars (default: 5)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Duration in headless mode (default: 30s)",
    )
    args = parser.parse_args()

    # Create app config
    app_config = AppConfig(
        sim=True,
        wheel_base=30.0,
        sim_speed=100.0,
        v_max=1.0,
        car_info=[(i + 1, i + 1) for i in range(args.cars)],
    )

    # Get boundary
    boundary = app_config.boundary()

    # Create environment and car configs
    env, car_configs = create_environment(
        num_cars=args.cars,
        boundary=boundary,
        wheel_base=app_config.wheel_base,
    )

    # Generate circle paths
    paths = generate_circle_paths(args.cars, boundary)

    print(f"Created {args.cars} cars")
    print(f"Circle path has {len(paths[0])} waypoints")

    if args.headless:
        run_headless(env, car_configs, paths, duration=args.duration)
        return 0
    else:
        return run_with_gui(env, car_configs, paths, app_config)


if __name__ == "__main__":
    raise SystemExit(main())
