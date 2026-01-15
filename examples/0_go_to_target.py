#!/usr/bin/env python3
"""
Example 0: Go To Target (Single Car)

This is the simplest example demonstrating how to:
1. Create a simulation environment with a single car
2. Command the car to move to a target position
3. Optionally align to a specific orientation

This example shows the Coordinator API for single-car control.

Usage:
    # Run with GUI (default)
    python examples/0_go_to_target.py

    # Run headless (no GUI, prints to console)
    python examples/0_go_to_target.py --headless
"""
from __future__ import annotations

import argparse
import math
import time

from micromvp.core.coordinator import Coordinator, CoordinatorConfig
from micromvp.core.models import CarConfig
from micromvp.env.sim_env import SimConfig, SimEnvironment
from micromvp.utils.config import AppConfig


def run_headless(
    env: SimEnvironment,
    car_config: CarConfig,
    target_x: float,
    target_y: float,
    target_theta: float,
) -> int:
    """Run the go-to-target example without GUI."""
    print("=" * 60)
    print("Example 0: Go To Target (Headless Mode)")
    print("=" * 60)

    # Create coordinator with single car
    coordinator = Coordinator(
        environment=env,
        car_configs=[car_config],
        config=CoordinatorConfig(control_hz=100.0),
    )

    # Get the car and command it (using smooth Bezier curve)
    car = coordinator.cars[0]
    car.goto(
        x=target_x,
        y=target_y,
        theta=target_theta,
        tolerance_pos=10.0,
        tolerance_theta=0.1,
        smooth=True,  # Use smooth Bezier curve approach
    )
    print(f"\nTarget: ({target_x}, {target_y}), theta={math.degrees(target_theta):.0f}° (smooth curve)")

    # Start coordinator
    coordinator.start()

    # Wait for task completion
    print("\nMoving to target...")
    start_time = time.time()
    last_print = 0

    while not coordinator.all_tasks_done():
        elapsed = time.time() - start_time
        if int(elapsed) - last_print >= 1:
            last_print = int(elapsed)
            dist = math.hypot(target_x - car.x, target_y - car.y)
            print(f"  t={elapsed:.0f}s: pos=({car.x:.0f}, {car.y:.0f}), "
                  f"theta={math.degrees(car.theta):.0f}°, "
                  f"dist={dist:.1f}")

        time.sleep(0.1)
        if elapsed > 30.0:
            print("  Timeout!")
            break

    coordinator.stop()

    # Report results
    print("\n--- Results ---")
    print(f"  Final position: ({car.x:.1f}, {car.y:.1f})")
    print(f"  Final orientation: {math.degrees(car.theta):.1f}°")
    print(f"  Task state: {car.task_state.name}")

    final_dist = math.hypot(target_x - car.x, target_y - car.y)
    final_angle_err = abs(target_theta - car.theta)
    print(f"  Position error: {final_dist:.1f} pixels")
    print(f"  Angle error: {math.degrees(final_angle_err):.1f}°")

    success = car.is_task_done
    print(f"\n{'SUCCESS!' if success else 'FAILED (timeout)'}")
    print("=" * 60)

    return 0 if success else 1


def run_with_gui(
    env: SimEnvironment,
    car_config: CarConfig,
    target_x: float,
    target_y: float,
    target_theta: float,
    app_config: AppConfig,
) -> int:
    """Run the go-to-target example with GUI."""
    from micromvp.ui.qt_app import run_app

    print("=" * 60)
    print("Example 0: Go To Target (GUI Mode)")
    print("=" * 60)
    print("\nControls:")
    print("  - Speed slider: Adjust car speed")
    print("  - Run/Stop: Start/pause motion")
    print("  - Click on canvas: Add waypoints")
    print()

    # Create coordinator with single car
    coordinator = Coordinator(
        environment=env,
        car_configs=[car_config],
        config=CoordinatorConfig(control_hz=100.0),
    )

    # Get the car and command it to go to target (using smooth Bezier curve)
    car = coordinator.cars[0]
    car.goto(
        x=target_x,
        y=target_y,
        theta=target_theta,
        tolerance_pos=10.0,
        tolerance_theta=0.1,
        smooth=True,  # Use smooth Bezier curve approach
    )
    print(f"Target: ({target_x}, {target_y}), theta={math.degrees(target_theta):.0f}° (smooth curve)")

    # Start coordinator BEFORE GUI
    coordinator.start()

    print("Starting GUI...\n")
    try:
        return run_app(config=app_config, coordinator=coordinator)
    finally:
        coordinator.stop()
        env.close()


def main() -> int:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Go To Target Example - Single car navigates to a target pose",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without GUI (console output only)",
    )
    args = parser.parse_args()

    # Configuration
    wheel_base = 30.0
    initial_x, initial_y, initial_theta = 100.0, 100.0, 0.0
    target_x, target_y = 500.0, 400.0
    target_theta = math.pi / 2  # Face upward (90 degrees)

    # Create app config
    app_config = AppConfig(
        sim=True,
        wheel_base=wheel_base,
        sim_speed=100.0,
        v_max=1.0,
        car_info=[(1, 1)],  # Single car
    )

    # Create environment
    sim_config = SimConfig(
        sim_speed=100.0,
        wheel_base=wheel_base,
    )
    initial_poses = {1: (initial_x, initial_y, initial_theta)}
    env = SimEnvironment(config=sim_config, initial_poses=initial_poses)

    # Create car config
    car_config = CarConfig.from_wheel_base(
        robot_id=1,
        tag_id=1,
        wheel_base=wheel_base,
        v_max=1.0,
    )

    print(f"Created car at ({initial_x}, {initial_y})")

    if args.headless:
        return run_headless(env, car_config, target_x, target_y, target_theta)
    else:
        return run_with_gui(env, car_config, target_x, target_y, target_theta, app_config)


if __name__ == "__main__":
    raise SystemExit(main())
