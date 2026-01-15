#!/usr/bin/env python3
"""
Example 2: Figure-8 Loop with Multiple Cars

This example demonstrates how to:
1. Use figure-8 (infinity/lemniscate) pattern
2. Coordinate multiple cars on the same path
3. Handle collision avoidance when cars pass each other

The figure-8 pattern is more challenging because cars will cross
paths at the center, requiring the collision avoidance system.

Usage:
    python examples/2_run_8shape_loop.py
    python examples/2_run_8shape_loop.py --headless --cars 4
"""
from __future__ import annotations

import argparse
import time
from typing import Dict, List, Tuple

from micromvp.core.coordinator import Coordinator, CoordinatorConfig
from micromvp.core.models import CarConfig, Point
from micromvp.core.patterns import figure8_pattern
from micromvp.core.planner import random_arrangement
from micromvp.env.sim_env import SimConfig, SimEnvironment
from micromvp.utils.config import AppConfig, Boundary


def create_environment(
    num_cars: int,
    boundary: Boundary,
    wheel_base: float,
) -> Tuple[SimEnvironment, List[CarConfig]]:
    """Create simulation environment with random initial positions."""
    starts = random_arrangement(num_cars, boundary, wheel_base * 2)

    initial_poses: Dict[int, Tuple[float, float, float]] = {
        i + 1: (x, y, theta) for i, (x, y, theta) in enumerate(starts)
    }

    sim_config = SimConfig(sim_speed=100.0, wheel_base=wheel_base)
    env = SimEnvironment(config=sim_config, initial_poses=initial_poses)

    car_configs = [
        CarConfig.from_wheel_base(robot_id=i + 1, tag_id=i + 1, wheel_base=wheel_base)
        for i in range(num_cars)
    ]

    return env, car_configs


def run_headless(
    env: SimEnvironment,
    car_configs: List[CarConfig],
    paths: List[List[Point]],
    duration: float = 30.0,
) -> None:
    """Run the simulation without GUI."""
    print("=" * 60)
    print("Running Figure-8 Loop Example (Headless Mode)")
    print("=" * 60)

    coordinator = Coordinator(
        environment=env,
        car_configs=car_configs,
        config=CoordinatorConfig(control_hz=100.0, collision_avoidance=True),
    )

    # Phase 1: Move to start positions (smooth curve)
    print("\n--- Phase 1: Moving to start positions (smooth curve) ---")
    start_points = [path[0] for path in paths]
    for i, car in enumerate(coordinator.cars):
        car.prepare_for_path(paths[i], tolerance_pos=15.0)
        x, y = start_points[i]
        print(f"  Car {car.robot_id}: smooth goto ({x:.0f}, {y:.0f})")

    coordinator.start()

    # Wait for all cars
    start_time = time.time()
    while not coordinator.all_tasks_done():
        time.sleep(0.1)
        if time.time() - start_time > 30.0:
            print("  Timeout!")
            break

    print(f"\n  All cars at start! ({time.time() - start_time:.1f}s)")

    # Phase 2: Follow figure-8 path
    print("\n--- Phase 2: Following figure-8 path ---")
    print("  Note: Collision avoidance active at crossing point!")
    coordinator.set_paths(paths, loop=True)

    print(f"\n  Running for {duration} seconds...")
    start_time = time.time()
    last_print = 0

    while (time.time() - start_time) < duration:
        elapsed = time.time() - start_time
        if int(elapsed) - last_print >= 5:
            last_print = int(elapsed)
            snapshot = coordinator.snapshot()
            print(f"\n  [t={elapsed:.0f}s]")
            for cs in snapshot.cars:
                print(f"    Car {cs.car_id}: ({cs.x:.0f}, {cs.y:.0f})")
        time.sleep(0.1)

    coordinator.stop()
    print("\n" + "=" * 60)
    print("Complete!")


def run_with_gui(
    env: SimEnvironment,
    car_configs: List[CarConfig],
    paths: List[List[Point]],
    app_config: AppConfig,
) -> int:
    """Run with GUI."""
    from micromvp.ui.qt_app import run_app

    print("=" * 60)
    print("Running Figure-8 Loop Example (GUI Mode)")
    print("=" * 60)

    coordinator = Coordinator(
        environment=env,
        car_configs=car_configs,
        config=CoordinatorConfig(control_hz=100.0, collision_avoidance=True),
    )

    # Command cars to go to start positions first (smooth curve approach)
    print("Commanding cars to move to start positions (smooth curve)...")
    start_points = [path[0] for path in paths]
    for i, car in enumerate(coordinator.cars):
        car.prepare_for_path(paths[i], tolerance_pos=15.0)
        print(f"  Car {car.robot_id}: smooth goto ({start_points[i][0]:.0f}, {start_points[i][1]:.0f})")

    # Start coordinator BEFORE GUI so user sees the move-to-start phase
    coordinator.start()

    print("Starting GUI (cars moving to start positions)...\n")
    try:
        return run_app(
            config=app_config,
            coordinator=coordinator,
            initial_paths=paths,
            goto_then_follow=True,  # Enable goto-then-follow mode
        )
    finally:
        coordinator.stop()


def main() -> int:
    parser = argparse.ArgumentParser(description="Figure-8 Loop Example")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--cars", type=int, default=5)
    parser.add_argument("--duration", type=float, default=30.0)
    args = parser.parse_args()

    app_config = AppConfig(
        sim=True,
        wheel_base=30.0,
        car_info=[(i + 1, i + 1) for i in range(args.cars)],
    )

    boundary = app_config.boundary()
    env, car_configs = create_environment(args.cars, boundary, app_config.wheel_base)
    paths = figure8_pattern(args.cars, boundary)

    print(f"Created {args.cars} cars")
    print(f"Figure-8 path has {len(paths[0])} waypoints")

    if args.headless:
        run_headless(env, car_configs, paths, args.duration)
        return 0
    else:
        return run_with_gui(env, car_configs, paths, app_config)


if __name__ == "__main__":
    raise SystemExit(main())
