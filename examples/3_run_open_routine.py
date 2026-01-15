#!/usr/bin/env python3
"""
Example 3: Open Routine (Non-looping Path)

This example demonstrates how to:
1. Create custom waypoint paths (not predefined patterns)
2. Run cars on non-looping (open) paths that complete at the end
3. Wait for all cars to complete their tasks
4. Reset and run a different routine

This is useful for tasks like:
- Point-to-point navigation
- Sequential task execution
- Cube pushing along a defined trajectory

Usage:
    python examples/3_run_open_routine.py
    python examples/3_run_open_routine.py --headless
"""
from __future__ import annotations

import argparse
import math
import time
from typing import Dict, List, Tuple

from micromvp.core.coordinator import Coordinator, CoordinatorConfig
from micromvp.core.models import CarConfig, Point
from micromvp.env.sim_env import SimConfig, SimEnvironment
from micromvp.utils.config import AppConfig, Boundary


def create_zigzag_path(
    start: Point,
    end: Point,
    num_turns: int = 3,
    amplitude: float = 50.0,
) -> List[Point]:
    """
    Create a zigzag path from start to end.

    Args:
        start: Starting point (x, y)
        end: Ending point (x, y)
        num_turns: Number of zigzag turns
        amplitude: How far to deviate from straight line

    Returns:
        List of waypoints
    """
    path = [start]
    dx = (end[0] - start[0]) / (num_turns + 1)
    dy = (end[1] - start[1]) / (num_turns + 1)

    # Direction perpendicular to path
    length = math.hypot(dx, dy)
    if length > 0:
        perp_x = -dy / length
        perp_y = dx / length
    else:
        perp_x, perp_y = 0, 1

    for i in range(1, num_turns + 1):
        # Alternate direction
        sign = 1 if i % 2 == 1 else -1
        x = start[0] + dx * i + perp_x * amplitude * sign
        y = start[1] + dy * i + perp_y * amplitude * sign
        path.append((x, y))

    path.append(end)
    return path


def create_custom_paths(boundary: Boundary, num_cars: int) -> List[List[Point]]:
    """
    Create custom open paths for each car.

    Cars will traverse different patterns across the field.
    """
    paths = []
    cx, cy = (boundary.left + boundary.right) / 2, (boundary.top + boundary.bottom) / 2
    margin = 50

    if num_cars >= 1:
        # Car 1: Zigzag from bottom-left to top-right
        paths.append(create_zigzag_path(
            (boundary.left + margin, boundary.bottom - margin),
            (boundary.right - margin, boundary.top + margin),
            num_turns=4,
            amplitude=80,
        ))

    if num_cars >= 2:
        # Car 2: Straight across horizontally
        paths.append([
            (boundary.left + margin, cy),
            (cx, cy),
            (boundary.right - margin, cy),
        ])

    if num_cars >= 3:
        # Car 3: Arc path (quarter circle)
        arc_points = []
        center = (cx, cy)
        radius = min(boundary.width, boundary.height) / 3
        for angle in range(0, 91, 10):
            rad = math.radians(angle)
            x = center[0] + radius * math.cos(rad)
            y = center[1] + radius * math.sin(rad)
            arc_points.append((x, y))
        paths.append(arc_points)

    # For additional cars, create diagonal paths
    for i in range(3, num_cars):
        offset = (i - 3) * 50
        paths.append([
            (boundary.left + margin + offset, boundary.top + margin + offset),
            (cx + offset, cy),
            (boundary.right - margin - offset, boundary.bottom - margin - offset),
        ])

    return paths


def run_headless(
    env: SimEnvironment,
    car_configs: List[CarConfig],
    paths: List[List[Point]],
) -> None:
    """Run the simulation without GUI."""
    print("=" * 60)
    print("Running Open Routine Example (Headless Mode)")
    print("=" * 60)

    coordinator = Coordinator(
        environment=env,
        car_configs=car_configs,
        config=CoordinatorConfig(control_hz=100.0, collision_avoidance=True),
    )

    coordinator.start()

    # =========================================================
    # Routine 1: Run custom open paths
    # =========================================================
    print("\n--- Routine 1: Custom open paths (non-looping) ---")

    # Set paths with loop=False (open paths)
    for i, car in enumerate(coordinator.cars):
        if i < len(paths):
            car.follow_path(paths[i], loop=False)
            print(f"  Car {car.robot_id}: {len(paths[i])} waypoints")

    print("\n  Running until all cars complete...")
    start_time = time.time()
    last_print = 0

    while not coordinator.all_tasks_done():
        elapsed = time.time() - start_time
        if int(elapsed) - last_print >= 2:
            last_print = int(elapsed)
            snapshot = coordinator.snapshot()
            print(f"\n  [t={elapsed:.0f}s]")
            for cs in snapshot.cars:
                print(f"    Car {cs.car_id}: ({cs.x:.0f}, {cs.y:.0f}), "
                      f"state={cs.task_state.name}")

        time.sleep(0.1)

        if elapsed > 60.0:
            print("  Timeout!")
            break

    print(f"\n  Routine 1 complete! ({time.time() - start_time:.1f}s)")

    # =========================================================
    # Routine 2: All cars return to center
    # =========================================================
    print("\n--- Routine 2: All cars return to center ---")

    boundary = AppConfig().boundary()
    cx = (boundary.left + boundary.right) / 2
    cy = (boundary.top + boundary.bottom) / 2

    # Use goto to bring all cars to center area
    for i, car in enumerate(coordinator.cars):
        offset = (i - len(coordinator.cars) // 2) * 60
        car.goto(x=cx + offset, y=cy, tolerance_pos=15.0)
        print(f"  Car {car.robot_id}: goto center")

    # Wait for completion
    start_time = time.time()
    while not coordinator.all_tasks_done():
        time.sleep(0.1)
        if time.time() - start_time > 30.0:
            print("  Timeout!")
            break

    print(f"  Routine 2 complete! ({time.time() - start_time:.1f}s)")

    # Final state
    print("\n--- Final Positions ---")
    snapshot = coordinator.snapshot()
    for cs in snapshot.cars:
        print(f"  Car {cs.car_id}: ({cs.x:.0f}, {cs.y:.0f})")

    coordinator.stop()
    print("\n" + "=" * 60)
    print("All routines complete!")


def run_with_gui(
    env: SimEnvironment,
    car_configs: List[CarConfig],
    paths: List[List[Point]],
    app_config: AppConfig,
) -> int:
    """Run with GUI."""
    from micromvp.ui.qt_app import run_app

    print("=" * 60)
    print("Running Open Routine Example (GUI Mode)")
    print("=" * 60)
    print("\nNote: This example uses open (non-looping) paths.")
    print("Cars will stop when they reach the end of their path.")
    print("Use the GUI to:\n")
    print("  - Click 'Apply Pattern' to set a new looping pattern")
    print("  - Draw new paths by clicking on the canvas")
    print()

    coordinator = Coordinator(
        environment=env,
        car_configs=car_configs,
        config=CoordinatorConfig(control_hz=100.0, collision_avoidance=True),
    )

    # Set initial open paths
    for i, car in enumerate(coordinator.cars):
        if i < len(paths):
            car.follow_path(paths[i], loop=False)

    coordinator.start()

    try:
        # Pass initial_paths for display (no goto_then_follow since cars start at path beginning)
        return run_app(
            config=app_config,
            coordinator=coordinator,
            initial_paths=paths,
            goto_then_follow=False,  # Paths already set, no goto phase needed
        )
    finally:
        coordinator.stop()


def main() -> int:
    parser = argparse.ArgumentParser(description="Open Routine Example")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--cars", type=int, default=3)
    args = parser.parse_args()

    app_config = AppConfig(
        sim=True,
        wheel_base=30.0,
        car_info=[(i + 1, i + 1) for i in range(args.cars)],
    )

    boundary = app_config.boundary()

    # Create environment with cars at starting positions of their paths
    paths = create_custom_paths(boundary, args.cars)

    initial_poses: Dict[int, Tuple[float, float, float]] = {}
    for i in range(args.cars):
        if i < len(paths) and paths[i]:
            x, y = paths[i][0]
            initial_poses[i + 1] = (x, y, 0.0)
        else:
            initial_poses[i + 1] = (200.0 + i * 100, 200.0, 0.0)

    sim_config = SimConfig(sim_speed=100.0, wheel_base=app_config.wheel_base)
    env = SimEnvironment(config=sim_config, initial_poses=initial_poses)

    car_configs = [
        CarConfig.from_wheel_base(robot_id=i + 1, tag_id=i + 1, wheel_base=app_config.wheel_base)
        for i in range(args.cars)
    ]

    print(f"Created {args.cars} cars with custom open paths")
    for i, path in enumerate(paths):
        print(f"  Car {i + 1}: {len(path)} waypoints")

    if args.headless:
        run_headless(env, car_configs, paths)
        return 0
    else:
        return run_with_gui(env, car_configs, paths, app_config)


if __name__ == "__main__":
    raise SystemExit(main())
