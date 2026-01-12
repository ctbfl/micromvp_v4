#!/usr/bin/env python3
"""
Visual test for Dubins path planning.

Generates random start/goal poses and visualizes:
- The computed Dubins path
- Sampled waypoints with front bumper indicators (perpendicular to heading)
- Arrows showing orientation direction at start/goal
"""

import math
import random

import matplotlib.pyplot as plt
import numpy as np

# Add project to path for imports
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from micromvp.core.ddr import compute_dubins_path, sample_dubins_path


def draw_car_bumper(ax, x, y, theta, width=20, color='blue', alpha=1.0, linewidth=2):
    """
    Draw a short line representing the car's front bumper (perpendicular to heading).

    For a differential drive car, theta is the direction of motion.
    The front bumper is perpendicular to this direction.
    """
    # Perpendicular direction (rotate theta by 90 degrees)
    perp_theta = theta + math.pi / 2
    half_width = width / 2

    # Bumper endpoints
    x1 = x + half_width * math.cos(perp_theta)
    y1 = y + half_width * math.sin(perp_theta)
    x2 = x - half_width * math.cos(perp_theta)
    y2 = y - half_width * math.sin(perp_theta)

    ax.plot([x1, x2], [y1, y2], color=color, linewidth=linewidth, alpha=alpha)


def draw_pose_arrow(ax, x, y, theta, length=30, color='blue', alpha=1.0):
    """
    Draw an arrow showing position and orientation.
    Arrow points in the direction of theta (heading/motion direction).
    """
    dx = length * math.cos(theta)
    dy = length * math.sin(theta)

    ax.annotate('', xy=(x + dx, y + dy), xytext=(x, y),
                arrowprops=dict(arrowstyle='->', color=color, lw=2, alpha=alpha))
    ax.plot(x, y, 'o', color=color, markersize=8, alpha=alpha)


def visualize_dubins_path(start, goal, min_radius=45.0):
    """
    Visualize a Dubins path from start to goal.

    Args:
        start: (x, y, theta) tuple
        goal: (x, y, theta) tuple
        min_radius: minimum turn radius
    """
    start_x, start_y, start_theta = start
    goal_x, goal_y, goal_theta = goal

    # Compute Dubins path
    path = compute_dubins_path(
        start_x, start_y, start_theta,
        goal_x, goal_y, goal_theta,
        min_radius
    )

    if path is None:
        print("No valid Dubins path found!")
        return None, None

    print(f"Path type: {path.path_type}")
    print(f"Segments (normalized): {path.segments}")
    print(f"Total length: {path.total_length:.1f} pixels")

    # Sample waypoints
    waypoints = sample_dubins_path(start_x, start_y, start_theta, path, step_size=10.0)

    # Create figure
    fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (pixels)')
    ax.set_ylabel('Y (pixels)')
    ax.set_title(f'Dubins Path: {path.path_type} (length={path.total_length:.1f})')

    # Draw path as continuous line
    xs = [wp[0] for wp in waypoints]
    ys = [wp[1] for wp in waypoints]
    ax.plot(xs, ys, 'b-', linewidth=1.5, alpha=0.5, label='Path')

    # Draw sampled waypoints with front bumper indicators
    for i, (wx, wy, wtheta) in enumerate(waypoints):
        if i % 5 == 0:  # Draw every 5th waypoint to avoid clutter
            alpha = 0.3 + 0.5 * (i / len(waypoints))  # Fade in along path
            draw_car_bumper(ax, wx, wy, wtheta, width=15, color='steelblue', alpha=alpha)

    # Draw start pose with arrow (green)
    draw_pose_arrow(ax, start_x, start_y, start_theta, length=35, color='green')
    draw_car_bumper(ax, start_x, start_y, start_theta, width=25, color='green', linewidth=3)

    # Draw goal pose with arrow (red)
    draw_pose_arrow(ax, goal_x, goal_y, goal_theta, length=35, color='red')
    draw_car_bumper(ax, goal_x, goal_y, goal_theta, width=25, color='red', linewidth=3)

    # Legend
    ax.plot([], [], 'g-', linewidth=2, label='Start pose')
    ax.plot([], [], 'r-', linewidth=2, label='Goal pose')
    ax.legend(loc='upper right')

    # Auto-scale with some padding
    all_x = xs + [start_x, goal_x]
    all_y = ys + [start_y, goal_y]
    margin = min_radius * 1.5
    ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
    ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

    return fig, ax


def random_pose(x_range=(100, 500), y_range=(100, 400)):
    """Generate a random pose within given ranges."""
    x = random.uniform(*x_range)
    y = random.uniform(*y_range)
    theta = random.uniform(0, 2 * math.pi)
    return (x, y, theta)


def main():
    # Field boundaries (similar to actual app)
    x_range = (100, 200)
    y_range = (100, 200)
    min_radius = 45.0  # Same as config default

    # Generate random start and goal poses (different each run)
    start = random_pose(x_range, y_range)
    goal = random_pose(x_range, y_range)

    print("=" * 50)
    print("Dubins Path Planning Test")
    print("=" * 50)
    print(f"Start: x={start[0]:.1f}, y={start[1]:.1f}, theta={math.degrees(start[2]):.1f} deg")
    print(f"Goal:  x={goal[0]:.1f}, y={goal[1]:.1f}, theta={math.degrees(goal[2]):.1f} deg")
    print(f"Min turn radius: {min_radius}")
    print()

    # Visualize
    fig, ax = visualize_dubins_path(start, goal, min_radius)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
