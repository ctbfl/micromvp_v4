"""
Differential Drive Robot (DDR) kinematics for simulation.

This module provides the physics simulation for differential-drive robots.
All angles are in radians internally, with theta=0 pointing to +X direction.
"""
from __future__ import annotations

import math
from typing import Tuple


def normalize_angle_rad(angle: float) -> float:
    """Normalize angle to [0, 2*pi)."""
    return angle % (2 * math.pi)


def normalize_angle_deg(angle: float) -> float:
    """Normalize angle to [0, 360)."""
    return angle % 360.0


def deg_to_rad(deg: float) -> float:
    """Convert degrees to radians."""
    return deg * math.pi / 180.0


def rad_to_deg(rad: float) -> float:
    """Convert radians to degrees."""
    return rad * 180.0 / math.pi


def simulate_step(
    x: float,
    y: float,
    theta_deg: float,
    v_left: float,
    v_right: float,
    wheel_base: float,
    dt: float,
) -> Tuple[float, float, float]:
    """
    Simulate one time step of differential-drive kinematics.

    Uses the ICC (Instantaneous Center of Curvature) model with no lateral slip.

    Args:
        x: Current x position (workspace units)
        y: Current y position (workspace units)
        theta_deg: Current heading in degrees [0, 360), where 0 is +X direction
        v_left: Left wheel velocity (workspace units/second)
        v_right: Right wheel velocity (workspace units/second)
        wheel_base: Distance between wheels (workspace units)
        dt: Time step (seconds)

    Returns:
        Tuple of (new_x, new_y, new_theta_deg)
    """
    if dt <= 0.0:
        return x, y, theta_deg

    if v_left == 0.0 and v_right == 0.0:
        return x, y, theta_deg

    # Convert to radians for calculation
    theta = deg_to_rad(theta_deg)

    # Linear and angular velocities
    v = 0.5 * (v_right + v_left)  # Linear velocity at center
    omega = (v_right - v_left) / wheel_base  # Angular velocity

    # Near-zero angular velocity: straight line motion
    if abs(omega) < 1e-9:
        new_x = x + v * dt * math.cos(theta)
        new_y = y + v * dt * math.sin(theta)
        return new_x, new_y, theta_deg

    # Arc motion using ICC model
    # ICC is at distance R = v/omega from the robot center
    new_theta = theta + omega * dt
    r = v / omega

    # Position update using rotation around ICC
    new_x = x + r * (math.sin(new_theta) - math.sin(theta))
    new_y = y - r * (math.cos(new_theta) - math.cos(theta))

    # Convert back to degrees and normalize
    new_theta_deg = normalize_angle_deg(rad_to_deg(new_theta))

    return new_x, new_y, new_theta_deg


def compute_wheel_velocities(
    linear_velocity: float,
    angular_velocity: float,
    wheel_base: float,
) -> Tuple[float, float]:
    """
    Compute wheel velocities from linear and angular velocity commands.

    Args:
        linear_velocity: Desired linear velocity (workspace units/second)
        angular_velocity: Desired angular velocity (radians/second)
        wheel_base: Distance between wheels (workspace units)

    Returns:
        Tuple of (v_left, v_right) wheel velocities
    """
    # v = (v_r + v_l) / 2
    # omega = (v_r - v_l) / wheel_base
    # Solving: v_r = v + omega * wheel_base / 2
    #          v_l = v - omega * wheel_base / 2
    half_wb = wheel_base / 2.0
    v_right = linear_velocity + angular_velocity * half_wb
    v_left = linear_velocity - angular_velocity * half_wb
    return v_left, v_right
