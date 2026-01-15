from __future__ import annotations

import math
from typing import List, Tuple

from micromvp.utils.geometry import distance


Point = Tuple[float, float]


def normalize_angle(angle: float) -> float:
    """Normalize angle to [0, 2*pi)."""
    angle = angle % (2 * math.pi)
    return angle


def calculate_atan(diff_x: float, diff_y: float) -> float:
    if diff_x == 0:
        return math.pi / 2 if diff_y > 0 else math.pi * 3 / 2
    if diff_y == 0:
        return 0.0 if diff_x > 0 else math.pi
    angle = math.atan(diff_y / diff_x)
    if diff_x > 0 and diff_y > 0:
        return angle
    if diff_x < 0 and diff_y > 0:
        return angle + math.pi
    if diff_x < 0 and diff_y < 0:
        return angle + math.pi
    return angle + 2 * math.pi


def calculate_speeds(x: float, y: float, theta: float, path: List[Point], v: float, wb: float) -> Tuple[float, float]:
    """Calculate left/right wheel speeds to follow a path."""
    while True:
        if not path:
            return 0.0, 0.0
        c = 0.6
        if len(path) < 10:
            c = len(path) / 10.0 * 0.6
        if c < 0.2:
            c = 0.2
        if distance(x, y, path[0][0], path[0][1]) <= wb * c:
            path.pop(0)
        else:
            break

    px, py = path[0]
    vx, vy = px - x, py - y
    v = v / 2
    theta = normalize_angle(theta)

    direction = calculate_atan(vx, vy)
    angle_error = math.atan2(math.sin(direction - theta), math.cos(direction - theta))
    abs_err = abs(angle_error)

    # If facing away, pivot in place aggressively.
    if abs_err > math.pi * 0.75:
        turn = v
        return (-turn, turn) if angle_error > 0 else (turn, -turn)

    # If large heading error, keep outer wheel full speed and reduce inner wheel.
    if abs_err > math.pi / 6:
        inner_scale = max(0.0, 1.0 - (abs_err / math.pi) * 2.0)
        if angle_error > 0:
            return v * inner_scale, v
        return v, v * inner_scale

    dist = distance(x, y, px, py)
    angle_at_center = normalize_angle(direction - theta)
    if angle_at_center != 0:
        radius = (dist / 2) / math.sin(angle_at_center)
        if radius < wb / 2:
            v_l = v * (-(wb / 2 - radius) / radius)
            v_r = v * ((radius + wb / 2) / radius)
        else:
            v_l = v * ((radius - wb / 2) / radius)
            v_r = v * ((radius + wb / 2) / radius)
        if v_l > 1.0:
            return 1.0, v_r / v_l
        if v_r > 1.0:
            return v_l / v_r, 1.0
        return v_l, v_r
    return v, v


def simulate_step(
    x: float,
    y: float,
    theta: float,
    v_l: float,
    v_r: float,
    wheel_base: float,
    dt: float,
) -> Tuple[float, float, float]:
    """Differential-drive kinematics with no lateral slip."""
    if dt <= 0.0:
        return x, y, theta
    if v_l == 0.0 and v_r == 0.0:
        return x, y, theta

    v = 0.5 * (v_r + v_l)
    omega = (v_r - v_l) / wheel_base

    if abs(omega) < 1e-6:
        x += v * dt * math.cos(theta)
        y += v * dt * math.sin(theta)
        return x, y, theta

    new_theta = normalize_angle(theta + omega * dt)
    r = v / omega
    x += r * (math.sin(new_theta) - math.sin(theta))
    y -= r * (math.cos(new_theta) - math.cos(theta))
    return x, y, new_theta
