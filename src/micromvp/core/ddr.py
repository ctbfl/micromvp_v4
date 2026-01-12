from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple, TYPE_CHECKING

from micromvp.utils.geometry import distance

if TYPE_CHECKING:
    from micromvp.core.models import Waypoint

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


def angle_diff(target: float, current: float) -> float:
    """Compute shortest angle difference (target - current), normalized to [-pi, pi]."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


def calculate_speeds_to_pose(
    x: float,
    y: float,
    theta: float,
    target_x: float,
    target_y: float,
    target_theta: float,
    v_max: float,
    wheel_base: float,
    position_threshold: float = 10.0,
    angle_threshold: float = 0.1,
) -> Tuple[float, float]:
    """
    Calculate wheel speeds to reach target pose (x, y, theta).

    Two-phase control:
    1. If position error > threshold: drive to position (ignoring target theta)
    2. If position error < threshold: rotate in place to match target theta

    Returns (v_left, v_right) normalized to [-1, 1].
    """
    dx = target_x - x
    dy = target_y - y
    dist = math.hypot(dx, dy)

    # Phase 2: Position reached, align orientation
    if dist <= position_threshold:
        theta_error = angle_diff(target_theta, theta)
        if abs(theta_error) <= angle_threshold:
            return (0.0, 0.0)  # Goal reached
        # Pivot in place with speed proportional to error
        turn_speed = min(v_max * 0.5, abs(theta_error) / math.pi * v_max)
        turn_speed = max(turn_speed, 0.15)  # Minimum turn speed
        if theta_error > 0:
            return (-turn_speed, turn_speed)
        else:
            return (turn_speed, -turn_speed)

    # Phase 1: Drive to position (use existing logic)
    path = [(target_x, target_y)]
    return calculate_speeds(x, y, theta, path, v_max, wheel_base)


# ============================================================================
# Dubins Path Implementation
# ============================================================================

class DubinsSegmentType(Enum):
    LEFT = 'L'
    RIGHT = 'R'
    STRAIGHT = 'S'


@dataclass
class DubinsPath:
    """Representation of a Dubins path."""
    path_type: str  # e.g., "LSL", "RSR", "LSR", "RSL", "RLR", "LRL"
    segments: Tuple[float, float, float]  # length of each segment (normalized by radius)
    total_length: float  # total path length in world units
    min_radius: float


def _mod2pi(angle: float) -> float:
    """Normalize angle to [0, 2*pi)."""
    return angle % (2 * math.pi)


def _dubins_LSL(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    """Compute LSL path parameters."""
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sa - sb)
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    tmp = math.atan2(cb - ca, d + sa - sb)
    t = _mod2pi(-alpha + tmp)
    q = _mod2pi(beta - tmp)
    return (t, p, q)


def _dubins_RSR(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    """Compute RSR path parameters."""
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sb - sa)
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    tmp = math.atan2(ca - cb, d - sa + sb)
    t = _mod2pi(alpha - tmp)
    q = _mod2pi(-beta + tmp)
    return (t, p, q)


def _dubins_LSR(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    """Compute LSR path parameters."""
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    p_sq = -2 + d * d + 2 * c_ab + 2 * d * (sa + sb)
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
    t = _mod2pi(-alpha + tmp)
    q = _mod2pi(-_mod2pi(beta) + tmp)
    return (t, p, q)


def _dubins_RSL(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    """Compute RSL path parameters."""
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    p_sq = -2 + d * d + 2 * c_ab - 2 * d * (sa + sb)
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
    t = _mod2pi(alpha - tmp)
    q = _mod2pi(beta - tmp)
    return (t, p, q)


def _dubins_RLR(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    """Compute RLR path parameters."""
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    tmp = (6.0 - d * d + 2 * c_ab + 2 * d * (sa - sb)) / 8.0
    if abs(tmp) > 1:
        return None
    p = _mod2pi(2 * math.pi - math.acos(tmp))
    t = _mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + _mod2pi(p / 2.0))
    q = _mod2pi(alpha - beta - t + _mod2pi(p))
    return (t, p, q)


def _dubins_LRL(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    """Compute LRL path parameters."""
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    tmp = (6.0 - d * d + 2 * c_ab + 2 * d * (sb - sa)) / 8.0
    if abs(tmp) > 1:
        return None
    p = _mod2pi(2 * math.pi - math.acos(tmp))
    t = _mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
    q = _mod2pi(_mod2pi(beta) - alpha - t + _mod2pi(p))
    return (t, p, q)


def compute_dubins_path(
    start_x: float,
    start_y: float,
    start_theta: float,
    goal_x: float,
    goal_y: float,
    goal_theta: float,
    min_radius: float,
) -> Optional[DubinsPath]:
    """
    Compute the shortest Dubins path from start pose to goal pose.

    A Dubins path consists of exactly 3 segments, each being:
    - L: left turn (CCW arc of radius min_radius)
    - R: right turn (CW arc of radius min_radius)
    - S: straight line

    Returns the shortest path among 6 possible types: LSL, RSR, LSR, RSL, RLR, LRL.
    Returns None if no valid path exists (rare edge cases).
    """
    dx = goal_x - start_x
    dy = goal_y - start_y
    d = math.hypot(dx, dy) / min_radius

    if d < 1e-6:
        # Start and goal are at same position, just rotate
        theta_diff = angle_diff(goal_theta, start_theta)
        if theta_diff >= 0:
            return DubinsPath(
                path_type="L",
                segments=(abs(theta_diff), 0.0, 0.0),
                total_length=abs(theta_diff) * min_radius,
                min_radius=min_radius,
            )
        else:
            return DubinsPath(
                path_type="R",
                segments=(abs(theta_diff), 0.0, 0.0),
                total_length=abs(theta_diff) * min_radius,
                min_radius=min_radius,
            )

    world_angle = math.atan2(dy, dx)
    alpha = _mod2pi(start_theta - world_angle)
    beta = _mod2pi(goal_theta - world_angle)

    path_funcs = [
        (_dubins_LSL, "LSL"),
        (_dubins_RSR, "RSR"),
        (_dubins_LSR, "LSR"),
        (_dubins_RSL, "RSL"),
        (_dubins_RLR, "RLR"),
        (_dubins_LRL, "LRL"),
    ]

    best_path: Optional[DubinsPath] = None
    best_length = float("inf")

    for func, path_type in path_funcs:
        result = func(alpha, beta, d)
        if result is not None:
            t, p, q = result
            length = (t + p + q) * min_radius
            if length < best_length:
                best_length = length
                best_path = DubinsPath(
                    path_type=path_type,
                    segments=(t, p, q),
                    total_length=length,
                    min_radius=min_radius,
                )

    return best_path


def sample_dubins_path(
    start_x: float,
    start_y: float,
    start_theta: float,
    path: DubinsPath,
    step_size: float = 5.0,
) -> List[Tuple[float, float, float]]:
    """
    Sample points along a Dubins path.

    Returns list of (x, y, theta) tuples with proper orientations at each sample.
    """
    from micromvp.core.models import Pose

    samples: List[Tuple[float, float, float]] = [(start_x, start_y, start_theta)]
    x, y, theta = start_x, start_y, start_theta
    r = path.min_radius

    segment_types = list(path.path_type)
    segment_lengths = list(path.segments)

    for seg_type, seg_len in zip(segment_types, segment_lengths):
        if seg_len < 1e-6:
            continue

        arc_length = seg_len * r
        num_steps = max(1, int(arc_length / step_size))
        step = seg_len / num_steps

        for _ in range(num_steps):
            if seg_type == 'S':
                # Straight segment
                x += step * r * math.cos(theta)
                y += step * r * math.sin(theta)
            elif seg_type == 'L':
                # Left turn (CCW)
                x += r * (math.sin(theta + step) - math.sin(theta))
                y += r * (-math.cos(theta + step) + math.cos(theta))
                theta = _mod2pi(theta + step)
            elif seg_type == 'R':
                # Right turn (CW)
                x += r * (-math.sin(theta - step) + math.sin(theta))
                y += r * (math.cos(theta - step) - math.cos(theta))
                theta = _mod2pi(theta - step)

            samples.append((x, y, theta))

    return samples
