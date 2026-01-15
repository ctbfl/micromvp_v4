import math
from typing import List, Tuple

Point = Tuple[float, float]
Pose = Tuple[float, float, float]  # (x, y, theta)


def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x1 - x2, y1 - y2)


def check_collision(threshold: float, x1: float, y1: float, x2: float, y2: float) -> bool:
    return distance(x1, y1, x2, y2) <= threshold


def angle_diff(target: float, current: float) -> float:
    """
    Compute shortest angle difference (target - current) in [-pi, pi].

    Args:
        target: Target angle in radians
        current: Current angle in radians

    Returns:
        Shortest signed angle difference in [-pi, pi]
    """
    diff = target - current
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff


def compute_path_heading(path: List[Point], index: int = 0, lookahead: int = 3) -> float:
    """
    Compute heading direction at a point on the path.

    Uses the direction from point[index] towards subsequent points,
    averaging over 'lookahead' segments for robustness.

    Args:
        path: List of (x, y) waypoints
        index: Point index to compute heading at (default: 0 for start)
        lookahead: Number of segments to average for direction

    Returns:
        Heading angle in radians [0, 2*pi)
    """
    if not path or len(path) < 2:
        return 0.0

    # Clamp index
    index = max(0, min(index, len(path) - 2))

    # Compute average direction over lookahead segments
    dx_sum = 0.0
    dy_sum = 0.0
    count = 0

    for i in range(index, min(index + lookahead, len(path) - 1)):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        length = math.hypot(dx, dy)
        if length > 1e-6:
            dx_sum += dx / length
            dy_sum += dy / length
            count += 1

    if count == 0:
        return 0.0

    return math.atan2(dy_sum / count, dx_sum / count)


def generate_bezier_path(
    start_pose: Pose,
    end_pose: Pose,
    num_points: int = 20,
    start_control_ratio: float = 0.3,
    end_control_ratio: float = 0.5,
) -> List[Point]:
    """
    Generate smooth path from start pose to end pose using cubic Bezier curve.

    The curve respects both position and orientation constraints:
    - Tangent at start is along start_pose theta
    - Tangent at end is along end_pose theta

    Args:
        start_pose: (x0, y0, theta0) - start position and heading
        end_pose: (x1, y1, theta1) - end position and heading
        num_points: Number of waypoints to generate
        start_control_ratio: Control point distance ratio at start (0-1)
        end_control_ratio: Control point distance ratio at end (0-1, larger = lower end curvature)

    Returns:
        List of (x, y) waypoints along the Bezier curve
    """
    x0, y0, theta0 = start_pose
    x1, y1, theta1 = end_pose

    # Distance between start and end
    dist = math.hypot(x1 - x0, y1 - y0)

    if dist < 1e-6:
        # Start and end are the same point
        return [(x0, y0)]

    # Control point distances
    d0 = start_control_ratio * dist
    d1 = end_control_ratio * dist

    # Bezier control points:
    # P0 = start position
    # P1 = P0 + d0 * direction(theta0)
    # P2 = P3 - d1 * direction(theta1)
    # P3 = end position
    p0 = (x0, y0)
    p1 = (x0 + d0 * math.cos(theta0), y0 + d0 * math.sin(theta0))
    p2 = (x1 - d1 * math.cos(theta1), y1 - d1 * math.sin(theta1))
    p3 = (x1, y1)

    # Generate points along Bezier curve
    path = []
    for i in range(num_points):
        t = i / (num_points - 1) if num_points > 1 else 0.0
        point = _cubic_bezier_point(p0, p1, p2, p3, t)
        path.append(point)

    return path


def _cubic_bezier_point(
    p0: Point,
    p1: Point,
    p2: Point,
    p3: Point,
    t: float,
) -> Point:
    """
    Compute point on cubic Bezier curve at parameter t.

    B(t) = (1-t)^3 * P0 + 3*(1-t)^2*t * P1 + 3*(1-t)*t^2 * P2 + t^3 * P3

    Args:
        p0, p1, p2, p3: Control points
        t: Parameter in [0, 1]

    Returns:
        Point (x, y) on the curve
    """
    t = max(0.0, min(1.0, t))
    mt = 1.0 - t

    # Bernstein basis polynomials
    b0 = mt * mt * mt
    b1 = 3.0 * mt * mt * t
    b2 = 3.0 * mt * t * t
    b3 = t * t * t

    x = b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0]
    y = b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1]

    return (x, y)
