"""
Follow Path Controller - Stanley path following for differential drive robots.

Axis convention:
- X: right
- Y: up
- 0 deg: +X
- deg increases CCW (+Y is 90 deg)

This controller:
1) Finds closest projection on the polyline (segment projection), starting from progress index
2) Computes path tangent heading at that segment
3) Computes signed cross-track error (cte) and heading error
4) Stanley control law -> curvature
5) Curvature -> left/right wheel speed
6) Progress index never goes backward
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

from micromvp.controller.base import Controller
from micromvp.core.models import (
    Action,
    RobotObservation,
    WorkspaceConfig,
)

Point = Tuple[float, float]


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _wrap_to_pi(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad <= -math.pi:
        rad += 2.0 * math.pi
    return rad


def _project_point_to_segment(p: Point, a: Point, b: Point) -> Tuple[Point, float]:
    ax, ay = a
    bx, by = b
    px, py = p

    vx = bx - ax
    vy = by - ay
    denom = vx * vx + vy * vy
    if denom <= 1e-12:
        return a, 0.0

    t = ((px - ax) * vx + (py - ay) * vy) / denom
    t = _clamp(t, 0.0, 1.0)
    return (ax + t * vx, ay + t * vy), t


class StanleyFollowPathController(Controller):
    """
    Stanley path following controller.

    Status labels:
    - "IDLE": No path assigned
    - "FOLLOWING": Actively following path
    - "FINISHED": Reached end of path
    """

    def __init__(
        self,
        robot_id: int,
        ws_config: WorkspaceConfig,
        lookahead_distance: Optional[float] = None,
        max_speed: float = 0.3,
        goal_tolerance: Optional[float] = None,
    ) -> None:
        super().__init__(robot_id, ws_config)

        self.car_size = max(ws_config.car_width, ws_config.car_height)

        # Keep API same: lookahead_distance still exists, used as curvature length scale
        self._lookahead_distance = lookahead_distance or (1.5 * self.car_size)
        self._max_speed = min(1.0, max(0.0, max_speed))

        # (2) Harder finish condition: 20% * max(car_width, car_height)
        self._goal_tolerance = goal_tolerance or (0.2 * self.car_size)

        # --- Stanley gains / behavior knobs ---
        # (1) Faster heading alignment: increase heading gain
        self._k_heading = 1.6

        # Cross-track correction gain (too large can induce S-shape)
        self._k_cte = 1.2

        # Small epsilon for low-speed stability
        self._v_eps = 1e-3

        # Rotation/curvature limit to avoid oscillation / oversteer
        # This caps the effective "control_angle" (steering intent).
        self._control_angle_max_deg = 55.0

        # Also cap curvature directly (optional but useful when wheel_base is large)
        self._curvature_max = 6.0  # in 1/(path-units)

        # Path state
        self._path: List[Point] = []
        self._path_index: int = 0
        self._prev_timestamp: Optional[float] = None

    @property
    def path(self) -> List[Point]:
        return self._path.copy()

    @property
    def path_index(self) -> int:
        return self._path_index

    @property
    def lookahead_distance(self) -> float:
        return self._lookahead_distance

    @property
    def max_speed(self) -> float:
        return self._max_speed

    def _resample_path(self, path: List[Point], max_step: float) -> List[Point]:
        if len(path) <= 1:
            return list(path)

        out: List[Point] = []
        for i in range(len(path) - 1):
            x0, y0 = path[i]
            x1, y1 = path[i + 1]
            dx = x1 - x0
            dy = y1 - y0
            d = math.sqrt(dx * dx + dy * dy)

            if d < 1e-12:
                continue

            n = int(math.ceil(d / max_step))
            # emit start point
            if not out:
                out.append((x0, y0))

            for k in range(1, n + 1):
                t = k / n
                out.append((x0 + t * dx, y0 + t * dy))

        return out

    def set_path(self, path: List[Point]) -> None:
        max_step = 0.01 * self.car_size  # 或 0.01*car_size
        self._path_raw = list(path)          # 可选：保留原始
        self._path = self._resample_path(self._path_raw, max_step)
        self._path_index = 0
        if self._path:
            self._car_state.status_label = "FOLLOWING"
            self._car_state.metadata["path"] = self._path
            self._car_state.metadata["path_index"] = self._path_index
        else:
            self._car_state.status_label = "IDLE"
            self._car_state.metadata.pop("path", None)
            self._car_state.metadata.pop("path_index", None)

    def clear_path(self) -> None:
        self._path = []
        self._path_index = 0
        self._car_state.status_label = "IDLE"
        self._car_state.metadata.pop("path", None)
        self._car_state.metadata.pop("path_index", None)
        self._car_state.metadata.pop("target_point", None)

    def step(self, observation: RobotObservation) -> Action:
        self.update(observation)
        return self.calculate_action()

    def update(self, observation: RobotObservation) -> None:
        prev_x = self._car_state.x
        prev_y = self._car_state.y
        prev_theta = self._car_state.theta
        prev_time = self._prev_timestamp

        self._car_state.x = observation.x
        self._car_state.y = observation.y
        self._car_state.theta = observation.theta  # degrees, CCW positive

        if prev_time is not None and observation.timestamp > prev_time:
            dt = observation.timestamp - prev_time
            if dt > 0:
                dx = observation.x - prev_x
                dy = observation.y - prev_y
                distance = math.sqrt(dx * dx + dy * dy)
                self._car_state.linear_velocity = distance / dt

                dtheta = observation.theta - prev_theta
                if dtheta > 180:
                    dtheta -= 360
                elif dtheta < -180:
                    dtheta += 360
                self._car_state.angular_velocity = dtheta / dt

        self._prev_timestamp = observation.timestamp
        self._last_observation = observation

    def calculate_action(self) -> Action:
        if not self._path:
            self._car_state.status_label = "IDLE"
            return Action.stop()

        robot_x = self._car_state.x
        robot_y = self._car_state.y
        robot_theta_deg = self._car_state.theta

        # Finish check (harder tolerance)
        final_point = self._path[-1]
        dist_to_goal = math.sqrt(
            (robot_x - final_point[0]) ** 2 + (robot_y - final_point[1]) ** 2
        )
        if dist_to_goal < self._goal_tolerance:
            self._car_state.status_label = "FINISHED"
            self._car_state.metadata.pop("target_point", None)
            return Action.stop()

        # Closest reference on polyline
        ref_point, theta_ref_rad, e_cte, seg_idx = self._find_closest_reference(
            (robot_x, robot_y)
        )
        self._path_index = max(self._path_index, seg_idx)

        self._car_state.metadata["target_point"] = ref_point
        self._car_state.metadata["path_index"] = self._path_index
        self._car_state.status_label = "FOLLOWING"

        robot_theta_rad = math.radians(robot_theta_deg)
        e_heading = _wrap_to_pi(theta_ref_rad - robot_theta_rad)

        # Base speed: still reduce when heading error large
        turn_factor = max(0.20, math.cos(abs(e_heading)))
        v = self._max_speed * turn_factor

        # Stanley correction (sign chosen for y-up, CCW-positive)
        corr = math.atan2(self._k_cte * e_cte, v + self._v_eps)
        control_angle = _wrap_to_pi(self._k_heading * e_heading - corr)

        # (1) Improve rotation response but suppress over-rush:
        # - cap control angle
        ca_max = math.radians(self._control_angle_max_deg)
        control_angle = _clamp(control_angle, -ca_max, ca_max)

        # Convert to curvature using length scale
        L = max(1e-6, self._lookahead_distance)
        curvature = 2.0 * math.sin(control_angle) / L

        # Cap curvature to prevent aggressive oscillations
        curvature = _clamp(curvature, -self._curvature_max, self._curvature_max)

        # Additional speed reduction when steering demand is high (reduces overshoot / S-shape)
        steer_slow = _clamp(1.0 - 0.65 * (abs(control_angle) / ca_max), 0.35, 1.0)
        v *= steer_slow

        # Convert curvature -> wheel speeds
        wheel_base = self._ws_config.wheel_base
        diff = curvature * wheel_base / 2.0
        left_speed = v * (1.0 - diff)
        right_speed = v * (1.0 + diff)

        # Normalize to keep within [-max_speed, max_speed]
        max_wheel = max(abs(left_speed), abs(right_speed))
        if max_wheel > self._max_speed:
            scale = self._max_speed / max_wheel
            left_speed *= scale
            right_speed *= scale

        left_speed = _clamp(left_speed, -1.0, 1.0)
        right_speed = _clamp(right_speed, -1.0, 1.0)

        # Debug
        self._car_state.metadata["cte"] = e_cte
        self._car_state.metadata["heading_error_deg"] = math.degrees(e_heading)
        self._car_state.metadata["theta_ref_deg"] = math.degrees(theta_ref_rad)
        self._car_state.metadata["curvature"] = curvature
        self._car_state.metadata["control_angle_deg"] = math.degrees(control_angle)
        self._car_state.metadata["goal_tol"] = self._goal_tolerance

        return Action(left_speed=left_speed, right_speed=right_speed)

    def _find_closest_reference(self, pos: Point) -> Tuple[Point, float, float, int]:
        """
        Returns:
            ref_point: closest projected point on path
            theta_ref_rad: path tangent heading at that segment (radians, CCW+)
            e_cte: signed cross-track error (positive = robot is left of path tangent)
            seg_idx: segment start index
        """
        x, y = pos
        n = len(self._path)
        if n == 1:
            px, py = self._path[0]
            theta_ref = math.radians(self._car_state.theta)
            e_cte = math.sqrt((x - px) ** 2 + (y - py) ** 2)
            return (px, py), theta_ref, e_cte, 0

        start_i = min(self._path_index, n - 2)

        best_dist2 = float("inf")
        best_proj: Point = self._path[start_i]
        best_theta = 0.0
        best_e = 0.0
        best_i = start_i

        for i in range(start_i, n - 1):
            a = self._path[i]
            b = self._path[i + 1]
            proj, _t = _project_point_to_segment((x, y), a, b)

            dx = x - proj[0]
            dy = y - proj[1]
            dist2 = dx * dx + dy * dy
            if dist2 < best_dist2:
                best_dist2 = dist2
                best_proj = proj
                best_i = i

                seg_dx = b[0] - a[0]
                seg_dy = b[1] - a[1]
                theta_ref = math.atan2(seg_dy, seg_dx)
                best_theta = theta_ref

                # Left normal in y-up frame: n = [-sin(theta), cos(theta)]
                nx = -math.sin(theta_ref)
                ny = math.cos(theta_ref)
                best_e = nx * (x - proj[0]) + ny * (y - proj[1])

        return best_proj, best_theta, best_e, best_i

    def reset(self) -> None:
        super().reset()
        self._path = []
        self._path_index = 0
        self._prev_timestamp = None
        self._car_state.metadata.pop("path", None)
        self._car_state.metadata.pop("path_index", None)
        self._car_state.metadata.pop("target_point", None)
        self._car_state.metadata.pop("cte", None)
        self._car_state.metadata.pop("heading_error_deg", None)
        self._car_state.metadata.pop("theta_ref_deg", None)
        self._car_state.metadata.pop("curvature", None)
        self._car_state.metadata.pop("control_angle_deg", None)
        self._car_state.metadata.pop("goal_tol", None)

    def set_speed(self, speed: float) -> None:
        self._max_speed = max(0.0, min(1.0, speed))
