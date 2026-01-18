"""
PurePursuit + CTE-PD Follow Path Controller for differential drive robots.

Axis convention:
- X: right
- Y: up
- 0 deg: +X
- deg increases CCW (+Y is 90 deg)

Algorithm (unit-insensitive, car_size-based):
1) set_path(): resample polyline so adjacent point gap <= max_point_gap_ratio * car_size
2) Precompute prefix arc-length s[i] along the resampled path
3) Each step: restrict all searches to a forward arc-length window:
      s in [s[path_index], s[path_index] + no_skip_ratio * car_size]
   so self-intersections cannot "steal" the controller to a later branch.
4) Pure Pursuit gives a smooth global steering intent (curvature_pp)
5) CTE-PD (near-path only) damps oscillation and improves tracking, bounded by car_size-scaled limits
6) curvature_cmd -> differential drive wheel speeds

Keeps all public/exposed API style the same as your original controller.
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


class PurePursuit_PD_FollowPathController(Controller):
    """
    Pure pursuit + cross-track-error PD controller (with no-skip arc-length gating).

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
        # New knobs (unit-insensitive, car_size-based)
        max_point_gap_ratio: float = 0.01,  # adjacent path point gap <= ratio * car_size
        no_skip_ratio: float = 0.5,         # allow at most this many car_size arc-length progress per update window
    ) -> None:
        super().__init__(robot_id, ws_config)

        self.car_size = max(ws_config.car_width, ws_config.car_height)

        self._max_point_gap_ratio = max(1e-4, float(max_point_gap_ratio))
        self._no_skip_ratio = max(0.1, float(no_skip_ratio))

        # Lookahead as car_size baseline (you can change multiplier if you want)
        self._lookahead_distance = lookahead_distance or (0.5 * self.car_size)
        self._max_speed = min(1.0, max(0.0, max_speed))

        # More strict finish tolerance: 20% car_size
        self._goal_tolerance = goal_tolerance or (0.2 * self.car_size)

        # ---- CTE-PD behavior knobs (dimensionless style, car_size-scaled internally) ----
        # Derivative smoothing
        self._cte_dot_alpha = 0.25  # 0~1, larger = less smoothing

        # Speed scheduling
        self._min_turn_factor = 0.25

        # Path state
        self._path_raw: List[Point] = []
        self._path: List[Point] = []
        self._path_s: List[float] = []  # prefix arc-length
        self._path_index: int = 0
        self._prev_timestamp: Optional[float] = None

        # PD state
        self._prev_cte: Optional[float] = None
        self._prev_cte_time: Optional[float] = None
        self._cte_dot_filt: float = 0.0

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

    # ---------------------------
    # Path preprocessing
    # ---------------------------

    def _resample_path(self, path: List[Point], max_step: float) -> List[Point]:
        if len(path) <= 1:
            return list(path)

        out: List[Point] = []
        for i in range(len(path) - 1):
            x0, y0 = path[i]
            x1, y1 = path[i + 1]
            dx = x1 - x0
            dy = y1 - y0
            d = math.hypot(dx, dy)

            if d < 1e-12:
                continue

            n = int(math.ceil(d / max_step))

            if not out:
                out.append((x0, y0))

            for k in range(1, n + 1):
                t = k / n
                out.append((x0 + t * dx, y0 + t * dy))

        return out

    def _build_prefix_s(self) -> None:
        self._path_s = [0.0]
        for i in range(len(self._path) - 1):
            x0, y0 = self._path[i]
            x1, y1 = self._path[i + 1]
            self._path_s.append(self._path_s[-1] + math.hypot(x1 - x0, y1 - y0))

    def _max_reachable_index(self, start_idx: int) -> int:
        if not self._path_s:
            return start_idx
        start_idx = _clamp(start_idx, 0, len(self._path_s) - 1)  # type: ignore[arg-type]
        start_idx = int(start_idx)

        s0 = self._path_s[start_idx]
        s_max = s0 + self._no_skip_ratio * self.car_size

        j = start_idx
        n = len(self._path_s)
        while j + 1 < n and self._path_s[j + 1] <= s_max:
            j += 1
        return j

    # ---------------------------
    # Public API
    # ---------------------------

    def set_path(self, path: List[Point]) -> None:
        self.car_size = max(self._ws_config.car_width, self._ws_config.car_height)

        self._path_raw = list(path)
        max_step = self._max_point_gap_ratio * self.car_size
        self._path = self._resample_path(self._path_raw, max_step)
        self._build_prefix_s()

        self._path_index = 0
        self._prev_cte = None
        self._prev_cte_time = None
        self._cte_dot_filt = 0.0

        if self._path:
            self._car_state.status_label = "FOLLOWING"
            self._car_state.metadata["path"] = self._path
            self._car_state.metadata["path_index"] = self._path_index
        else:
            self._car_state.status_label = "IDLE"
            self._car_state.metadata.pop("path", None)
            self._car_state.metadata.pop("path_index", None)

    def clear_path(self) -> None:
        self._path_raw = []
        self._path = []
        self._path_s = []
        self._path_index = 0
        self._car_state.status_label = "IDLE"
        self._car_state.metadata.pop("path", None)
        self._car_state.metadata.pop("path_index", None)
        self._car_state.metadata.pop("target_point", None)

        self._prev_cte = None
        self._prev_cte_time = None
        self._cte_dot_filt = 0.0

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
        self._car_state.theta = observation.theta  # degrees

        if prev_time is not None and observation.timestamp > prev_time:
            dt = observation.timestamp - prev_time
            if dt > 0:
                dx = observation.x - prev_x
                dy = observation.y - prev_y
                distance = math.hypot(dx, dy)
                self._car_state.linear_velocity = distance / dt

                dtheta = observation.theta - prev_theta
                if dtheta > 180:
                    dtheta -= 360
                elif dtheta < -180:
                    dtheta += 360
                self._car_state.angular_velocity = dtheta / dt

        self._prev_timestamp = observation.timestamp
        self._last_observation = observation

    # ---------------------------
    # Core controller
    # ---------------------------

    def calculate_action(self) -> Action:
        if not self._path:
            self._car_state.status_label = "IDLE"
            return Action.stop()

        robot_x = self._car_state.x
        robot_y = self._car_state.y
        robot_theta = self._car_state.theta  # degrees

        # size baseline (unit-insensitive)
        self.car_size = max(self._ws_config.car_width, self._ws_config.car_height)
        wheel_base = self._ws_config.wheel_base

        # finish check (car_size baseline)
        final_point = self._path[-1]
        dist_to_goal = math.hypot(robot_x - final_point[0], robot_y - final_point[1])
        if dist_to_goal < self._goal_tolerance:
            self._car_state.status_label = "FINISHED"
            self._car_state.metadata.pop("target_point", None)
            return Action.stop()

        # target point (gated to avoid skipping at self-intersections)
        target_point = self._find_target_point(robot_x, robot_y)
        if target_point is None:
            target_point = final_point

        # closest reference for CTE (also gated)
        ref_point, theta_ref_rad, cte_raw, seg_idx = self._find_closest_reference((robot_x, robot_y))
        self._path_index = max(self._path_index, seg_idx)

        # metadata (keep existing)
        self._car_state.metadata["target_point"] = target_point
        self._car_state.metadata["path_index"] = self._path_index
        self._car_state.status_label = "FOLLOWING"

        # pure pursuit curvature
        curvature_pp, heading_error_rad, dist_to_target = self._pure_pursuit_curvature(
            robot_x, robot_y, robot_theta, target_point
        )

        # -------------------------
        # Unit-insensitive gating for PD
        # -------------------------
        dist_to_path = abs(cte_raw)

        cte_pd_enable_dist = 2.5 * self.car_size
        cte_clip = 1.0 * self.car_size
        cte_deadband = 0.10 * self.car_size

        # curvature scales as 1/length => use 1/car_size
        curv_pd_max = 2.0 / max(self.car_size, 1e-9)
        curvature_max = 3.5 / max(self.car_size, 1e-9)

        rotate_in_place_rad = math.radians(110.0)

        # speed scheduling
        turn_factor = max(self._min_turn_factor, math.cos(abs(heading_error_rad)))
        base_speed = self._max_speed * turn_factor

        # PD enable
        use_pd = dist_to_path <= cte_pd_enable_dist

        # clamp + deadband
        cte_used = cte_raw
        if abs(cte_used) < cte_deadband:
            cte_used = 0.0
        cte_used = _clamp(cte_used, -cte_clip, cte_clip)

        now_t = self._prev_timestamp
        if not use_pd or now_t is None:
            self._prev_cte = None
            self._prev_cte_time = None
            self._cte_dot_filt = 0.0
            cte_dot_filt = 0.0
            cte_pd_curv = 0.0
        else:
            if self._prev_cte is None or self._prev_cte_time is None:
                cte_dot = 0.0
            else:
                dt = now_t - self._prev_cte_time
                cte_dot = (cte_used - self._prev_cte) / dt if dt > 1e-6 else 0.0

            self._cte_dot_filt = (1.0 - self._cte_dot_alpha) * self._cte_dot_filt + self._cte_dot_alpha * cte_dot
            cte_dot_filt = self._cte_dot_filt

            self._prev_cte = cte_used
            self._prev_cte_time = now_t

            # dimensionless knobs
            kp = 1.2
            kd = 0.35

            cte_n = cte_used / max(self.car_size, 1e-9)
            cte_dot_n = cte_dot_filt / max(self.car_size, 1e-9)

            cte_pd_curv = (kp * cte_n + kd * cte_dot_n) / max(self.car_size, 1e-9)
            cte_pd_curv = _clamp(cte_pd_curv, -curv_pd_max, curv_pd_max)

        curvature_cmd = curvature_pp + cte_pd_curv
        curvature_cmd = _clamp(curvature_cmd, -curvature_max, curvature_max)

        curv_slow = _clamp(1.0 - 0.65 * (abs(curvature_cmd) / max(curvature_max, 1e-9)), 0.35, 1.0)
        base_speed *= curv_slow

        # rotate-in-place capture when far + facing away
        if (not use_pd) and (abs(heading_error_rad) > rotate_in_place_rad):
            w = 0.6 * self._max_speed
            left_speed = -w if heading_error_rad > 0 else w
            right_speed = w if heading_error_rad > 0 else -w

            self._car_state.metadata["mode"] = "ROTATE_IN_PLACE"
            self._car_state.metadata["pd_enabled"] = False
            self._car_state.metadata["cte"] = cte_raw
            self._car_state.metadata["cte_used"] = cte_used
            self._car_state.metadata["cte_dot_filt"] = 0.0
            self._car_state.metadata["curvature_pp"] = curvature_pp
            self._car_state.metadata["curvature_cmd"] = 0.0
            self._car_state.metadata["heading_error_deg"] = math.degrees(heading_error_rad)
            self._car_state.metadata["theta_ref_deg"] = math.degrees(theta_ref_rad)
            self._car_state.metadata["ref_point"] = ref_point
            self._car_state.metadata["dist_to_target"] = dist_to_target
            self._car_state.metadata["goal_tol"] = self._goal_tolerance
            self._car_state.metadata["no_skip_ratio"] = self._no_skip_ratio
            self._car_state.metadata["max_point_gap_ratio"] = self._max_point_gap_ratio
            self._car_state.metadata["cte_pd_enable_dist"] = cte_pd_enable_dist

            return Action(
                left_speed=_clamp(left_speed, -1.0, 1.0),
                right_speed=_clamp(right_speed, -1.0, 1.0),
            )

        diff = curvature_cmd * wheel_base / 2.0
        left_speed = base_speed * (1.0 - diff)
        right_speed = base_speed * (1.0 + diff)

        max_wheel = max(abs(left_speed), abs(right_speed))
        if max_wheel > self._max_speed:
            scale = self._max_speed / max_wheel
            left_speed *= scale
            right_speed *= scale

        left_speed = _clamp(left_speed, -1.0, 1.0)
        right_speed = _clamp(right_speed, -1.0, 1.0)

        # Debug metadata
        self._car_state.metadata["mode"] = "TRACK" if use_pd else "ACQUIRE"
        self._car_state.metadata["pd_enabled"] = use_pd
        self._car_state.metadata["cte"] = cte_raw
        self._car_state.metadata["cte_used"] = cte_used
        self._car_state.metadata["cte_dot_filt"] = cte_dot_filt if use_pd else 0.0
        self._car_state.metadata["curvature_pp"] = curvature_pp
        self._car_state.metadata["curvature_cmd"] = curvature_cmd
        self._car_state.metadata["heading_error_deg"] = math.degrees(heading_error_rad)
        self._car_state.metadata["theta_ref_deg"] = math.degrees(theta_ref_rad)
        self._car_state.metadata["ref_point"] = ref_point
        self._car_state.metadata["dist_to_target"] = dist_to_target
        self._car_state.metadata["goal_tol"] = self._goal_tolerance
        self._car_state.metadata["cte_pd_enable_dist"] = cte_pd_enable_dist
        self._car_state.metadata["curvature_max"] = curvature_max
        self._car_state.metadata["curv_pd_max"] = curv_pd_max
        self._car_state.metadata["no_skip_ratio"] = self._no_skip_ratio
        self._car_state.metadata["max_point_gap_ratio"] = self._max_point_gap_ratio

        return Action(left_speed=left_speed, right_speed=right_speed)

    # ---------------------------
    # Gated target finding (no-skip)
    # ---------------------------

    def _find_target_point(self, robot_x: float, robot_y: float) -> Optional[Point]:
        if not self._path:
            return None

        start = self._path_index
        imax = self._max_reachable_index(start)

        best_idx = start
        best_dist = float("inf")

        for i in range(start, imax + 1):
            px, py = self._path[i]
            dist = math.hypot(robot_x - px, robot_y - py)
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        self._path_index = max(self._path_index, best_idx)

        start = self._path_index
        imax = self._max_reachable_index(start)

        for i in range(start, imax + 1):
            px, py = self._path[i]
            dist = math.hypot(robot_x - px, robot_y - py)
            if dist >= self._lookahead_distance:
                self._path_index = i
                return (px, py)

        return self._path[min(imax, len(self._path) - 1)]

    # ---------------------------
    # Pure Pursuit curvature
    # ---------------------------

    def _pure_pursuit_curvature(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta_deg: float,
        target: Point,
    ) -> Tuple[float, float, float]:
        dx = target[0] - robot_x
        dy = target[1] - robot_y
        distance = math.hypot(dx, dy)
        if distance < 1e-9:
            return 0.0, 0.0, 0.0

        angle_to_target = math.atan2(dy, dx)
        robot_theta_rad = math.radians(robot_theta_deg)
        heading_error_rad = _wrap_to_pi(angle_to_target - robot_theta_rad)

        L = max(distance, self._lookahead_distance)
        curvature_pp = 2.0 * math.sin(heading_error_rad) / L
        return curvature_pp, heading_error_rad, distance

    # ---------------------------
    # Gated closest reference (no-skip)
    # ---------------------------

    def _find_closest_reference(self, pos: Point) -> Tuple[Point, float, float, int]:
        x, y = pos
        n = len(self._path)
        if n == 1:
            px, py = self._path[0]
            theta_ref = math.radians(self._car_state.theta)
            cte = math.hypot(x - px, y - py)
            return (px, py), theta_ref, cte, 0

        start_i = min(self._path_index, n - 2)
        end_i = min(self._max_reachable_index(start_i), n - 1)
        seg_end = max(start_i, min(end_i - 1, n - 2))

        best_dist2 = float("inf")
        best_proj: Point = self._path[start_i]
        best_theta = 0.0
        best_cte = 0.0
        best_i = start_i

        for i in range(start_i, seg_end + 1):
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

                nx = -math.sin(theta_ref)
                ny = math.cos(theta_ref)
                best_cte = nx * (x - proj[0]) + ny * (y - proj[1])

        return best_proj, best_theta, best_cte, best_i

    # ---------------------------
    # Reset / speed
    # ---------------------------

    def reset(self) -> None:
        super().reset()
        self._path_raw = []
        self._path = []
        self._path_s = []
        self._path_index = 0
        self._prev_timestamp = None

        self._prev_cte = None
        self._prev_cte_time = None
        self._cte_dot_filt = 0.0

        self._car_state.metadata.pop("path", None)
        self._car_state.metadata.pop("path_index", None)
        self._car_state.metadata.pop("target_point", None)

    def set_speed(self, speed: float) -> None:
        self._max_speed = max(0.0, min(1.0, speed))
