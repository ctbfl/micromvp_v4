"""
PurePursuit + CTE-PD Follow Path Controller for differential drive robots.

Axis convention:
- X: right
- Y: up
- 0 deg: +X
- deg increases CCW (+Y is 90 deg)

Algorithm:
1) Find closest point on the path polyline (segment projection), starting from progress index
2) Find a pure-pursuit target point at lookahead distance (still using your original discrete-point method)
3) Compute PP curvature from heading-to-target
4) Compute signed cross-track error (CTE) to the polyline + its derivative (PD with low-pass)
5) Combine: curvature_cmd = curvature_pp + k_p*cte + k_d*d(cte)/dt
6) Convert curvature_cmd to wheel speeds (same mapping you used)

Keeps all public/exposed API the same style as your original controller.
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
    Pure pursuit + cross-track-error PD controller.

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
        self._lookahead_distance = lookahead_distance or (0.5 * self.car_size)
        self._max_speed = min(1.0, max(0.0, max_speed))
        # 按你之前的需求：更严格的终点半径（20% car_size）
        self._goal_tolerance = goal_tolerance or (0.2 * self.car_size)

        # ---- CTE-PD gains (tune these first) ----
        # 注意：这里直接加到 curvature 上，所以单位是:
        #   k_p: 1/length
        #   k_d: 1/(length * second)
        self._cte_kp = 1.2
        self._cte_kd = 0.25

        # CTE deadband (像你三传感器“压中线就不纠”的感觉)
        self._cte_deadband = 0.10 * self.car_size

        # Low-pass filter for cte_dot (avoid noise amplification)
        self._cte_dot_alpha = 0.25  # 0~1, larger = less smoothing

        # Curvature cap to prevent over-steer oscillation
        self._curvature_max = 6.0  # 1/length

        # Speed reduction when steering demand is large
        self._min_turn_factor = 0.25

        # Path state
        self._path: List[Point] = []
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
        self._path = []
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
        """
        PurePursuit + CTE-PD (unit-insensitive, car_size-normalized) action.

        Stages:
        - Acquire (far from path): use pure pursuit only (no CTE-PD), and if heading is huge, rotate-in-place.
        - Track (near path): enable CTE-PD with cte clamp + deadband + filtered derivative.
        """
        if not self._path:
            self._car_state.status_label = "IDLE"
            return Action.stop()

        robot_x = self._car_state.x
        robot_y = self._car_state.y
        robot_theta = self._car_state.theta  # degrees

        # --- size baseline (unit-insensitive) ---
        self.car_size = max(self._ws_config.car_width, self._ws_config.car_height)
        Ld = max(1e-9, self._lookahead_distance)  # already scaled from car_size
        wheel_base = self._ws_config.wheel_base

        # --- finish check (use car_size baseline) ---
        final_point = self._path[-1]
        dist_to_goal = math.sqrt(
            (robot_x - final_point[0]) ** 2 + (robot_y - final_point[1]) ** 2
        )
        if dist_to_goal < self._goal_tolerance:
            self._car_state.status_label = "FINISHED"
            self._car_state.metadata.pop("target_point", None)
            return Action.stop()

        # --- 1) pure pursuit target ---
        target_point = self._find_target_point(robot_x, robot_y)
        if target_point is None:
            target_point = final_point

        # --- 2) closest reference for CTE (segment projection) ---
        ref_point, theta_ref_rad, cte_raw, seg_idx = self._find_closest_reference((robot_x, robot_y))
        self._path_index = max(self._path_index, seg_idx)

        # metadata (keep existing)
        self._car_state.metadata["target_point"] = target_point
        self._car_state.metadata["path_index"] = self._path_index
        self._car_state.status_label = "FOLLOWING"

        # --- 3) PP curvature & heading error to target ---
        curvature_pp, heading_error_rad, dist_to_target = self._pure_pursuit_curvature(
            robot_x, robot_y, robot_theta, target_point
        )

        # -------------------------
        # Unit-insensitive gating
        # -------------------------
        # Far -> do NOT use CTE-PD (avoid "cte huge => curvature saturated => spin")
        # Use only car_size/Ld ratios so it scales with environment.
        dist_to_path = abs(cte_raw)

        # Enable PD only when close enough to the path
        # (2~3 car sizes is a good capture radius across scales)
        cte_pd_enable_dist = 2.5 * self.car_size

        # PD cte clamp: never let PD "see" an error larger than ~1 car size
        cte_clip = 1.0 * self.car_size

        # Deadband: within this band we don't correct (sensor-like behavior)
        cte_deadband = 0.10 * self.car_size

        # Max PD curvature contribution (limit how much PD can override PP)
        # Scale as 1/car_size (because curvature ~ 1/length)
        curv_pd_max = 2.0 / max(self.car_size, 1e-9)

        # Overall curvature cap (also 1/car_size scaled)
        curvature_max = 3.5 / max(self.car_size, 1e-9)

        # Rotate-in-place threshold (degrees) to avoid driving forward when facing away
        rotate_in_place_deg = 110.0
        rotate_in_place_rad = math.radians(rotate_in_place_deg)

        # -------------------------
        # Speed scheduling (unit-insensitive)
        # -------------------------
        # Base speed reduced by heading error and curvature demand.
        # (still in normalized wheel speed units)
        turn_factor = max(self._min_turn_factor, math.cos(abs(heading_error_rad)))
        base_speed = self._max_speed * turn_factor

        # -------------------------
        # 4) Compute CTE-PD term (only when near path)
        # -------------------------
        use_pd = dist_to_path <= cte_pd_enable_dist

        # Convert "cte" to a normalized-ish value by clamping using car_size baseline
        cte_used = cte_raw
        if abs(cte_used) < cte_deadband:
            cte_used = 0.0
        cte_used = _clamp(cte_used, -cte_clip, cte_clip)

        # Derivative of cte (filtered) - only meaningful when PD enabled
        now_t = self._prev_timestamp
        if not use_pd or now_t is None:
            # reset derivative state when PD disabled (prevents huge transient)
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
                if dt > 1e-6:
                    cte_dot = (cte_used - self._prev_cte) / dt
                else:
                    cte_dot = 0.0

            # Low-pass filter on derivative
            self._cte_dot_filt = (1.0 - self._cte_dot_alpha) * self._cte_dot_filt + self._cte_dot_alpha * cte_dot
            cte_dot_filt = self._cte_dot_filt

            self._prev_cte = cte_used
            self._prev_cte_time = now_t

            # -------------------------
            # IMPORTANT: Make gains unit-insensitive
            # curvature_pd = kp*(cte/car_size)/car_size  + kd*(cte_dot/car_size)/car_size
            # i.e. kp, kd are dimensionless knobs, curvature scales as 1/car_size.
            # -------------------------
            cte_n = cte_used / max(self.car_size, 1e-9)
            cte_dot_n = cte_dot_filt / max(self.car_size, 1e-9)

            # Dimensionless knobs (tune):
            kp = 1.2
            kd = 0.35

            cte_pd_curv = (kp * cte_n + kd * cte_dot_n) / max(self.car_size, 1e-9)
            cte_pd_curv = _clamp(cte_pd_curv, -curv_pd_max, curv_pd_max)

        # -------------------------
        # 5) Combine curvature
        # -------------------------
        curvature_cmd = curvature_pp + cte_pd_curv
        curvature_cmd = _clamp(curvature_cmd, -curvature_max, curvature_max)

        # Additional speed reduction for large curvature demand (helps suppress S-shape)
        curv_slow = _clamp(1.0 - 0.65 * (abs(curvature_cmd) / max(curvature_max, 1e-9)), 0.35, 1.0)
        base_speed *= curv_slow

        # -------------------------
        # 6) Rotate-in-place capture when facing away (Acquire mode)
        # -------------------------
        # If far from path OR heading is huge, don't push forward; rotate to reduce heading error.
        if (not use_pd) and (abs(heading_error_rad) > rotate_in_place_rad):
            # Rotate direction based on heading_error sign (CCW positive)
            w = 0.6 * self._max_speed  # normalized spin intensity
            left_speed = -w if heading_error_rad > 0 else w
            right_speed = w if heading_error_rad > 0 else -w

            self._car_state.metadata["mode"] = "ROTATE_IN_PLACE"
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
            self._car_state.metadata["pd_enabled"] = False

            return Action(left_speed=_clamp(left_speed, -1.0, 1.0), right_speed=_clamp(right_speed, -1.0, 1.0))

        # -------------------------
        # 7) Curvature -> wheel speeds
        # -------------------------
        diff = curvature_cmd * wheel_base / 2.0
        left_speed = base_speed * (1.0 - diff)
        right_speed = base_speed * (1.0 + diff)

        # normalize to keep within [-max_speed, max_speed]
        max_wheel = max(abs(left_speed), abs(right_speed))
        if max_wheel > self._max_speed:
            scale = self._max_speed / max_wheel
            left_speed *= scale
            right_speed *= scale

        left_speed = _clamp(left_speed, -1.0, 1.0)
        right_speed = _clamp(right_speed, -1.0, 1.0)

        # -------------------------
        # Debug metadata
        # -------------------------
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

        return Action(left_speed=left_speed, right_speed=right_speed)


    def _find_target_point(self, robot_x: float, robot_y: float) -> Optional[Point]:
        """
        Kept same as your original: choose a discrete point at >= lookahead distance.
        """
        if not self._path:
            return None

        best_idx = self._path_index
        best_dist = float("inf")

        for i in range(self._path_index, len(self._path)):
            px, py = self._path[i]
            dist = math.sqrt((robot_x - px) ** 2 + (robot_y - py) ** 2)
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        self._path_index = max(self._path_index, best_idx)

        for i in range(self._path_index, len(self._path)):
            px, py = self._path[i]
            dist = math.sqrt((robot_x - px) ** 2 + (robot_y - py) ** 2)
            if dist >= self._lookahead_distance:
                self._path_index = i
                return (px, py)

        if self._path:
            return self._path[-1]
        return None

    def _pure_pursuit_curvature(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta_deg: float,
        target: Point,
    ) -> Tuple[float, float, float]:
        """
        Returns:
            curvature_pp: 2*sin(alpha)/L
            heading_error_rad: alpha in radians (-pi, pi]
            distance: distance to target
        """
        dx = target[0] - robot_x
        dy = target[1] - robot_y
        distance = math.sqrt(dx * dx + dy * dy)
        if distance < 1e-9:
            return 0.0, 0.0, 0.0

        angle_to_target = math.atan2(dy, dx)  # radians
        robot_theta_rad = math.radians(robot_theta_deg)
        heading_error_rad = _wrap_to_pi(angle_to_target - robot_theta_rad)

        L = max(distance, self._lookahead_distance)
        curvature_pp = 2.0 * math.sin(heading_error_rad) / L
        return curvature_pp, heading_error_rad, distance

    def _find_closest_reference(self, pos: Point) -> Tuple[Point, float, float, int]:
        """
        Find closest projection on the polyline (segment projection) for a smooth CTE.

        Returns:
            ref_point: closest projected point on path
            theta_ref_rad: path tangent heading at that segment (radians, CCW+)
            cte: signed cross-track error (positive = robot is left of path tangent)
            seg_idx: segment start index
        """
        x, y = pos
        n = len(self._path)
        if n == 1:
            px, py = self._path[0]
            theta_ref = math.radians(self._car_state.theta)
            cte = math.sqrt((x - px) ** 2 + (y - py) ** 2)
            return (px, py), theta_ref, cte, 0

        start_i = min(self._path_index, n - 2)

        best_dist2 = float("inf")
        best_proj: Point = self._path[start_i]
        best_theta = 0.0
        best_cte = 0.0
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
                best_cte = nx * (x - proj[0]) + ny * (y - proj[1])

        return best_proj, best_theta, best_cte, best_i

    def reset(self) -> None:
        super().reset()
        self._path = []
        self._path_index = 0
        self._prev_timestamp = None

        self._prev_cte = None
        self._prev_cte_time = None
        self._cte_dot_filt = 0.0

        self._car_state.metadata.pop("path", None)
        self._car_state.metadata.pop("path_index", None)
        self._car_state.metadata.pop("target_point", None)

        # extra debug keys
        self._car_state.metadata.pop("cte", None)
        self._car_state.metadata.pop("cte_used", None)
        self._car_state.metadata.pop("cte_dot_filt", None)
        self._car_state.metadata.pop("curvature_pp", None)
        self._car_state.metadata.pop("curvature_cmd", None)
        self._car_state.metadata.pop("heading_error_deg", None)
        self._car_state.metadata.pop("theta_ref_deg", None)
        self._car_state.metadata.pop("ref_point", None)
        self._car_state.metadata.pop("dist_to_target", None)
        self._car_state.metadata.pop("goal_tol", None)

    def set_speed(self, speed: float) -> None:
        self._max_speed = max(0.0, min(1.0, speed))
