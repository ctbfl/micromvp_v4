from __future__ import annotations

import math
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from queue import Queue
from typing import Dict, List, Optional, Tuple

from micromvp.core.ddr import (
    angle_diff,
    calculate_speeds,
    calculate_speeds_to_pose,
    compute_dubins_path,
    sample_dubins_path,
    simulate_step,
)
from micromvp.core.models import CarState, Pose, Waypoint, WorldState, points_to_waypoints
from micromvp.env.base import Environment
from micromvp.utils.config import AppConfig


Point = Tuple[float, float]


@dataclass(slots=True)
class Command:
    name: str
    payload: object = None


class BaseController(ABC):
    """Abstract base controller defining the control loop interface."""

    def __init__(self, config: AppConfig, environment: Environment):
        self._config = config
        self._environment = environment
        self._lock = threading.Lock()
        self._commands: "Queue[Command]" = Queue()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        self._cars: List[CarState] = [
            CarState(car_id=car_id, tag_id=tag_id)
            for car_id, tag_id in config.car_info
        ]

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._commands.put(Command("shutdown"))
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def enqueue(self, command: Command) -> None:
        self._commands.put(command)

    def snapshot(self) -> WorldState:
        """Return current world state snapshot."""
        target_map = self._current_targets()
        with self._lock:
            return WorldState(
                cars=[self._copy_car_state(car) for car in self._cars],
                targets=target_map,
            )

    def _run_loop(self) -> None:
        self._running = True
        while self._running:
            self._drain_commands()
            self._update_from_feedback()
            self._follow_paths()
            time.sleep(0.01)

    def _drain_commands(self) -> None:
        """Process queued commands."""
        while True:
            try:
                cmd = self._commands.get_nowait()
            except Exception:
                break
            if cmd.name == "shutdown":
                self._running = False
                return
            self._handle_command(cmd)

    def _handle_command(self, cmd: Command) -> None:
        """Handle a single command. Subclasses may override to add custom commands."""
        if cmd.name == "run":
            self._running = True
        elif cmd.name == "clear_paths":
            with self._lock:
                for car in self._cars:
                    car.path = []
        elif cmd.name == "add_point":
            car_id, point = cmd.payload
            with self._lock:
                for car in self._cars:
                    if car.tag_id == car_id:
                        # Convert Point to Waypoint with theta=0 for backward compat
                        if isinstance(point, tuple) and len(point) == 2:
                            waypoint = Waypoint(x=point[0], y=point[1], theta=0.0)
                        else:
                            waypoint = point
                        car.path.append(waypoint)
                        break
        elif cmd.name == "add_waypoint":
            # New command for adding waypoints with orientation
            car_id, waypoint = cmd.payload
            with self._lock:
                for car in self._cars:
                    if car.tag_id == car_id:
                        car.path.append(waypoint)
                        break
        elif cmd.name == "set_positions":
            positions = cmd.payload
            with self._lock:
                for car, pos in zip(self._cars, positions):
                    car.x, car.y, car.theta = pos
        elif cmd.name == "set_paths":
            paths = cmd.payload
            with self._lock:
                for car, path in zip(self._cars, paths):
                    # Convert Point paths to Waypoint paths if needed
                    if path and isinstance(path[0], tuple) and len(path[0]) == 2:
                        car.path = points_to_waypoints(path)
                    else:
                        car.path = path

    def _update_from_feedback(self) -> None:
        """Update car states from environment feedback."""
        data = self._environment.get_feedback()
        with self._lock:
            for car in self._cars:
                pose = data.get(car.tag_id)
                if pose is not None:
                    car.x, car.y, car.theta = pose.x, pose.y, pose.theta

    @abstractmethod
    def _follow_paths(self) -> None:
        """Execute one control step. Subclasses must implement."""
        raise NotImplementedError

    @abstractmethod
    def _current_targets(self) -> Dict[int, Waypoint]:
        """Return current target for each car. Subclasses must implement."""
        raise NotImplementedError

    @staticmethod
    def _copy_car_state(car: CarState) -> CarState:
        return CarState(
            car_id=car.car_id,
            tag_id=car.tag_id,
            x=car.x,
            y=car.y,
            theta=car.theta,
            l_speed=car.l_speed,
            r_speed=car.r_speed,
            path=list(car.path),
        )


class PositionOnlyController(BaseController):
    """
    改进版 Position-only 跟踪控制器（兼容 Waypoint 含 theta 的输入，但默认不强制中间段对齐 theta）

    主要改进：
    1) 目标点不再用 “时间相位沿路径走”，而是：当前位置投影到 polyline + 前瞻距离 L
       -> 大幅改善 “target 过近/在身后/抖动” 的问题
    2) 动态前瞻：速度越快看得越远；靠近路径终点逐渐缩短，但不小于 L_min
    3) 近距离角速度软化：离 target 太近时减少转向强度，推方块更稳
    4) 可选末端姿态对齐：仅在接近路径终点时使用 calculate_speeds_to_pose（且角度阈值可松）
    5) 缓存路径弧长，避免每帧 O(N) 重算
    """

    def __init__(self, config: AppConfig, environment: Environment):
        super().__init__(config, environment)

        # ====== 基本参数 ======
        self._v_max = float(config.v_max)
        self._wb = float(config.wheel_base)

        # ====== 前瞻与软化参数（都按 wheel_base 的倍数设定，基本不怕单位坑） ======
        self._L0 = getattr(config, "lookahead_base", 0.8 * self._wb)        # 基础前瞻
        self._L_min = getattr(config, "lookahead_min", 0.45 * self._wb)     # 前瞻下限（推物体别太小）
        self._L_max = getattr(config, "lookahead_max", 2.2 * self._wb)      # 前瞻上限
        self._k_v = getattr(config, "lookahead_k_v", 0.6)                   # L = L0 + k_v * v_est  (v_est 同单位/秒)

        # 近距离角速度软化：d < d0 时逐渐降低转向强度（减少左右摆）
        self._d0 = getattr(config, "omega_soft_dist", 0.9 * self._wb)
        self._omega_min_scale = getattr(config, "omega_min_scale", 0.25)

        # 末端姿态对齐（可选）
        self._enable_final_pose_align = bool(getattr(config, "enable_final_pose_align", True))
        self._final_align_dist = getattr(config, "final_align_dist", 1.2 * self._wb)  # 离终点弧长 < 该值才考虑对齐 theta
        self._final_pos_threshold = getattr(config, "final_pos_threshold", 0.35 * self._wb)
        self._final_angle_threshold = getattr(config, "final_angle_threshold", 0.20)  # rad，别太严，否则会抖

        # ====== 多车同步（保留原逻辑） ======
        self._syn = False

        # ====== 碰撞避免状态（保留原逻辑） ======
        self._yield_until: Dict[int, float] = {}
        self._last_positions: Dict[int, Tuple[float, float, float]] = {}

        # ====== 路径缓存：每辆车缓存 polyline + 累计弧长 ======
        # key 用 tag_id 更稳，不怕 cars 列表顺序变化
        self._path_cache: Dict[int, Dict[str, object]] = {}
        # 缓存最近投影点的 segment index，加速局部搜索
        self._proj_hint_seg: Dict[int, int] = {}

    # ---------------------------
    # Commands
    # ---------------------------
    def _handle_command(self, cmd: Command) -> None:
        if cmd.name == "set_env_speed_scale":
            self._environment.set_speed_scale(float(cmd.payload))
        elif cmd.name == "set_sync":
            self._syn = bool(cmd.payload)
        else:
            super()._handle_command(cmd)
            # 路径可能被修改，清理缓存（最简单可靠）
            if cmd.name in ("clear_paths", "add_point", "add_waypoint", "set_paths"):
                with self._lock:
                    for car in self._cars:
                        self._path_cache.pop(car.tag_id, None)
                        self._proj_hint_seg.pop(car.tag_id, None)

    # ---------------------------
    # Main loop step
    # ---------------------------
    def _follow_paths(self) -> None:
        speeds: List[Tuple[float, float]] = []

        with self._lock:
            cars_snapshot = [self._copy_car_state(c) for c in self._cars]
            positions = [(c.x, c.y, c.theta) for c in cars_snapshot]

        for car, (x, y, theta) in zip(cars_snapshot, positions):
            if not car.path:
                speeds.append((0.0, 0.0))
                continue

            # polyline points: 使用 waypoint 的 (x,y)，忽略中间 theta（末端对齐时再用）
            pts: List[Tuple[float, float]] = [(wp.x, wp.y) for wp in car.path]
            cache = self._get_or_build_cache(car.tag_id, pts)

            # 1) 投影：当前位置投影到 polyline 得到 s_star
            s_star, seg_hint = self._project_s_to_polyline(
                x, y,
                cache["pts"],            # List[(x,y)]
                cache["cum_s"],          # List[float] same length as pts
                hint_seg=self._proj_hint_seg.get(car.tag_id, 0),
            )
            self._proj_hint_seg[car.tag_id] = seg_hint

            total_s = float(cache["total_s"])
            remaining_s = max(0.0, total_s - s_star)

            # 2) 估计当前速度（用上一次命令值即可），用于动态前瞻
            #    v_est 单位与 wheel speed 一致（你的 calculate_speeds 输出的“轮速单位”）
            v_est = 0.5 * (car.l_speed + car.r_speed)

            # 动态前瞻 + 终点衰减（靠近终点时缩短，但不小于 L_min）
            L = self._L0 + self._k_v * abs(v_est)
            L = max(self._L_min, min(self._L_max, L))
            # 终点衰减：剩余弧长小于 2L 时逐渐缩短
            if remaining_s < 2.0 * L:
                L = max(self._L_min, 0.5 * remaining_s)

            # 3) 取前瞻目标点
            s_tgt = min(total_s, s_star + L)
            tx, ty = self._point_at_s(cache["pts"], cache["cum_s"], s_tgt)

            # 4) 计算轮速（主要用 position-only）
            #    这里直接把 target 当成 1 点 path 交给 calculate_speeds
            vl, vr = calculate_speeds(
                x, y, theta,
                [(tx, ty)],
                self._v_max,
                self._wb,
            )

            # 5) 近距离角速度软化：离 target 很近时减小转向（推方块更稳）
            d = math.hypot(tx - x, ty - y)
            if self._d0 > 1e-6:
                scale = d / self._d0
                if scale < 1.0:
                    scale = max(self._omega_min_scale, scale)
                    # 把 (vl,vr) 转成 (v,w) 再缩放 w
                    v = 0.5 * (vl + vr)
                    w = (vr - vl) / self._wb
                    w *= scale
                    vl = v - 0.5 * self._wb * w
                    vr = v + 0.5 * self._wb * w

            # 6) 可选：接近终点时做姿态对齐（但别对中间点强制对齐）
            if (
                self._enable_final_pose_align
                and remaining_s <= self._final_align_dist
                and len(car.path) > 0
            ):
                final_wp = car.path[-1]
                # 如果最终 waypoint 的 theta 有意义（你现在强制含 theta）
                # 用 pose 控制做最后对齐；阈值设松一点避免抖
                vl2, vr2 = calculate_speeds_to_pose(
                    x, y, theta,
                    float(final_wp.x), float(final_wp.y), float(final_wp.theta),
                    self._v_max,
                    self._wb,
                    float(self._final_pos_threshold),
                    float(self._final_angle_threshold),
                )
                # 只在“真的很接近终点位置”时才切到 pose 控制，避免太早纠结角度
                if math.hypot(final_wp.x - x, final_wp.y - y) <= 2.0 * self._final_pos_threshold:
                    vl, vr = vl2, vr2

            speeds.append((vl, vr))

        # 同步（保留原逻辑）
        if self._syn:
            self._synchronize(speeds, cars_snapshot)

        # 碰撞避免（保留原逻辑）
        speeds = self._avoid_collisions(positions, speeds)

        # 写回并发送 action
        with self._lock:
            for idx, car in enumerate(self._cars):
                car.l_speed, car.r_speed = speeds[idx]

        actions = {car.tag_id: speeds[idx] for idx, car in enumerate(self._cars) if car.tag_id != 0}
        self._environment.send_actions(actions)

    # ---------------------------
    # Target reporting
    # ---------------------------
    def _current_targets(self) -> Dict[int, Waypoint]:
        """给 UI 用：返回当前前瞻 target（theta 给 0，不强调姿态）"""
        targets: Dict[int, Waypoint] = {}
        with self._lock:
            cars_snapshot = [self._copy_car_state(c) for c in self._cars]

        for car in cars_snapshot:
            if not car.path:
                continue
            pts = [(wp.x, wp.y) for wp in car.path]
            cache = self._get_or_build_cache(car.tag_id, pts)

            x, y, _ = car.x, car.y, car.theta
            s_star, seg_hint = self._project_s_to_polyline(
                x, y, cache["pts"], cache["cum_s"],
                hint_seg=self._proj_hint_seg.get(car.tag_id, 0),
            )
            self._proj_hint_seg[car.tag_id] = seg_hint

            total_s = float(cache["total_s"])
            remaining_s = max(0.0, total_s - s_star)

            v_est = 0.5 * (car.l_speed + car.r_speed)
            L = self._L0 + self._k_v * abs(v_est)
            L = max(self._L_min, min(self._L_max, L))
            if remaining_s < 2.0 * L:
                L = max(self._L_min, 0.5 * remaining_s)

            s_tgt = min(total_s, s_star + L)
            tx, ty = self._point_at_s(cache["pts"], cache["cum_s"], s_tgt)

            targets[car.tag_id] = Waypoint(x=float(tx), y=float(ty), theta=0.0)

        return targets

    # ---------------------------
    # Path cache
    # ---------------------------
    def _get_or_build_cache(self, tag_id: int, pts: List[Tuple[float, float]]) -> Dict[str, object]:
        """
        缓存 polyline 的累计弧长。只要 pts 不变就复用。
        """
        cached = self._path_cache.get(tag_id)
        if cached is not None:
            old_pts = cached.get("pts")
            if old_pts == pts:
                return cached

        # build
        cum_s: List[float] = [0.0] * len(pts)
        total = 0.0
        for i in range(1, len(pts)):
            dx = float(pts[i][0]) - float(pts[i - 1][0])
            dy = float(pts[i][1]) - float(pts[i - 1][1])
            seg = (dx * dx + dy * dy) ** 0.5
            total += seg
            cum_s[i] = total

        cache = {
            "pts": pts,
            "cum_s": cum_s,
            "total_s": float(total),
        }
        self._path_cache[tag_id] = cache
        return cache

    # ---------------------------
    # Geometry helpers: projection + sampling by arc length
    # ---------------------------
    @staticmethod
    def _point_at_s(
        pts: List[Tuple[float, float]],
        cum_s: List[float],
        s: float,
    ) -> Tuple[float, float]:
        """在 polyline 上按弧长 s 取点（0 <= s <= total_s）"""
        n = len(pts)
        if n == 0:
            return (0.0, 0.0)
        if n == 1:
            return (float(pts[0][0]), float(pts[0][1]))

        total = float(cum_s[-1])
        if total <= 1e-9:
            return (float(pts[0][0]), float(pts[0][1]))

        s = max(0.0, min(total, float(s)))

        # 找到 cum_s[i-1] <= s <= cum_s[i]
        # 线性扫描足够快；如果你的点数很大可以二分
        for i in range(1, n):
            if cum_s[i] >= s:
                s0 = cum_s[i - 1]
                s1 = cum_s[i]
                seg_len = s1 - s0
                if seg_len <= 1e-9:
                    return (float(pts[i][0]), float(pts[i][1]))
                t = (s - s0) / seg_len
                x0, y0 = float(pts[i - 1][0]), float(pts[i - 1][1])
                x1, y1 = float(pts[i][0]), float(pts[i][1])
                return (x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)

        return (float(pts[-1][0]), float(pts[-1][1]))

    @staticmethod
    def _project_s_to_polyline(
        x: float,
        y: float,
        pts: List[Tuple[float, float]],
        cum_s: List[float],
        hint_seg: int = 0,
    ) -> Tuple[float, int]:
        """
        把点 (x,y) 投影到 polyline，返回：
        - s_star：投影点对应的弧长
        - best_seg：最优 segment index（投影落在 segment [i-1, i]，返回 i）
        """
        n = len(pts)
        if n == 0:
            return (0.0, 0)
        if n == 1:
            return (0.0, 0)

        # 局部窗口搜索：优先在 hint_seg 附近找，性能更稳
        # segment 用 i 表示 [i-1, i]，i in [1, n-1]
        i0 = max(1, min(n - 1, int(hint_seg)))
        win = 30  # 窗口大小：点很密也够用；想更稳可以调大
        lo = max(1, i0 - win)
        hi = min(n - 1, i0 + win)

        best_d2 = float("inf")
        best_s = 0.0
        best_i = i0

        # 如果 hint 不靠谱（比如刚换了路径），全局扫一次兜底
        # 这里用一个启发：如果窗口扫完 best_d2 仍很大，就扩大范围
        def scan(i_start: int, i_end: int) -> None:
            nonlocal best_d2, best_s, best_i
            for i in range(i_start, i_end + 1):
                x0, y0 = float(pts[i - 1][0]), float(pts[i - 1][1])
                x1, y1 = float(pts[i][0]), float(pts[i][1])
                vx, vy = (x1 - x0), (y1 - y0)
                wx, wy = (x - x0), (y - y0)
                seg_len2 = vx * vx + vy * vy
                if seg_len2 <= 1e-12:
                    # 退化 segment，当作点
                    px, py = x0, y0
                    t = 0.0
                else:
                    t = (wx * vx + wy * vy) / seg_len2
                    if t < 0.0:
                        t = 0.0
                    elif t > 1.0:
                        t = 1.0
                    px = x0 + t * vx
                    py = y0 + t * vy

                dx = x - px
                dy = y - py
                d2 = dx * dx + dy * dy
                if d2 < best_d2:
                    best_d2 = d2
                    # 投影点弧长：cum_s[i-1] + t * seg_len
                    seg_len = seg_len2 ** 0.5
                    best_s = float(cum_s[i - 1]) + float(t) * float(seg_len)
                    best_i = i

        scan(lo, hi)

        # 兜底：如果窗口内没找到足够近的（比如瞬移/换路径），就全局扫
        # 阈值用路径尺度自适应不太好取，这里给一个保守策略：窗口外再扫一次更大的窗口
        if best_d2 == float("inf"):
            scan(1, n - 1)
        else:
            # 如果你想更“保险”，可以把下面这行打开：当窗口最优仍然很远时全局扫
            # scan(1, n - 1)
            pass

        return (best_s, best_i)

    # ---------------------------
    # Synchronize (unchanged)
    # ---------------------------
    def _synchronize(self, speeds: List[Tuple[float, float]], cars: List[CarState]) -> None:
        length = max((len(car.path) for car in cars), default=0)
        for i, car in enumerate(cars):
            diff = length - len(car.path)
            diff = float(12 - diff) / 12.0
            if diff < 0.0:
                diff = 0.0
            v_l = speeds[i][0] * (diff ** 0.5)
            v_r = speeds[i][1] * (diff ** 0.5)
            speeds[i] = (v_l, v_r)

    # ---------------------------
    # Collision avoidance (same as your original)
    # ---------------------------
    def _avoid_collisions(
        self,
        positions: List[Tuple[float, float, float]],
        speeds: List[Tuple[float, float]],
    ) -> List[Tuple[float, float]]:
        n = len(speeds)
        if n <= 1:
            return speeds

        now = time.monotonic()
        wb = float(self._config.wheel_base)

        r_car = 0.8 * wb
        d_safe = 2.0 * r_car + 0.4 * wb
        d_hard = 1.2 * r_car

        T = 0.6
        dt = 0.06
        steps = max(1, int(T / dt))

        yield_hold_s = 0.25
        max_slow = 0.85

        ids = [car.tag_id for car in self._cars]

        smoothed_positions: List[Tuple[float, float, float]] = []
        for i, (x, y, th) in enumerate(positions):
            tid = ids[i]
            prev = self._last_positions.get(tid)
            if prev is None:
                smoothed_positions.append((x, y, th))
            else:
                a = 0.65
                xs = a * x + (1 - a) * prev[0]
                ys = a * y + (1 - a) * prev[1]
                smoothed_positions.append((xs, ys, th))
            self._last_positions[tid] = (smoothed_positions[-1][0], smoothed_positions[-1][1], th)

        out = list(speeds)

        pred_scale = float(self._config.sim_speed) if getattr(self._config, "sim", False) else 1.0

        traj: List[List[Tuple[float, float]]] = []
        for i in range(n):
            x, y, th = smoothed_positions[i]
            vl_cmd, vr_cmd = out[i]
            vl = vl_cmd * pred_scale
            vr = vr_cmd * pred_scale
            pts = [(x, y)]
            for _ in range(steps):
                x, y, th = simulate_step(x, y, th, vl, vr, wb, dt)
                pts.append((x, y))
            traj.append(pts)

        for i in range(n):
            for j in range(i + 1, n):
                min_d = float("inf")
                for k in range(len(traj[i])):
                    dx = traj[i][k][0] - traj[j][k][0]
                    dy = traj[i][k][1] - traj[j][k][1]
                    d = (dx * dx + dy * dy) ** 0.5
                    if d < min_d:
                        min_d = d

                if min_d >= d_safe:
                    continue

                id_i, id_j = ids[i], ids[j]
                yield_i = id_i > id_j
                yield_j = id_j > id_i
                if id_i == id_j:
                    yield_i = yield_j = True

                if now < self._yield_until.get(id_i, 0.0):
                    yield_i = True
                if now < self._yield_until.get(id_j, 0.0):
                    yield_j = True

                if min_d <= d_hard:
                    alpha = 1.0
                else:
                    alpha = (d_safe - min_d) / max(1e-6, (d_safe - d_hard))
                    alpha = max(0.0, min(1.0, alpha))

                if yield_i:
                    self._yield_until[id_i] = max(self._yield_until.get(id_i, 0.0), now + yield_hold_s)
                if yield_j:
                    self._yield_until[id_j] = max(self._yield_until.get(id_j, 0.0), now + yield_hold_s)

                if yield_i:
                    out[i] = self._apply_yield(out[i], alpha, max_slow)
                if yield_j:
                    out[j] = self._apply_yield(out[j], alpha, max_slow)

        return out

    def _apply_yield(
        self,
        wheel_speeds: Tuple[float, float],
        alpha: float,
        max_slow: float,
    ) -> Tuple[float, float]:
        vl, vr = wheel_speeds
        slow = 1.0 - max_slow * alpha
        vl2 = vl * slow
        vr2 = vr * slow

        if alpha > 0.8:
            pivot = 0.25 * alpha
            if (vl + vr) >= 0:
                vl2 -= pivot
                vr2 += pivot
            else:
                vl2 += pivot
                vr2 -= pivot

        return (vl2, vr2)



class PoseControlMode(Enum):
    """Control mode for PoseController."""
    DIRECT = "direct"      # Direct wheel speeds from pose error
    PATH_BASED = "path"    # Generate Dubins path, sample intermediate waypoints


class PoseController(BaseController):
    """
    Controller that handles full pose control (position + orientation).

    Supports two modes:
    1. DIRECT: Compute wheel speeds directly from current pose to target pose
       (two-phase: position first, then orientation)
    2. PATH_BASED: Generate Dubins curve path to target, sample intermediate waypoints
    """

    def __init__(
        self,
        config: AppConfig,
        environment: Environment,
        mode: PoseControlMode = PoseControlMode.DIRECT,
    ):
        super().__init__(config, environment)
        self._mode = mode
        self._v_max = config.v_max
        self._min_turn_radius = getattr(config, "min_turn_radius", config.wheel_base * 1.5)
        self._position_threshold = getattr(config, "position_threshold", config.wheel_base * 0.3)
        self._angle_threshold = getattr(config, "angle_threshold", 0.1)

        # For PATH_BASED mode: cached sampled paths
        self._sampled_paths: Dict[int, List[Tuple[float, float, float]]] = {}
        self._path_indices: Dict[int, int] = {}  # Current index in sampled path

        # Collision avoidance state (same as PositionOnlyController)
        self._yield_until: Dict[int, float] = {}
        self._last_positions: Dict[int, Tuple[float, float, float]] = {}

    def _handle_command(self, cmd: Command) -> None:
        """Handle pose controller specific commands."""
        if cmd.name == "set_mode":
            self._mode = PoseControlMode(cmd.payload)
        elif cmd.name == "set_env_speed_scale":
            self._environment.set_speed_scale(float(cmd.payload))
        else:
            super()._handle_command(cmd)

    def _follow_paths(self) -> None:
        """Execute pose control."""
        speeds: List[Tuple[float, float]] = []

        with self._lock:
            positions = [(car.x, car.y, car.theta) for car in self._cars]
            paths = [list(car.path) for car in self._cars]

        for i, ((x, y, theta), path) in enumerate(zip(positions, paths)):
            if not path:
                speeds.append((0.0, 0.0))
                continue

            target = path[0]  # First waypoint is current target

            if self._mode == PoseControlMode.DIRECT:
                speed = self._direct_pose_control(x, y, theta, target)
            else:
                speed = self._path_based_control(x, y, theta, target, car_id=i)

            speeds.append(speed)

            # Check if waypoint reached, advance to next
            if self._waypoint_reached(x, y, theta, target):
                paths[i] = paths[i][1:]  # Pop first waypoint
                # Clear cached path for this car when target changes
                if i in self._sampled_paths:
                    del self._sampled_paths[i]
                if i in self._path_indices:
                    del self._path_indices[i]

        # Apply collision avoidance
        speeds = self._avoid_collisions(positions, speeds)

        # Update car states and send actions
        with self._lock:
            for idx, car in enumerate(self._cars):
                car.l_speed, car.r_speed = speeds[idx]
                car.path = paths[idx]

        actions = {car.tag_id: speeds[idx] for idx, car in enumerate(self._cars) if car.tag_id != 0}
        self._environment.send_actions(actions)

    def _direct_pose_control(
        self, x: float, y: float, theta: float, target: Waypoint
    ) -> Tuple[float, float]:
        """
        Direct mode: compute wheel speeds from current pose to target pose.
        Uses two-phase control (position first, then orientation).
        """
        return calculate_speeds_to_pose(
            x, y, theta,
            target.x, target.y, target.theta,
            self._v_max,
            self._config.wheel_base,
            self._position_threshold,
            self._angle_threshold,
        )

    def _path_based_control(
        self, x: float, y: float, theta: float, target: Waypoint, car_id: int
    ) -> Tuple[float, float]:
        """
        Path-based mode: generate Dubins path to target, follow it.
        """
        # Compute Dubins path if not cached
        if car_id not in self._sampled_paths:
            dubins_path = compute_dubins_path(
                x, y, theta,
                target.x, target.y, target.theta,
                self._min_turn_radius,
            )
            if dubins_path is not None:
                self._sampled_paths[car_id] = sample_dubins_path(
                    x, y, theta, dubins_path, step_size=5.0
                )
                self._path_indices[car_id] = 0
            else:
                # Fallback to direct control
                return self._direct_pose_control(x, y, theta, target)

        sampled_path = self._sampled_paths.get(car_id, [])
        if not sampled_path:
            return (0.0, 0.0)

        # Find next waypoint on sampled path (look ahead)
        idx = self._path_indices.get(car_id, 0)
        lookahead_dist = self._config.wheel_base * 0.5

        while idx < len(sampled_path) - 1:
            wp_x, wp_y, wp_theta = sampled_path[idx]
            dist = math.hypot(wp_x - x, wp_y - y)
            if dist > lookahead_dist:
                break
            idx += 1

        self._path_indices[car_id] = idx

        if idx >= len(sampled_path):
            # Reached end of sampled path, use direct control for final alignment
            return self._direct_pose_control(x, y, theta, target)

        next_wp = sampled_path[idx]
        return calculate_speeds_to_pose(
            x, y, theta,
            next_wp[0], next_wp[1], next_wp[2],
            self._v_max,
            self._config.wheel_base,
            self._position_threshold,
            self._angle_threshold,
        )

    def _waypoint_reached(self, x: float, y: float, theta: float, target: Waypoint) -> bool:
        """Check if car has reached the target waypoint (position + orientation)."""
        dist = math.hypot(target.x - x, target.y - y)
        angle_err = abs(angle_diff(target.theta, theta))
        return dist <= self._position_threshold and angle_err <= self._angle_threshold

    def _current_targets(self) -> Dict[int, Waypoint]:
        """Return current target waypoint for each car."""
        targets: Dict[int, Waypoint] = {}
        with self._lock:
            for car in self._cars:
                if car.path:
                    targets[car.tag_id] = car.path[0]
        return targets

    def _avoid_collisions(
        self,
        positions: List[Tuple[float, float, float]],
        speeds: List[Tuple[float, float]],
    ) -> List[Tuple[float, float]]:
        """Vision-driven collision avoidance (same as PositionOnlyController)."""
        n = len(speeds)
        if n <= 1:
            return speeds

        now = time.monotonic()
        wb = float(self._config.wheel_base)

        r_car = 0.8 * wb
        d_safe = 2.0 * r_car + 0.4 * wb
        d_hard = 1.2 * r_car

        T = 0.6
        dt = 0.06
        steps = max(1, int(T / dt))

        yield_hold_s = 0.25
        max_slow = 0.85

        ids = [car.tag_id for car in self._cars]

        smoothed_positions: List[Tuple[float, float, float]] = []
        for i, (x, y, th) in enumerate(positions):
            tid = ids[i]
            prev = self._last_positions.get(tid)
            if prev is None:
                smoothed_positions.append((x, y, th))
            else:
                a = 0.65
                xs = a * x + (1 - a) * prev[0]
                ys = a * y + (1 - a) * prev[1]
                smoothed_positions.append((xs, ys, th))
            self._last_positions[tid] = (smoothed_positions[-1][0], smoothed_positions[-1][1], th)

        out = list(speeds)

        pred_scale = float(self._config.sim_speed) if getattr(self._config, "sim", False) else 1.0

        traj: List[List[Tuple[float, float]]] = []
        for i in range(n):
            x, y, th = smoothed_positions[i]
            vl_cmd, vr_cmd = out[i]
            vl = vl_cmd * pred_scale
            vr = vr_cmd * pred_scale
            pts = [(x, y)]
            for _ in range(steps):
                x, y, th = simulate_step(x, y, th, vl, vr, wb, dt)
                pts.append((x, y))
            traj.append(pts)

        for i in range(n):
            for j in range(i + 1, n):
                min_d = float("inf")
                for k in range(len(traj[i])):
                    dx = traj[i][k][0] - traj[j][k][0]
                    dy = traj[i][k][1] - traj[j][k][1]
                    d = (dx * dx + dy * dy) ** 0.5
                    if d < min_d:
                        min_d = d

                if min_d >= d_safe:
                    continue

                id_i, id_j = ids[i], ids[j]
                yield_i = id_i > id_j
                yield_j = id_j > id_i
                if id_i == id_j:
                    yield_i = yield_j = True

                if now < self._yield_until.get(id_i, 0.0):
                    yield_i = True
                if now < self._yield_until.get(id_j, 0.0):
                    yield_j = True

                if min_d <= d_hard:
                    alpha = 1.0
                else:
                    alpha = (d_safe - min_d) / max(1e-6, (d_safe - d_hard))
                    alpha = max(0.0, min(1.0, alpha))

                if yield_i:
                    self._yield_until[id_i] = max(self._yield_until.get(id_i, 0.0), now + yield_hold_s)
                if yield_j:
                    self._yield_until[id_j] = max(self._yield_until.get(id_j, 0.0), now + yield_hold_s)

                if yield_i:
                    out[i] = self._apply_yield(out[i], alpha, max_slow)
                if yield_j:
                    out[j] = self._apply_yield(out[j], alpha, max_slow)

        return out

    def _apply_yield(
        self,
        wheel_speeds: Tuple[float, float],
        alpha: float,
        max_slow: float,
    ) -> Tuple[float, float]:
        """Reduce forward motion; if very close, add small pivot."""
        vl, vr = wheel_speeds
        slow = 1.0 - max_slow * alpha
        vl2 = vl * slow
        vr2 = vr * slow

        if alpha > 0.8:
            pivot = 0.25 * alpha
            if (vl + vr) >= 0:
                vl2 -= pivot
                vr2 += pivot
            else:
                vl2 += pivot
                vr2 -= pivot

        return (vl2, vr2)


# Backward compatibility alias
Controller = PositionOnlyController


def build_paths_from_pattern(locs: List[Point], paths: List[List[Point]]) -> List[List[Point]]:
    """Build refined paths from pattern. Backward compatible version."""
    from micromvp.core.planner import shuffle_paths, refine_paths
    paths = shuffle_paths(locs, paths)
    return refine_paths(paths)
