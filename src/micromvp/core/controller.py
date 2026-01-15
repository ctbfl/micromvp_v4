from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from queue import Queue
from typing import Dict, List, Optional, Tuple, Any

from enum import Enum
from micromvp.core.ddr import calculate_speeds, calculate_speeds_to_pose, angle_diff, simulate_step
from micromvp.core.models import CarState, WorldState
from micromvp.env.base import Environment
from micromvp.utils.config import AppConfig

Point = Tuple[float, float]


@dataclass(slots=True)
class Command:
    name: str
    payload: object = None


class Controller:
    """
    多车差速驱动（diff-drive）路径跟踪控制器。

    该 Controller 维护一个后台控制线程，周期性地：
    1) 从环境获取每辆车的视觉反馈位姿 (x, y, theta)
    2) 对每辆车根据其路径选择一个“当前追踪目标点” target
    3) 调用局部控制器 calculate_speeds() 将当前位姿驱动向 target，得到左右轮速度 (v_l, v_r)
    4) （可选）对多车速度做同步缩放（用于多车节奏对齐）
    5) 对所有车辆速度做短时域预测碰撞避免（vision + simulate_step），必要时减速/让行
    6) 将动作发送给环境

    路径数据：
    - position-only 路径：路径点为 (x, y) 的点列（polyline），或带 .x/.y 属性的对象点列
    - 控制器不会“消费”路径点（不会 pop），而是根据车辆当前位置在路径上投影得到进度 s*，
    然后沿路径前瞻距离 L，取 s*+L 对应的插值点作为 target。
    - loop_path=True 时目标弧长会对 total_s 取模，实现闭环循环跟踪；否则为开环走到末端。

    说明：
    - 该实现不再使用旧版的“按时间推进 distance_along”的方式来选 target；
    保留的 _target_rate/_target_phase 仅用于兼容旧命令（默认不影响控制）。
    """

    def __init__(self, config: AppConfig, environment: Environment):

        """
        初始化控制器状态与后台线程所需资源。

        参数
        ----
        config:
            运行参数与车辆信息（wheel_base、v_max、car_info、sim_speed、以及本控制器的跟踪/避障超参）
        environment:
            环境接口，负责提供视觉反馈 get_feedback() 与发送动作 send_actions()

        初始化内容
        --------
        - 创建 CarState 列表（每辆车一个），包含 tag_id（视觉识别ID）与 car_id（内部编号）
        - 路径跟踪参数：loop_path、lookahead_*、omega_soft_* 等
        - 路径几何缓存：每辆车的累计弧长 cum_s / total_s，用于快速“投影 + 前瞻”取 target
        - 碰撞避免相关状态：让行保持时间 _yield_until、视觉抖动平滑缓存 _last_positions
        - 兼容字段：_target_rate/_target_phase 保留以兼容旧命令 set_target_rate（默认不参与控制）
        """
        

        self._config = config
        self._environment = environment
        self._lock = threading.Lock()
        self._commands: "Queue[Command]" = Queue()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        self._cars: List[CarState] = [
            CarState(car_id=car_id, tag_id=tag_id) for car_id, tag_id in config.car_info
        ]

        # ---- motion params ----
        self._v_max = float(config.v_max)
        self._wb = float(config.wheel_base)

        # ---- 旧参数，兼容使用 ----
        self._syn = False
        self._target_rate = float(getattr(config, "target_points_per_sec", 20.0))  # 兼容旧配置
        self._start_time = time.monotonic()
        self._target_phase = 0.0

        # ---- new tracking knobs (unit scaled by wheel_base, robust to mm/px/m) ----
        self._loop_path = bool(getattr(config, "loop_path", True))  # 默认 True：保持旧版循环
        self._lookahead_base = float(getattr(config, "lookahead_base", 0.8 * self._wb))
        self._lookahead_min = float(getattr(config, "lookahead_min", 0.45 * self._wb))
        self._lookahead_max = float(getattr(config, "lookahead_max", 2.2 * self._wb))
        self._lookahead_k_v = float(getattr(config, "lookahead_k_v", 0.6))  # L = base + k_v*|v_est|

        # soft-turn when close to target
        self._omega_soft_dist = float(getattr(config, "omega_soft_dist", 0.9 * self._wb))
        self._omega_min_scale = float(getattr(config, "omega_min_scale", 0.25))

        # ---- path cache: tag_id -> cache dict ----
        self._path_cache: Dict[int, Dict[str, Any]] = {}
        self._proj_hint_seg: Dict[int, int] = {}  # tag_id -> segment index hint for fast projection

        # ---- collision avoidance state ----
        self._yield_until: Dict[int, float] = {}
        self._last_positions: Dict[int, Tuple[float, float, float]] = {}

    # ------------------------------------------------------------------
    # Public API (unchanged)
    # ------------------------------------------------------------------
    def start(self) -> None:
        """
        启动后台控制线程。

        注意：重复调用 start 不会启动多个线程；如果线程已存在则直接返回。
        线程函数为 _run_loop，daemon=True。
        """
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """
        请求停止后台控制线程。

        实现方式：
        - 向命令队列塞入 "shutdown"
        - join 等待最多 2 秒
        """

        self._commands.put(Command("shutdown"))
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def enqueue(self, command: Command) -> None:
        """
        向控制器命令队列提交命令。

        命令会在控制线程中被 _drain_commands() 消费并执行，从而与主线程隔离。
        """
        self._commands.put(command)

    def snapshot(self) -> WorldState:
        """
        生成当前世界状态快照，用于 UI/调试可视化。

        返回内容包括：
        - cars: 每辆车的 CarState（包含当前位置、当前速度指令、路径列表等）
        - targets: 当前每辆车的“追踪目标点”（由 _current_targets 计算）

        注意：snapshot 只读，不修改任何状态。
        """
        target_map = self._current_targets()
        with self._lock:
            return WorldState(
                cars=[
                    CarState(
                        car_id=car.car_id,
                        tag_id=car.tag_id,
                        x=car.x,
                        y=car.y,
                        theta=car.theta,
                        l_speed=car.l_speed,
                        r_speed=car.r_speed,
                        path=list(car.path),
                    )
                    for car in self._cars
                ],
                targets=target_map,
            )

    # ------------------------------------------------------------------
    # Loop
    # ------------------------------------------------------------------
    def _run_loop(self) -> None:
        """
        控制主循环（后台线程执行）。

        每个周期：
        1) 处理外部命令（_drain_commands）
        2) 从环境读取视觉反馈更新车辆位姿（_update_from_feedback）
        3) 根据路径计算控制指令并发送（_follow_paths）
        4) sleep 10ms（控制周期约 100Hz）
        """
        self._running = True
        while self._running:
            self._drain_commands()
            self._update_from_feedback()
            self._follow_paths()
            time.sleep(0.01)

    def _drain_commands(self) -> None:
        """
        消费并执行命令队列中的所有待处理命令（非阻塞）。

        当前支持的命令：
        - shutdown: 停止控制线程
        - run: 将 _running 置 True（一般不需要）
        - stop: 将 environment 速度缩放设为 0（紧急停）
        - set_env_speed_scale: 调整 environment 速度缩放
        - clear_paths: 清空所有车辆路径，同时清空路径缓存
        - add_point: 给指定 tag_id 的车追加一个路径点（(x,y) 或带 .x/.y 的对象）
        - set_positions: 直接设置车辆位姿（仿真/初始化用）
        - set_paths: 批量设置所有车辆路径，同时清空路径缓存
        - set_sync: 开关速度同步（启发式节奏对齐）

        兼容命令（默认不影响控制）：
        - set_target_rate: 保留旧字段 _target_rate/_target_phase，主要用于兼容旧 UI/调试，不参与 target 计算
        """
        while True:
            try:
                cmd = self._commands.get_nowait()
            except Exception:
                break

            if cmd.name == "shutdown":
                self._running = False
                return

            if cmd.name == "run":
                self._running = True

            elif cmd.name == "stop":
                # 旧代码里是 self._speed_scale=0，但没用到；这里保留语义：直接把环境速度缩放拉到 0
                try:
                    self._environment.set_speed_scale(0.0)
                except Exception:
                    pass

            elif cmd.name == "set_target_rate":
                # 兼容保留：用于旧版 phase 的概念；这里我们不靠它驱动 target，但仍保留字段用于 UI/调试
                new_rate = max(0.1, float(cmd.payload))
                now = time.monotonic()
                elapsed = now - self._start_time
                self._target_phase = (self._target_phase + elapsed * self._target_rate)
                self._start_time = now
                self._target_rate = new_rate

            elif cmd.name == "set_env_speed_scale":
                self._environment.set_speed_scale(float(cmd.payload))

            elif cmd.name == "set_sync":
                self._syn = bool(cmd.payload)

            elif cmd.name == "clear_paths":
                with self._lock:
                    for car in self._cars:
                        car.path = []
                self._path_cache.clear()
                self._proj_hint_seg.clear()

            elif cmd.name == "add_point":
                car_id, point = cmd.payload
                with self._lock:
                    for car in self._cars:
                        if car.tag_id == car_id:
                            car.path.append(point)
                            break
                self._path_cache.pop(car_id, None)
                self._proj_hint_seg.pop(car_id, None)
                self._syn = False

            elif cmd.name == "set_positions":
                positions = cmd.payload
                with self._lock:
                    for car, pos in zip(self._cars, positions):
                        car.x, car.y, car.theta = pos

            elif cmd.name == "set_paths":
                paths = cmd.payload
                with self._lock:
                    for car, path in zip(self._cars, paths):
                        car.path = path
                self._path_cache.clear()
                self._proj_hint_seg.clear()
                self._syn = False

            else:
                # 未知命令：忽略
                pass

    def _update_from_feedback(self) -> None:
        """
        从环境读取视觉反馈并更新车辆位姿。

        environment.get_feedback() 返回映射：tag_id -> Pose(x,y,theta)
        对每辆车：
        - 若反馈存在则覆盖 car.x/car.y/car.theta
        - 若缺失则保持上一帧值（可能意味着视觉丢失）
        """

        data = self._environment.get_feedback()
        with self._lock:
            for car in self._cars:
                pose = data.get(car.tag_id)
                if pose is None:
                    continue
                car.x, car.y, car.theta = pose.x, pose.y, pose.theta

    # ------------------------------------------------------------------
    # Path following
    # ------------------------------------------------------------------
    def _follow_paths(self) -> None:
        """
        根据路径计算并发送控制指令（核心控制逻辑）。

        该版本采用“投影到路径 + 前瞻（lookahead）”的路径跟踪方式：
        - 每个周期根据车辆当前位置在 polyline 上找到最近投影点（得到弧长进度 s*）
        - 再沿路径向前取 s* + L 处的点作为当前追踪目标 target（L 为前瞻距离）
        - 用 calculate_speeds() 生成左右轮速度，使车辆持续跟随路径（而非逐点 pop）

        输入数据来源
        ----------
        - cars_snapshot：从 self._cars 拷贝得到的车辆状态（x, y, theta, 上一帧轮速、路径点列）
        这样可以避免控制计算时与命令线程/视觉更新产生并发写冲突。

        单车控制流程
        ----------
        若该车无路径：输出 (0, 0) 停车。

        否则：
        1) 解析路径点：
            - pts: (x, y) 点列（必需）
            - thetas: 可选的 theta 列表（只有当路径点对象包含 theta 时才有，用于末端姿态对齐）
        2) 弧长缓存：
            - 对 pts 计算并缓存累计弧长 cum_s 与总长度 total_s，避免每帧重复 O(N) 计算
        3) 投影到路径（progress estimation）：
            - 将当前 (x, y) 投影到 polyline 的最近 segment 上，得到弧长坐标 s_star（车辆在路径上的进度）
            - 使用 self._proj_hint_seg 记录上次命中的 segment index，用于局部窗口搜索加速
        4) 前瞻距离 L（lookahead policy）：
            - L = lookahead_base + lookahead_k_v * |v_est|，并裁剪到 [lookahead_min, lookahead_max]
            - v_est 由上一帧轮速估计（0.5*(l_speed+r_speed)），用于“速度越大看得越远”
            - 若为开环路径（loop_path=False），接近终点时会缩短 L，降低 overshoot 风险
        5) 目标点选择：
            - 闭环（loop_path=True）：s_tgt = (s_star + L) % total_s
            - 开环：s_tgt = min(total_s, s_star + L)
            - target = point_at_s(s_tgt) 在 polyline 上插值得到 (tx, ty)
        6) 生成轮速：
            - 调用 calculate_speeds(current_pose, [target], v_max, wheel_base) 得到 (v_l, v_r)
        7) 近距离转向软化（stabilization for pushing）：
            - 当车辆离 target 很近（d < omega_soft_dist）时，按比例缩小角速度 w=(v_r-v_l)/wheel_base
                以减少左右摆动，提升推方块/贴边跟踪的稳定性


        多车后处理
        ----------
        - 若 _syn=True：调用 _synchronize 做弱同步（按路径点数量启发式缩放速度）
        - 调用 _avoid_collisions：基于短视野预测的让行/减速滤波，降低碰撞风险

        输出与副作用
        ----------
        - 将最终轮速写回 self._cars[i].l_speed / r_speed
        - 通过 environment.send_actions({tag_id: (v_l, v_r)}) 发送动作
        - 注意：该跟踪方式不会“消费路径点”（不会 pop），路径是否循环由 loop_path 控制
        """

        with self._lock:
            cars_snapshot = [
                CarState(
                    car_id=car.car_id,
                    tag_id=car.tag_id,
                    x=car.x,
                    y=car.y,
                    theta=car.theta,
                    l_speed=car.l_speed,
                    r_speed=car.r_speed,
                    path=list(car.path),
                )
                for car in self._cars
            ]

        positions = [(c.x, c.y, c.theta) for c in cars_snapshot]
        speeds: List[Tuple[float, float]] = []

        for car in cars_snapshot:
            if not car.path:
                speeds.append((0.0, 0.0))
                continue

            pts, thetas = self._extract_xy_theta(car.path)
            if len(pts) == 1:
                # 单点：直接追这个点
                tx, ty = pts[0]
                vl, vr = calculate_speeds(car.x, car.y, car.theta, [(tx, ty)], self._v_max, self._wb)
                speeds.append((vl, vr))
                continue

            cache = self._get_or_build_cache(car.tag_id, pts)

            # 1) 投影得到 s_star
            hint = self._proj_hint_seg.get(car.tag_id, 1)
            s_star, best_seg = self._project_s_to_polyline(car.x, car.y, cache["pts"], cache["cum_s"], hint_seg=hint)
            self._proj_hint_seg[car.tag_id] = best_seg

            total_s = float(cache["total_s"])
            remaining = max(0.0, total_s - s_star)

            # 2) 前瞻距离 L（动态 + 终点衰减）
            v_est = 0.5 * (car.l_speed + car.r_speed)
            L = self._lookahead_base + self._lookahead_k_v * abs(v_est)
            L = max(self._lookahead_min, min(self._lookahead_max, L))

            if not self._loop_path:
                # 开环：靠近末端时缩短 L，防止 overshoot，但不小于 min
                if remaining < 2.0 * L:
                    L = max(self._lookahead_min, 0.5 * remaining)

            # 3) 目标弧长
            if self._loop_path and total_s > 1e-9:
                s_tgt = (s_star + L) % total_s
            else:
                s_tgt = min(total_s, s_star + L)

            tx, ty = self._point_at_s(cache["pts"], cache["cum_s"], s_tgt)

            # 4) position-only 追踪
            vl, vr = calculate_speeds(car.x, car.y, car.theta, [(tx, ty)], self._v_max, self._wb)

            # 5) 近距离转向软化（减少左右摆）
            d = math.hypot(tx - car.x, ty - car.y)
            if self._omega_soft_dist > 1e-6 and d < self._omega_soft_dist:
                scale = max(self._omega_min_scale, d / self._omega_soft_dist)
                v = 0.5 * (vl + vr)
                w = (vr - vl) / self._wb
                w *= scale
                vl = v - 0.5 * self._wb * w
                vr = v + 0.5 * self._wb * w

            speeds.append((vl, vr))

        # 同步（保留）
        if self._syn:
            self._synchronize(speeds, cars_snapshot)

        # 碰撞避免（保留）
        speeds = self._avoid_collisions(positions, speeds)

        # 写回并发送
        with self._lock:
            for i, car in enumerate(self._cars):
                car.l_speed, car.r_speed = speeds[i]

        actions = {car.tag_id: speeds[i] for i, car in enumerate(self._cars) if car.tag_id != 0}
        self._environment.send_actions(actions)

    # ------------------------------------------------------------------
    # Target reporting for UI (kept shape: Dict[int, Tuple[float,float]] like old)
    # ------------------------------------------------------------------
    def _current_targets(self) -> Dict[int, Tuple[float, float]]:
        targets: Dict[int, Tuple[float, float]] = {}

        with self._lock:
            cars_snapshot = [
                CarState(
                    car_id=car.car_id,
                    tag_id=car.tag_id,
                    x=car.x,
                    y=car.y,
                    theta=car.theta,
                    l_speed=car.l_speed,
                    r_speed=car.r_speed,
                    path=list(car.path),
                )
                for car in self._cars
            ]

        for car in cars_snapshot:
            if not car.path:
                continue
            pts, _ = self._extract_xy_theta(car.path)
            if len(pts) == 1:
                targets[car.tag_id] = pts[0]
                continue

            cache = self._get_or_build_cache(car.tag_id, pts)
            hint = self._proj_hint_seg.get(car.tag_id, 1)
            s_star, best_seg = self._project_s_to_polyline(car.x, car.y, cache["pts"], cache["cum_s"], hint_seg=hint)
            self._proj_hint_seg[car.tag_id] = best_seg

            total_s = float(cache["total_s"])
            v_est = 0.5 * (car.l_speed + car.r_speed)
            L = self._lookahead_base + self._lookahead_k_v * abs(v_est)
            L = max(self._lookahead_min, min(self._lookahead_max, L))

            if self._loop_path and total_s > 1e-9:
                s_tgt = (s_star + L) % total_s
            else:
                s_tgt = min(total_s, s_star + L)

            tx, ty = self._point_at_s(cache["pts"], cache["cum_s"], s_tgt)
            targets[car.tag_id] = (tx, ty)

        return targets

    # ------------------------------------------------------------------
    # Helpers: extract points + optional theta
    # ------------------------------------------------------------------
    @staticmethod
    def _extract_xy_theta(path: List[Any]) -> Tuple[List[Point], Optional[List[float]]]:
        """
        提取路径点的 (x,y)。

        兼容：
        - (x,y) tuple
        - obj.x, obj.y

        返回值保持旧签名：(pts, None)
        - 第二个返回值保留为 None，仅用于兼容旧调用方（当前控制逻辑不使用 theta）。
        """
        pts: List[Point] = []
        for p in path:
            if isinstance(p, tuple) and len(p) == 2:
                pts.append((float(p[0]), float(p[1])))
            else:
                pts.append((float(getattr(p, "x")), float(getattr(p, "y"))))
        return pts, None


    # ------------------------------------------------------------------
    # Cache: cumulative arc-length
    # ------------------------------------------------------------------
    def _get_or_build_cache(self, tag_id: int, pts: List[Point]) -> Dict[str, Any]:
        cached = self._path_cache.get(tag_id)
        if cached is not None and cached.get("pts") == pts:
            return cached

        cum_s: List[float] = [0.0] * len(pts)
        total = 0.0
        for i in range(1, len(pts)):
            dx = pts[i][0] - pts[i - 1][0]
            dy = pts[i][1] - pts[i - 1][1]
            seg = math.hypot(dx, dy)
            total += seg
            cum_s[i] = total

        out = {"pts": pts, "cum_s": cum_s, "total_s": float(total)}
        self._path_cache[tag_id] = out
        return out

    # ------------------------------------------------------------------
    # Geometry: point at arc-length
    # ------------------------------------------------------------------
    @staticmethod
    def _point_at_s(pts: List[Point], cum_s: List[float], s: float) -> Point:
        n = len(pts)
        if n == 0:
            return (0.0, 0.0)
        if n == 1:
            return pts[0]

        total = float(cum_s[-1])
        if total <= 1e-9:
            return pts[0]

        s = float(s)
        if s <= 0.0:
            return pts[0]
        if s >= total:
            return pts[-1]

        # 线性扫描；如果点特别多可以改二分
        for i in range(1, n):
            if cum_s[i] >= s:
                s0 = cum_s[i - 1]
                s1 = cum_s[i]
                seg_len = s1 - s0
                if seg_len <= 1e-9:
                    return pts[i]
                t = (s - s0) / seg_len
                x0, y0 = pts[i - 1]
                x1, y1 = pts[i]
                return (x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)

        return pts[-1]

    # ------------------------------------------------------------------
    # Geometry: project to polyline -> arc-length s*
    # ------------------------------------------------------------------
    @staticmethod
    def _project_s_to_polyline(
        x: float,
        y: float,
        pts: List[Point],
        cum_s: List[float],
        hint_seg: int = 1,
    ) -> Tuple[float, int]:
        n = len(pts)
        if n <= 1:
            return (0.0, 0)

        # segment index i means segment [i-1, i]
        i0 = max(1, min(n - 1, int(hint_seg)))
        win = 30
        lo = max(1, i0 - win)
        hi = min(n - 1, i0 + win)

        best_d2 = float("inf")
        best_s = 0.0
        best_i = i0

        def scan(a: int, b: int) -> None:
            nonlocal best_d2, best_s, best_i
            for i in range(a, b + 1):
                x0, y0 = pts[i - 1]
                x1, y1 = pts[i]
                vx, vy = (x1 - x0), (y1 - y0)
                wx, wy = (x - x0), (y - y0)
                seg_len2 = vx * vx + vy * vy
                if seg_len2 <= 1e-12:
                    t = 0.0
                    px, py = x0, y0
                    seg_len = 0.0
                else:
                    t = (wx * vx + wy * vy) / seg_len2
                    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
                    px = x0 + t * vx
                    py = y0 + t * vy
                    seg_len = math.sqrt(seg_len2)

                dx = x - px
                dy = y - py
                d2 = dx * dx + dy * dy
                if d2 < best_d2:
                    best_d2 = d2
                    best_i = i
                    best_s = float(cum_s[i - 1]) + float(t) * float(seg_len)

        scan(lo, hi)

        # 如果你担心 hint 不准，可以打开全局扫描（更慢但更稳）：
        # scan(1, n - 1)

        return (best_s, best_i)

    # ------------------------------------------------------------------
    # Synchronize (kept)
    # ------------------------------------------------------------------
    @staticmethod
    def _synchronize(speeds: List[Tuple[float, float]], cars: List[CarState]) -> None:
        length = max((len(car.path) for car in cars), default=0)
        for i, car in enumerate(cars):
            diff = length - len(car.path)
            diff = float(12 - diff) / 12.0
            if diff < 0.0:
                diff = 0.0
            v_l = speeds[i][0] * (diff ** 0.5)
            v_r = speeds[i][1] * (diff ** 0.5)
            speeds[i] = (v_l, v_r)

    # ------------------------------------------------------------------
    # Collision avoidance (与你原版几乎一致)
    # ------------------------------------------------------------------
    def _avoid_collisions(
        self,
        positions: List[Tuple[float, float, float]],
        speeds: List[Tuple[float, float]],
    ) -> List[Tuple[float, float]]:
        """
        基于视觉位姿 + 短时域前向预测的碰撞避免滤波器。

        输入
        ----
        positions:
            视觉反馈的当前位姿列表 [(x, y, theta), ...]，顺序与 self._cars 一致
        speeds:
            控制器原始输出的左右轮速度列表 [(v_l, v_r), ...]，顺序与 self._cars 一致

        处理流程
        --------
        1) 对视觉位置做轻量 EMA 平滑（只平滑 x,y；theta 不平滑以避免角度 wrap 问题）
        2) 用 simulate_step 按 dt 对每辆车做 T 秒的轨迹预测（使用当前速度指令）
           - 在仿真模式下 pred_scale 会放大预测用速度，以匹配环境里 sim_speed 的位移尺度
        3) 对每对车辆 (i,j) 计算预测轨迹在整个时域内的最小距离 min_d
           - 若 min_d < d_safe，认为存在潜在冲突
        4) 决策让行（yield）：默认 tag_id 大的车让 tag_id 小的车（确定性避免随机振荡）
           - 引入 yield_hold_s 的“让行保持时间”，防止两车反复互让造成抖动
        5) 根据冲突严重程度 alpha（在 d_hard~d_safe 间线性插值）对让行车辆减速
           - 若非常近（alpha>0.8）额外施加小幅 pivot，帮助打破对顶僵局

        输出
        ----
        返回调整后的速度列表 out，长度与输入 speeds 相同。
        """
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
                    d = math.sqrt(dx * dx + dy * dy)
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

    @staticmethod
    def _apply_yield(
        wheel_speeds: Tuple[float, float],
        alpha: float,
        max_slow: float,
    ) -> Tuple[float, float]:
        """
        将“让行强度” alpha 映射到速度修正。

        设计目标
        --------
        - 尽量保持运动方向不变：先整体缩放 v_l,v_r 来降低前进速度
        - 冲突非常严重时（alpha 大）添加轻微原地转向（pivot），帮助避开正面僵持

        参数
        ----
        wheel_speeds:
            原始左右轮速度 (v_l, v_r)
        alpha:
            冲突强度 [0,1]。越大表示越危险，需要越强干预
        max_slow:
            最大减速比例（例如 0.85 表示最严重时最多减速到 15%）

        返回
        ----
        修正后的 (v_l, v_r)
        """
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
