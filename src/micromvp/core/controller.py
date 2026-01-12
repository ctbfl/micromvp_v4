from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from queue import Queue
from typing import Dict, List, Optional, Tuple

from micromvp.core.ddr import calculate_speeds, simulate_step
from micromvp.core.models import CarState, WorldState
from micromvp.env.base import Environment
from micromvp.utils.config import AppConfig


Point = Tuple[float, float]


@dataclass(slots=True)
class Command:
    name: str
    payload: object = None


class Controller:
    def __init__(self, config: AppConfig, environment: Environment):
        self._config = config
        self._environment = environment
        self._lock = threading.Lock()
        self._commands: "Queue[Command]" = Queue()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._syn = False
        self._v_max = config.v_max
        self._target_rate = config.target_points_per_sec
        self._start_time = time.monotonic()
        self._target_phase = 0.0

        self._cars: List[CarState] = [
            CarState(car_id=car_id, tag_id=tag_id) for car_id, tag_id in config.car_info
        ]
        # --- collision avoidance state ---
        self._yield_until: Dict[int, float] = {}  # tag_id -> monotonic time until which this car yields
        self._last_positions: Dict[int, Tuple[float, float, float]] = {}  # tag_id -> (x,y,theta) for jitter smoothing

    def _avoid_collisions(
        self,
        positions: List[Tuple[float, float, float]],
        speeds: List[Tuple[float, float]],
    ) -> List[Tuple[float, float]]:
        """
        Vision-driven + short-horizon prediction collision avoidance.
        - Uses current (x,y,theta) from vision and current wheel speeds (v_l,v_r).
        - Predicts forward with simulate_step for a short horizon and slows/yields when conflicts are detected.
        """
        n = len(speeds)
        if n <= 1:
            return speeds

        now = time.monotonic()
        wb = float(self._config.wheel_base)

        # ---------------- Tunables (pixel-ish units in your UI space) ----------------
        # car "radius" and safety distances
        r_car = 0.8 * wb                # start: ~24 when wb=30
        d_safe = 2.0 * r_car + 0.4 * wb # start: ~60
        d_hard = 1.2 * r_car            # emergency close

        # prediction horizon
        T = 0.6     # seconds of lookahead
        dt = 0.06   # step
        steps = max(1, int(T / dt))

        # yielding behavior
        yield_hold_s = 0.25  # once yield triggered, keep yielding for this duration to avoid oscillation
        max_slow = 0.85      # at most slow down by 85%

        # priority: smaller tag_id has higher priority
        ids = [car.tag_id for car in self._cars]

        # --------- 1) light smoothing of vision jitter (optional but recommended) ---------
        smoothed_positions: List[Tuple[float, float, float]] = []
        for i, (x, y, th) in enumerate(positions):
            tid = ids[i]
            prev = self._last_positions.get(tid)
            if prev is None:
                smoothed_positions.append((x, y, th))
            else:
                # simple EMA (keeps responsiveness but reduces jitter)
                a = 0.65
                xs = a * x + (1 - a) * prev[0]
                ys = a * y + (1 - a) * prev[1]
                # theta smoothing is tricky; keep raw theta (or you can smooth with angle wrap)
                smoothed_positions.append((xs, ys, th))
            self._last_positions[tid] = (smoothed_positions[-1][0], smoothed_positions[-1][1], th)

        out = list(speeds)

        # --------- 2) precompute predicted trajectories ---------
        # 预测速度缩放：仿真里 1.0 的指令通常会被环境放大（sim_speed）
        pred_scale = float(self._config.sim_speed) if getattr(self._config, "sim", False) else 1.0

        traj: List[List[Tuple[float, float]]] = []
        for i in range(n):
            x, y, th = smoothed_positions[i]
            vl_cmd, vr_cmd = out[i]

            # ✅ 只在预测里放大，模拟真实位移尺度
            vl = vl_cmd * pred_scale
            vr = vr_cmd * pred_scale

            pts = [(x, y)]
            for _ in range(steps):
                x, y, th = simulate_step(x, y, th, vl, vr, wb, dt)
                pts.append((x, y))
            traj.append(pts)


        # --------- 3) pairwise conflict detection + resolution ---------
        for i in range(n):
            for j in range(i + 1, n):
                # min predicted distance over horizon
                min_d = float("inf")
                for k in range(len(traj[i])):
                    dx = traj[i][k][0] - traj[j][k][0]
                    dy = traj[i][k][1] - traj[j][k][1]
                    d = (dx * dx + dy * dy) ** 0.5
                    if d < min_d:
                        min_d = d

                if min_d >= d_safe:
                    continue

                # Decide who yields: higher id yields
                id_i, id_j = ids[i], ids[j]
                yield_i = id_i > id_j
                yield_j = id_j > id_i
                if id_i == id_j:
                    yield_i = yield_j = True

                # Apply hysteresis: if currently in yield hold window, keep yielding
                if now < self._yield_until.get(id_i, 0.0):
                    yield_i = True
                if now < self._yield_until.get(id_j, 0.0):
                    yield_j = True

                # intervention strength alpha
                if min_d <= d_hard:
                    alpha = 1.0
                else:
                    alpha = (d_safe - min_d) / max(1e-6, (d_safe - d_hard))
                    alpha = max(0.0, min(1.0, alpha))

                # If conflict detected, set yield hold timers (prevents oscillation)
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
        """
        Reduce forward motion; if very close, add small pivot to break deadlock.
        """
        vl, vr = wheel_speeds

        # primary: slow down both wheels
        slow = 1.0 - max_slow * alpha   # alpha=1 -> slow=1-max_slow
        vl2 = vl * slow
        vr2 = vr * slow

        # emergency: pivot a little to avoid face-to-face deadlock
        if alpha > 0.8:
            pivot = 0.25 * alpha
            # deterministic turn direction to avoid randomness
            if (vl + vr) >= 0:
                vl2 -= pivot
                vr2 += pivot
            else:
                vl2 += pivot
                vr2 -= pivot

        return (vl2, vr2)


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

    def _run_loop(self) -> None:
        self._running = True
        while self._running:
            self._drain_commands()
            self._update_from_feedback()
            self._follow_paths()
            time.sleep(0.01)

    def _drain_commands(self) -> None:
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
                self._speed_scale = 0.0
            elif cmd.name == "set_target_rate":
                new_rate = max(0.1, float(cmd.payload))
                now = time.monotonic()
                elapsed = now - self._start_time
                self._target_phase = (self._target_phase + elapsed * self._target_rate)
                self._start_time = now
                self._target_rate = new_rate
            elif cmd.name == "set_env_speed_scale":
                self._environment.set_speed_scale(float(cmd.payload))
            elif cmd.name == "clear_paths":
                with self._lock:
                    for car in self._cars:
                        car.path = []
            elif cmd.name == "add_point":
                car_id, point = cmd.payload
                with self._lock:
                    for car in self._cars:
                        if car.tag_id == car_id:
                            car.path.append(point)
                            self._syn = False
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
                        car.path = path
                self._syn = False

    def _update_from_feedback(self) -> None:
        data = self._environment.get_feedback()
        with self._lock:
            for car in self._cars:
                pose = data.get(car.tag_id)
                if pose is None:
                    continue
                car.x, car.y, car.theta = pose.x, pose.y, pose.theta

    def _follow_paths(self) -> None:
        speeds: List[Tuple[float, float]] = []
        with self._lock:
            positions = [(car.x, car.y, car.theta) for car in self._cars]
            paths = [list(car.path) for car in self._cars]

        v_max = self._v_max
        time_s = time.monotonic() - self._start_time
        for (x, y, theta), path in zip(positions, paths):
            if not path:
                speeds.append((0.0, 0.0))
                continue
            target = self._target_on_path(path, self._target_phase + time_s * self._target_rate)
            target_path = [target]
            speeds.append(calculate_speeds(x, y, theta, target_path, v_max, self._config.wheel_base))

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
        if self._syn:
            self._synchronize(speeds, cars_snapshot)
        # NEW: collision avoidance filter (uses vision positions + short-horizon prediction)
        speeds = self._avoid_collisions(positions, speeds)

        with self._lock:
            for idx, car in enumerate(self._cars):
                car.l_speed, car.r_speed = speeds[idx]
                car.path = paths[idx]

        actions = {car.tag_id: speeds[idx] for idx, car in enumerate(self._cars) if car.tag_id != 0}
        self._environment.send_actions(actions)


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

    def _current_targets(self) -> Dict[int, Tuple[float, float]]:
        time_s = time.monotonic() - self._start_time
        targets: Dict[int, Tuple[float, float]] = {}
        with self._lock:
            for car in self._cars:
                if not car.path:
                    continue
                targets[car.tag_id] = self._target_on_path(
                    car.path,
                    self._target_phase + time_s * self._target_rate,
                )
        return targets

    def _target_on_path(self, path: List[Tuple[float, float]], distance_along: float) -> Tuple[float, float]:
        if len(path) == 1:
            return path[0]
        lengths = [0.0]
        total = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
            seg = (dx * dx + dy * dy) ** 0.5
            total += seg
            lengths.append(total)
        if total <= 0.0:
            return path[0]
        s = distance_along % total
        for i in range(1, len(lengths)):
            if lengths[i] >= s:
                prev = lengths[i - 1]
                seg_len = lengths[i] - prev
                if seg_len <= 0.0:
                    return path[i]
                t = (s - prev) / seg_len
                x = path[i - 1][0] + (path[i][0] - path[i - 1][0]) * t
                y = path[i - 1][1] + (path[i][1] - path[i - 1][1]) * t
                return (x, y)
        return path[-1]


def build_paths_from_pattern(locs: List[Point], paths: List[List[Point]]) -> List[List[Point]]:
    paths = shuffle_paths(locs, paths)
    return refine_paths(paths)
