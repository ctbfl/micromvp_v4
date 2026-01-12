from __future__ import annotations

import time
from typing import Dict, Tuple

from micromvp.core.ddr import simulate_step
from micromvp.core.transport import Pose
from micromvp.env.base import Environment
from micromvp.utils.config import AppConfig


class SimEnvironment(Environment):
    def __init__(self, config: AppConfig, initial: Dict[int, Pose]) -> None:
        self._config = config
        self._poses = dict(initial)
        self._actions: Dict[int, Tuple[float, float]] = {}
        self._last_time = time.perf_counter()
        self._speed_scale = 1.0

    def get_feedback(self) -> Dict[int, Pose]:
        now = time.perf_counter()
        dt = now - self._last_time
        self._last_time = now
        if dt <= 0.0:
            dt = 0.0
        if dt > 0.05:
            dt = 0.05
        for tag_id, pose in list(self._poses.items()):
            v_l, v_r = self._actions.get(tag_id, (0.0, 0.0))
            x, y, theta = simulate_step(
                pose.x,
                pose.y,
                pose.theta,
                v_l * self._config.sim_speed * self._speed_scale,
                v_r * self._config.sim_speed * self._speed_scale,
                self._config.wheel_base,
                dt,
            )
            self._poses[tag_id] = Pose(x=x, y=y, theta=theta)
        return dict(self._poses)

    def send_actions(self, actions: Dict[int, Tuple[float, float]]) -> None:
        self._actions = dict(actions)

    def set_speed_scale(self, scale: float) -> None:
        self._speed_scale = max(0.0, float(scale))
