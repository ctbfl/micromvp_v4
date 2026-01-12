from __future__ import annotations

from typing import Dict, Tuple

from micromvp.core.transport import Pose
from micromvp.env.base import Environment
from micromvp.io.aruco_observer import ArucoObserver


class RealEnvironment(Environment):
    def __init__(self, observer: ArucoObserver, sender) -> None:
        self._observer = observer
        self._sender = sender
        self._speed_scale = 1.0

    def get_feedback(self) -> Dict[int, Pose]:
        return self._observer.get_poses()

    def send_actions(self, actions: Dict[int, Tuple[float, float]]) -> None:
        s = float(self._speed_scale)
        if s <= 0.0:
            # 立刻停：直接全 0
            scaled = {k: (0.0, 0.0) for k in actions.keys()}
        else:
            scaled = {k: (v[0] * s, v[1] * s) for k, v in actions.items()}
        self._sender.send_actions(scaled)

    def set_speed_scale(self, scale: float) -> None:
        self._speed_scale = max(0.0, float(scale))

    def close(self) -> None:
        # 如果 sender 有 close（WifiSink 有），就关掉
        if hasattr(self._sender, "close"):
            self._sender.close()
