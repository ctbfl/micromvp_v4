from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Dict, Tuple

from micromvp.core.transport import Pose


class Environment(ABC):
    @abstractmethod
    def get_feedback(self) -> Dict[int, Pose]:
        raise NotImplementedError

    @abstractmethod
    def send_actions(self, actions: Dict[int, Tuple[float, float]]) -> None:
        raise NotImplementedError

    def set_speed_scale(self, scale: float) -> None:
        _ = scale
