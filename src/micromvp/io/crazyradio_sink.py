from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple


@dataclass(slots=True)
class CrazyRadioConfig:
    channel: int = 100
    address: bytes = b"\xE7\xE7\xE7\xE7\xE7"
    payload_size: int = 32
    payload_level: int = 2
    min_car_id: int = 1
    max_car_id: int = 20
    begin_check: int = ord("C")
    end_check: int = ord("M")


class CrazyRadioSink:
    """Placeholder speed sender. Replace with real Crazyradio driver."""

    def __init__(self, config: CrazyRadioConfig) -> None:
        self._config = config

    def send_actions(self, actions: Dict[int, Tuple[float, float]]) -> None:
        _ = actions
