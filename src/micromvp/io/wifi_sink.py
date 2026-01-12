from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass
from typing import Dict, Tuple, Optional

# actions: {car_id: (vl, vr)}
Actions = Dict[int, Tuple[float, float]]


@dataclass
class WifiSinkConfig:
    host: str = "192.168.4.1"
    port: int = 9000
    connect_timeout_s: float = 2.0
    send_timeout_s: float = 0.5
    reconnect_backoff_s: float = 0.3
    tcp_nodelay: bool = True

    # If >0, throttle send_actions to at most this Hz (e.g., 50).
    max_send_hz: float = 0.0

    # Optional: include extra fields for debugging / protocol evolution
    include_timestamp: bool = True
    include_seq: bool = True


class WifiSink:
    """
    PC-side sink:
      - Connects to AP car (ID=0) via TCP
      - Sends wheel commands for multiple cars as JSON lines (newline-delimited)

    Message format (one line per send_actions):
      {
        "type": "wheel",
        "seq": 123,
        "t_ms": 1712345678901,
        "actions": {
          "1": [0.1, 0.12],
          "2": [-0.2, -0.2]
        }
      }\n

    AP firmware reads a line and forwards via UDP.
    """

    def __init__(self, cfg: WifiSinkConfig | None = None) -> None:
        self.cfg = cfg or WifiSinkConfig()
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self._seq = 0
        self._last_send_ts = 0.0

    # ---------- Public API expected by RealEnvironment ----------
    def send_actions(self, actions: Actions) -> None:
        """
        Send wheel commands for multiple cars.
        actions values are (vl, vr), typically normalized [-1, 1].
        """
        if not actions:
            return

        # --- NEW: AP(0) is gateway-only; never send motion to car 0 ---
        if 0 in actions:
            actions = {k: v for k, v in actions.items() if k != 0}
            if not actions:
                return

        # Optional rate limiting
        if self.cfg.max_send_hz and self.cfg.max_send_hz > 0:
            min_dt = 1.0 / self.cfg.max_send_hz
            now = time.time()
            dt = now - self._last_send_ts
            if dt < min_dt:
                time.sleep(min_dt - dt)

        payload = self._build_payload(actions)
        line = (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")

        with self._lock:
            # Ensure connection
            if self._sock is None:
                self._connect_locked()

            # Send (with one reconnect attempt)
            try:
                assert self._sock is not None
                self._sock.sendall(line)
                self._last_send_ts = time.time()
            except (OSError, AssertionError):
                # Reconnect once and retry
                self._close_locked()
                time.sleep(self.cfg.reconnect_backoff_s)
                self._connect_locked()
                assert self._sock is not None
                self._sock.sendall(line)
                self._last_send_ts = time.time()


    def close(self) -> None:
        with self._lock:
            self._close_locked()

    # ---------- Internal helpers ----------
    def _build_payload(self, actions: Actions) -> dict:
        self._seq += 1
        d = {
            "type": "wheel",
            "actions": {str(k): [float(vl), float(vr)] for k, (vl, vr) in actions.items()},
        }
        if self.cfg.include_seq:
            d["seq"] = self._seq
        if self.cfg.include_timestamp:
            d["t_ms"] = int(time.time() * 1000)
        return d

    def _connect_locked(self) -> None:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.cfg.connect_timeout_s)
        s.connect((self.cfg.host, self.cfg.port))

        # After connect, set send timeout
        s.settimeout(self.cfg.send_timeout_s)

        if self.cfg.tcp_nodelay:
            try:
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                pass

        self._sock = s

    def _close_locked(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
