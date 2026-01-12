from __future__ import annotations

import threading
import time
from typing import Dict, List

import zmq


class ZmqPositionSubscriber:
    def __init__(self, endpoint: str) -> None:
        self._endpoint = endpoint
        self._lock = threading.Lock()
        self._data: Dict[int, List[float]] = {}
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._running = False

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    def get_all(self) -> Dict[int, List[float]]:
        with self._lock:
            return {k: list(v) for k, v in self._data.items()}

    def _run(self) -> None:
        context = zmq.Context.instance()
        socket = context.socket(zmq.SUB)
        socket.connect(self._endpoint)
        socket.setsockopt(zmq.SUBSCRIBE, b"")
        while self._running:
            try:
                message = socket.recv().decode("utf-8", errors="replace")
            except Exception:
                time.sleep(0.01)
                continue
            parts = message.split()
            if len(parts) != 9:
                continue
            try:
                tag_id = int(parts[0])
                values = [float(v) for v in parts[1:]]
            except ValueError:
                continue
            with self._lock:
                self._data[tag_id] = values
        socket.close(0)
