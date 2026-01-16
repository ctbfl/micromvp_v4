"""
UDP action sender for real robot control.

Sends wheel speed commands to robots via UDP.
Each robot has its own IP/port for receiving commands.

Protocol:
- Packet format: struct "<Hhh" = (seq: uint16, left_speed_milli: int16, right_speed_milli: int16)
- Speed values are in range [-1000, 1000] representing normalized [-1.0, 1.0]
- Sequence number wraps around at 65535

Based on: /home/omen/junshan/micromvp_push/network/try_control_car_keyboard.py
"""
from __future__ import annotations

import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

from micromvp.core.models import Action


@dataclass
class RobotEndpoint:
    """Network endpoint for a single robot."""
    robot_id: int
    ip: str
    port: int = 9001


@dataclass
class UDPActionConfig:
    """Configuration for UDP action sender."""
    # Robot endpoints: list of (robot_id, ip, port)
    endpoints: list = field(default_factory=list)

    # Command sending rate (Hz) - sends commands at this frequency
    send_hz: float = 50.0

    # Invert right wheel (some robots have reversed motor wiring)
    invert_right_wheel: bool = True


class UDPActionSender:
    """
    UDP-based action sender for real robots.

    Manages socket connections and sends wheel speed commands
    to multiple robots at a configurable rate.
    """

    def __init__(self, config: UDPActionConfig) -> None:
        self._config = config
        self._running = False

        # Socket for sending
        self._sock: Optional[socket.socket] = None

        # Robot endpoints: robot_id -> (ip, port)
        self._endpoints: Dict[int, Tuple[str, int]] = {}
        for ep in config.endpoints:
            if isinstance(ep, RobotEndpoint):
                self._endpoints[ep.robot_id] = (ep.ip, ep.port)
            elif isinstance(ep, (tuple, list)) and len(ep) >= 3:
                self._endpoints[ep[0]] = (ep[1], ep[2])

        # Sequence numbers: robot_id -> seq
        self._seq: Dict[int, int] = {rid: 0 for rid in self._endpoints}

        # Pending actions: robot_id -> Action
        self._actions: Dict[int, Action] = {}
        self._action_lock = threading.Lock()

        # Background sender thread
        self._thread: Optional[threading.Thread] = None

    def start(self) -> bool:
        """
        Start the action sender.

        Returns:
            True if started successfully
        """
        if self._running:
            return True

        # Create UDP socket
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Start background sender thread
        self._running = True
        self._thread = threading.Thread(target=self._send_loop, daemon=True)
        self._thread.start()

        print(f"UDPActionSender started for {len(self._endpoints)} robots at {self._config.send_hz} Hz")
        return True

    def stop(self) -> None:
        """Stop the action sender and send stop commands."""
        if not self._running:
            return

        # Send stop commands to all robots
        for robot_id in self._endpoints:
            self._send_action(robot_id, Action.stop())
            self._send_action(robot_id, Action.stop())

        self._running = False

        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

        print("UDPActionSender stopped")

    def set_action(self, robot_id: int, action: Action) -> None:
        """
        Set action for a robot.

        The action will be sent at the configured rate until changed.

        Args:
            robot_id: Robot ID
            action: Action to apply
        """
        with self._action_lock:
            self._actions[robot_id] = action

    def set_actions(self, actions: Dict[int, Action]) -> None:
        """
        Set actions for multiple robots.

        Args:
            actions: Dict mapping robot_id to Action
        """
        with self._action_lock:
            for robot_id, action in actions.items():
                self._actions[robot_id] = action

    def send_immediate(self, robot_id: int, action: Action) -> bool:
        """
        Send action immediately (bypassing the rate limiter).

        Useful for emergency stops.

        Args:
            robot_id: Robot ID
            action: Action to send

        Returns:
            True if sent successfully
        """
        return self._send_action(robot_id, action)

    def stop_all(self) -> None:
        """Send stop command to all robots immediately."""
        for robot_id in self._endpoints:
            self._send_action(robot_id, Action.stop())

    def add_robot(self, robot_id: int, ip: str, port: int = 9001) -> None:
        """Add a new robot endpoint."""
        self._endpoints[robot_id] = (ip, port)
        self._seq[robot_id] = 0

    def remove_robot(self, robot_id: int) -> None:
        """Remove a robot endpoint."""
        self._endpoints.pop(robot_id, None)
        self._seq.pop(robot_id, None)
        with self._action_lock:
            self._actions.pop(robot_id, None)

    # -------------------------------------------------------------------------
    # Internal
    # -------------------------------------------------------------------------

    def _send_loop(self) -> None:
        """Background loop for sending actions at configured rate."""
        period = 1.0 / max(1.0, self._config.send_hz)

        while self._running:
            start = time.perf_counter()

            # Get current actions
            with self._action_lock:
                actions = dict(self._actions)

            # Send to each robot
            for robot_id in self._endpoints:
                action = actions.get(robot_id, Action.stop())
                self._send_action(robot_id, action)

            # Rate limiting
            elapsed = time.perf_counter() - start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _send_action(self, robot_id: int, action: Action) -> bool:
        """Send action to a specific robot."""
        endpoint = self._endpoints.get(robot_id)
        if endpoint is None or self._sock is None:
            return False

        ip, port = endpoint

        # Get sequence number
        seq = self._seq.get(robot_id, 0)
        self._seq[robot_id] = (seq + 1) & 0xFFFF

        # Clamp and convert speeds
        left = max(-1.0, min(1.0, action.left_speed))
        right = max(-1.0, min(1.0, action.right_speed))

        # Convert to milli-units (int16 range)
        left_milli = int(left * 1000)
        right_milli = int(right * 1000)

        # Pack and send
        try:
            pkt = struct.pack("<Hhh", seq, left_milli, right_milli)
            self._sock.sendto(pkt, (ip, port))
            return True
        except Exception as e:
            print(f"Failed to send to robot {robot_id}: {e}")
            return False
