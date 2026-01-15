"""
Real hardware environment - Vision + WiFi communication bridge.

RealEnvironment bridges the control system to real hardware:
- Observations come from ArucoObserver (camera + ArUco marker tracking)
- Actions are sent via WifiSink (TCP to AP gateway)
"""
from __future__ import annotations

import time
from typing import Dict, TYPE_CHECKING

from micromvp.core.models import Action, RobotObservation
from micromvp.env.base import Environment

if TYPE_CHECKING:
    from micromvp.io.aruco_observer import ArucoObserver
    from micromvp.io.wifi_sink import WifiSink


class RealEnvironment(Environment):
    """
    Real hardware environment using vision tracking and WiFi communication.

    Observations are obtained from ArUco marker tracking via ZMQ.
    Actions are sent to cars via TCP through the AP gateway.
    """

    def __init__(
        self,
        observer: "ArucoObserver",
        sender: "WifiSink",
    ) -> None:
        """
        Initialize real environment with I/O components.

        Args:
            observer: ArucoObserver for pose tracking
            sender: WifiSink for command transmission
        """
        self._observer = observer
        self._sender = sender
        self._speed_scale = 1.0

    def observe(self) -> Dict[int, RobotObservation]:
        """
        Get observations from vision system.

        Returns observations for all currently tracked robots.
        Robots with stale/lost tracking won't be included.
        """
        poses = self._observer.get_poses()
        timestamp = time.time()

        observations: Dict[int, RobotObservation] = {}
        for tag_id, pose in poses.items():
            observations[tag_id] = RobotObservation(
                robot_id=tag_id,
                x=pose.x,
                y=pose.y,
                theta=pose.theta,
                timestamp=timestamp,
                valid=True,
            )

        return observations

    def apply_actions(self, actions: Dict[int, Action]) -> None:
        """
        Send actions to robots via WiFi.

        Actions are scaled by speed_scale before sending.
        Scale = 0 means immediate stop for all.
        """
        scale = float(self._speed_scale)

        if scale <= 0.0:
            # Immediate stop: send zeros for all
            scaled_actions = {k: (0.0, 0.0) for k in actions.keys()}
        else:
            # Scale and convert to tuple format
            scaled_actions = {
                k: (v.left_speed * scale, v.right_speed * scale)
                for k, v in actions.items()
            }

        self._sender.send_actions(scaled_actions)

    def set_speed_scale(self, scale: float) -> None:
        """
        Set speed scale (0.0 = stopped, 1.0 = normal).

        This affects all subsequent actions sent to hardware.
        """
        self._speed_scale = max(0.0, float(scale))

    def close(self) -> None:
        """Clean up resources."""
        if hasattr(self._sender, "close"):
            self._sender.close()
