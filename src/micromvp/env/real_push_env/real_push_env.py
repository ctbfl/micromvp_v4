"""
Real Push Environment for 1st Spring St real-world setup.

Contract:
- GUI always exists (your PyQt app).
- Preview is optional debug CV window, controlled by `no_preview`.
- To show preview window safely, call env.render() from Qt main thread (e.g., QTimer).
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

from micromvp.core.models import Action, RobotObservation, WorkspaceConfig
from micromvp.env.base import Environment

from .observer import ArucoObserver, ObserverConfig, WORKSPACE_W_CM, WORKSPACE_H_CM
from .udp_action import SerialActionSender, SerialActionConfig, RobotEndpoint


@dataclass
class RealPushConfig:
    # Workspace dimensions (cm)
    width: float = WORKSPACE_W_CM
    height: float = WORKSPACE_H_CM

    # Car geometry (cm)
    car_width: float = 4.8
    car_height: float = 4.8
    offset_w: float = 2.4
    offset_h: float = 2.4

    # Dynamics (cm/s)
    wheel_base: float = 4.2
    max_wheel_speed: float = 10.0
    frequency: float = 30.0

    # Robot endpoints [(id, ip, port), ...]
    robot_endpoints: List[Tuple[int, str, int]] = field(default_factory=list)

    # Camera settings
    camera_device: int = 0
    camera_resolution: str = "720p"
    camera_fps: int = 60
    undistort: bool = False
    calibration_file: str = os.path.join(os.path.dirname(__file__), "camera.yaml")

    # Observer settings
    workspace_dict: str = "DICT_5X5_50"
    car_dict: str = "DICT_4X4_50"
    car_marker_size_mm: float = 36.0
    warmup_frames: int = 30

    # The ONLY preview option you want
    no_preview: bool = False

    # Action settings
    send_hz: float = 60.0
    invert_right_wheel: bool = True


class RealPushEnv(Environment):
    def __init__(self, config: RealPushConfig) -> None:
        self._config = config
        self._started = False

        self._workspace_config = WorkspaceConfig(
            width=config.width,
            height=config.height,
            car_width=config.car_width,
            car_height=config.car_height,
            offset_w=config.offset_w,
            offset_h=config.offset_h,
            wheel_base=config.wheel_base,
            max_wheel_speed=config.max_wheel_speed,
            frequency=config.frequency,
            car_id_list=[ep[0] for ep in config.robot_endpoints],
        )

        obs_config = ObserverConfig(
            camera_device=config.camera_device,
            resolution=config.camera_resolution,
            fps=config.camera_fps,
            undistort=config.undistort,
            calibration_file=config.calibration_file,
            workspace_dict=config.workspace_dict,
            car_dict=config.car_dict,
            car_marker_size_mm=config.car_marker_size_mm,
            warmup_frames=config.warmup_frames,
            no_preview=config.no_preview,
        )
        self._observer = ArucoObserver(obs_config)

        action_config = SerialActionConfig(
            endpoints=[RobotEndpoint(robot_id=ep[0], ip=ep[1], port=ep[2]) for ep in config.robot_endpoints],
            send_hz=config.send_hz,
            invert_right_wheel=config.invert_right_wheel,
        )
        self._action_sender = SerialActionSender(action_config)
        self._speed_scale = 1.0

    @property
    def workspace_config(self) -> WorkspaceConfig:
        return self._workspace_config

    def start(self, wait_for_ready: bool = True, timeout: float = 5.0) -> bool:
        if self._started:
            return True

        print("[RealPushEnv] Starting subsystems...")

        if not self._observer.start():
            print("[RealPushEnv] Error: Failed to start observer.")
            return False

        if not self._action_sender.start():
            print("[RealPushEnv] Error: Failed to start action sender.")
            self._observer.stop()
            return False

        self._started = True

        if wait_for_ready:
            print(f"[RealPushEnv] Checking workspace status (timeout={timeout}s)...")
            t0 = time.time()
            while time.time() - t0 < timeout:
                if self._observer.is_workspace_ready():
                    print("[RealPushEnv] Workspace ready!")
                    return True
                time.sleep(0.05)
            print("[RealPushEnv] Warning: workspace not ready within timeout.")

        return True

    def observe(self) -> Dict[int, RobotObservation]:
        car_obs = self._observer.get_observations()
        observations: Dict[int, RobotObservation] = {}
        for car_id, obs in car_obs.items():
            observations[car_id] = RobotObservation(
                robot_id=car_id,
                x=obs.x_cm,
                y=obs.y_cm,
                theta=obs.yaw_deg,
                timestamp=obs.timestamp,
            )
        return observations

    def render(self) -> None:
        """
        IMPORTANT:
        This MUST be called in Qt main thread (e.g., QTimer timeout callback).
        It will show CV preview window only if no_preview=False.
        """
        self._observer.render()

    def apply_actions(self, actions: Dict[int, Action]) -> None:
        if self._speed_scale != 1.0:
            scaled = {}
            for rid, act in actions.items():
                scaled[rid] = Action(
                    left_speed=act.left_speed * self._speed_scale,
                    right_speed=act.right_speed * self._speed_scale,
                )
            self._action_sender.set_actions(scaled)
        else:
            self._action_sender.set_actions(actions)

    def set_speed_scale(self, scale: float) -> None:
        self._speed_scale = max(0.0, min(1.0, float(scale)))
        if self._speed_scale == 0.0:
            self._action_sender.stop_all()

    def stop_all(self) -> None:
        self._action_sender.stop_all()

    def close(self) -> None:
        if not self._started:
            return
        print("[RealPushEnv] Closing...")
        self._action_sender.stop()
        # Ideally call close() from GUI main thread so destroyWindow is safe
        self._observer.stop()
        self._started = False
        print("[RealPushEnv] Closed.")

    # Robot management
    def add_robot(self, robot_id: int, ip: str, port: int = 9001) -> None:
        self._action_sender.add_robot(robot_id, ip, port)
        if robot_id not in self._workspace_config.car_id_list:
            self._workspace_config.car_id_list.append(robot_id)

    def remove_robot(self, robot_id: int) -> None:
        self._action_sender.remove_robot(robot_id)
        if robot_id in self._workspace_config.car_id_list:
            self._workspace_config.car_id_list.remove(robot_id)
