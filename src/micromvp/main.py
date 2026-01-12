from __future__ import annotations

import argparse

from micromvp.core.controller import (
    BaseController,
    Command,
    Controller,
    PoseControlMode,
    PoseController,
    PositionOnlyController,
)
from micromvp.core.planner import random_arrangement
from micromvp.core.transport import Pose
from micromvp.env.base import Environment
from micromvp.env.real_env import RealEnvironment
from micromvp.env.sim_env import SimEnvironment
from micromvp.io.aruco_observer import ArucoObserver
from micromvp.io.wifi_sink import WifiSink, WifiSinkConfig
from micromvp.ui.qt_app import run_app
from micromvp.utils.config import AppConfig


def create_controller(config: AppConfig, environment: Environment) -> BaseController:
    """Factory function to create the appropriate controller based on config."""
    if config.use_pose_control:
        mode = PoseControlMode(config.pose_control_mode)
        return PoseController(config, environment, mode)
    return PositionOnlyController(config, environment)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="microMVP modern controller")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--sim", action="store_true", help="Run in simulation mode")
    mode.add_argument("--real", action="store_true", help="Run with live positioning")
    parser.add_argument("--vision-sub", default="tcp://localhost:5556", help="ZMQ vision endpoint")
    parser.add_argument("--cars", default=None, help="Comma-separated car IDs (e.g. 1,2,3)")
    parser.add_argument("--pose-control", action="store_true", help="Enable pose (position+orientation) control")
    parser.add_argument("--pose-mode", choices=["direct", "path"], default="direct",
                        help="Pose control mode: 'direct' (two-phase) or 'path' (Dubins curves)")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    config = AppConfig()
    if args.cars:
        ids = [int(x.strip()) for x in args.cars.split(",") if x.strip()]
        config.car_info = [(car_id, car_id) for car_id in ids]

    config.sim = True if args.sim or not args.real else False
    config.zmq_endpoint = args.vision_sub

    # Pose control configuration from command line
    config.use_pose_control = args.pose_control
    config.pose_control_mode = args.pose_mode

    if config.sim:
        starts = random_arrangement(len(config.car_info), config.boundary(), config.wheel_base * 2)
        initial = {
            tag_id: Pose(x=pos[0], y=pos[1], theta=pos[2])
            for (car_id, tag_id), pos in zip(config.car_info, starts)
        }
        environment = SimEnvironment(config=config, initial=initial)

    else:
        observer = ArucoObserver(config.zmq_endpoint)
        observer.start()

        # PC -> TCP -> AP(0号机)
        sender = WifiSink(WifiSinkConfig(
            host="192.168.4.1",
            port=9000,
            max_send_hz=80,   # 可选：限速更稳
        ))
        environment = RealEnvironment(observer=observer, sender=sender)

    # Use factory to create appropriate controller
    controller = create_controller(config=config, environment=environment)
    controller.start()

    exit_code = run_app(config=config, controller=controller)
    controller.stop()
    if not config.sim:
        observer.stop()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
