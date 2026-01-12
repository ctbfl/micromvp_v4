from __future__ import annotations

import argparse
import sys

from micromvp.core.controller import Command, Controller
from micromvp.core.planner import random_arrangement
from micromvp.core.transport import Pose
from micromvp.env.real_env import RealEnvironment
from micromvp.env.sim_env import SimEnvironment
from micromvp.io.aruco_observer import ArucoObserver
from micromvp.io.wifi_sink import WifiSink, WifiSinkConfig
from micromvp.ui.qt_app import run_app
from micromvp.utils.config import AppConfig


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="microMVP modern controller")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--sim", action="store_true", help="Run in simulation mode")
    mode.add_argument("--real", action="store_true", help="Run with live positioning")
    parser.add_argument("--vision-sub", default="tcp://localhost:5556", help="ZMQ vision endpoint")
    parser.add_argument("--cars", default=None, help="Comma-separated car IDs (e.g. 1,2,3)")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    config = AppConfig()
    if args.cars:
        ids = [int(x.strip()) for x in args.cars.split(",") if x.strip()]
        config.car_info = [(car_id, car_id) for car_id in ids]

    config.sim = True if args.sim or not args.real else False
    config.zmq_endpoint = args.vision_sub

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


    controller = Controller(config=config, environment=environment)
    controller.start()

    exit_code = run_app(config=config, controller=controller)
    controller.stop()
    if not config.sim:
        observer.stop()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
