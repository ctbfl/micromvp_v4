"""
MicroMVP Entry Point.

This module provides the main entry point for the MicroMVP application.
It sets up the environment, creates cars, and launches the control loop with GUI.

Architecture:
    Environment (Sim/Real) <- Coordinator -> [Car, Car, ...] <- GUI

Usage:
    # Simulation mode (default)
    micromvp --sim

    # Real hardware mode
    micromvp --real --vision-sub tcp://localhost:5556 --cars 1,2,3
"""
from __future__ import annotations

import argparse
from typing import Dict, List, Tuple

from micromvp.core.coordinator import Coordinator, CoordinatorConfig
from micromvp.core.models import CarConfig
from micromvp.core.planner import random_arrangement
from micromvp_v4.src.micromvp.env.real_env.real_env import RealEnvironment
from micromvp_v4.src.micromvp.env.sim_env.sim_env import SimConfig, SimEnvironment
from micromvp.io.aruco_observer import ArucoObserver
from micromvp.io.wifi_sink import WifiSink, WifiSinkConfig
from micromvp.gui.qt_app import run_app
from micromvp.utils.config import AppConfig


def _parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="MicroMVP - Multi-robot Control System")

    # Mode selection
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--sim",
        action="store_true",
        help="Run in simulation mode (default)",
    )
    mode.add_argument(
        "--real",
        action="store_true",
        help="Run with real hardware",
    )

    # Connection settings
    parser.add_argument(
        "--vision-sub",
        default="tcp://localhost:5556",
        help="ZMQ vision endpoint (default: tcp://localhost:5556)",
    )

    # Car configuration
    parser.add_argument(
        "--cars",
        default=None,
        help="Comma-separated car IDs (e.g., 1,2,3). Default: 1,2,3,4,5",
    )

    return parser.parse_args()


def _create_car_configs(
    car_info: List[Tuple[int, int]],
    wheel_base: float,
    v_max: float,
) -> List[CarConfig]:
    """Create CarConfig objects for all cars."""
    return [
        CarConfig.from_wheel_base(
            robot_id=car_id,
            tag_id=tag_id,
            wheel_base=wheel_base,
            v_max=v_max,
        )
        for car_id, tag_id in car_info
    ]


def _create_initial_poses(
    car_info: List[Tuple[int, int]],
    app_config: AppConfig,
) -> Dict[int, Tuple[float, float, float]]:
    """Create random initial poses for simulation."""
    starts = random_arrangement(
        len(car_info),
        app_config.boundary(),
        app_config.wheel_base * 2,
    )
    return {
        tag_id: (pos[0], pos[1], pos[2])
        for (_, tag_id), pos in zip(car_info, starts)
    }


def main() -> int:
    """Main entry point."""
    args = _parse_args()

    # Initialize application config
    app_config = AppConfig()

    # Parse car IDs if provided
    if args.cars:
        ids = [int(x.strip()) for x in args.cars.split(",") if x.strip()]
        app_config.car_info = [(car_id, car_id) for car_id in ids]

    # Determine mode
    is_sim = args.sim or not args.real
    app_config.sim = is_sim
    app_config.zmq_endpoint = args.vision_sub

    # Create car configurations
    car_configs = _create_car_configs(
        app_config.car_info,
        app_config.wheel_base,
        app_config.v_max,
    )

    # Create environment
    observer = None
    if is_sim:
        # Simulation mode
        sim_config = SimConfig(
            sim_speed=app_config.sim_speed,
            wheel_base=app_config.wheel_base,
        )
        initial_poses = _create_initial_poses(app_config.car_info, app_config)
        environment = SimEnvironment(config=sim_config, initial_poses=initial_poses)
    else:
        # Real hardware mode
        observer = ArucoObserver(app_config.zmq_endpoint)
        observer.start()

        sender = WifiSink(WifiSinkConfig(
            host="192.168.4.1",
            port=9000,
            max_send_hz=80,
        ))
        environment = RealEnvironment(observer=observer, sender=sender)

    # Create coordinator
    coordinator_config = CoordinatorConfig(
        control_hz=100.0,
        collision_avoidance=True,
    )
    coordinator = Coordinator(
        environment=environment,
        car_configs=car_configs,
        config=coordinator_config,
    )

    # Start coordinator control loop
    coordinator.start()

    # Run GUI (blocking)
    try:
        exit_code = run_app(config=app_config, coordinator=coordinator)
    finally:
        # Cleanup
        coordinator.stop()
        if observer is not None:
            observer.stop()
        environment.close()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
