"""
Coordinator module - High-level multi-car orchestration.

The Coordinator is an OPTIONAL component that provides a convenient way to
manage multiple cars. It encapsulates the main control loop and handles:
- Observation distribution to cars
- Action collection and execution
- Collision avoidance (optional)
- Task coordination

Users can bypass the Coordinator and write their own control loop directly.

Example usage:
    env = SimEnvironment(config, initial_poses)
    coordinator = Coordinator(env, car_configs)

    # Set up pattern
    for i, car in enumerate(coordinator.cars):
        car.follow_path(paths[i])

    # Run (blocking)
    coordinator.run()

    # Or non-blocking step:
    coordinator.step()
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from queue import Queue
from typing import Any, Callable, Dict, List, Optional, Tuple

from micromvp.core.car import Car
from micromvp.core.ddr import simulate_step
from micromvp.core.models import (
    Action,
    CarConfig,
    CarState,
    Point,
    RobotObservation,
    TaskState,
    WorldState,
)
from micromvp.env.base import Environment


@dataclass
class CoordinatorConfig:
    """Configuration for the Coordinator."""
    control_hz: float = 100.0           # Control loop frequency
    collision_avoidance: bool = True    # Enable collision avoidance

    # Collision avoidance parameters
    collision_d_safe: float = 140.0     # Safe distance (start avoiding) - increased
    collision_d_hard: float = 70.0      # Hard distance (max avoidance) - increased
    collision_d_stop: float = 90.0      # Distance at which yielding car stops completely
    collision_prediction_t: float = 0.8 # Prediction horizon (seconds) - increased
    collision_prediction_dt: float = 0.05  # Prediction step size
    collision_max_slow: float = 0.95    # Maximum slowdown factor (before full stop)
    collision_yield_hold: float = 0.4   # Yield hold time (seconds) - increased

    # Speed synchronization (optional)
    sync_enabled: bool = False


@dataclass(slots=True)
class Command:
    """Command for coordinator message queue."""
    name: str
    payload: Any = None


class Coordinator:
    """
    High-level coordinator for multi-car control.

    The Coordinator manages multiple Car agents and provides:
    - Central control loop running in background thread
    - Collision avoidance between cars
    - Task state management
    - Snapshot generation for UI
    - Command queue for async control

    Note: The Coordinator is optional. Users can directly use Car.get_action()
    and Environment.step() in their own control loop if desired.
    """

    def __init__(
        self,
        environment: Environment,
        car_configs: List[CarConfig],
        config: Optional[CoordinatorConfig] = None,
    ) -> None:
        """
        Initialize coordinator with environment and cars.

        Args:
            environment: The Environment (SimEnvironment or RealEnvironment)
            car_configs: List of CarConfig for each car
            config: Optional coordinator configuration
        """
        self._env = environment
        self._config = config or CoordinatorConfig()

        # Create Car agents
        self._cars: List[Car] = [Car(cfg) for cfg in car_configs]

        # Thread control
        self._lock = threading.Lock()
        self._commands: Queue[Command] = Queue()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Collision avoidance state
        self._yield_until: Dict[int, float] = {}
        self._last_positions: Dict[int, Tuple[float, float, float]] = {}

        # Speed scale (for pause/resume)
        self._speed_scale = 1.0

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------
    @property
    def cars(self) -> List[Car]:
        """Get list of Car agents (read-only access to list)."""
        return self._cars

    @property
    def environment(self) -> Environment:
        """Get the environment."""
        return self._env

    # ------------------------------------------------------------------
    # Public API: Thread control
    # ------------------------------------------------------------------
    def start(self) -> None:
        """Start the background control loop."""
        if self._thread is not None:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop the background control loop."""
        self._commands.put(Command("shutdown"))
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    # ------------------------------------------------------------------
    # Public API: Commands
    # ------------------------------------------------------------------
    def enqueue(self, command: Command) -> None:
        """Add command to queue for processing in control loop."""
        self._commands.put(command)

    def set_speed_scale(self, scale: float) -> None:
        """Set global speed scale (also affects environment)."""
        self._speed_scale = max(0.0, float(scale))
        self._env.set_speed_scale(scale)

    def pause(self) -> None:
        """Pause all motion."""
        self.set_speed_scale(0.0)

    def resume(self) -> None:
        """Resume motion at normal speed."""
        self.set_speed_scale(1.0)

    # ------------------------------------------------------------------
    # Public API: High-level task commands
    # ------------------------------------------------------------------
    def set_paths(self, paths: List[List[Point]], loop: bool = True) -> None:
        """
        Set paths for all cars.

        Args:
            paths: List of paths (one per car, in order)
            loop: Whether to loop paths
        """
        with self._lock:
            for i, car in enumerate(self._cars):
                if i < len(paths):
                    car.follow_path(paths[i], loop=loop)

    def clear_paths(self) -> None:
        """Clear all car paths and stop them."""
        with self._lock:
            for car in self._cars:
                car.stop()

    def add_point_to_car(self, car_id: int, point: Point) -> None:
        """Add a point to a specific car's path."""
        with self._lock:
            for car in self._cars:
                if car.robot_id == car_id or car.tag_id == car_id:
                    car.add_path_point(point)
                    break

    def stop_all(self) -> None:
        """Stop all cars immediately."""
        with self._lock:
            for car in self._cars:
                car.stop()

    # ------------------------------------------------------------------
    # Public API: State queries
    # ------------------------------------------------------------------
    def snapshot(self) -> WorldState:
        """
        Get current world state snapshot for UI rendering.

        Returns a copy of current state, safe for UI thread access.
        """
        with self._lock:
            car_states = [car.to_car_state() for car in self._cars]
            targets = {
                car.tag_id: car.current_target
                for car in self._cars
                if car.current_target is not None
            }
        return WorldState(cars=car_states, targets=targets)

    def all_tasks_done(self) -> bool:
        """Check if all cars have completed their tasks."""
        with self._lock:
            return all(car.is_task_done or car.is_idle for car in self._cars)

    def wait_until_done(self, timeout: Optional[float] = None) -> bool:
        """
        Wait until all cars complete their tasks.

        Args:
            timeout: Maximum wait time in seconds (None = infinite)

        Returns:
            True if all tasks completed, False if timeout
        """
        start = time.time()
        while not self.all_tasks_done():
            if timeout is not None and (time.time() - start) > timeout:
                return False
            time.sleep(0.01)
        return True

    # ------------------------------------------------------------------
    # Single step (for manual control)
    # ------------------------------------------------------------------
    def step(self) -> Dict[int, RobotObservation]:
        """
        Execute one control step manually.

        This allows users to run the coordinator in their own loop
        instead of using the background thread.

        Returns:
            Current observations after step
        """
        self._drain_commands()
        obs = self._env.observe()
        self._update_cars(obs)
        actions = self._collect_actions(obs)
        if self._config.collision_avoidance:
            actions = self._apply_collision_avoidance(obs, actions)
        self._env.apply_actions(actions)
        return obs

    # ------------------------------------------------------------------
    # Background thread loop
    # ------------------------------------------------------------------
    def _run_loop(self) -> None:
        """Main control loop (runs in background thread)."""
        period = 1.0 / self._config.control_hz
        while self._running:
            start = time.perf_counter()

            self._drain_commands()
            if not self._running:
                break

            # Get observations
            obs = self._env.observe()

            # Update car states from observations
            self._update_cars(obs)

            # Collect actions from all cars
            actions = self._collect_actions(obs)

            # Apply collision avoidance
            if self._config.collision_avoidance:
                actions = self._apply_collision_avoidance(obs, actions)

            # Send actions to environment
            self._env.apply_actions(actions)

            # Sleep to maintain control rate
            elapsed = time.perf_counter() - start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _drain_commands(self) -> None:
        """Process all pending commands."""
        while True:
            try:
                cmd = self._commands.get_nowait()
            except Exception:
                break

            if cmd.name == "shutdown":
                self._running = False
                return

            elif cmd.name == "set_speed_scale":
                self.set_speed_scale(float(cmd.payload))

            elif cmd.name == "clear_paths":
                self.clear_paths()

            elif cmd.name == "add_point":
                car_id, point = cmd.payload
                self.add_point_to_car(car_id, point)

            elif cmd.name == "set_paths":
                paths = cmd.payload
                self.set_paths(paths)

            elif cmd.name == "stop":
                self.stop_all()

    def _update_cars(self, obs: Dict[int, RobotObservation]) -> None:
        """Update car states from observations."""
        with self._lock:
            for car in self._cars:
                robot_obs = obs.get(car.tag_id)
                if robot_obs is not None:
                    car._update_from_observation(robot_obs)

    def _collect_actions(self, obs: Dict[int, RobotObservation]) -> Dict[int, Action]:
        """Collect actions from all cars."""
        actions: Dict[int, Action] = {}
        with self._lock:
            for car in self._cars:
                robot_obs = obs.get(car.tag_id)
                if robot_obs is not None:
                    action = car.get_action(robot_obs)
                    actions[car.tag_id] = action
        return actions

    # ------------------------------------------------------------------
    # Collision Avoidance
    # ------------------------------------------------------------------
    def _apply_collision_avoidance(
        self,
        obs: Dict[int, RobotObservation],
        actions: Dict[int, Action],
    ) -> Dict[int, Action]:
        """
        Apply collision avoidance to actions.

        Uses short-horizon trajectory prediction to detect potential collisions.
        Lower tag_id cars have priority - higher tag_id cars will slow down or
        stop completely to wait for the priority car to pass.

        Special case: If the priority car is stopped (not moving), the yielding
        car will navigate around it instead of waiting indefinitely.
        """
        cfg = self._config
        n = len(self._cars)
        if n <= 1:
            return actions

        now = time.monotonic()

        # Build position list and check which cars are stopped
        positions: List[Tuple[float, float, float]] = []
        speeds: List[Tuple[float, float]] = []
        ids: List[int] = []
        car_is_stopped: List[bool] = []

        for car in self._cars:
            positions.append(car.pose)
            action = actions.get(car.tag_id, Action.stop())
            speeds.append(action.as_tuple())
            ids.append(car.tag_id)

            # Check if car is effectively stopped (very low speed or IDLE/DONE state)
            speed_magnitude = abs(action.left_speed) + abs(action.right_speed)
            is_stopped = (
                speed_magnitude < 0.05 or
                car.task_state == TaskState.IDLE or
                car.task_state == TaskState.DONE
            )
            car_is_stopped.append(is_stopped)

        # Smooth positions for stability
        smoothed_positions = self._smooth_positions(positions, ids)

        # Also compute current distances for immediate collision detection
        current_dist: Dict[Tuple[int, int], float] = {}
        for i in range(n):
            for j in range(i + 1, n):
                dx = smoothed_positions[i][0] - smoothed_positions[j][0]
                dy = smoothed_positions[i][1] - smoothed_positions[j][1]
                current_dist[(i, j)] = math.sqrt(dx * dx + dy * dy)

        # Predict trajectories
        traj = self._predict_trajectories(smoothed_positions, speeds)

        # Apply collision avoidance
        out_actions = dict(actions)

        for i in range(n):
            for j in range(i + 1, n):
                # Find minimum distance in predicted trajectory
                min_d = float("inf")
                for k in range(len(traj[i])):
                    dx = traj[i][k][0] - traj[j][k][0]
                    dy = traj[i][k][1] - traj[j][k][1]
                    d = math.sqrt(dx * dx + dy * dy)
                    if d < min_d:
                        min_d = d

                # Also consider current distance
                curr_d = current_dist[(i, j)]
                min_d = min(min_d, curr_d)

                if min_d >= cfg.collision_d_safe:
                    continue

                # Determine who yields (higher tag_id yields - lower ID has priority)
                id_i, id_j = ids[i], ids[j]
                yield_i = id_i > id_j
                yield_j = id_j > id_i

                # Check if the priority car is stopped
                # If priority car is stopped, yielding car should navigate around, not wait
                priority_car_stopped = False
                if yield_i and car_is_stopped[j]:  # j has priority, j is stopped
                    priority_car_stopped = True
                elif yield_j and car_is_stopped[i]:  # i has priority, i is stopped
                    priority_car_stopped = True

                # Check yield hold time (once yielding, keep yielding for a while)
                # But don't apply yield hold if priority car is stopped
                if not priority_car_stopped:
                    if now < self._yield_until.get(id_i, 0.0):
                        yield_i = True
                    if now < self._yield_until.get(id_j, 0.0):
                        yield_j = True

                # Compute avoidance strength based on distance
                if min_d <= cfg.collision_d_hard:
                    alpha = 1.0
                else:
                    alpha = (cfg.collision_d_safe - min_d) / max(1e-6, cfg.collision_d_safe - cfg.collision_d_hard)
                    alpha = max(0.0, min(1.0, alpha))

                # Determine if yielding car should stop completely
                # Don't stop if priority car is already stopped - navigate around instead
                should_stop = min_d <= cfg.collision_d_stop and not priority_car_stopped

                # Apply yield hold (only if priority car is moving)
                if not priority_car_stopped:
                    if yield_i:
                        self._yield_until[id_i] = max(self._yield_until.get(id_i, 0.0), now + cfg.collision_yield_hold)
                    if yield_j:
                        self._yield_until[id_j] = max(self._yield_until.get(id_j, 0.0), now + cfg.collision_yield_hold)

                # Apply avoidance
                if yield_i:
                    out_actions[id_i] = self._apply_yield(
                        out_actions[id_i], alpha, should_stop,
                        navigate_around=priority_car_stopped,
                        other_pos=smoothed_positions[j],
                        my_pos=smoothed_positions[i],
                    )
                if yield_j:
                    out_actions[id_j] = self._apply_yield(
                        out_actions[id_j], alpha, should_stop,
                        navigate_around=priority_car_stopped,
                        other_pos=smoothed_positions[i],
                        my_pos=smoothed_positions[j],
                    )

        return out_actions

    def _smooth_positions(
        self,
        positions: List[Tuple[float, float, float]],
        ids: List[int],
    ) -> List[Tuple[float, float, float]]:
        """Apply EMA smoothing to positions for stability."""
        alpha = 0.65
        smoothed: List[Tuple[float, float, float]] = []

        for i, (x, y, th) in enumerate(positions):
            tid = ids[i]
            prev = self._last_positions.get(tid)
            if prev is None:
                smoothed.append((x, y, th))
            else:
                xs = alpha * x + (1 - alpha) * prev[0]
                ys = alpha * y + (1 - alpha) * prev[1]
                smoothed.append((xs, ys, th))
            self._last_positions[tid] = (smoothed[-1][0], smoothed[-1][1], th)

        return smoothed

    def _predict_trajectories(
        self,
        positions: List[Tuple[float, float, float]],
        speeds: List[Tuple[float, float]],
    ) -> List[List[Tuple[float, float]]]:
        """Predict future trajectories for collision detection."""
        cfg = self._config
        steps = max(1, int(cfg.collision_prediction_t / cfg.collision_prediction_dt))

        # Get wheel base from first car's config
        wb = self._cars[0].config.wheel_base if self._cars else 30.0

        traj: List[List[Tuple[float, float]]] = []
        for i, (x, y, th) in enumerate(positions):
            vl, vr = speeds[i]
            pts = [(x, y)]
            for _ in range(steps):
                x, y, th = simulate_step(x, y, th, vl, vr, wb, cfg.collision_prediction_dt)
                pts.append((x, y))
            traj.append(pts)

        return traj

    def _apply_yield(
        self,
        action: Action,
        alpha: float,
        should_stop: bool = False,
        navigate_around: bool = False,
        other_pos: Optional[Tuple[float, float, float]] = None,
        my_pos: Optional[Tuple[float, float, float]] = None,
    ) -> Action:
        """
        Apply yielding behavior to an action.

        Args:
            action: Original action
            alpha: Avoidance strength (0.0 = no avoidance, 1.0 = maximum)
            should_stop: If True, stop completely instead of just slowing down
            navigate_around: If True, try to navigate around stopped obstacle
            other_pos: Position of the other car (x, y, theta)
            my_pos: Position of this car (x, y, theta)
        """
        if should_stop:
            # Stop completely - let priority car pass
            return Action.stop()

        vl, vr = action.left_speed, action.right_speed

        if navigate_around and other_pos is not None and my_pos is not None:
            # Navigate around a stopped car
            # Calculate which side to go around (perpendicular to line between cars)
            dx = other_pos[0] - my_pos[0]
            dy = other_pos[1] - my_pos[1]
            dist = math.sqrt(dx * dx + dy * dy)

            if dist > 1e-6:
                # Direction from me to other car
                dir_to_other = math.atan2(dy, dx)
                # My heading
                my_heading = my_pos[2]
                # Relative angle
                rel_angle = dir_to_other - my_heading

                # Normalize to [-pi, pi]
                while rel_angle > math.pi:
                    rel_angle -= 2 * math.pi
                while rel_angle < -math.pi:
                    rel_angle += 2 * math.pi

                # Slow down proportionally to distance
                speed_factor = max(0.3, min(1.0, dist / self._config.collision_d_safe))
                base_speed = 0.4 * speed_factor

                # Turn away from the obstacle
                # If obstacle is on the right (rel_angle > 0), turn left
                # If obstacle is on the left (rel_angle < 0), turn right
                turn_strength = 0.3 * alpha

                if abs(rel_angle) < math.pi / 2:
                    # Obstacle is in front - need to turn away
                    if rel_angle > 0:  # Obstacle on right, turn left
                        vl2 = base_speed - turn_strength
                        vr2 = base_speed + turn_strength
                    else:  # Obstacle on left, turn right
                        vl2 = base_speed + turn_strength
                        vr2 = base_speed - turn_strength
                else:
                    # Obstacle is behind - just proceed with slight avoidance
                    vl2 = vl * 0.7
                    vr2 = vr * 0.7

                return Action(left_speed=vl2, right_speed=vr2)

        # Normal yielding behavior - slow down
        slow = 1.0 - self._config.collision_max_slow * alpha
        vl2 = vl * slow
        vr2 = vr * slow

        # Add slight evasive turn when getting closer (alpha > 0.7)
        if alpha > 0.7:
            pivot = 0.15 * alpha
            if (vl + vr) >= 0:  # Moving forward - turn right
                vl2 -= pivot
                vr2 += pivot
            else:  # Moving backward - turn right in reverse
                vl2 += pivot
                vr2 -= pivot

        return Action(left_speed=vl2, right_speed=vr2)
