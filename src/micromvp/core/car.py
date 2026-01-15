"""
Car module - Independent robot agent with integrated controller.

The Car class is the core entity in the system. Each car is an independent agent
that maintains its own state, controller logic, and high-level behavior API.

Key principles:
- Car does NOT directly interact with Environment
- Car does NOT execute actions - it only produces them
- Car state is updated solely via RobotObservation
- High-level APIs (goto, follow_path, stop) modify controller state, not actions
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from micromvp.core.ddr import calculate_speeds, normalize_angle
from micromvp.core.models import (
    Action,
    CarConfig,
    CarState,
    Point,
    Pose,
    RobotObservation,
    TargetPose,
    TaskState,
)
from micromvp.utils.geometry import (
    angle_diff,
    compute_path_heading,
    generate_bezier_path,
)


@dataclass
class PathCache:
    """Cached path geometry for efficient path following."""
    pts: List[Point]
    cum_s: List[float]  # Cumulative arc lengths
    total_s: float


class Car:
    """
    Independent robot agent with integrated controller.

    A Car is a self-contained entity that:
    - Owns its static configuration (CarConfig)
    - Maintains runtime state (pose, task state)
    - Contains controller logic for path following
    - Provides high-level API (goto, follow_path, stop)
    - Produces actions via get_action(observation)

    Usage:
        config = CarConfig.from_wheel_base(robot_id=1, tag_id=1, wheel_base=30.0)
        car = Car(config)
        car.follow_path([(100, 100), (200, 200), (300, 100)])

        # In control loop:
        obs = env.observe()[car.robot_id]
        action = car.get_action(obs)
        env.apply_actions({car.robot_id: action})
    """

    def __init__(self, config: CarConfig) -> None:
        """
        Initialize car with static configuration.

        Args:
            config: Static car configuration (robot_id, wheel_base, etc.)
        """
        self._config = config

        # Runtime state (updated from observations)
        self._x: float = 0.0
        self._y: float = 0.0
        self._theta: float = 0.0
        self._last_obs_time: float = 0.0

        # Last commanded speeds (for velocity estimation)
        self._last_l_speed: float = 0.0
        self._last_r_speed: float = 0.0

        # Task state
        self._task_state: TaskState = TaskState.IDLE

        # Path following state
        self._path: List[Point] = []
        self._path_cache: Optional[PathCache] = None
        self._proj_hint_seg: int = 1
        self._loop_path: bool = True

        # Goto state
        self._target_pose: Optional[TargetPose] = None

        # Smooth goto state
        self._smooth_goto: bool = False
        self._rotate_target_theta: Optional[float] = None  # Target theta for ROTATING state
        self._smooth_goto_path: List[Point] = []  # Generated Bezier path
        self._smooth_goto_needs_init: bool = False  # Deferred init flag
        self._smooth_rotate_threshold: float = math.pi / 2

    # ------------------------------------------------------------------
    # Properties (read-only access to state)
    # ------------------------------------------------------------------
    @property
    def robot_id(self) -> int:
        return self._config.robot_id

    @property
    def tag_id(self) -> int:
        return self._config.tag_id

    @property
    def config(self) -> CarConfig:
        return self._config

    @property
    def x(self) -> float:
        return self._x

    @property
    def y(self) -> float:
        return self._y

    @property
    def theta(self) -> float:
        return self._theta

    @property
    def pose(self) -> Pose:
        return (self._x, self._y, self._theta)

    @property
    def position(self) -> Point:
        return (self._x, self._y)

    @property
    def task_state(self) -> TaskState:
        return self._task_state

    @property
    def path(self) -> List[Point]:
        return list(self._path)

    @property
    def current_target(self) -> Optional[Point]:
        """Get current tracking target point (for UI visualization)."""
        if self._task_state == TaskState.IDLE:
            return None
        if self._task_state == TaskState.GOTO and self._target_pose:
            return self._target_pose.position
        if self._task_state == TaskState.FOLLOW and self._path:
            return self._compute_lookahead_target()
        return None

    @property
    def is_task_done(self) -> bool:
        return self._task_state == TaskState.DONE

    @property
    def is_idle(self) -> bool:
        return self._task_state == TaskState.IDLE

    # ------------------------------------------------------------------
    # High-level API (modifies controller state, not actions)
    # ------------------------------------------------------------------
    def goto(
        self,
        x: float,
        y: float,
        theta: Optional[float] = None,
        tolerance_pos: float = 10.0,
        tolerance_theta: float = 0.1,
        smooth: bool = False,
        rotate_threshold: float = math.pi / 2,
    ) -> None:
        """
        Command car to move to target pose.

        This modifies the controller's target state. The actual movement
        happens when get_action() is called with observations.

        Args:
            x: Target x coordinate
            y: Target y coordinate
            theta: Target orientation (None = don't care about orientation)
            tolerance_pos: Position tolerance in pixels
            tolerance_theta: Orientation tolerance in radians
            smooth: If True and theta is provided, generate a smooth Bezier path
            rotate_threshold: If heading error exceeds this, rotate in place first
        """
        self._target_pose = TargetPose(
            x=x,
            y=y,
            theta=theta,
            tolerance_pos=tolerance_pos,
            tolerance_theta=tolerance_theta,
        )
        self._smooth_goto = smooth
        self._smooth_goto_path = []
        self._rotate_target_theta = None
        self._smooth_goto_needs_init = False  # Will be set to True for deferred init
        self._smooth_rotate_threshold = rotate_threshold

        if smooth and theta is not None:
            # Defer initialization until first observation is received
            # This ensures we have accurate current pose for path generation
            self._smooth_goto_needs_init = True
            self._task_state = TaskState.GOTO  # Temporary, will be updated on first get_action
        else:
            self._task_state = TaskState.GOTO

        self._path = []
        self._path_cache = None

    def follow_path(self, points: List[Point], loop: bool = True) -> None:
        """
        Command car to follow a path.

        This modifies the controller's path state. The actual path following
        happens when get_action() is called with observations.

        Args:
            points: List of (x, y) waypoints
            loop: Whether to loop back to start when reaching end
        """
        self._path = list(points)
        self._loop_path = loop
        self._path_cache = None
        self._proj_hint_seg = 1
        self._target_pose = None
        self._task_state = TaskState.FOLLOW if points else TaskState.IDLE

    def stop(self) -> None:
        """Stop the car and clear all tasks."""
        self._task_state = TaskState.IDLE
        self._path = []
        self._path_cache = None
        self._target_pose = None
        self._smooth_goto = False
        self._smooth_goto_path = []
        self._rotate_target_theta = None
        self._smooth_goto_needs_init = False

    def prepare_for_path(self, path: List[Point], tolerance_pos: float = 10.0) -> None:
        """
        Prepare to follow a path by going to start with correct heading.

        This generates a smooth Bezier curve from the current pose to the
        path's start point, with the ending heading aligned to the path direction.

        After this completes (is_task_done), call follow_path(path) to continue.

        Args:
            path: The path to prepare for (will go to path[0] with correct heading)
            tolerance_pos: Position tolerance for reaching the start
        """
        if not path or len(path) < 2:
            return

        target_theta = compute_path_heading(path)
        self.goto(
            x=path[0][0],
            y=path[0][1],
            theta=target_theta,
            tolerance_pos=tolerance_pos,
            smooth=True,
        )

    def add_path_point(self, point: Point) -> None:
        """Add a point to the current path."""
        self._path.append(point)
        self._path_cache = None
        if self._task_state == TaskState.IDLE:
            self._task_state = TaskState.FOLLOW

    def clear_path(self) -> None:
        """Clear the current path but keep task state."""
        self._path = []
        self._path_cache = None

    def set_path(self, points: List[Point], loop: bool = True) -> None:
        """Alias for follow_path for compatibility."""
        self.follow_path(points, loop)

    # ------------------------------------------------------------------
    # Core method: get_action(observation) -> Action
    # ------------------------------------------------------------------
    def get_action(self, obs: RobotObservation) -> Action:
        """
        Compute action based on observation and current task.

        This is the core method that produces wheel speed commands.
        It should be called every control cycle with fresh observations.

        Args:
            obs: Robot observation from environment

        Returns:
            Action with left/right wheel speeds
        """
        # Update state from observation
        self._update_from_observation(obs)

        # Handle deferred smooth goto initialization
        if self._smooth_goto_needs_init and obs.valid:
            self._init_smooth_goto()

        # Compute action based on task state
        if self._task_state == TaskState.IDLE:
            return Action.stop()

        if self._task_state == TaskState.DONE:
            return Action.stop()

        if self._task_state == TaskState.ROTATING:
            return self._compute_rotating_action()

        if self._task_state == TaskState.GOTO:
            return self._compute_goto_action()

        if self._task_state == TaskState.FOLLOW:
            return self._compute_follow_action()

        return Action.stop()

    # ------------------------------------------------------------------
    # Internal state update
    # ------------------------------------------------------------------
    def _update_from_observation(self, obs: RobotObservation) -> None:
        """Update internal state from observation."""
        if not obs.valid:
            return
        self._x = obs.x
        self._y = obs.y
        self._theta = obs.theta
        self._last_obs_time = obs.timestamp

    def _init_smooth_goto(self) -> None:
        """
        Initialize smooth goto after first valid observation.

        This is called once per smooth goto command, after we have
        accurate current pose from observations.
        """
        self._smooth_goto_needs_init = False

        if self._target_pose is None or self._target_pose.theta is None:
            return

        target = self._target_pose
        dist = math.hypot(target.x - self._x, target.y - self._y)

        # Minimum distance needed for a good Bezier curve approach
        min_approach_dist = self._config.wheel_base * 3.0

        # Check if we need to rotate first
        target_direction = math.atan2(target.y - self._y, target.x - self._x)
        heading_error = abs(angle_diff(target_direction, self._theta))

        if dist < min_approach_dist:
            # Too close for a good Bezier curve
            # Check heading alignment with target orientation
            final_heading_error = abs(angle_diff(target.theta, self._theta))

            if final_heading_error < 0.3:  # ~17 degrees - already well aligned
                # Close and aligned - just go directly to target
                self._task_state = TaskState.GOTO
                self._smooth_goto = False
            elif dist < self._config.wheel_base * 1.5:
                # Very close - rotate in place to target orientation, then go to target
                self._rotate_target_theta = target.theta
                self._task_state = TaskState.ROTATING
                # After rotation, will transition to GOTO to reach exact position
                self._smooth_goto = False
            else:
                # Moderately close - back up first to create space
                self._setup_backup_approach()
        elif heading_error > self._smooth_rotate_threshold:
            # Need to rotate first towards target direction
            self._rotate_target_theta = target_direction
            self._task_state = TaskState.ROTATING
        else:
            # Generate Bezier path directly
            self._generate_smooth_path()
            self._task_state = TaskState.FOLLOW

    def _setup_backup_approach(self) -> None:
        """
        Setup a backup-then-approach maneuver for when car is too close to target.

        Creates a path that:
        1. Backs up away from target
        2. Then curves smoothly to approach with correct heading
        """
        if self._target_pose is None or self._target_pose.theta is None:
            return

        target = self._target_pose

        # Calculate backup point: move backwards from current position
        backup_dist = self._config.wheel_base * 3.0
        backup_x = self._x - backup_dist * math.cos(self._theta)
        backup_y = self._y - backup_dist * math.sin(self._theta)

        # Now create a path: backup point -> target with correct approach
        # The backup point should have heading towards target
        approach_heading = math.atan2(target.y - backup_y, target.x - backup_x)

        # Generate Bezier from backup point to target
        backup_pose = (backup_x, backup_y, approach_heading)
        end_pose = (target.x, target.y, target.theta)

        bezier_path = generate_bezier_path(
            start_pose=backup_pose,
            end_pose=end_pose,
            num_points=25,
            start_control_ratio=0.4,
            end_control_ratio=0.6,
        )

        # Create full path: current position -> backup point -> bezier path
        # First, add points to back up
        backup_steps = 10
        full_path = []
        for i in range(backup_steps):
            t = (i + 1) / backup_steps
            px = self._x + t * (backup_x - self._x)
            py = self._y + t * (backup_y - self._y)
            full_path.append((px, py))

        # Add the bezier approach path
        full_path.extend(bezier_path)

        self._smooth_goto_path = full_path
        self._path = full_path
        self._path_cache = None
        self._proj_hint_seg = 1
        self._loop_path = False
        self._task_state = TaskState.FOLLOW

    # ------------------------------------------------------------------
    # Goto control
    # ------------------------------------------------------------------
    def _compute_goto_action(self) -> Action:
        """Compute action to reach target pose."""
        if self._target_pose is None:
            self._task_state = TaskState.IDLE
            return Action.stop()

        target = self._target_pose
        dx = target.x - self._x
        dy = target.y - self._y
        dist = math.hypot(dx, dy)

        # Check if position reached
        if dist <= target.tolerance_pos:
            # Check if orientation matters and is reached
            if target.theta is None:
                self._task_state = TaskState.DONE
                return Action.stop()

            angle_err = self._angle_diff(target.theta, self._theta)
            if abs(angle_err) <= target.tolerance_theta:
                self._task_state = TaskState.DONE
                return Action.stop()

            # Rotate in place to target orientation
            return self._compute_rotation_action(angle_err)

        # Move towards target position
        vl, vr = calculate_speeds(
            self._x, self._y, self._theta,
            [(target.x, target.y)],
            self._config.v_max,
            self._config.wheel_base,
        )

        self._last_l_speed = vl
        self._last_r_speed = vr
        return Action(left_speed=vl, right_speed=vr)

    def _compute_rotation_action(self, angle_error: float) -> Action:
        """Compute action to rotate in place by angle_error."""
        v = self._config.v_max * 0.3  # Slower for rotation
        if angle_error > 0:
            return Action(left_speed=-v, right_speed=v)
        else:
            return Action(left_speed=v, right_speed=-v)

    @staticmethod
    def _angle_diff(target: float, current: float) -> float:
        """Compute shortest angle difference (target - current)."""
        diff = target - current
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    # ------------------------------------------------------------------
    # Smooth goto control (ROTATING state)
    # ------------------------------------------------------------------
    def _compute_rotating_action(self) -> Action:
        """
        Compute action for ROTATING state (in-place rotation before smooth goto).

        When rotation is complete:
        - If smooth_goto is True: transitions to FOLLOW state with Bezier path
        - If smooth_goto is False: transitions to GOTO state to reach exact position
        """
        if self._rotate_target_theta is None:
            # Should not happen, but handle gracefully
            if self._smooth_goto:
                self._generate_smooth_path()
                self._task_state = TaskState.FOLLOW
            else:
                # Go to GOTO to reach exact target position
                self._task_state = TaskState.GOTO
            return Action.stop()

        angle_error = self._angle_diff(self._rotate_target_theta, self._theta)
        tolerance = 0.15  # ~8.5 degrees tolerance for rotation completion

        if abs(angle_error) <= tolerance:
            # Rotation complete
            self._rotate_target_theta = None

            if self._smooth_goto:
                # Generate smooth path and switch to FOLLOW
                self._generate_smooth_path()
                self._task_state = TaskState.FOLLOW
            else:
                # No smooth curve needed - switch to GOTO to reach exact position
                self._task_state = TaskState.GOTO
            return Action.stop()

        # Continue rotating
        v = self._config.v_max * 0.35
        if angle_error > 0:
            return Action(left_speed=-v, right_speed=v)
        else:
            return Action(left_speed=v, right_speed=-v)

    def _generate_smooth_path(self) -> None:
        """
        Generate a smooth Bezier path from current pose to target pose.

        Updates self._path with the generated waypoints.
        """
        if self._target_pose is None or self._target_pose.theta is None:
            return

        target = self._target_pose
        start_pose = (self._x, self._y, self._theta)
        end_pose = (target.x, target.y, target.theta)

        # Generate Bezier path with asymmetric control ratios
        # Larger end_control_ratio creates lower curvature at the endpoint
        bezier_path = generate_bezier_path(
            start_pose=start_pose,
            end_pose=end_pose,
            num_points=25,
            start_control_ratio=0.35,
            end_control_ratio=0.55,  # Lower curvature at end
        )

        self._smooth_goto_path = bezier_path
        self._path = bezier_path
        self._path_cache = None
        self._proj_hint_seg = 1
        self._loop_path = False  # Smooth goto path should not loop

    # ------------------------------------------------------------------
    # Path following control
    # ------------------------------------------------------------------
    def _compute_follow_action(self) -> Action:
        """Compute action to follow current path."""
        if not self._path:
            self._task_state = TaskState.IDLE
            return Action.stop()

        pts = self._path

        # Single point: go to it
        if len(pts) == 1:
            tx, ty = pts[0]
            vl, vr = calculate_speeds(
                self._x, self._y, self._theta,
                [(tx, ty)],
                self._config.v_max,
                self._config.wheel_base,
            )
            self._last_l_speed = vl
            self._last_r_speed = vr
            return Action(left_speed=vl, right_speed=vr)

        # Build or retrieve path cache
        cache = self._get_or_build_cache(pts)

        # Project current position onto path
        s_star, best_seg = self._project_to_path(self._x, self._y, cache)
        self._proj_hint_seg = best_seg

        total_s = cache.total_s
        remaining = max(0.0, total_s - s_star)

        # Compute lookahead distance
        v_est = 0.5 * (self._last_l_speed + self._last_r_speed)
        L = self._config.lookahead_base + self._config.lookahead_k_v * abs(v_est)
        L = max(self._config.lookahead_min, min(self._config.lookahead_max, L))

        # For non-looping paths, reduce lookahead near end
        if not self._loop_path and remaining < 2.0 * L:
            L = max(self._config.lookahead_min, 0.5 * remaining)
            # Check if we've reached the end
            if remaining < self._config.lookahead_min:
                self._task_state = TaskState.DONE
                return Action.stop()

        # Compute target arc length
        if self._loop_path and total_s > 1e-9:
            s_tgt = (s_star + L) % total_s
        else:
            s_tgt = min(total_s, s_star + L)

        # Get target point
        tx, ty = self._point_at_s(cache, s_tgt)

        # Compute wheel speeds
        vl, vr = calculate_speeds(
            self._x, self._y, self._theta,
            [(tx, ty)],
            self._config.v_max,
            self._config.wheel_base,
        )

        # Apply soft turning near target
        d = math.hypot(tx - self._x, ty - self._y)
        if self._config.omega_soft_dist > 1e-6 and d < self._config.omega_soft_dist:
            scale = max(self._config.omega_min_scale, d / self._config.omega_soft_dist)
            v = 0.5 * (vl + vr)
            w = (vr - vl) / self._config.wheel_base
            w *= scale
            vl = v - 0.5 * self._config.wheel_base * w
            vr = v + 0.5 * self._config.wheel_base * w

        self._last_l_speed = vl
        self._last_r_speed = vr
        return Action(left_speed=vl, right_speed=vr)

    def _compute_lookahead_target(self) -> Optional[Point]:
        """Compute current lookahead target point for visualization."""
        if not self._path or len(self._path) < 2:
            return self._path[0] if self._path else None

        cache = self._get_or_build_cache(self._path)
        s_star, _ = self._project_to_path(self._x, self._y, cache)

        v_est = 0.5 * (self._last_l_speed + self._last_r_speed)
        L = self._config.lookahead_base + self._config.lookahead_k_v * abs(v_est)
        L = max(self._config.lookahead_min, min(self._config.lookahead_max, L))

        total_s = cache.total_s
        if self._loop_path and total_s > 1e-9:
            s_tgt = (s_star + L) % total_s
        else:
            s_tgt = min(total_s, s_star + L)

        return self._point_at_s(cache, s_tgt)

    # ------------------------------------------------------------------
    # Path geometry helpers
    # ------------------------------------------------------------------
    def _get_or_build_cache(self, pts: List[Point]) -> PathCache:
        """Get or build path cache for geometry calculations."""
        if self._path_cache is not None and self._path_cache.pts == pts:
            return self._path_cache

        cum_s = [0.0] * len(pts)
        total = 0.0
        for i in range(1, len(pts)):
            dx = pts[i][0] - pts[i - 1][0]
            dy = pts[i][1] - pts[i - 1][1]
            seg = math.hypot(dx, dy)
            total += seg
            cum_s[i] = total

        self._path_cache = PathCache(pts=pts, cum_s=cum_s, total_s=total)
        return self._path_cache

    def _project_to_path(
        self,
        x: float,
        y: float,
        cache: PathCache,
    ) -> Tuple[float, int]:
        """Project point onto path, returning arc length and segment index."""
        pts = cache.pts
        cum_s = cache.cum_s
        n = len(pts)

        if n <= 1:
            return (0.0, 0)

        # Use hint for windowed search
        i0 = max(1, min(n - 1, self._proj_hint_seg))
        win = 30
        lo = max(1, i0 - win)
        hi = min(n - 1, i0 + win)

        best_d2 = float("inf")
        best_s = 0.0
        best_i = i0

        for i in range(lo, hi + 1):
            x0, y0 = pts[i - 1]
            x1, y1 = pts[i]
            vx, vy = x1 - x0, y1 - y0
            wx, wy = x - x0, y - y0
            seg_len2 = vx * vx + vy * vy

            if seg_len2 <= 1e-12:
                t = 0.0
                px, py = x0, y0
                seg_len = 0.0
            else:
                t = (wx * vx + wy * vy) / seg_len2
                t = max(0.0, min(1.0, t))
                px = x0 + t * vx
                py = y0 + t * vy
                seg_len = math.sqrt(seg_len2)

            dx = x - px
            dy = y - py
            d2 = dx * dx + dy * dy

            if d2 < best_d2:
                best_d2 = d2
                best_i = i
                best_s = cum_s[i - 1] + t * seg_len

        return (best_s, best_i)

    @staticmethod
    def _point_at_s(cache: PathCache, s: float) -> Point:
        """Get point at arc length s along path."""
        pts = cache.pts
        cum_s = cache.cum_s
        n = len(pts)

        if n == 0:
            return (0.0, 0.0)
        if n == 1:
            return pts[0]

        total = cache.total_s
        if total <= 1e-9:
            return pts[0]

        s = float(s)
        if s <= 0.0:
            return pts[0]
        if s >= total:
            return pts[-1]

        for i in range(1, n):
            if cum_s[i] >= s:
                s0 = cum_s[i - 1]
                s1 = cum_s[i]
                seg_len = s1 - s0
                if seg_len <= 1e-9:
                    return pts[i]
                t = (s - s0) / seg_len
                x0, y0 = pts[i - 1]
                x1, y1 = pts[i]
                return (x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)

        return pts[-1]

    # ------------------------------------------------------------------
    # Snapshot for UI
    # ------------------------------------------------------------------
    def to_car_state(self) -> CarState:
        """Create CarState snapshot for UI rendering."""
        return CarState(
            car_id=self._config.robot_id,
            tag_id=self._config.tag_id,
            x=self._x,
            y=self._y,
            theta=self._theta,
            l_speed=self._last_l_speed,
            r_speed=self._last_r_speed,
            path=list(self._path),
            task_state=self._task_state,
        )

    # ------------------------------------------------------------------
    # Direct pose setting (for simulation/testing)
    # ------------------------------------------------------------------
    def set_pose(self, x: float, y: float, theta: float) -> None:
        """Directly set pose (for initialization/testing only)."""
        self._x = x
        self._y = y
        self._theta = theta
