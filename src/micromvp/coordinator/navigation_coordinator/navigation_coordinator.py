"""
Navigation Coordinator - Single robot navigation with obstacle avoidance.

This coordinator provides:
1. Web server API for external control:
   - setup_obstacle(obstacles): Set obstacle geometry
   - goto(x, y, theta): Navigate to position with final orientation
   - follow_path(path): Follow a sequence of waypoints

2. GUI integration:
   - Speed slider to adjust robot movement speed
   - Rotation input to manually rotate robot to target angle
   - Hand-drawn path following from canvas
   - Path visualization until completion

3. Single robot control with selectable ID

Priority: GUI commands override external web server commands.
"""
from __future__ import annotations

import json
import math
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from typing import Dict, List, Any, Optional, Tuple
from functools import partial

from micromvp.controller.base import Controller
from micromvp.controller.navigation_controller import NavigationController, NavigationState
from micromvp.coordinator.base import Coordinator
from micromvp.core.models import (
    Action,
    CarState,
    RobotObservation,
    WorkspaceConfig,
)


Point = Tuple[float, float]


class NavigationCoordinator(Coordinator):
    """
    Coordinator for single robot navigation with obstacle avoidance.

    Features:
    - Web server with setup_obstacle, goto, follow_path APIs
    - GUI speed slider and rotation input
    - Hand-drawn path following
    - RVG-based path planning for obstacle avoidance
    - GUI commands take priority over web server commands

    Usage:
        coordinator = NavigationCoordinator(
            ws_config,
            controllers={robot_id: NavigationController(...)},
            active_robot_id=robot_id,
            webserver_port=8080
        )

        # Register GUI callbacks
        gui.register_callback("set_robot_speed", coordinator.on_speed_change)
        gui.register_callback("set_rotation_target", coordinator.on_rotation_input)
        gui.register_callback("on_curve_drawn", coordinator.on_curve_drawn)
    """

    def __init__(
        self,
        ws_config: WorkspaceConfig,
        controllers: Dict[int, Controller],
        active_robot_id: Optional[int] = None,
        webserver_port: int = 8080,
        robot_geometry: Optional[List[Tuple[float, float]]] = None,
    ) -> None:
        """
        Initialize navigation coordinator.

        Args:
            ws_config: Workspace configuration from environment
            controllers: Dict mapping robot_id to Controller instance
                         Should be NavigationController for full functionality
            active_robot_id: The robot ID to control (default: first in list)
            webserver_port: Port for the web server API (default: 8080)
            robot_geometry: Robot shape as list of (x,y) vertices for RVG
                           Default is a rectangle based on car dimensions
        """
        super().__init__(ws_config, controllers)

        # Active robot selection
        if active_robot_id is not None:
            self._active_robot_id = active_robot_id
        elif ws_config.car_id_list:
            self._active_robot_id = ws_config.car_id_list[0]
        else:
            self._active_robot_id = None

        # Obstacle storage for path planning
        self._obstacles: List[List[Tuple[float, float]]] = []
        self._obstacles_lock = threading.Lock()

        # External command tracking (for priority management)
        self._external_command_active = False
        self._external_path: Optional[List[Point]] = None

        # Robot geometry for RVG solver
        self._robot_geometry = robot_geometry or self._default_robot_geometry()

        # Web server
        self._webserver_port = webserver_port
        self._webserver: Optional[HTTPServer] = None
        self._webserver_thread: Optional[threading.Thread] = None

        # Current path being displayed (for visualization)
        self._display_path: Optional[List[Point]] = None
        self._display_path_lock = threading.Lock()

        # Pending path for pre-rotation before path following
        # When a path is received, we first rotate toward the start point,
        # then start following the path after rotation completes
        self._pending_path: Optional[List[Point]] = None
        self._pending_goto_theta: Optional[float] = None  # Final theta for goto command
        self._pending_path_lock = threading.Lock()

        # Minimum distance to trigger pre-rotation (skip if already very close to path start)
        self._pre_rotation_min_distance = max(ws_config.car_width, ws_config.car_height) * 0.5

        # Start web server
        self._start_webserver()

    def _default_robot_geometry(self) -> List[Tuple[float, float]]:
        """Create default robot geometry from workspace config."""
        w = self._ws_config.car_width / 2
        h = self._ws_config.car_height / 2
        return [(-w, -h), (w, -h), (w, h), (-w, h)]

    # -------------------------------------------------------------------------
    # Web Server Implementation
    # -------------------------------------------------------------------------

    def _start_webserver(self) -> None:
        """Start the HTTP server in a background thread."""
        handler = partial(_NavigationRequestHandler, coordinator=self)
        try:
            self._webserver = HTTPServer(("0.0.0.0", self._webserver_port), handler)
            self._webserver_thread = threading.Thread(
                target=self._webserver.serve_forever,
                daemon=True
            )
            self._webserver_thread.start()
            print(f"[NavigationCoordinator] Web server started on port {self._webserver_port}")
        except OSError as e:
            print(f"[NavigationCoordinator] Failed to start web server: {e}")
            self._webserver = None

    def _stop_webserver(self) -> None:
        """Stop the web server."""
        if self._webserver:
            self._webserver.shutdown()
            self._webserver = None

    # -------------------------------------------------------------------------
    # Web API Handlers
    # -------------------------------------------------------------------------

    def api_setup_obstacle(self, obstacles: List[List[List[float]]]) -> Dict[str, Any]:
        """
        Set obstacles for path planning.

        Args:
            obstacles: List of obstacle polygons. Each polygon is a list of
                      [x, y] points in counter-clockwise order.

        Returns:
            API response dict
        """
        with self._obstacles_lock:
            self._obstacles = [
                [(pt[0], pt[1]) for pt in polygon]
                for polygon in obstacles
            ]
        return {"status": "ok", "obstacle_count": len(self._obstacles)}

    def api_goto(self, x: float, y: float, theta: float) -> Dict[str, Any]:
        """
        Navigate robot to position with final orientation.

        Uses RVG solver to find obstacle-avoiding path. First rotates toward
        the path start, then follows the path, and finally rotates to the
        target orientation.

        Args:
            x: Target x position
            y: Target y position
            theta: Target orientation in degrees [0, 360)

        Returns:
            API response dict
        """
        controller = self._get_active_controller()
        if controller is None:
            return {"status": "error", "message": "No active robot"}

        # Get current position
        current_x = controller.car_state.x
        current_y = controller.car_state.y
        current_theta = controller.car_state.theta

        # Plan path using RVG
        with self._obstacles_lock:
            obstacles_copy = list(self._obstacles)

        path = self._plan_path(
            (current_x, current_y, current_theta),
            (x, y, theta),
            obstacles_copy
        )

        if path is None:
            return {"status": "error", "message": "Path planning failed"}

        # Set external command
        self._external_command_active = True
        self._external_path = path
        self._set_display_path(path)

        # Start path with pre-rotation toward path start
        self._start_path_with_pre_rotation(path, goto_theta=theta)

        return {
            "status": "ok",
            "path_length": len(path),
            "target": {"x": x, "y": y, "theta": theta}
        }

    def api_follow_path(self, path: List[List[float]]) -> Dict[str, Any]:
        """
        Follow a sequence of waypoints.

        First rotates toward the path start, then follows the path.

        Args:
            path: List of [x, y] waypoints

        Returns:
            API response dict
        """
        controller = self._get_active_controller()
        if controller is None:
            return {"status": "error", "message": "No active robot"}

        # Convert to tuples
        path_points = [(pt[0], pt[1]) for pt in path]

        # Set external command
        self._external_command_active = True
        self._external_path = path_points
        self._set_display_path(path_points)

        # Start path with pre-rotation toward path start
        self._start_path_with_pre_rotation(path_points)

        return {"status": "ok", "path_length": len(path_points)}

    # -------------------------------------------------------------------------
    # Path Planning with RVG
    # -------------------------------------------------------------------------

    def _plan_path(
        self,
        start: Tuple[float, float, float],
        goal: Tuple[float, float, float],
        obstacles: List[List[Tuple[float, float]]]
    ) -> Optional[List[Point]]:
        """
        Plan a path using RVG solver.

        Args:
            start: Start pose (x, y, theta)
            goal: Goal pose (x, y, theta)
            obstacles: List of obstacle polygons

        Returns:
            Path as list of (x, y) points, or None if planning fails
        """
        try:
            from rvg import vertex, polygon, rvg
            import numpy as np
        except ImportError:
            print("[NavigationCoordinator] RVG not available, using direct path")
            return [start[:2], goal[:2]]

        try:
            # Create border from workspace
            w = self._ws_config.width
            h = self._ws_config.height
            border_verts = [
                vertex(0, 0),
                vertex(w, 0),
                vertex(w, h),
                vertex(0, h)
            ]
            border = polygon(border_verts, False)

            # Create robot polygon
            robot_verts = [vertex(pt[0], pt[1]) for pt in self._robot_geometry]
            robot_poly = polygon(robot_verts, False)

            # Create obstacle polygons
            obstacle_polys = []
            for obs in obstacles:
                if len(obs) >= 3:
                    obs_verts = [vertex(pt[0], pt[1]) for pt in obs]
                    obstacle_polys.append(polygon(obs_verts))

            # Create RVG solver
            solver = rvg(
                robot=robot_poly,
                border=border,
                obstacles=obstacle_polys,
                resolution=18,
                numThreads=1,
                verbose=False,
                fineApprox=True
            )
            solver.setWeight(euclideanWeight=1.0, rotationalWeight=0.1)

            # Create start and goal vertices
            start_v = vertex(start[0], start[1], 0, 2 * np.pi, 0)
            goal_v = vertex(goal[0], goal[1], 0, 2 * np.pi, 0)

            # Find shortest path
            path_result = solver.shortestPath(start_v, goal_v)

            # Convert result to list of points
            if path_result is None or len(path_result) == 0:
                return [start[:2], goal[:2]]

            path = []
            for v in path_result:
                path.append((v.getX(), v.getY()))

            return path if path else [start[:2], goal[:2]]

        except Exception as e:
            print(f"[NavigationCoordinator] Path planning error: {e}")
            # Fall back to direct path
            return [start[:2], goal[:2]]

    # -------------------------------------------------------------------------
    # GUI Callbacks
    # -------------------------------------------------------------------------

    def on_speed_change(self, speed: float) -> None:
        """
        Handle speed slider change from GUI.

        Args:
            speed: New speed value [0, 1]
        """
        controller = self._get_active_controller()
        if controller is not None and hasattr(controller, "set_speed"):
            controller.set_speed(speed)

    def on_rotation_input(self, input_text: str) -> None:
        """
        Handle rotation target input from GUI.

        Args:
            input_text: Text input (should be angle 0-359.99)
        """
        try:
            angle = float(input_text)
            if 0 <= angle < 360:
                self._gui_rotate_to(angle)
        except ValueError:
            pass  # Ignore invalid input

    def on_curve_drawn(self, points: List[Tuple[float, float]]) -> None:
        """
        Handle curve drawn event from GUI.

        GUI commands take priority, so this clears external commands.
        First rotates toward the path start, then follows the path.

        Args:
            points: List of (x, y) points defining the drawn curve
        """
        if not points:
            return

        # GUI takes priority - clear external command and pending path
        self._external_command_active = False
        self._external_path = None
        self._clear_pending_path()

        controller = self._get_active_controller()
        if controller is None:
            return

        # Set display path
        self._set_display_path(points)

        # Start path with pre-rotation toward path start
        self._start_path_with_pre_rotation(points)

    def on_key_press(self, key: str) -> None:
        """
        Handle key press event from GUI.

        Args:
            key: Key name (lowercase)
        """
        key = key.lower()

        # C to clear path
        if key == "c":
            self._clear_current_task()

        # Space for emergency stop
        elif key == "space":
            self._emergency_stop()

    def on_key_release(self, key: str) -> None:
        """Handle key release event from GUI."""
        pass  # No action needed

    def on_car_click(self, robot_id: int) -> None:
        """
        Handle car click event from GUI.

        Args:
            robot_id: ID of the clicked robot
        """
        if robot_id in self._controllers:
            self._active_robot_id = robot_id

    # -------------------------------------------------------------------------
    # Internal Methods
    # -------------------------------------------------------------------------

    def _get_active_controller(self) -> Optional[NavigationController]:
        """Get the currently active controller."""
        if self._active_robot_id is None:
            return None
        controller = self._controllers.get(self._active_robot_id)
        if isinstance(controller, NavigationController):
            return controller
        return None

    def _gui_rotate_to(self, angle: float) -> None:
        """
        Start rotation to target angle (GUI command).

        GUI commands take priority over external commands.

        Args:
            angle: Target angle in degrees [0, 360)
        """
        # GUI takes priority
        self._external_command_active = False
        self._external_path = None
        self._set_display_path(None)

        controller = self._get_active_controller()
        if controller is not None:
            controller.rotate_to(angle)

    def _clear_current_task(self) -> None:
        """Clear current task and stop robot."""
        self._external_command_active = False
        self._external_path = None
        self._set_display_path(None)
        self._clear_pending_path()

        controller = self._get_active_controller()
        if controller is not None:
            controller.clear_path()
            controller.cancel_rotation()

    def _emergency_stop(self) -> None:
        """Emergency stop all robots."""
        self._external_command_active = False
        self._external_path = None
        self._set_display_path(None)
        self._clear_pending_path()

        for controller in self._controllers.values():
            if isinstance(controller, NavigationController):
                controller.clear_path()
                controller.cancel_rotation()
            controller.stop()

    def _set_display_path(self, path: Optional[List[Point]]) -> None:
        """Set the path to display in GUI."""
        with self._display_path_lock:
            self._display_path = path

    def _start_path_with_pre_rotation(
        self,
        path: List[Point],
        goto_theta: Optional[float] = None
    ) -> bool:
        """
        Start path following with pre-rotation toward path start.

        First rotates the robot to face the first point of the path,
        then starts following the path after rotation completes.

        Args:
            path: List of (x, y) waypoints
            goto_theta: Optional final orientation for goto command

        Returns:
            True if pre-rotation started, False if path following started directly
        """
        controller = self._get_active_controller()
        if controller is None or not path:
            return False

        # Get current position
        current_x = controller.car_state.x
        current_y = controller.car_state.y

        # Calculate distance and angle to first path point
        first_point = path[0]
        dx = first_point[0] - current_x
        dy = first_point[1] - current_y
        distance = math.sqrt(dx * dx + dy * dy)

        # If very close to path start, skip pre-rotation
        if distance < self._pre_rotation_min_distance:
            # Start path following directly
            controller.set_path(path)
            if goto_theta is not None:
                controller._car_state.metadata["goto_target_theta"] = goto_theta
            return False

        # Calculate angle to first point
        target_angle = math.degrees(math.atan2(dy, dx))
        if target_angle < 0:
            target_angle += 360

        # Store pending path
        with self._pending_path_lock:
            self._pending_path = path
            self._pending_goto_theta = goto_theta

        # Start rotation toward path start
        controller.rotate_to(target_angle)
        return True

    def _check_pending_path_transition(self, controller: NavigationController) -> None:
        """
        Check if pre-rotation completed and start pending path if so.

        Called from process() to handle the transition from pre-rotation
        to path following.

        Args:
            controller: The navigation controller to check
        """
        with self._pending_path_lock:
            if self._pending_path is None:
                return

            # Check if rotation completed (or nearly complete)
            if controller._nav_state in (NavigationState.ROTATION_DONE,
                                          NavigationState.ROTATION_STABLE):
                # Rotation complete - start path following
                path = self._pending_path
                goto_theta = self._pending_goto_theta
                self._pending_path = None
                self._pending_goto_theta = None

                # Cancel rotation state and start path
                controller.cancel_rotation()
                controller.set_path(path)
                if goto_theta is not None:
                    controller._car_state.metadata["goto_target_theta"] = goto_theta

    def _clear_pending_path(self) -> None:
        """Clear any pending path waiting for pre-rotation."""
        with self._pending_path_lock:
            self._pending_path = None
            self._pending_goto_theta = None

    # -------------------------------------------------------------------------
    # Coordinator Interface
    # -------------------------------------------------------------------------

    def process(
        self, observations: Dict[int, RobotObservation]
    ) -> Dict[int, Action]:
        """
        Process observations and return actions.

        Handles:
        - Pre-rotation to path start transition
        - Path following to final rotation transition (for goto)

        Args:
            observations: Dict mapping robot_id to RobotObservation

        Returns:
            Dict mapping robot_id to Action
        """
        actions: Dict[int, Action] = {}

        for robot_id, controller in self._controllers.items():
            if robot_id not in observations:
                actions[robot_id] = Action.stop()
                continue

            if isinstance(controller, NavigationController):
                # Check for pending path transition (pre-rotation complete)
                if robot_id == self._active_robot_id:
                    self._check_pending_path_transition(controller)

                # Handle goto final rotation transition
                if controller._nav_state == NavigationState.FINISHED_PATH:
                    # Check if this was a goto command
                    target_theta = controller._car_state.metadata.get("goto_target_theta")
                    if target_theta is not None:
                        # Start rotation to final orientation
                        controller._car_state.metadata.pop("goto_target_theta", None)
                        controller.rotate_to(target_theta)

                # Clear display path when task completes
                if (controller._nav_state in (NavigationState.IDLE,
                                               NavigationState.FINISHED_PATH,
                                               NavigationState.ROTATION_DONE)):
                    if self._external_command_active and robot_id == self._active_robot_id:
                        self._external_command_active = False
                        self._set_display_path(None)

            actions[robot_id] = controller.step(observations[robot_id])

        return actions

    def gather_car_state(self) -> List[CarState]:
        """
        Gather car states with active robot metadata.

        Returns:
            List of CarState objects
        """
        states = []
        for controller in self._controllers.values():
            state = controller.car_state
            state.metadata["is_active"] = (
                controller.robot_id == self._active_robot_id
            )
            states.append(state)
        return states

    def get_additional_drawings(self) -> List[Dict[str, Any]]:
        """
        Get additional drawings for GUI.

        Draws:
        - Selection indicator around active robot
        - Current path being followed
        - Target point
        - Obstacles

        Returns:
            List of drawing commands
        """
        drawings = []

        # Draw selection indicator for active robot
        if self._active_robot_id is not None:
            controller = self._controllers.get(self._active_robot_id)
            if controller is not None:
                state = controller.car_state
                drawings.append({
                    "uuid": f"selection_indicator_{self._active_robot_id}",
                    "type": "circle",
                    "center": (state.x, state.y),
                    "radius": max(self._ws_config.car_width, self._ws_config.car_height) * 0.8,
                    "color": "green",
                    "width": 2,
                })

        # Draw current path
        with self._display_path_lock:
            display_path = self._display_path

        if display_path and len(display_path) >= 2:
            drawings.append({
                "uuid": "navigation_path",
                "type": "path",
                "points": display_path,
                "color": "#00FF00",
                "width": 2,
            })

        # Draw target point from controller
        if self._active_robot_id is not None:
            controller = self._controllers.get(self._active_robot_id)
            if controller is not None and isinstance(controller, NavigationController):
                state = controller.car_state

                # Draw path from controller metadata
                path = state.metadata.get("path")
                if path and len(path) >= 2 and display_path is None:
                    drawings.append({
                        "uuid": f"path_{self._active_robot_id}",
                        "type": "path",
                        "points": path,
                        "color": "#00FF00",
                        "width": 2,
                    })

                # Draw target point
                target = state.metadata.get("target_point")
                if target and state.status_label == "FOLLOWING":
                    drawings.append({
                        "uuid": f"target_{self._active_robot_id}",
                        "type": "point",
                        "position": target,
                        "radius": 5,
                        "color": "#FF0000",
                    })

                    # Draw line from robot to target
                    drawings.append({
                        "uuid": f"target_line_{self._active_robot_id}",
                        "type": "line",
                        "start": (state.x, state.y),
                        "end": target,
                        "color": "#FF000080",
                        "width": 1,
                    })

                # Draw rotation target indicator
                target_theta = state.metadata.get("target_theta")
                if target_theta is not None:
                    # Draw a line showing target direction
                    length = max(self._ws_config.car_width, self._ws_config.car_height) * 1.5
                    rad = math.radians(target_theta)
                    end_x = state.x + length * math.cos(rad)
                    end_y = state.y + length * math.sin(rad)
                    drawings.append({
                        "uuid": f"rotation_target_{self._active_robot_id}",
                        "type": "line",
                        "start": (state.x, state.y),
                        "end": (end_x, end_y),
                        "color": "#0088FF",
                        "width": 2,
                    })

        # Draw obstacles
        with self._obstacles_lock:
            obstacles_copy = list(self._obstacles)

        for i, obstacle in enumerate(obstacles_copy):
            if len(obstacle) >= 3:
                # Close the polygon
                points = list(obstacle) + [obstacle[0]]
                drawings.append({
                    "uuid": f"obstacle_{i}",
                    "type": "path",
                    "points": points,
                    "color": "#FF6600",
                    "width": 2,
                })

        return drawings

    def select_robot(self, robot_id: int) -> bool:
        """
        Select a robot as active.

        Args:
            robot_id: ID of robot to select

        Returns:
            True if robot was found and selected
        """
        if robot_id in self._controllers:
            self._active_robot_id = robot_id
            return True
        return False

    def reset(self) -> None:
        """Reset coordinator and all controllers."""
        self._external_command_active = False
        self._external_path = None
        self._set_display_path(None)
        self._clear_pending_path()
        with self._obstacles_lock:
            self._obstacles = []
        super().reset()

    def close(self) -> None:
        """Clean up resources."""
        self._stop_webserver()
        super().close()


class _NavigationRequestHandler(BaseHTTPRequestHandler):
    """HTTP request handler for navigation API."""

    def __init__(self, *args, coordinator: NavigationCoordinator, **kwargs):
        self.coordinator = coordinator
        super().__init__(*args, **kwargs)

    def log_message(self, format, *args):
        """Suppress default logging."""
        pass

    def _send_json_response(self, data: Dict[str, Any], status: int = 200) -> None:
        """Send a JSON response."""
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def _read_json_body(self) -> Optional[Dict[str, Any]]:
        """Read and parse JSON body."""
        try:
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length)
            return json.loads(body.decode())
        except (ValueError, json.JSONDecodeError):
            return None

    def do_OPTIONS(self) -> None:
        """Handle CORS preflight."""
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self) -> None:
        """Handle GET requests."""
        if self.path == "/status":
            controller = self.coordinator._get_active_controller()
            if controller:
                state = controller.car_state
                self._send_json_response({
                    "status": "ok",
                    "robot_id": self.coordinator._active_robot_id,
                    "position": {"x": state.x, "y": state.y, "theta": state.theta},
                    "state": state.status_label,
                })
            else:
                self._send_json_response({
                    "status": "ok",
                    "robot_id": None,
                    "position": None,
                    "state": "NO_ROBOT"
                })
        else:
            self._send_json_response({"status": "error", "message": "Not found"}, 404)

    def do_POST(self) -> None:
        """Handle POST requests."""
        data = self._read_json_body()

        if self.path == "/setup_obstacle":
            if data is None or "obstacles" not in data:
                self._send_json_response(
                    {"status": "error", "message": "Invalid request body"}, 400
                )
                return
            result = self.coordinator.api_setup_obstacle(data["obstacles"])
            self._send_json_response(result)

        elif self.path == "/goto":
            if data is None:
                self._send_json_response(
                    {"status": "error", "message": "Invalid request body"}, 400
                )
                return
            x = data.get("x")
            y = data.get("y")
            theta = data.get("theta", 0)
            if x is None or y is None:
                self._send_json_response(
                    {"status": "error", "message": "Missing x or y"}, 400
                )
                return
            result = self.coordinator.api_goto(x, y, theta)
            self._send_json_response(result)

        elif self.path == "/follow_path":
            if data is None or "path" not in data:
                self._send_json_response(
                    {"status": "error", "message": "Invalid request body"}, 400
                )
                return
            result = self.coordinator.api_follow_path(data["path"])
            self._send_json_response(result)

        else:
            self._send_json_response({"status": "error", "message": "Not found"}, 404)
