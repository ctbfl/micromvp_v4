#!/usr/bin/env python3
"""
Example 4: Draw Path by Hand

This example demonstrates interactive path drawing:
1. A single car is randomly positioned on the canvas
2. User draws a path by clicking and dragging the mouse
3. When drawing finishes (mouse release), the car:
   - First goes to the start of the drawn path
   - Then follows the path to the end
4. After completing the path, the path disappears
5. User can draw a new path

This is useful for:
- Testing manual path input
- Interactive control of a single car
- Demonstrating goto-then-follow behavior

Usage:
    python examples/4_draw_path.py
"""
from __future__ import annotations

import random
import time
from pathlib import Path
from typing import List, Optional

from PyQt6 import QtCore, QtGui, QtWidgets

from micromvp.core.coordinator import Coordinator, CoordinatorConfig
from micromvp.core.models import CarConfig, TaskState
from micromvp.env.sim_env import SimConfig, SimEnvironment
from micromvp.ui.qt_canvas import CanvasConfig, FieldCanvas
from micromvp.utils.config import AppConfig, Boundary

Point = tuple[float, float]


class DrawPathWindow(QtWidgets.QMainWindow):
    """
    Specialized window for draw-path example.

    This window handles the draw -> goto -> follow -> clear cycle.
    """

    def __init__(
        self,
        config: AppConfig,
        coordinator: Coordinator,
    ) -> None:
        super().__init__()
        self._config = config
        self._coordinator = coordinator
        self._drawn_path: List[Point] = []
        self._pending_path: Optional[List[Point]] = None
        self._waiting_for_drawing = True

        self.setWindowTitle("MicroMVP - Draw Path Example")
        self._setup_ui()

        # Timer for UI refresh (~60 FPS)
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(16)

    def _setup_ui(self) -> None:
        """Set up the UI layout."""
        central = QtWidgets.QWidget(self)
        layout = QtWidgets.QHBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # Left control panel
        panel = self._create_control_panel(central)
        layout.addWidget(panel)

        # Right canvas
        canvas = self._create_canvas(central)
        layout.addWidget(canvas)

        self.setCentralWidget(central)

    def _create_control_panel(self, parent: QtWidgets.QWidget) -> QtWidgets.QFrame:
        """Create the left control panel."""
        panel = QtWidgets.QFrame(parent)
        panel.setFixedWidth(180)
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setSpacing(10)

        # Instructions
        instructions = QtWidgets.QLabel(
            "Instructions:\n\n"
            "1. Draw a path on the\n"
            "   canvas by clicking\n"
            "   and dragging\n\n"
            "2. Release mouse to\n"
            "   start the car\n\n"
            "3. Car will go to start,\n"
            "   then follow path\n\n"
            "4. After completion,\n"
            "   draw a new path",
            panel,
        )
        instructions.setWordWrap(True)
        layout.addWidget(instructions)

        layout.addSpacing(20)

        # Status label
        self._status_label = QtWidgets.QLabel("Status: Waiting for path...", panel)
        self._status_label.setWordWrap(True)
        layout.addWidget(self._status_label)

        layout.addSpacing(20)

        # Speed slider
        layout.addWidget(QtWidgets.QLabel("Vehicle Speed", panel))
        self._speed_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal, panel)
        self._speed_slider.setMinimum(0)
        self._speed_slider.setMaximum(200)
        self._speed_slider.setValue(100)
        self._speed_slider.valueChanged.connect(self._on_speed_changed)
        layout.addWidget(self._speed_slider)

        layout.addSpacing(20)

        # Clear button
        self._clear_button = QtWidgets.QPushButton("Clear Path", panel)
        self._clear_button.clicked.connect(self._on_clear)
        layout.addWidget(self._clear_button)

        layout.addStretch(1)

        return panel

    def _create_canvas(self, parent: QtWidgets.QWidget) -> FieldCanvas:
        """Create the field canvas for visualization."""
        # Load car image
        asset_path = Path(__file__).resolve().parents[1] / "assets" / "carImage.png"
        pixmap = QtGui.QPixmap(str(asset_path))
        scaled = pixmap.scaled(
            int(self._config.wheel_base * 2.0 / (9.0 / 8.0)),
            int(self._config.wheel_base * 2.0),
            QtCore.Qt.AspectRatioMode.KeepAspectRatio,
            QtCore.Qt.TransformationMode.SmoothTransformation,
        )

        canvas_config = CanvasConfig(
            boundary=self._config.boundary(),
            wheel_base=self._config.wheel_base,
            car_pixmap=scaled,
            car_heading_offset_deg=self._config.car_heading_offset_deg,
        )

        self._canvas = FieldCanvas(canvas_config, parent=parent)
        self._canvas.setFixedSize(
            self._config.painter_size[0],
            self._config.painter_size[1],
        )

        # Connect callbacks
        self._canvas.point_added = self._on_point_added
        self._canvas.drawing_finished = self._on_drawing_finished

        return self._canvas

    def _on_point_added(self, point: Point) -> None:
        """Handle point added during drawing."""
        if not self._waiting_for_drawing:
            return  # Ignore drawing while car is moving

        self._drawn_path.append(point)
        self._canvas.set_prototype_path(self._drawn_path)

    def _on_drawing_finished(self) -> None:
        """Handle mouse release - finalize the drawn path."""
        if not self._waiting_for_drawing:
            return

        if len(self._drawn_path) < 2:
            # Path too short, clear and wait for new drawing
            self._drawn_path = []
            self._canvas.set_prototype_path([])
            return

        # Path is ready - start goto-then-follow
        self._waiting_for_drawing = False
        path = list(self._drawn_path)

        # Command car to go to start of path with correct heading
        car = self._coordinator.cars[0]
        start_x, start_y = path[0]
        car.prepare_for_path(path, tolerance_pos=10.0)

        # Store path for later (will be set after goto completes)
        self._pending_path = path
        self._status_label.setText("Status: Going to start...")
        print(f"Path drawn with {len(path)} points. Going to start ({start_x:.0f}, {start_y:.0f}) with smooth curve...")

    def _on_speed_changed(self, value: int) -> None:
        """Handle speed slider change."""
        self._coordinator.set_speed_scale(value / 100.0)

    def _on_clear(self) -> None:
        """Handle clear button - reset everything."""
        self._drawn_path = []
        self._pending_path = None
        self._waiting_for_drawing = True
        self._canvas.set_prototype_path([])

        # Stop the car
        car = self._coordinator.cars[0]
        car.stop()

        self._status_label.setText("Status: Waiting for path...")

    def _refresh(self) -> None:
        """Refresh canvas with current world state."""
        world = self._coordinator.snapshot()
        self._canvas.set_world(world)
        self._canvas.set_time(time.time())

        car = self._coordinator.cars[0]

        # Check state transitions
        if self._pending_path is not None:
            # Waiting for goto to complete
            if car.is_task_done:
                # Goto complete, now follow the path
                path = self._pending_path
                self._pending_path = None
                car.follow_path(path, loop=False)
                self._status_label.setText("Status: Following path...")
                print("Reached start, now following path...")

        elif not self._waiting_for_drawing:
            # Following path, check if done
            if car.is_task_done:
                # Path complete, clear and wait for new drawing
                self._drawn_path = []
                self._canvas.set_prototype_path([])
                self._waiting_for_drawing = True
                self._status_label.setText("Status: Path complete! Draw a new path...")
                print("Path complete! Ready for new path.")


def run_draw_path_app(config: AppConfig, coordinator: Coordinator) -> int:
    """Run the draw-path application."""
    app = QtWidgets.QApplication([])
    window = DrawPathWindow(config, coordinator)
    window.show()
    return app.exec()


def main() -> int:
    """Main entry point."""
    print("=" * 60)
    print("Example 4: Draw Path by Hand")
    print("=" * 60)
    print("\nInstructions:")
    print("  1. Draw a path on the canvas by clicking and dragging")
    print("  2. Release mouse to start the car")
    print("  3. Car goes to start point, then follows the path")
    print("  4. After completion, draw a new path")
    print()

    # Configuration
    wheel_base = 30.0

    app_config = AppConfig(
        sim=True,
        wheel_base=wheel_base,
        sim_speed=100.0,
        v_max=1.0,
        car_info=[(1, 1)],  # Single car
    )

    boundary = app_config.boundary()

    # Random initial position within boundary
    margin = 50
    initial_x = random.uniform(boundary.left + margin, boundary.right - margin)
    initial_y = random.uniform(boundary.top + margin, boundary.bottom - margin)
    initial_theta = random.uniform(0, 6.28)

    print(f"Car starting at random position: ({initial_x:.0f}, {initial_y:.0f})")

    # Create environment
    sim_config = SimConfig(
        sim_speed=100.0,
        wheel_base=wheel_base,
    )
    initial_poses = {1: (initial_x, initial_y, initial_theta)}
    env = SimEnvironment(config=sim_config, initial_poses=initial_poses)

    # Create car config
    car_config = CarConfig.from_wheel_base(
        robot_id=1,
        tag_id=1,
        wheel_base=wheel_base,
        v_max=1.0,
    )

    # Create coordinator
    coordinator = Coordinator(
        environment=env,
        car_configs=[car_config],
        config=CoordinatorConfig(control_hz=100.0),
    )

    # Start coordinator
    coordinator.start()

    print("Starting GUI...\n")
    try:
        return run_draw_path_app(config=app_config, coordinator=coordinator)
    finally:
        coordinator.stop()
        env.close()


if __name__ == "__main__":
    raise SystemExit(main())
