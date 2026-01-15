"""
Qt GUI Application for MicroMVP.

The GUI has two main parts:
1. Canvas: Displays observations faithfully (car positions and orientations)
2. Control Panel: Interacts with Coordinator (speed, patterns, pause/resume)

The GUI does NOT contain any control logic - it only:
- Displays WorldState snapshots from Coordinator
- Sends commands to Coordinator via its API
"""
from __future__ import annotations

import time
from pathlib import Path
from typing import List, Optional

from PyQt6 import QtCore, QtGui, QtWidgets

from micromvp.core.coordinator import Coordinator
from micromvp.core.patterns import circle_pattern, figure8_pattern
from micromvp.core.planner import refine_paths, shuffle_paths
from micromvp.ui.qt_canvas import CanvasConfig, FieldCanvas
from micromvp.utils.config import AppConfig

Point = tuple[float, float]


class MainWindow(QtWidgets.QMainWindow):
    """Main application window."""

    def __init__(
        self,
        config: AppConfig,
        coordinator: Coordinator,
        initial_paths: Optional[List[List[Point]]] = None,
        goto_then_follow: bool = False,
    ) -> None:
        """
        Initialize main window.

        Args:
            config: Application configuration
            coordinator: The coordinator managing cars
            initial_paths: Optional paths to display and follow after goto completes
            goto_then_follow: If True, cars are doing goto and should follow paths when done
        """
        super().__init__()
        self._config = config
        self._coordinator = coordinator
        self._prototype_path: List[Point] = []
        self._pending_paths: Optional[List[List[Point]]] = initial_paths if goto_then_follow else None
        self._goto_then_follow = goto_then_follow

        self.setWindowTitle("microMVP")
        self._setup_ui()

        # Set initial prototype path if provided (for bug 3 fix)
        if initial_paths and len(initial_paths) > 0:
            self._prototype_path = list(initial_paths[0])
            self._canvas.set_prototype_path(self._prototype_path)

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

        # Run/Stop/Clear buttons
        self._run_button = QtWidgets.QPushButton("Run", panel)
        self._stop_button = QtWidgets.QPushButton("Stop", panel)
        self._clear_button = QtWidgets.QPushButton("Clear", panel)
        layout.addWidget(self._run_button)
        layout.addWidget(self._stop_button)
        layout.addWidget(self._clear_button)

        # Vehicle Speed slider
        layout.addWidget(QtWidgets.QLabel("Vehicle Speed", panel))
        self._speed_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal, panel)
        self._speed_slider.setMinimum(0)
        self._speed_slider.setMaximum(200)
        self._speed_slider.setValue(100)
        layout.addWidget(self._speed_slider)

        # Show Targets toggle
        self._show_targets_toggle = QtWidgets.QCheckBox("Show Targets", panel)
        self._show_targets_toggle.setChecked(True)
        layout.addWidget(self._show_targets_toggle)

        # Car selection for manual point adding
        layout.addWidget(QtWidgets.QLabel("Car", panel))
        self._car_select = QtWidgets.QComboBox(panel)
        for car_id, _ in self._config.car_info:
            self._car_select.addItem(str(car_id))
        layout.addWidget(self._car_select)

        # Pattern selection
        layout.addWidget(QtWidgets.QLabel("Pattern", panel))
        self._pattern_select = QtWidgets.QComboBox(panel)
        self._pattern_select.addItems(["circle", "figure8"])
        layout.addWidget(self._pattern_select)

        self._pattern_button = QtWidgets.QPushButton("Apply Pattern", panel)
        layout.addWidget(self._pattern_button)

        layout.addStretch(1)

        # Connect signals
        self._run_button.clicked.connect(self._on_run)
        self._stop_button.clicked.connect(self._on_stop)
        self._clear_button.clicked.connect(self._on_clear)
        self._speed_slider.valueChanged.connect(self._on_speed_changed)
        self._show_targets_toggle.toggled.connect(self._on_show_targets_changed)
        self._pattern_button.clicked.connect(self._on_apply_pattern)

        return panel

    def _create_canvas(self, parent: QtWidgets.QWidget) -> FieldCanvas:
        """Create the field canvas for visualization."""
        # Load car image
        asset_path = Path(__file__).resolve().parents[3] / "assets" / "carImage.png"
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
        self._canvas.point_added = self._on_canvas_point

        return self._canvas

    # ------------------------------------------------------------------
    # UI Refresh
    # ------------------------------------------------------------------
    def _refresh(self) -> None:
        """Refresh canvas with current world state."""
        world = self._coordinator.snapshot()
        self._canvas.set_world(world)
        self._canvas.set_time(time.time())

        # Check if pending paths should be applied (goto-then-follow mode)
        if self._pending_paths is not None and self._coordinator.all_tasks_done():
            paths = self._pending_paths
            self._pending_paths = None  # Clear pending to avoid re-applying
            self._coordinator.set_paths(paths, loop=True)
            print("All cars reached start positions - now following paths")

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------
    def _on_run(self) -> None:
        """Handle Run button click - resume at normal speed."""
        self._coordinator.set_speed_scale(1.0)
        self._speed_slider.setValue(100)

    def _on_stop(self) -> None:
        """Handle Stop button click - pause all motion."""
        self._coordinator.set_speed_scale(0.0)
        self._speed_slider.setValue(0)

    def _on_clear(self) -> None:
        """Handle Clear button click - clear all paths."""
        self._coordinator.clear_paths()
        self._prototype_path = []
        self._canvas.set_prototype_path(self._prototype_path)

    def _on_speed_changed(self, value: int) -> None:
        """Handle speed slider change."""
        self._coordinator.set_speed_scale(value / 100.0)

    def _on_show_targets_changed(self, checked: bool) -> None:
        """Handle show targets toggle."""
        self._canvas.set_show_targets(checked)

    def _on_canvas_point(self, point: tuple[float, float]) -> None:
        """Handle point added on canvas."""
        car_id = int(self._car_select.currentText())
        self._coordinator.add_point_to_car(car_id, point)
        self._prototype_path.append(point)
        self._canvas.set_prototype_path(self._prototype_path)

    def _on_apply_pattern(self) -> None:
        """Handle Apply Pattern button click.

        This implements goto-then-follow: cars first move to their start
        positions, then follow the path. This ensures proper alignment
        before beginning the pattern.
        """
        world = self._coordinator.snapshot()
        locs = [(car.x, car.y) for car in world.cars]
        bound = self._config.boundary()

        # Generate pattern
        pattern_name = self._pattern_select.currentText()
        if pattern_name == "figure8":
            template_paths = figure8_pattern(len(world.cars), bound)
        else:
            template_paths = circle_pattern(len(world.cars), bound)

        # Update prototype display (so path is visible immediately)
        self._prototype_path = list(template_paths[0]) if template_paths else []
        self._canvas.set_prototype_path(self._prototype_path)

        # Assign paths to cars (optimal assignment based on current locations)
        paths = shuffle_paths(locs, template_paths)
        paths = refine_paths(paths)

        # First, command each car to go to its path's start point
        cars = self._coordinator.cars
        for i, car in enumerate(cars):
            if i < len(paths) and len(paths[i]) > 0:
                start_x, start_y = paths[i][0]
                car.goto(x=start_x, y=start_y, tolerance_pos=15.0)

        # Store paths as pending - they will be applied when all cars reach start
        self._pending_paths = paths
        print(f"Applying {pattern_name} pattern - cars moving to start positions...")


def run_app(
    config: AppConfig,
    coordinator: Coordinator,
    initial_paths: Optional[List[List[Point]]] = None,
    goto_then_follow: bool = False,
) -> int:
    """
    Run the Qt application.

    Args:
        config: Application configuration
        coordinator: The coordinator managing cars
        initial_paths: Optional paths to display and follow after goto completes
        goto_then_follow: If True, cars are doing goto and should follow paths when done

    Returns:
        Application exit code
    """
    app = QtWidgets.QApplication([])
    window = MainWindow(
        config,
        coordinator,
        initial_paths=initial_paths,
        goto_then_follow=goto_then_follow,
    )
    window.show()
    return app.exec()
