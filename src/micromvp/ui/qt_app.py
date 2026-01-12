from __future__ import annotations

import time
from pathlib import Path
from typing import List

from PyQt6 import QtCore, QtGui, QtWidgets

from micromvp.core.controller import BaseController, Command, Controller
from micromvp.core.patterns import circle_pattern, figure8_pattern
from micromvp.core.planner import refine_paths, shuffle_paths
from micromvp.ui.qt_canvas import CanvasConfig, FieldCanvas
from micromvp.utils.config import AppConfig


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, config: AppConfig, controller: BaseController) -> None:
        super().__init__()
        self._config = config
        self._controller = controller
        self._prototype_path: List[tuple[float, float]] = []
        self.setWindowTitle("microMVP")

        self._setup_ui()
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(16)

    def _setup_ui(self) -> None:
        central = QtWidgets.QWidget(self)
        layout = QtWidgets.QHBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        panel = QtWidgets.QFrame(central)
        panel.setFixedWidth(180)
        panel_layout = QtWidgets.QVBoxLayout(panel)
        panel_layout.setSpacing(10)

        self._run_button = QtWidgets.QPushButton("Run", panel)
        self._stop_button = QtWidgets.QPushButton("Stop", panel)
        self._clear_button = QtWidgets.QPushButton("Clear", panel)
        panel_layout.addWidget(self._run_button)
        panel_layout.addWidget(self._stop_button)
        panel_layout.addWidget(self._clear_button)

        panel_layout.addWidget(QtWidgets.QLabel("Vehicle Speed", panel))
        self._env_speed_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal, panel)
        self._env_speed_slider.setMinimum(0)
        self._env_speed_slider.setMaximum(200)
        self._env_speed_slider.setValue(100)
        panel_layout.addWidget(self._env_speed_slider)

        panel_layout.addWidget(QtWidgets.QLabel("Target Speed", panel))
        self._target_speed_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal, panel)
        self._target_speed_slider.setMinimum(1)
        self._target_speed_slider.setMaximum(100)
        self._target_speed_slider.setValue(int(self._config.target_points_per_sec))
        panel_layout.addWidget(self._target_speed_slider)

        self._show_targets_toggle = QtWidgets.QCheckBox("Show Targets", panel)
        self._show_targets_toggle.setChecked(True)
        panel_layout.addWidget(self._show_targets_toggle)

        panel_layout.addWidget(QtWidgets.QLabel("Car", panel))
        self._car_select = QtWidgets.QComboBox(panel)
        for car_id, _ in self._config.car_info:
            self._car_select.addItem(str(car_id))
        panel_layout.addWidget(self._car_select)

        panel_layout.addWidget(QtWidgets.QLabel("Pattern", panel))
        self._pattern_select = QtWidgets.QComboBox(panel)
        self._pattern_select.addItems(["circle", "figure8"])
        panel_layout.addWidget(self._pattern_select)
        self._pattern_button = QtWidgets.QPushButton("Apply", panel)
        panel_layout.addWidget(self._pattern_button)

        panel_layout.addStretch(1)

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
        self._canvas = FieldCanvas(canvas_config, parent=central)
        self._canvas.setFixedSize(self._config.painter_size[0], self._config.painter_size[1])
        self._canvas.point_added = self._handle_canvas_point

        layout.addWidget(panel)
        layout.addWidget(self._canvas)
        self.setCentralWidget(central)

        self._run_button.clicked.connect(lambda: self._controller.enqueue(Command("set_env_speed_scale", 1.0)))
        self._stop_button.clicked.connect(lambda: self._controller.enqueue(Command("set_env_speed_scale", 0.0)))
        self._clear_button.clicked.connect(self._clear_paths)
        self._pattern_button.clicked.connect(self._apply_pattern)
        self._env_speed_slider.valueChanged.connect(self._set_env_speed_scale)
        self._target_speed_slider.valueChanged.connect(self._set_target_speed)
        self._show_targets_toggle.toggled.connect(self._set_show_targets)

    def _refresh(self) -> None:
        world = self._controller.snapshot()
        self._canvas.set_world(world)
        self._canvas.set_time(time.time())

    def _set_env_speed_scale(self, value: int) -> None:
        self._controller.enqueue(Command("set_env_speed_scale", value / 100.0))

    def _set_target_speed(self, value: int) -> None:
        self._controller.enqueue(Command("set_target_rate", float(value)))

    def _set_show_targets(self, checked: bool) -> None:
        self._canvas.set_show_targets(checked)

    def _clear_paths(self) -> None:
        self._controller.enqueue(Command("clear_paths"))
        self._prototype_path = []
        self._canvas.set_prototype_path(self._prototype_path)

    def _handle_canvas_point(self, point: tuple[float, float]) -> None:
        car_id = int(self._car_select.currentText())
        self._controller.enqueue(Command("add_point", (car_id, point)))
        self._prototype_path.append(point)
        self._canvas.set_prototype_path(self._prototype_path)

    def _apply_pattern(self) -> None:
        world = self._controller.snapshot()
        locs = [(car.x, car.y) for car in world.cars]
        bound = self._config.boundary()
        if self._pattern_select.currentText() == "figure8":
            template_paths = figure8_pattern(len(world.cars), bound)
        else:
            template_paths = circle_pattern(len(world.cars), bound)
        self._prototype_path = list(template_paths[0]) if template_paths else []
        self._canvas.set_prototype_path(self._prototype_path)

        paths = shuffle_paths(locs, template_paths)
        paths = refine_paths(paths)
        self._controller.enqueue(Command("set_paths", paths))


def run_app(config: AppConfig, controller: BaseController) -> int:
    app = QtWidgets.QApplication([])
    window = MainWindow(config, controller)
    window.show()
    return app.exec()
