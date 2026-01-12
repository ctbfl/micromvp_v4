from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

from PyQt6 import QtCore, QtGui, QtWidgets

from micromvp.core.models import WorldState
from micromvp.utils.config import Boundary
from micromvp.utils.geometry import check_collision


Point = Tuple[float, float]


@dataclass(slots=True)
class CanvasConfig:
    # World coords:
    #   X: left -> right
    #   Y: bottom -> top
    boundary: Boundary              # interpreted as world (left, bottom, width, height) via boundary.top as bottom (legacy)
    wheel_base: float
    car_pixmap: QtGui.QPixmap

    # 0 if pixmap faces right at 0-rotation; 90 if faces up at 0-rotation (common)
    car_heading_offset_deg: float = 90.0

    clamp_click_to_boundary: bool = True

    # drawing safety
    sanity_margin: float = 2000.0
    max_segment_len: float = 300.0
    max_route_points: int = 2000


class FieldCanvas(QtWidgets.QWidget):
    """
    Canvas semantics (as you requested):
      - There is ONE universal prototype path (global), not per-car.
      - Cars are independent from that prototype path.
      - Optionally you may still draw each car's current target point/line
        if your WorldState already carries them (see _draw_car_targets()).
    """
    point_added: Callable[[Point], None]

    def __init__(self, config: CanvasConfig, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent=parent)
        self._config = config
        self._world: Optional[WorldState] = None

        # Universal prototype path (WORLD coords)
        self._prototype_path: List[Point] = []

        self._recording = False
        self._time_s = 0.0
        self._show_targets = True
        self.setMouseTracking(True)

    # ----------------------------
    # external setters
    # ----------------------------
    def set_world(self, world: WorldState) -> None:
        self._world = world
        self.update()

    def set_prototype_path(self, path: List[Point]) -> None:
        """Set universal prototype path in WORLD coords."""
        self._prototype_path = list(path) if path else []
        self.update()

    def set_time(self, time_s: float) -> None:
        self._time_s = float(time_s)

    def set_show_targets(self, show: bool) -> None:
        self._show_targets = bool(show)
        self.update()

    # ----------------------------
    # coordinate transforms
    # ----------------------------
    def _world_to_screen(self, x_w: float, y_w: float) -> QtCore.QPointF:
        h = float(self.height())
        return QtCore.QPointF(float(x_w), h - float(y_w))

    def _screen_to_world(self, x_s: float, y_s: float) -> Point:
        h = float(self.height())
        return (float(x_s), h - float(y_s))

    def _boundary_world(self) -> Tuple[float, float, float, float]:
        """
        Return boundary in WORLD coords as (left, bottom, width, height).

        NOTE: we treat Boundary.top as bottom in world coords (legacy naming).
        If your Boundary truly stores top (Qt-style), you must change the data source or adjust here.
        """
        b = self._config.boundary
        left = float(b.left)
        bottom = float(b.top)   # legacy: "top" stores bottom in world coords
        width = float(b.width)
        height = float(b.height)
        return left, bottom, width, height

    def _boundary_world_contains(self, x_w: float, y_w: float, margin: float = 0.0) -> bool:
        left, bottom, width, height = self._boundary_world()
        return (
            (left - margin) <= x_w <= (left + width + margin)
            and (bottom - margin) <= y_w <= (bottom + height + margin)
        )

    def _boundary_rect_screen(self) -> QtCore.QRectF:
        left, bottom, width, height = self._boundary_world()
        h = float(self.height())
        top_world = bottom + height
        top_screen = h - top_world
        return QtCore.QRectF(left, top_screen, width, height)

    @staticmethod
    def _finite(a: float) -> bool:
        return isinstance(a, (int, float)) and math.isfinite(a)

    # ----------------------------
    # polyline drawing helpers
    # ----------------------------
    def _sanitize_path(self, path: List[Point]) -> List[Point]:
        if not path:
            return []
        margin = float(self._config.sanity_margin)
        out: List[Point] = []
        for (x, y) in path:
            if not (self._finite(x) and self._finite(y)):
                continue
            if not self._boundary_world_contains(x, y, margin=margin):
                continue
            out.append((float(x), float(y)))

        max_n = int(self._config.max_route_points)
        if max_n > 0 and len(out) > max_n:
            step = max(1, len(out) // max_n)
            out = out[::step]
        return out

    def _draw_polyline_safe(self, painter: QtGui.QPainter, path: List[Point]) -> None:
        if len(path) < 2:
            return
        max_seg = float(self._config.max_segment_len)
        max_seg2 = max_seg * max_seg

        qp = QtGui.QPainterPath()
        prev = None
        for (x, y) in path:
            ps = self._world_to_screen(x, y)
            if prev is None:
                qp.moveTo(ps)
                prev = (x, y)
                continue
            dx = x - prev[0]
            dy = y - prev[1]
            if (dx * dx + dy * dy) > max_seg2:
                qp.moveTo(ps)  # break
            else:
                qp.lineTo(ps)
            prev = (x, y)

        painter.drawPath(qp)

    # ----------------------------
    # paint
    # ----------------------------
    def paintEvent(self, event: QtGui.QPaintEvent) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
        painter.fillRect(self.rect(), QtGui.QColor(255, 255, 255))

        # boundary
        painter.setPen(QtGui.QPen(QtGui.QColor(128, 128, 128), 1))
        painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        painter.drawRect(self._boundary_rect_screen())

        # 1) draw ONE universal prototype path (black)
        route_pen = QtGui.QPen(QtGui.QColor(0, 0, 0), 2)
        route_pen.setCosmetic(True)
        painter.setPen(route_pen)
        painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        safe_proto = self._sanitize_path(list(self._prototype_path))
        self._draw_polyline_safe(painter, safe_proto)

        # 2) draw cars (no relation to prototype path)
        if self._world is None:
            return

        for car in self._world.cars:
            if not (self._finite(car.x) and self._finite(car.y) and self._finite(car.theta)):
                continue

            center_s = self._world_to_screen(car.x, car.y)

            # World theta: CCW positive; Qt rotation: clockwise positive => use -theta
            angle_deg = (-math.degrees(car.theta)) + float(self._config.car_heading_offset_deg)

            transform = QtGui.QTransform().rotate(angle_deg)
            rotated = self._config.car_pixmap.transformed(
                transform,
                QtCore.Qt.TransformationMode.SmoothTransformation,
            )
            rect = rotated.rect()
            rect.moveCenter(QtCore.QPoint(int(center_s.x()), int(center_s.y())))
            painter.drawPixmap(rect.topLeft(), rotated)

            # center marker
            painter.setBrush(QtGui.QColor(255, 0, 0))
            painter.setPen(QtCore.Qt.PenStyle.NoPen)
            painter.drawEllipse(center_s, 3, 3)

            # id
            painter.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0), 1))
            painter.drawText(
                int(center_s.x() - self._config.wheel_base / 2),
                int(center_s.y() - self._config.wheel_base / 2),
                str(car.car_id),
            )

        if self._show_targets and self._world.targets:
            painter.setPen(QtGui.QPen(QtGui.QColor(80, 80, 80), 1, QtCore.Qt.PenStyle.DashLine))
            painter.setBrush(QtGui.QColor(255, 80, 80))
            for car in self._world.cars:
                target = self._world.targets.get(car.tag_id)
                if target is None:
                    continue
                target_s = self._world_to_screen(target[0], target[1])
                car_s = self._world_to_screen(car.x, car.y)
                painter.drawLine(
                    int(car_s.x()),
                    int(car_s.y()),
                    int(target_s.x()),
                    int(target_s.y()),
                )
                painter.drawEllipse(target_s, 4, 4)

        # 3) collision warning
        painter.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0), 1))
        for i, car in enumerate(self._world.cars):
            for j, other in enumerate(self._world.cars):
                if i >= j:
                    continue
                if check_collision(self._config.wheel_base * 1.5, car.x, car.y, other.x, other.y):
                    mid_wx = (car.x + other.x) / 2.0
                    mid_wy = (car.y + other.y) / 2.0
                    mid_s = self._world_to_screen(mid_wx, mid_wy)
                    painter.drawText(mid_s, "TOO CLOSE!")

    # ----------------------------
    # mouse events (emit WORLD points)
    # ----------------------------
    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self._recording = True
            self._emit_point(event.position())

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:
        if self._recording:
            self._emit_point(event.position())

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self._recording = False

    def _emit_point(self, position: QtCore.QPointF) -> None:
        if not (hasattr(self, "point_added") and self.point_added):
            return

        x_w, y_w = self._screen_to_world(position.x(), position.y())
        if not (self._finite(x_w) and self._finite(y_w)):
            return

        if self._config.clamp_click_to_boundary:
            if not self._boundary_world_contains(x_w, y_w, margin=0.0):
                return

        self.point_added((x_w, y_w))
