"""
Main Canvas component for MicroMVP GUI.

Implements:
- Coordinate transformation (workspace units to screen pixels)
- UUID-based retained mode drawing
- Car rendering with ID labels
- Click interactions (canvas, car, curve drawing)
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QBrush, QColor, QPainter, QPainterPath, QPen, QPixmap
from PyQt6.QtWidgets import (
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsItemGroup,
    QGraphicsLineItem,
    QGraphicsPathItem,
    QGraphicsPixmapItem,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
    QGraphicsView,
    QSizePolicy,
)

from micromvp.core.models import CarState, WorkspaceConfig


Point = Tuple[float, float]


def load_car_pixmap() -> QPixmap:
    """
    Load car image from package resources:
      src/micromvp/assets/carImage.png  ->  micromvp.assets/carImage.png
    If missing, raises a clear error (no fallback).
    """
    import importlib.resources as pkg_resources

    car_png = pkg_resources.files("micromvp.assets") / "carImage.png"
    with pkg_resources.as_file(car_png) as p:
        pm = QPixmap(str(p))
    if pm.isNull():
        raise RuntimeError("Failed to load pixmap from micromvp/assets/carImage.png")
    return pm


class CarGraphicsItem(QGraphicsItemGroup):
    """
    Car graphics item composed of:
      - Pixmap (car image), centered at (0,0) == car representation point.
      - Collision box (rect) using offset_w/offset_h and car_width/car_height.
      - ID label.

    Conventions:
      - Workspace theta=0 points to +X direction.
      - carImage.png visually points "up" on screen.
      - The view flips Y via scale(..., -...), so rotation is negated (same as old code).
      - Add IMAGE_HEADING_OFFSET_DEG to map theta=0 (+X) to image's "up" direction.

    If the visual heading is off by 90/180, tweak IMAGE_HEADING_OFFSET_DEG.
    """

    IMAGE_HEADING_OFFSET_DEG = 90.0

    def __init__(
        self,
        car_id: int,
        car_width: float,
        car_height: float,
        offset_w: float,
        offset_h: float,
        car_pixmap: QPixmap,
        parent: Optional[QGraphicsItem] = None,
        show_collision_box: bool = True,
    ):
        super().__init__(parent)
        self.car_id = car_id
        self._car_width = float(car_width)
        self._car_height = float(car_height)
        self._offset_w = float(offset_w)
        self._offset_h = float(offset_h)

        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)

        # -------------------------
        # 1) Car image (pixmap)
        # -------------------------
        self._pixmap_item = QGraphicsPixmapItem(car_pixmap)
        self._pixmap_item.setTransformationMode(Qt.TransformationMode.SmoothTransformation)

        # Scale so that rendered width matches workspace car_width
        pm_w = max(1, car_pixmap.width())
        scale = self._car_width / float(pm_w)
        self._pixmap_item.setScale(scale)

        # Center pixmap at (0,0) (representation point)
        self._pixmap_item.setOffset(-car_pixmap.width() / 2.0, -car_pixmap.height() / 2.0)

        self.addToGroup(self._pixmap_item)

        # -------------------------
        # 2) Collision box (rect)
        # -------------------------
        self._collision_rect_item: Optional[QGraphicsRectItem] = None
        if show_collision_box:
            # Wheel-axis center is (0,0).
            # Collision box uses offsets exactly like the old polygon definition.
            rect_x = -self._offset_w
            rect_y = -self._offset_h
            rect_w = self._car_width
            rect_h = self._car_height

            self._collision_rect_item = QGraphicsRectItem(rect_x, rect_y, rect_w, rect_h)
            self._collision_rect_item.setPen(QPen(QColor(30, 60, 90), 2))
            self._collision_rect_item.setBrush(QBrush(Qt.GlobalColor.transparent))
            self.addToGroup(self._collision_rect_item)

        # -------------------------
        # 3) ID label
        # -------------------------
        self._label = QGraphicsSimpleTextItem(str(car_id))
        self._label.setBrush(QBrush(Qt.GlobalColor.white))

        # Put label at the collision box top-left if available; otherwise near center
        if self._collision_rect_item is not None:
            self._label.setPos(-self._offset_w + 2.0, -self._offset_h + 2.0)
        else:
            self._label.setPos(2.0, 2.0)

        self.addToGroup(self._label)

    def set_pose(self, x: float, y: float, theta_deg: float) -> None:
        """Set car pose in workspace coordinates."""
        self.setPos(x, y)

        # View flips Y-axis, so negate rotation sign (same as old polygon approach).
        # Image points up, workspace theta=0 is +X, so add a heading offset.
        self.setRotation((theta_deg) + self.IMAGE_HEADING_OFFSET_DEG)



class MVPCanvas(QGraphicsView):
    """
    Main canvas for displaying workspace, cars, and additional drawings.

    Signals:
        canvas_clicked(x, y): Emitted when canvas is clicked (workspace coordinates)
        car_clicked(car_id): Emitted when a car is clicked
        curve_drawn(points): Emitted when user draws a curve (workspace coordinates)
    """

    canvas_clicked = pyqtSignal(float, float)
    car_clicked = pyqtSignal(int)
    curve_drawn = pyqtSignal(list)

    def __init__(
        self,
        workspace_config: WorkspaceConfig,
        parent=None,
    ):
        super().__init__(parent)
        self._ws_config = workspace_config

        # Scene setup
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)

        # Rendering settings
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        self.setViewportUpdateMode(QGraphicsView.ViewportUpdateMode.FullViewportUpdate)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        # Background
        self.setBackgroundBrush(QBrush(QColor(245, 245, 245)))

        # Load car image once (no fallback)
        self._car_pixmap = load_car_pixmap()

        # Drawing caches
        self._car_items: Dict[int, CarGraphicsItem] = {}
        self._drawing_cache: Dict[str, QGraphicsItem] = {}

        # Coordinate transformation state
        self._scale = 1.0
        self._canvas_height = 0.0

        # Curve drawing state
        self._draw_curve_enabled = False
        self._click_canvas_enabled = False
        self._is_drawing = False
        self._current_path: List[Point] = []
        self._path_item: Optional[QGraphicsPathItem] = None
        self._click_point_item: Optional[QGraphicsEllipseItem] = None

        # Draw workspace boundary
        self._draw_workspace_boundary()

    def enable_curve_drawing(self, enabled: bool) -> None:
        """Enable or disable curve drawing mode."""
        self._draw_curve_enabled = enabled

    def enable_canvas_click(self, enabled: bool) -> None:
        """Enable or disable general canvas click callback (empty area)."""
        self._click_canvas_enabled = enabled

    def _show_click_point(self, x: float, y: float, r: float = 2.5) -> None:
        """Show/update a small point at (x,y) in workspace coordinates."""
        if self._click_point_item is None:
            self._click_point_item = QGraphicsEllipseItem()
            self._click_point_item.setPen(QPen(QColor(255, 80, 80), 2))
            self._click_point_item.setBrush(QBrush(QColor(255, 80, 80)))
            self._scene.addItem(self._click_point_item)
        self._click_point_item.setRect(x - r, y - r, 2 * r, 2 * r)

    def _clear_click_point(self) -> None:
        """Remove the transient click point if it exists."""
        if self._click_point_item is not None:
            self._scene.removeItem(self._click_point_item)
            self._click_point_item = None

    def _draw_workspace_boundary(self) -> None:
        """Draw the workspace boundary rectangle."""
        ws_w = self._ws_config.width
        ws_h = self._ws_config.height

        pen = QPen(QColor(100, 100, 100), 2, Qt.PenStyle.DashLine)
        self._boundary_rect = self._scene.addRect(
            0, 0, ws_w, ws_h, pen, QBrush(Qt.GlobalColor.transparent)
        )

    def resizeEvent(self, event) -> None:
        """Handle resize to maintain aspect ratio and update scaling."""
        super().resizeEvent(event)
        self._update_transform()

    def _update_transform(self) -> None:
        """Update coordinate transformation based on current view size."""
        view_w = self.viewport().width()
        view_h = self.viewport().height()
        ws_w = self._ws_config.width
        ws_h = self._ws_config.height

        if view_w <= 0 or view_h <= 0:
            return

        # Calculate scale to fit workspace in view while maintaining aspect ratio
        scale_x = view_w / ws_w
        scale_y = view_h / ws_h
        self._scale = min(scale_x, scale_y)
        self._canvas_height = view_h

        # Scene rect in workspace coordinates
        self._scene.setSceneRect(0, 0, ws_w, ws_h)

        # Flip Y-axis via view transform
        self.resetTransform()
        self.scale(self._scale, -self._scale)
        self.centerOn(ws_w / 2, ws_h / 2)

    # -------------------------------------------------------------------------
    # Coordinate Transformation
    # -------------------------------------------------------------------------

    def workspace_to_scene(self, x: float, y: float) -> Tuple[float, float]:
        """
        Convert workspace coordinates to scene coordinates.

        Note: Scene coordinates already have Y-axis flipped via view transform,
        so we just return the workspace coordinates directly.
        """
        return (x, y)

    def scene_to_workspace(self, sx: float, sy: float) -> Tuple[float, float]:
        """Convert scene coordinates to workspace coordinates."""
        return (sx, sy)

    def view_to_workspace(self, vx: float, vy: float) -> Tuple[float, float]:
        """Convert view (mouse) coordinates to workspace coordinates."""
        scene_point = self.mapToScene(int(vx), int(vy))
        return (scene_point.x(), scene_point.y())

    # -------------------------------------------------------------------------
    # Car Rendering
    # -------------------------------------------------------------------------

    def update_cars(self, car_states: Dict[int, CarState]) -> None:
        """Update car positions and create/remove car items as needed."""
        current_ids = set(car_states.keys())
        existing_ids = set(self._car_items.keys())

        # Remove cars that no longer exist
        for car_id in existing_ids - current_ids:
            item = self._car_items.pop(car_id)
            self._scene.removeItem(item)

        # Update or create cars
        for car_id, state in car_states.items():
            if car_id not in self._car_items:
                item = CarGraphicsItem(
                    car_id=car_id,
                    car_width=self._ws_config.car_width,
                    car_height=self._ws_config.car_height,
                    offset_w=self._ws_config.offset_w,
                    offset_h=self._ws_config.offset_h,
                    car_pixmap=self._car_pixmap,
                    show_collision_box=True,
                )
                self._scene.addItem(item)
                self._car_items[car_id] = item

            item = self._car_items[car_id]
            item.set_pose(state.x, state.y, state.theta)

    # -------------------------------------------------------------------------
    # UUID-Based Drawing API
    # -------------------------------------------------------------------------

    def update_drawings(self, drawings: List[Dict[str, Any]]) -> None:
        """
        Update additional drawings using UUID-based retained mode.

        Each drawing dict should have:
        - uuid: str - Unique identifier
        - type: str - "line", "circle", "rect", "path", "point"
        - Additional type-specific parameters
        """
        current_uuids = set()

        for drawing in drawings:
            uuid = drawing.get("uuid")
            if not uuid:
                continue
            current_uuids.add(uuid)

            if uuid in self._drawing_cache:
                self._update_drawing_item(uuid, drawing)
            else:
                self._create_drawing_item(uuid, drawing)

        # Remove stale items
        stale_uuids = set(self._drawing_cache.keys()) - current_uuids
        for uuid in stale_uuids:
            item = self._drawing_cache.pop(uuid)
            self._scene.removeItem(item)

    def _create_drawing_item(self, uuid: str, drawing: Dict[str, Any]) -> None:
        """Create a new drawing item."""
        draw_type = drawing.get("type", "").lower()
        item: Optional[QGraphicsItem] = None

        if draw_type == "line":
            item = self._create_line(drawing)
        elif draw_type == "circle":
            item = self._create_circle(drawing)
        elif draw_type == "rect":
            item = self._create_rect(drawing)
        elif draw_type == "path":
            item = self._create_path(drawing)
        elif draw_type == "point":
            item = self._create_point(drawing)

        if item:
            self._scene.addItem(item)
            self._drawing_cache[uuid] = item

    def _update_drawing_item(self, uuid: str, drawing: Dict[str, Any]) -> None:
        """Update an existing drawing item."""
        item = self._drawing_cache[uuid]
        draw_type = drawing.get("type", "").lower()

        if draw_type == "line" and isinstance(item, QGraphicsLineItem):
            x1, y1 = drawing.get("start", (0, 0))
            x2, y2 = drawing.get("end", (0, 0))
            item.setLine(x1, y1, x2, y2)
            self._apply_pen(item, drawing)

        elif draw_type == "circle" and isinstance(item, QGraphicsEllipseItem):
            cx, cy = drawing.get("center", (0, 0))
            r = drawing.get("radius", 10)
            item.setRect(cx - r, cy - r, 2 * r, 2 * r)
            self._apply_pen_brush(item, drawing)

        elif draw_type == "rect" and isinstance(item, QGraphicsRectItem):
            x, y = drawing.get("position", (0, 0))
            w, h = drawing.get("size", (10, 10))
            item.setRect(x, y, w, h)
            self._apply_pen_brush(item, drawing)

        elif draw_type == "path" and isinstance(item, QGraphicsPathItem):
            points = drawing.get("points", [])
            path = self._build_path(points)
            item.setPath(path)
            self._apply_pen(item, drawing)

        elif draw_type == "point" and isinstance(item, QGraphicsEllipseItem):
            x, y = drawing.get("position", (0, 0))
            r = drawing.get("radius", 3)
            item.setRect(x - r, y - r, 2 * r, 2 * r)
            self._apply_pen_brush(item, drawing)

    def _create_line(self, drawing: Dict[str, Any]) -> QGraphicsLineItem:
        """Create a line item."""
        x1, y1 = drawing.get("start", (0, 0))
        x2, y2 = drawing.get("end", (0, 0))
        item = QGraphicsLineItem(x1, y1, x2, y2)
        self._apply_pen(item, drawing)
        return item

    def _create_circle(self, drawing: Dict[str, Any]) -> QGraphicsEllipseItem:
        """Create a circle item."""
        cx, cy = drawing.get("center", (0, 0))
        r = drawing.get("radius", 10)
        item = QGraphicsEllipseItem(cx - r, cy - r, 2 * r, 2 * r)
        self._apply_pen_brush(item, drawing)
        return item

    def _create_rect(self, drawing: Dict[str, Any]) -> QGraphicsRectItem:
        """Create a rectangle item."""
        x, y = drawing.get("position", (0, 0))
        w, h = drawing.get("size", (10, 10))
        item = QGraphicsRectItem(x, y, w, h)
        self._apply_pen_brush(item, drawing)
        return item

    def _create_path(self, drawing: Dict[str, Any]) -> QGraphicsPathItem:
        """Create a path item from list of points."""
        points = drawing.get("points", [])
        path = self._build_path(points)
        item = QGraphicsPathItem(path)
        self._apply_pen(item, drawing)
        return item

    def _create_point(self, drawing: Dict[str, Any]) -> QGraphicsEllipseItem:
        """Create a point (small circle) item."""
        x, y = drawing.get("position", (0, 0))
        r = drawing.get("radius", 3)
        item = QGraphicsEllipseItem(x - r, y - r, 2 * r, 2 * r)
        self._apply_pen_brush(item, drawing)
        return item

    def _build_path(self, points: List[Tuple[float, float]]) -> QPainterPath:
        """Build a QPainterPath from a list of points."""
        path = QPainterPath()
        if points:
            path.moveTo(points[0][0], points[0][1])
            for x, y in points[1:]:
                path.lineTo(x, y)
        return path

    def _apply_pen(self, item: QGraphicsItem, drawing: Dict[str, Any]) -> None:
        """Apply pen styling to an item."""
        color = drawing.get("color", "#FF0000")
        width = drawing.get("width", 2)
        pen = QPen(QColor(color), width)
        item.setPen(pen)

    def _apply_pen_brush(self, item: QGraphicsItem, drawing: Dict[str, Any]) -> None:
        """Apply pen and brush styling to an item."""
        color = drawing.get("color", "#FF0000")
        fill = drawing.get("fill", color)
        width = drawing.get("width", 2)
        item.setPen(QPen(QColor(color), width))
        item.setBrush(QBrush(QColor(fill)))

    # -------------------------------------------------------------------------
    # Mouse Interaction
    # -------------------------------------------------------------------------

    def mousePressEvent(self, event) -> None:
        """Handle mouse press events."""
        if event.button() == Qt.MouseButton.LeftButton:
            scene_pos = self.mapToScene(event.pos())
            ws_x, ws_y = scene_pos.x(), scene_pos.y()

            # Check if clicked on a car (or a child item of the car)
            items = self.items(event.pos())
            for it in items:
                # Could be the group itself, or children (pixmap/rect/label)
                if isinstance(it, CarGraphicsItem):
                    self.car_clicked.emit(it.car_id)
                    super().mousePressEvent(event)
                    return
                parent = it.parentItem()
                if isinstance(parent, CarGraphicsItem):
                    self.car_clicked.emit(parent.car_id)
                    super().mousePressEvent(event)
                    return

            # Empty area click
            if self._draw_curve_enabled:
                # If click callback enabled, show point immediately as press feedback
                if self._click_canvas_enabled:
                    self._show_click_point(ws_x, ws_y)

                self._is_drawing = True
                self._current_path = [(ws_x, ws_y)]
                self._path_item = QGraphicsPathItem()
                self._path_item.setPen(QPen(QColor(255, 100, 100), 2))
                self._scene.addItem(self._path_item)
            else:
                # Click-only mode: show point instantly + emit instantly (if enabled)
                if self._click_canvas_enabled:
                    self._show_click_point(ws_x, ws_y)
                    self.canvas_clicked.emit(ws_x, ws_y)

        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        """Handle mouse move events for curve drawing."""
        if self._is_drawing and self._path_item:
            scene_pos = self.mapToScene(event.pos())
            ws_x, ws_y = scene_pos.x(), scene_pos.y()
            self._current_path.append((ws_x, ws_y))

            # If this is clearly a curve, remove the press-feedback point
            if self._click_canvas_enabled and len(self._current_path) >= 5:
                self._clear_click_point()

            path = self._build_path(self._current_path)
            self._path_item.setPath(path)

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        """Handle mouse release events."""
        if event.button() == Qt.MouseButton.LeftButton and self._is_drawing:
            self._is_drawing = False

            # Remove temporary path item
            if self._path_item:
                self._scene.removeItem(self._path_item)
                self._path_item = None

            if self._click_canvas_enabled and len(self._current_path) < 10:
                # Point click: emit on release (so curve mode can disambiguate)
                x0, y0 = self._current_path[0]
                self.canvas_clicked.emit(x0, y0)
                # transient point should disappear on release
                self._clear_click_point()
            else:
                # Curve: ensure press point disappears
                if self._click_canvas_enabled:
                    self._clear_click_point()
                if len(self._current_path) > 5:
                    self.curve_drawn.emit(self._current_path)

            self._current_path = []

        # Click-only mode: remove transient point on release
        elif event.button() == Qt.MouseButton.LeftButton:
            if self._click_canvas_enabled:
                self._clear_click_point()


        super().mouseReleaseEvent(event)