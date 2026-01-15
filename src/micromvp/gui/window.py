"""
Main window for MicroMVP GUI application.

MVPWindow is the main entry point for the GUI, providing:
- Left sidebar with control panel and car inspector
- Right canvas for workspace visualization
- Callback registration API
- Thread-safe update mechanism
"""

from typing import Any, Callable, Dict, List, Optional

from PyQt6.QtCore import QObject, Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QMainWindow,
    QSplitter,
    QWidget,
)

from micromvp.core.models import CarState, WorkspaceConfig

from .canvas import MVPCanvas
from .sidebar import Sidebar


class UpdateSignalEmitter(QObject):
    """
    Helper class to emit signals from non-GUI threads.

    Qt requires GUI updates to happen on the main thread.
    This class provides a signal-based bridge for thread-safe updates.
    """

    update_requested = pyqtSignal(dict, list)


class MVPWindow(QMainWindow):
    """
    Main MicroMVP GUI Window.

    Layout:
    - Left: Sidebar (control panel + car inspector)
    - Right: Canvas (workspace visualization)

    Usage:
        gui = MVPWindow(gui_config, workspace_config)
        gui.register_callback("cmd_name", handler_func)
        gui.run()  # Blocks until window is closed

    From logic thread:
        gui.update(world_state, additional_drawings)
    """

    def __init__(
        self,
        gui_config: Dict[str, Any],
        workspace_config: WorkspaceConfig,
        parent: Optional[QWidget] = None,
    ):
        # Ensure QApplication exists
        self._app = QApplication.instance()
        if self._app is None:
            self._app = QApplication([])

        super().__init__(parent)

        self._ws_config = workspace_config
        self._gui_config = gui_config
        self._callbacks: Dict[str, Callable] = {}

        # Car states cache for inspector updates
        self._car_states: Dict[int, CarState] = {}
        self._selected_car_id: Optional[int] = None

        # Thread-safe update mechanism
        self._update_emitter = UpdateSignalEmitter()
        self._update_emitter.update_requested.connect(self._handle_update)

        # Setup UI
        self._setup_window()
        self._setup_layout()
        self._connect_signals()

    def _setup_window(self) -> None:
        """Configure main window properties."""
        self.setWindowTitle("MicroMVP Control Center")
        self.setMinimumSize(800, 600)
        self.resize(1200, 800)

    def _setup_layout(self) -> None:
        """Setup the main layout with sidebar and canvas."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # Create splitter for resizable panels
        splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter)

        # Left: Sidebar
        control_config = self._gui_config.get("control_panel", [])
        self._sidebar = Sidebar(control_config)
        splitter.addWidget(self._sidebar)

        # Vertical separator line
        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.VLine)
        separator.setFrameShadow(QFrame.Shadow.Sunken)
        splitter.addWidget(separator)

        # Right: Canvas
        self._canvas = MVPCanvas(self._ws_config)
        splitter.addWidget(self._canvas)

        # Configure splitter stretch factors
        splitter.setStretchFactor(0, 0)  # Sidebar: fixed
        splitter.setStretchFactor(1, 0)  # Separator: fixed
        splitter.setStretchFactor(2, 1)  # Canvas: expandable

        # Set initial sizes (sidebar ~300px, rest for canvas)
        splitter.setSizes([300, 2, 700])

        # Configure canvas drawing mode from config
        canvas_config = self._gui_config.get("canvas", {})
        if canvas_config.get("draw_curve_callback", False):
            self._canvas.enable_curve_drawing(True)
        if canvas_config.get("click_canvas_callback", False):
            self._canvas.enable_canvas_click(True)

    def _connect_signals(self) -> None:
        """Connect canvas signals to internal handlers."""
        self._canvas.canvas_clicked.connect(self._on_canvas_click)
        self._canvas.car_clicked.connect(self._on_car_click)
        self._canvas.curve_drawn.connect(self._on_curve_drawn)

    # -------------------------------------------------------------------------
    # Callback Registration API
    # -------------------------------------------------------------------------

    def register_callback(self, callback_name: str, func: Callable) -> None:
        """
        Register a callback function for a control or canvas event.

        Args:
            callback_name: Name matching config's callback_name or predefined events:
                - Control panel widgets: as defined in config
                - Canvas events: "on_canvas_click", "on_car_click", "on_curve_drawn"
            func: Callback function to invoke
        """
        self._callbacks[callback_name] = func

        # Also register with sidebar if it's a control panel callback
        self._sidebar.set_callback(callback_name, func)

    # -------------------------------------------------------------------------
    # Canvas Event Handlers
    # -------------------------------------------------------------------------

    def _on_canvas_click(self, x: float, y: float) -> None:
        """Handle canvas click event."""
        callback = self._callbacks.get("on_canvas_click")
        if callback:
            callback(x, y)

    def _on_car_click(self, car_id: int) -> None:
        """Handle car click event."""
        self._selected_car_id = car_id

        # Update inspector if we have the car's state
        if car_id in self._car_states:
            self._sidebar.update_inspector(self._car_states[car_id])

        # Notify external callback
        callback = self._callbacks.get("on_car_click")
        if callback:
            callback(car_id)

    def _on_curve_drawn(self, points: List[tuple]) -> None:
        """Handle curve drawn event."""
        callback = self._callbacks.get("on_curve_drawn")
        if callback:
            callback(points)

    # -------------------------------------------------------------------------
    # Update API (Thread-Safe)
    # -------------------------------------------------------------------------

    def update(
        self,
        world_state: Dict[int, CarState],
        additional_drawings: Optional[List[Dict[str, Any]]] = None,
    ) -> None:
        """
        Update the GUI with new state.

        This method is thread-safe and can be called from the logic thread.

        Args:
            world_state: Dictionary mapping car_id to CarState
            additional_drawings: List of drawing specifications for the canvas
        """
        if additional_drawings is None:
            additional_drawings = []

        # Convert CarState dict to serializable format for signal
        # (signals require JSON-serializable data in some Qt configurations)
        state_dict = {
            car_id: {
                "car_id": state.car_id,
                "x": state.x,
                "y": state.y,
                "theta": state.theta,
                "linear_velocity": state.linear_velocity,
                "angular_velocity": state.angular_velocity,
                "status_label": state.status_label,
                "metadata": state.metadata,
            }
            for car_id, state in world_state.items()
        }

        # Emit signal to main thread
        self._update_emitter.update_requested.emit(state_dict, additional_drawings)

    def _handle_update(
        self, state_dict: Dict[int, dict], additional_drawings: List[Dict[str, Any]]
    ) -> None:
        """Handle update on the main thread."""
        # Reconstruct CarState objects
        car_states = {}
        for car_id, data in state_dict.items():
            state = CarState(
                car_id=data["car_id"],
                x=data["x"],
                y=data["y"],
                theta=data["theta"],
                linear_velocity=data["linear_velocity"],
                angular_velocity=data["angular_velocity"],
                status_label=data["status_label"],
                metadata=data["metadata"],
            )
            car_states[car_id] = state

        # Cache states
        self._car_states = car_states

        # Update canvas
        self._canvas.update_cars(car_states)
        self._canvas.update_drawings(additional_drawings)

        # Update inspector if a car is selected
        if self._selected_car_id is not None and self._selected_car_id in car_states:
            self._sidebar.update_inspector(car_states[self._selected_car_id])

    def update_car_inspector(self, car_id: int) -> None:
        """
        Manually update the car inspector to show a specific car.

        Args:
            car_id: ID of the car to display
        """
        self._selected_car_id = car_id
        if car_id in self._car_states:
            self._sidebar.update_inspector(self._car_states[car_id])

    # -------------------------------------------------------------------------
    # Main Loop
    # -------------------------------------------------------------------------

    def run(self) -> int:
        """
        Start the GUI main loop.

        This method blocks until the window is closed.

        Returns:
            Application exit code
        """
        self.show()
        return self._app.exec()

    def close_window(self) -> None:
        """Programmatically close the window."""
        self.close()
