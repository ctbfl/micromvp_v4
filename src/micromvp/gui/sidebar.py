"""
Sidebar components for MicroMVP GUI.

Contains:
- ControlPanel: Dynamic control panel generated from config
- CarInspector: Car state display panel
"""

from typing import Any, Callable, Dict, List, Optional

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QFormLayout,
    QFrame,
    QGroupBox,
    QLabel,
    QScrollArea,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from micromvp.core.models import CarState

from .widgets import create_widget_from_config


class ControlPanel(QScrollArea):
    """
    Dynamic control panel that generates widgets from configuration.

    Supports scrolling when content exceeds available space.
    """

    def __init__(self, config: List[Dict[str, Any]], parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._config = config
        self._widgets: Dict[str, QWidget] = {}  # callback_name -> widget

        # Scroll area setup
        self.setWidgetResizable(True)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.setFrameShape(QFrame.Shape.NoFrame)

        # Container widget
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)

        # Build widgets from config
        for item_config in config:
            widget, callback_name = create_widget_from_config(item_config)
            layout.addWidget(widget)

            if callback_name:
                self._widgets[callback_name] = widget

        layout.addStretch()
        self.setWidget(container)

    def set_callback(self, callback_name: str, func: Callable) -> bool:
        """
        Bind a callback function to a widget.

        Returns:
            True if callback was bound successfully, False if widget not found.
        """
        widget = self._widgets.get(callback_name)
        if not widget:
            return False

        if hasattr(widget, "set_callback"):
            widget.set_callback(func)
            return True

        return False


class CarInspector(QGroupBox):
    """
    Panel displaying detailed information about the selected car.

    Shows all fields from CarState including metadata.
    """

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__("Car Inspector", parent)
        self._current_car_id: Optional[int] = None

        # Main layout
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(8, 12, 8, 8)

        # Scroll area for content
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        # Content container
        self._content = QWidget()
        self._form_layout = QFormLayout(self._content)
        self._form_layout.setContentsMargins(0, 0, 0, 0)
        self._form_layout.setSpacing(4)

        scroll.setWidget(self._content)
        main_layout.addWidget(scroll)

        # Field labels
        self._fields: Dict[str, QLabel] = {}

        # Initialize with empty state
        self._init_fields()
        self.clear()

    def _init_fields(self) -> None:
        """Initialize the standard field labels."""
        field_names = [
            ("car_id", "ID"),
            ("x", "X"),
            ("y", "Y"),
            ("theta", "Theta (deg)"),
            ("linear_velocity", "Lin Vel (v)"),
            ("angular_velocity", "Ang Vel (ω)"),
            ("status_label", "Status"),
        ]

        for field_key, label_text in field_names:
            value_label = QLabel("-")
            value_label.setStyleSheet("color: #0066cc;")
            self._fields[field_key] = value_label
            self._form_layout.addRow(f"{label_text}:", value_label)

        # Separator for metadata
        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.HLine)
        separator.setFrameShadow(QFrame.Shadow.Sunken)
        self._form_layout.addRow(separator)

        # Metadata header
        self._metadata_header = QLabel("Metadata")
        self._metadata_header.setStyleSheet("font-weight: bold; color: #666;")
        self._form_layout.addRow(self._metadata_header)

    def clear(self) -> None:
        """Clear the inspector display."""
        self._current_car_id = None
        for label in self._fields.values():
            label.setText("-")

        # Clear metadata rows (keep standard fields)
        self._clear_metadata_rows()
        self.setTitle("Car Inspector")

    def _clear_metadata_rows(self) -> None:
        """Remove dynamic metadata rows."""
        # Keep only the standard rows (field_count + separator + header)
        standard_row_count = len(self._fields) + 2
        while self._form_layout.rowCount() > standard_row_count:
            self._form_layout.removeRow(standard_row_count)

    def update_state(self, state: CarState) -> None:
        """Update the inspector with a car's state."""
        self._current_car_id = state.car_id
        self.setTitle(f"Car Inspector - #{state.car_id}")

        # Update standard fields
        self._fields["car_id"].setText(str(state.car_id))
        self._fields["x"].setText(f"{state.x:.2f}")
        self._fields["y"].setText(f"{state.y:.2f}")
        self._fields["theta"].setText(f"{state.theta:.1f}")
        self._fields["linear_velocity"].setText(f"{state.linear_velocity:.3f}")
        self._fields["angular_velocity"].setText(f"{state.angular_velocity:.3f}")
        self._fields["status_label"].setText(state.status_label)

        # Update metadata
        self._clear_metadata_rows()
        for key, value in state.metadata.items():
            value_label = QLabel(self._format_value(value))
            value_label.setStyleSheet("color: #666;")
            value_label.setWordWrap(True)
            self._form_layout.addRow(f"{key}:", value_label)

    def _format_value(self, value: Any) -> str:
        """Format a metadata value for display."""
        if isinstance(value, float):
            return f"{value:.4f}"
        elif isinstance(value, (list, tuple)):
            if len(value) <= 3:
                return str(value)
            return f"[{len(value)} items]"
        elif isinstance(value, dict):
            return f"{{{len(value)} keys}}"
        return str(value)

    @property
    def current_car_id(self) -> Optional[int]:
        """Get the currently displayed car ID."""
        return self._current_car_id


class Sidebar(QWidget):
    """
    Complete sidebar containing control panel and car inspector.
    """

    MIN_WIDTH = 250
    PREFERRED_WIDTH = 300

    def __init__(
        self,
        control_config: List[Dict[str, Any]],
        parent: Optional[QWidget] = None,
    ):
        super().__init__(parent)
        self.setMinimumWidth(self.MIN_WIDTH)
        self.setMaximumWidth(400)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(8)

        # Control Panel (top, expandable)
        self._control_panel = ControlPanel(control_config)
        layout.addWidget(self._control_panel, stretch=1)

        # Car Inspector (bottom, fixed height)
        self._car_inspector = CarInspector()
        self._car_inspector.setMinimumHeight(200)
        self._car_inspector.setMaximumHeight(350)
        layout.addWidget(self._car_inspector)

    @property
    def control_panel(self) -> ControlPanel:
        """Access the control panel."""
        return self._control_panel

    @property
    def car_inspector(self) -> CarInspector:
        """Access the car inspector."""
        return self._car_inspector

    def set_callback(self, callback_name: str, func: Callable) -> bool:
        """Register a callback with the control panel."""
        return self._control_panel.set_callback(callback_name, func)

    def update_inspector(self, state: CarState) -> None:
        """Update the car inspector with new state."""
        self._car_inspector.update_state(state)

    def clear_inspector(self) -> None:
        """Clear the car inspector."""
        self._car_inspector.clear()
