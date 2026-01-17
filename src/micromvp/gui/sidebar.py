"""
Sidebar components for MicroMVP GUI.

Contains:
- ControlPanel: Dynamic control panel generated from config
- CarInspector: Car state display panel
"""

from typing import Any, Callable, Dict, List, Optional, Tuple

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
    QSplitter,
)

from micromvp.core.models import CarState

from .widgets import DynamicLabelWidget, create_widget_from_config


class ControlPanel(QScrollArea):
    """
    Dynamic control panel that generates widgets from configuration.

    Supports scrolling when content exceeds available space.
    """

    def __init__(self, config: List[Dict[str, Any]], parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._config = config
        self._widgets: Dict[str, QWidget] = {}  # callback_name -> widget
        self._dynamic_labels: Dict[str, DynamicLabelWidget] = {}  # widget_name -> DynamicLabelWidget

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
            widget, name = create_widget_from_config(item_config)
            layout.addWidget(widget)

            # Store widget by name for callback binding or text updates
            if name:
                if isinstance(widget, DynamicLabelWidget):
                    self._dynamic_labels[name] = widget
                else:
                    self._widgets[name] = widget

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

    def update_widget_text(self, widget_name: str, text: str) -> bool:
        """
        Update the text of a DynamicLabelWidget.

        Args:
            widget_name: Name of the widget (from config's widget_name)
            text: New text to display

        Returns:
            True if widget was updated successfully, False if widget not found.
        """
        widget = self._dynamic_labels.get(widget_name)
        if widget:
            widget.set_text(text)
            return True
        return False


class CarInspector(QGroupBox):
    """
    Panel displaying detailed information about the selected car.

    Shows all fields from CarState including metadata.
    """

    HIDDEN_TEXT = "<hidden>"

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__("Car Inspector", parent)
        self._current_car_id: Optional[int] = None

        # metadata layout bookkeeping
        self._metadata_keys: Tuple[str, ...] = ()
        self._metadata_value_labels: Dict[str, QLabel] = {}

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

        # Standard field labels
        self._fields: Dict[str, QLabel] = {}

        # Initialize with empty state
        self._init_fields()
        self.clear()

    def _init_fields(self) -> None:
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
        self._separator = QFrame()
        self._separator.setFrameShape(QFrame.Shape.HLine)
        self._separator.setFrameShadow(QFrame.Shadow.Sunken)
        self._form_layout.addRow(self._separator)

        # Metadata header
        self._metadata_header = QLabel("Metadata")
        self._metadata_header.setStyleSheet("font-weight: bold; color: #666;")
        self._form_layout.addRow(self._metadata_header)

    def clear(self) -> None:
        self._current_car_id = None
        for label in self._fields.values():
            label.setText("-")

        self._clear_metadata_rows()
        self.setTitle("Car Inspector")

    def update_state(self, state: CarState) -> None:
        car_changed = (self._current_car_id is None) or (self._current_car_id != state.car_id)
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

        # Build metadata rows only when selecting a new car
        if car_changed:
            self._build_metadata_schema(state.metadata)

        # Update metadata values only (no flashing keys)
        for key in self._metadata_keys:
            value = state.metadata.get(key, None)
            self._metadata_value_labels[key].setText(self._format_value(value))

    def _build_metadata_schema(self, metadata: Dict[str, Any]) -> None:
        """
        Rebuild metadata rows once when a new car is selected.
        Assumption: for a given car, keys are stable afterwards.
        """
        self._clear_metadata_rows()

        keys = list(metadata.keys())
        # keep original insertion order from dict; if you prefer stable sort, uncomment:
        # keys = sorted(keys)

        self._metadata_keys = tuple(keys)
        self._metadata_value_labels = {}

        for key in self._metadata_keys:
            value_label = QLabel("-")
            value_label.setStyleSheet("color: #666;")
            value_label.setWordWrap(True)
            self._metadata_value_labels[key] = value_label
            self._form_layout.addRow(f"{key}:", value_label)

    def _clear_metadata_rows(self) -> None:
        """Remove all metadata rows (below the header)."""
        # Standard rows = len(fields) + separator + header
        standard_row_count = len(self._fields) + 2
        while self._form_layout.rowCount() > standard_row_count:
            self._form_layout.removeRow(standard_row_count)

        self._metadata_keys = ()
        self._metadata_value_labels = {}

    def _is_simple(self, value: Any) -> bool:
        """Simple enough to display directly."""
        if value is None:
            return True
        return isinstance(value, (bool, int, float, str))

    def _format_value(self, value: Any) -> str:
        """Format metadata value; complex types become <hidden>."""
        if not self._is_simple(value):
            return self.HIDDEN_TEXT

        if value is None:
            return "None"
        if isinstance(value, bool):
            return "True" if value else "False"
        if isinstance(value, float):
            return f"{value:.4f}"
        return str(value)

    @property
    def current_car_id(self) -> Optional[int]:
        return self._current_car_id


class Sidebar(QWidget):
    MIN_WIDTH = 250
    PREFERRED_WIDTH = 300

    def __init__(self, control_config, parent=None):
        super().__init__(parent)
        self.setMinimumWidth(self.MIN_WIDTH)
        self.setMaximumWidth(400)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(0)  # splitter 自己会留缝

        self._control_panel = ControlPanel(control_config)
        self._car_inspector = CarInspector()

        splitter = QSplitter(Qt.Orientation.Vertical)
        splitter.addWidget(self._control_panel)
        splitter.addWidget(self._car_inspector)

        # 可选：设初始比例（上大下小）
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        splitter.setCollapsible(0, False)
        splitter.setCollapsible(1, False)

        # 可选：允许 inspector 拉得更大
        # 重要：不要再 setMaximumHeight(350) 这种上限
        self._control_panel.setMinimumHeight(200)
        self._car_inspector.setMinimumHeight(200)

        layout.addWidget(splitter)

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

    def update_widget_text(self, widget_name: str, text: str) -> bool:
        """
        Update the text of a DynamicLabelWidget.

        Args:
            widget_name: Name of the widget (from config's widget_name)
            text: New text to display

        Returns:
            True if widget was updated successfully, False if widget not found.
        """
        return self._control_panel.update_widget_text(widget_name, text)
