"""
Custom control widgets for the MicroMVP GUI control panel.

Supports: Label, Toggle, Discrete Slider, Continuous Slider, Options (Dropdown), Input
"""

from typing import Any, Callable, List, Optional

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QSlider,
    QVBoxLayout,
    QWidget,
)


class LabelWidget(QWidget):
    """Display-only label widget."""

    def __init__(self, text: str, parent: Optional[QWidget] = None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 8, 0, 8)
        label = QLabel(text)
        label.setStyleSheet("font-weight: bold; color: #333;")
        layout.addWidget(label)


class ToggleWidget(QWidget):
    """Toggle (checkbox) widget with callback support."""

    def __init__(
        self,
        label: str,
        callback: Optional[Callable[[bool], None]] = None,
        default: bool = False,
        parent: Optional[QWidget] = None,
    ):
        super().__init__(parent)
        self._callback = callback

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 4, 0, 4)

        self._checkbox = QCheckBox(label)
        self._checkbox.setChecked(default)
        self._checkbox.stateChanged.connect(self._on_state_changed)
        layout.addWidget(self._checkbox)

    def _on_state_changed(self, state: int) -> None:
        if self._callback:
            self._callback(state == Qt.CheckState.Checked.value)

    def set_callback(self, callback: Callable[[bool], None]) -> None:
        self._callback = callback


class DiscreteSliderWidget(QWidget):
    """Discrete slider with predefined tier values."""

    def __init__(
        self,
        label: str,
        tiers: List[Any],
        callback: Optional[Callable[[Any], None]] = None,
        default_idx: int = 0,
        parent: Optional[QWidget] = None,
    ):
        super().__init__(parent)
        self._callback = callback
        self._tiers = tiers

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 4, 0, 4)

        # Label row with current value
        label_row = QHBoxLayout()
        label_row.addWidget(QLabel(label))
        self._value_label = QLabel(str(tiers[default_idx]))
        self._value_label.setStyleSheet("color: #0066cc;")
        label_row.addStretch()
        label_row.addWidget(self._value_label)
        layout.addLayout(label_row)

        # Slider
        self._slider = QSlider(Qt.Orientation.Horizontal)
        self._slider.setMinimum(0)
        self._slider.setMaximum(len(tiers) - 1)
        self._slider.setValue(default_idx)
        self._slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self._slider.setTickInterval(1)
        self._slider.valueChanged.connect(self._on_value_changed)
        layout.addWidget(self._slider)

    def _on_value_changed(self, index: int) -> None:
        value = self._tiers[index]
        self._value_label.setText(str(value))
        if self._callback:
            self._callback(value)

    def set_callback(self, callback: Callable[[Any], None]) -> None:
        self._callback = callback


class ContinuousSliderWidget(QWidget):
    """Continuous slider with float range."""

    STEPS = 100  # Internal slider resolution

    def __init__(
        self,
        label: str,
        value_range: List[float],
        callback: Optional[Callable[[float], None]] = None,
        default: Optional[float] = None,
        parent: Optional[QWidget] = None,
    ):
        super().__init__(parent)
        self._callback = callback
        self._min_val = value_range[0]
        self._max_val = value_range[1]

        if default is None:
            default = self._min_val

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 4, 0, 4)

        # Label row with current value
        label_row = QHBoxLayout()
        label_row.addWidget(QLabel(label))
        self._value_label = QLabel(f"{default:.2f}")
        self._value_label.setStyleSheet("color: #0066cc;")
        label_row.addStretch()
        label_row.addWidget(self._value_label)
        layout.addLayout(label_row)

        # Slider
        self._slider = QSlider(Qt.Orientation.Horizontal)
        self._slider.setMinimum(0)
        self._slider.setMaximum(self.STEPS)
        self._slider.setValue(self._value_to_step(default))
        self._slider.valueChanged.connect(self._on_value_changed)
        layout.addWidget(self._slider)

    def _value_to_step(self, value: float) -> int:
        ratio = (value - self._min_val) / (self._max_val - self._min_val)
        return int(ratio * self.STEPS)

    def _step_to_value(self, step: int) -> float:
        ratio = step / self.STEPS
        return self._min_val + ratio * (self._max_val - self._min_val)

    def _on_value_changed(self, step: int) -> None:
        value = self._step_to_value(step)
        self._value_label.setText(f"{value:.2f}")
        if self._callback:
            self._callback(value)

    def set_callback(self, callback: Callable[[float], None]) -> None:
        self._callback = callback


class OptionsWidget(QWidget):
    """Dropdown options widget."""

    def __init__(
        self,
        label: str,
        options: List[Any],
        callback: Optional[Callable[[Any], None]] = None,
        default: Optional[Any] = None,
        parent: Optional[QWidget] = None,
    ):
        super().__init__(parent)
        self._callback = callback
        self._options = options

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 4, 0, 4)

        layout.addWidget(QLabel(label))

        self._combo = QComboBox()
        for opt in options:
            self._combo.addItem(str(opt))

        if default is not None and default in options:
            self._combo.setCurrentIndex(options.index(default))

        self._combo.currentIndexChanged.connect(self._on_index_changed)
        layout.addWidget(self._combo, 1)

    def _on_index_changed(self, index: int) -> None:
        if self._callback and 0 <= index < len(self._options):
            self._callback(self._options[index])

    def set_callback(self, callback: Callable[[Any], None]) -> None:
        self._callback = callback


class InputWidget(QWidget):
    """Text input widget that triggers callback on Enter."""

    def __init__(
        self,
        label: str,
        callback: Optional[Callable[[str], None]] = None,
        placeholder: str = "",
        parent: Optional[QWidget] = None,
    ):
        super().__init__(parent)
        self._callback = callback

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 4, 0, 4)

        layout.addWidget(QLabel(label))

        self._input = QLineEdit()
        self._input.setPlaceholderText(placeholder)
        self._input.returnPressed.connect(self._on_return_pressed)
        layout.addWidget(self._input, 1)

    def _on_return_pressed(self) -> None:
        if self._callback:
            self._callback(self._input.text())
            self._input.clear()

    def set_callback(self, callback: Callable[[str], None]) -> None:
        self._callback = callback


def create_widget_from_config(config: dict) -> tuple[QWidget, Optional[str]]:
    """
    Factory function to create a widget from configuration dict.

    Returns:
        Tuple of (widget, callback_name or None)
    """
    widget_type = config.get("type", "").lower()
    callback_name = config.get("callback_name")

    if widget_type == "label":
        return LabelWidget(config.get("text", "")), None

    elif widget_type == "toggle":
        return ToggleWidget(
            label=config.get("label", ""),
            default=config.get("default", False),
        ), callback_name

    elif widget_type == "discrete_slider":
        tiers = config.get("tiers", [1])
        return DiscreteSliderWidget(
            label=config.get("label", ""),
            tiers=tiers,
            default_idx=config.get("default_idx", 0),
        ), callback_name

    elif widget_type == "continuous_slider":
        value_range = config.get("range", [0.0, 1.0])
        return ContinuousSliderWidget(
            label=config.get("label", ""),
            value_range=value_range,
            default=config.get("default"),
        ), callback_name

    elif widget_type == "options":
        return OptionsWidget(
            label=config.get("label", ""),
            options=config.get("options", []),
            default=config.get("default"),
        ), callback_name

    elif widget_type == "input":
        return InputWidget(
            label=config.get("label", ""),
            placeholder=config.get("placeholder", ""),
        ), callback_name

    else:
        # Unknown widget type, return empty label
        return LabelWidget(f"[Unknown: {widget_type}]"), None
