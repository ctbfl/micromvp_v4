# MicroMVP GUI Module

PyQt6-based graphical interface for the MicroMVP robot control system.

## Quick Start

```python
from micromvp.gui import MVPWindow
from micromvp.core.models import WorkspaceConfig

# Define workspace configuration (typically from Env)
ws_config = WorkspaceConfig(
    width=800, height=600,
    car_width=40, car_height=60,
    offset_w=20, offset_h=30,
    wheel_base=30, max_wheel_speed=100,
    frequency=20, car_id_list=[1, 2, 3]
)

# Define GUI configuration
gui_config = {
    "canvas": {
        "draw_curve_callback": True,
        "click_canvas_callback": True
    },
    "control_panel": [
        {"type": "toggle", "label": "Enable", "callback_name": "toggle_enable", "default": True},
        {"type": "continuous_slider", "label": "Speed", "range": [0, 1], "callback_name": "set_speed"},
    ]
}

# Create and run
gui = MVPWindow(gui_config, ws_config)
gui.register_callback("toggle_enable", lambda v: print(f"Enable: {v}"))
gui.register_callback("set_speed", lambda v: print(f"Speed: {v}"))
gui.run()  # Blocks until window is closed
```

## Architecture

```
MVPWindow (QMainWindow)
├── Sidebar (left)
│   ├── ControlPanel (top, scrollable)
│   │   └── Dynamic widgets from config
│   └── CarInspector (bottom)
│       └── Selected car state display
└── MVPCanvas (right)
    ├── Workspace boundary
    ├── Car graphics items
    └── Additional drawings (UUID-based)
```

## GUI Configuration

### Canvas Config

| Key | Type | Description |
|-----|------|-------------|
| `draw_curve_callback` | bool | Enable curve drawing with mouse drag |
| `click_canvas_callback` | bool | Enable canvas click position callback |

### Control Panel Widgets

Configure widgets via a list of dicts in `gui_config["control_panel"]`:

#### Label (display only)
```python
{"type": "label", "text": "=== Section Header ==="}
```

#### Toggle (checkbox)
```python
{
    "type": "toggle",
    "label": "Enable Feature",
    "callback_name": "toggle_feature",
    "default": True  # optional, default False
}
# Callback receives: bool
```

#### Discrete Slider
```python
{
    "type": "discrete_slider",
    "label": "Speed Tier",
    "tiers": [1, 2, 5, 10],
    "callback_name": "set_tier",
    "default_idx": 0  # optional, default 0
}
# Callback receives: element from tiers list
```

#### Continuous Slider
```python
{
    "type": "continuous_slider",
    "label": "Gain",
    "range": [0.1, 2.0],
    "callback_name": "set_gain",
    "default": 0.5  # optional, defaults to range[0]
}
# Callback receives: float
```

#### Options (dropdown)
```python
{
    "type": "options",
    "label": "Pattern",
    "options": ["circle", "8_shape", "square"],
    "callback_name": "change_pattern",
    "default": "circle"  # optional
}
# Callback receives: selected option value
```

#### Input (text field)
```python
{
    "type": "input",
    "label": "Command",
    "placeholder": "Enter command...",
    "callback_name": "send_command"
}
# Callback receives: str (triggered on Enter key)
```

## API Reference

### MVPWindow

#### `__init__(gui_config, workspace_config)`
Create the main window.

#### `register_callback(callback_name, func)`
Register a callback function for control panel widgets or canvas events.

**Predefined canvas callbacks:**
- `"on_canvas_click"` - `func(x: float, y: float)` - Canvas click position (workspace coords)
- `"on_car_click"` - `func(car_id: int)` - Clicked car ID
- `"on_curve_drawn"` - `func(points: List[Tuple[float, float]])` - Drawn curve points

#### `update(world_state, additional_drawings=None)`
Thread-safe method to update the GUI from the logic thread.

```python
# world_state: Dict[int, CarState]
# additional_drawings: List[Dict] (see Drawing API below)
gui.update(car_states, drawings)
```

#### `update_car_inspector(car_id)`
Manually update the inspector to show a specific car.

#### `run() -> int`
Start the GUI main loop (blocks until window closed).

#### `close_window()`
Programmatically close the window.

## Canvas Drawing API

The canvas uses UUID-based retained mode rendering. Each drawing object is identified by a unique `uuid` string.

### Update Cycle
1. Objects with existing UUID: properties updated in place
2. Objects with new UUID: created and cached
3. Cached objects not in current list: removed

### Drawing Types

#### Line
```python
{
    "uuid": "line_1",
    "type": "line",
    "start": (x1, y1),
    "end": (x2, y2),
    "color": "#FF0000",  # optional
    "width": 2           # optional
}
```

#### Circle
```python
{
    "uuid": "circle_1",
    "type": "circle",
    "center": (cx, cy),
    "radius": 10,
    "color": "#00FF00",  # outline color
    "fill": "#00FF00",   # fill color
    "width": 2
}
```

#### Rectangle
```python
{
    "uuid": "rect_1",
    "type": "rect",
    "position": (x, y),  # top-left in workspace coords
    "size": (w, h),
    "color": "#0000FF",
    "fill": "#0000FF",
    "width": 2
}
```

#### Path (polyline)
```python
{
    "uuid": "path_1",
    "type": "path",
    "points": [(x1, y1), (x2, y2), (x3, y3), ...],
    "color": "#FF00FF",
    "width": 2
}
```

#### Point
```python
{
    "uuid": "point_1",
    "type": "point",
    "position": (x, y),
    "radius": 3,
    "color": "#FFFF00",
    "fill": "#FFFF00"
}
```

## Coordinate System

- **Workspace origin**: Bottom-left corner (0, 0)
- **Y-axis**: Points upward (mathematical convention)
- **Theta = 0**: Points to +X direction (right)
- **Units**: Defined by `WorkspaceConfig` (typically pixels or cm)

The canvas automatically handles the Y-axis flip for PyQt rendering.

## Threading Model

- **Main thread**: GUI event loop (Qt requirement)
- **Logic thread**: Control loop calling `gui.update()`

The `update()` method is thread-safe via Qt signals, allowing safe updates from the logic thread.

```python
import threading

def logic_loop():
    while True:
        # ... compute car_states and drawings ...
        gui.update(car_states, drawings)
        time.sleep(1/20)  # 20 Hz

# Start logic thread
threading.Thread(target=logic_loop, daemon=True).start()

# Run GUI on main thread (blocks)
gui.run()
```

## Car Inspector

Clicking a car on the canvas automatically updates the Car Inspector panel with:
- Car ID
- Position (x, y)
- Orientation (theta)
- Velocities (linear, angular)
- Status label
- Custom metadata from `CarState.metadata`

## Module Structure

```
gui/
├── __init__.py      # Exports MVPWindow
├── window.py        # Main window class
├── canvas.py        # Canvas with car rendering and drawing API
├── sidebar.py       # Control panel and car inspector
└── widgets.py       # Widget implementations
```

## Dependencies

- PyQt6
- micromvp.core.models (CarState, WorkspaceConfig)
- micromvp.assets/carImage.png (car sprite)
