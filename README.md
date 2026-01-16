# MicroMVP v4

A modular robotics control framework for multi-robot systems with both simulation and real hardware support.

## Overview

MicroMVP v4 provides a clean separation between environment sensing/actuation, per-robot control logic, and high-level coordination. The same controller code works seamlessly with both simulation and real ArUco-tracked robots.

```
┌─────────────────────────────────────────────────────────────┐
│                          GUI                                │
│  (visualization, user input, canvas drawing)                │
└─────────────────────────────────────────────────────────────┘
                              ↑↓
┌─────────────────────────────────────────────────────────────┐
│                       Coordinator                           │
│  (distributes observations, collects actions, GUI bridge)   │
└─────────────────────────────────────────────────────────────┘
                              ↑↓
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│ Controller 1 │  │ Controller 2 │  │ Controller N │
│ (per-robot)  │  │ (per-robot)  │  │ (per-robot)  │
└──────────────┘  └──────────────┘  └──────────────┘
                              ↑↓
┌─────────────────────────────────────────────────────────────┐
│                       Environment                           │
│  (SimEnv or RealPushEnv - observe & apply actions)          │
└─────────────────────────────────────────────────────────────┘
```

## Installation

```bash
# Clone and install in development mode
cd micromvp_v4
pip install -e .
```

**Requirements:** Python 3.12+

**Dependencies:**
- PyQt6 - GUI framework
- NumPy - Numerical computation
- OpenCV (opencv-contrib-python) - ArUco marker detection
- PyZMQ - Networking
- PySerial / PyUSB - Hardware communication

## Quick Start

### Simulation with Keyboard Control

```bash
python examples/test_sim_env_keyboard.py
```

Controls: `W/A/S/D` to move, `1-9` to select robot, `Tab` to cycle, `Space` to stop.

### Real Hardware with Path Following

```bash
python examples/test_real_env_follow_path.py --robots "1:10.0.0.100"
```

Click a robot to select, draw a curve on the canvas, and watch it follow the path.

## Project Structure

```
micromvp_v4/
├── src/micromvp/
│   ├── core/                    # Data models and utilities
│   │   ├── models.py            # WorkspaceConfig, Action, CarState, etc.
│   │   ├── patterns.py          # Path generators (circle, figure-8)
│   │   └── ddr.py               # Differential drive kinematics
│   ├── env/                     # Environment implementations
│   │   ├── base.py              # Abstract Environment interface
│   │   ├── sim_env/             # Physics simulation
│   │   └── real_push_env/       # Real hardware (ArUco + UDP)
│   ├── controller/              # Per-robot control algorithms
│   │   ├── base.py              # Abstract Controller interface
│   │   ├── WASD_controller/     # Manual keyboard control
│   │   └── follow_path_controller/  # Pure pursuit path following
│   ├── coordinator/             # High-level orchestration
│   │   ├── base.py              # Abstract Coordinator interface
│   │   ├── keyboard_coordinator/    # Multi-robot keyboard control
│   │   └── follow_path_coordinator/ # Path assignment coordination
│   └── gui/                     # PyQt6 visualization
│       ├── window.py            # Main MVPWindow
│       ├── canvas.py            # Workspace rendering
│       └── sidebar.py           # Control panel & inspector
├── examples/                    # Usage examples
└── pyproject.toml               # Package configuration
```

## Core Concepts

### Data Models (`core/models.py`)

| Class | Purpose |
|-------|---------|
| `WorkspaceConfig` | Physical workspace definition (size, car geometry, frequency) |
| `RobotObservation` | Sensor data from environment (x, y, theta, timestamp) |
| `Action` | Motor command (left_speed, right_speed in [-1, 1]) |
| `CarState` | Robot state snapshot (pose, velocity, status, metadata) |

### Environment

The environment handles sensing and actuation without knowing about controllers.

```python
from micromvp.env import SimEnv, SimConfig

config = SimConfig(
    width=800, height=600,
    initial_poses=[(1, 100, 100, 0), (2, 200, 200, 90)]
)
env = SimEnv(config)

# Main loop
observations = env.observe()          # Get robot poses
env.apply_actions({1: Action(0.5, 0.5)})  # Send wheel commands
```

**Implementations:**
- **SimEnv** - In-memory physics simulation
- **RealPushEnv** - ArUco vision + UDP action transmission

### Controller

Each robot has its own controller instance managing its state and computing actions.

```python
from micromvp.controller import WASDController, FollowPathController

# Manual control
controller = WASDController(robot_id=1, ws_config=ws_config)
controller.set_keys({"w"})  # Forward
action = controller.step(observation)

# Path following
controller = FollowPathController(robot_id=1, ws_config=ws_config, max_speed=0.3)
controller.set_path([(100, 100), (200, 150), (300, 100)])
action = controller.step(observation)
```

**Status Labels:**
- WASDController: `IDLE`, `FORWARD`, `BACKWARD`, `TURN_LEFT`, `TURN_RIGHT`, etc.
- FollowPathController: `IDLE`, `FOLLOWING`, `FINISHED`

### Coordinator

The coordinator bridges environment, controllers, and GUI.

```python
from micromvp.coordinator import KeyboardCoordinator, FollowPathCoordinator

coordinator = KeyboardCoordinator(ws_config, controllers)

# Main loop
actions = coordinator.process(observations)  # Distribute obs, collect actions
car_states = coordinator.gather_car_state()  # For GUI rendering
drawings = coordinator.get_additional_drawings()  # Custom visualizations
```

### GUI

PyQt6-based visualization with configurable control panel.

```python
from micromvp.gui import MVPWindow

gui_config = {
    "canvas": {
        "click_canvas_callback": True,
        "draw_curve_callback": True,
    },
    "control_panel": [
        {"type": "label", "text": "=== Controls ==="},
        {"type": "toggle", "label": "Enable", "callback_name": "toggle_enable"},
        {"type": "continuous_slider", "label": "Speed", "range": [0, 1], "callback_name": "set_speed"},
    ]
}

gui = MVPWindow(gui_config, ws_config)
gui.register_callback("on_key_press", coordinator.on_key_press)
gui.register_callback("on_curve_drawn", coordinator.on_curve_drawn)
gui.run()  # Blocks until closed
```

**Canvas Drawing API:**
```python
drawings = [
    {"uuid": "path_1", "type": "path", "points": [(x1,y1), (x2,y2)], "color": "#00FF00", "width": 2},
    {"uuid": "target", "type": "point", "position": (x, y), "radius": 5, "color": "#FF0000", "fill": "#FF0000"},
    {"uuid": "line_1", "type": "line", "start": (x1,y1), "end": (x2,y2), "color": "#0000FF"},
    {"uuid": "circle_1", "type": "circle", "center": (cx,cy), "radius": 10, "color": "green"},
]
gui.update(car_states, drawings)
```

## Examples

| Example | Description |
|---------|-------------|
| `test_sim_env.py` | Minimal simulation without GUI |
| `test_empty_gui.py` | GUI widgets and canvas without environment |
| `test_sim_env_keyboard.py` | Simulation with keyboard control |
| `test_real_env_keyboard.py` | Real hardware with keyboard control |
| `test_real_env_follow_path.py` | Real hardware with path following |

## Full Example: Path Following

```python
import threading
import time
from micromvp.env import RealPushEnv, RealPushConfig
from micromvp.controller import FollowPathController
from micromvp.coordinator import FollowPathCoordinator
from micromvp.gui import MVPWindow

# Setup environment
config = RealPushConfig(robot_endpoints=[(1, "10.0.0.100", 9001)])
env = RealPushEnv(config)
env.start(wait_for_ready=True)
ws_config = env.workspace_config

# Setup controllers and coordinator
controllers = {rid: FollowPathController(rid, ws_config, max_speed=0.3)
               for rid in ws_config.car_id_list}
coordinator = FollowPathCoordinator(ws_config, controllers)

# Setup GUI
gui_config = {"canvas": {"draw_curve_callback": True, "click_canvas_callback": True}}
gui = MVPWindow(gui_config, ws_config)
gui.register_callback("on_car_click", coordinator.on_car_click)
gui.register_callback("on_curve_drawn", coordinator.on_curve_drawn)
gui.register_callback("on_key_press", coordinator.on_key_press)

# Logic loop (separate thread)
running = True
def logic_loop():
    while running:
        observations = env.observe()
        if observations:
            actions = coordinator.process(observations)
            env.apply_actions(actions)
            car_states = {s.car_id: s for s in coordinator.gather_car_state()}
            gui.update(car_states, coordinator.get_additional_drawings())
        time.sleep(1 / ws_config.frequency)

threading.Thread(target=logic_loop, daemon=True).start()
gui.run()
env.close()
```

## Real Hardware Setup

### Robot Configuration

Robots communicate via UDP. Each robot needs:
- Unique ArUco marker (4x4 dictionary)
- WiFi connection to the network
- UDP listener on configured port (default: 9001)

### Camera Setup

1. Place 5x5 ArUco markers to define the workspace boundary
2. Calibrate camera and save to `camera.yaml`
3. Position camera to see entire workspace

### Running

```bash
# Single robot
python examples/test_real_env_follow_path.py --robots "1:10.0.0.100"

# Multiple robots
python examples/test_real_env_follow_path.py --robots "1:10.0.0.100,2:10.0.0.101"

# Custom settings
python examples/test_real_env_follow_path.py \
    --robots "1:10.0.0.100" \
    --max-speed 0.2 \
    --lookahead 50 \
    --calib /path/to/camera.yaml
```

## Extending the Framework

### Custom Controller

```python
from micromvp.controller.base import Controller
from micromvp.core.models import Action, RobotObservation

class MyController(Controller):
    def step(self, observation: RobotObservation) -> Action:
        self.update(observation)
        return self.calculate_action()

    def update(self, observation: RobotObservation) -> None:
        self._car_state.x = observation.x
        self._car_state.y = observation.y
        self._car_state.theta = observation.theta

    def calculate_action(self) -> Action:
        # Your control logic here
        return Action(left_speed=0.0, right_speed=0.0)
```

### Custom Coordinator

```python
from micromvp.coordinator.base import Coordinator

class MyCoordinator(Coordinator):
    def process(self, observations):
        actions = {}
        for robot_id, controller in self._controllers.items():
            if robot_id in observations:
                actions[robot_id] = controller.step(observations[robot_id])
        return actions

    def get_additional_drawings(self):
        # Return custom visualizations
        return [{"uuid": "my_drawing", "type": "circle", ...}]
```

## Coordinate System

- **Origin**: Bottom-left corner (0, 0)
- **X-axis**: Points right (positive)
- **Y-axis**: Points up (positive)
- **Theta**: 0° = +X direction, counter-clockwise positive
- **Units**: Defined by WorkspaceConfig (typically pixels or cm)
