# microMVP (modern rewrite)

Multi-robot control system for differential-drive cars with vision tracking.

## Install

Create/activate your conda env, then:

```bash
python -m pip install -e .
```

## Quick Start

### Run GUI (Simulation)

```bash
micromvp --sim
```

### Run Examples

The `examples/` folder contains detailed API usage demonstrations:

```bash
# Example 0: Single car goto target (simplest API demo)
python examples/0_go_to_target.py

# Example 1: Multiple cars following circle pattern
python examples/1_run_circle_loop.py

# Example 2: Figure-8 pattern with collision avoidance
python examples/2_run_8shape_loop.py

# Example 3: Custom open paths (non-looping)
python examples/3_run_open_routine.py
```

All examples support `--headless` mode for console-only output:

```bash
python examples/1_run_circle_loop.py --headless --cars 5 --duration 30
```

## Architecture

The system follows a clean three-layer architecture:

```
GUI (PyQt6)  ←→  Coordinator  ←→  [Car, Car, ...]
                     ↓
              Environment (Sim/Real)
```

- **Environment**: World adapter (observe/apply_actions)
- **Car**: Independent agent with controller (goto/follow_path)
- **Coordinator**: Optional multi-car orchestration

## API Usage

### Direct Car Control (Simple)

```python
from micromvp.core.car import Car
from micromvp.core.models import CarConfig
from micromvp.env.sim_env import SimConfig, SimEnvironment

# Create environment and car
env = SimEnvironment(SimConfig(), {1: (100, 100, 0)})
car = Car(CarConfig.from_wheel_base(1, 1, 30.0))

# Command car to target
car.goto(x=500, y=400, theta=1.57)

# Control loop
while not car.is_task_done:
    obs = env.observe()
    action = car.get_action(obs[1])
    env.apply_actions({1: action})
```

### Multi-Car with Coordinator

```python
from micromvp.core.coordinator import Coordinator, CoordinatorConfig
from micromvp.core.models import CarConfig
from micromvp.env.sim_env import SimConfig, SimEnvironment

# Create environment with multiple cars
initial_poses = {1: (100, 100, 0), 2: (200, 100, 0)}
env = SimEnvironment(SimConfig(), initial_poses)

# Create coordinator
car_configs = [CarConfig.from_wheel_base(i, i, 30.0) for i in [1, 2]]
coordinator = Coordinator(env, car_configs)

# Set paths and run
coordinator.set_paths([path1, path2], loop=True)
coordinator.start()

# Wait for completion or run with GUI
coordinator.wait_until_done(timeout=30.0)
coordinator.stop()
```

## Real Hardware

### 1. Start Vision Publisher

```bash
python -m micromvp.vision.aruco_publisher 0 5556 --dict DICT_4X4_50
```

### 2. Run Controller

```bash
micromvp --real --vision-sub tcp://localhost:5556 --cars 1,2,3
```

## Configuration

Key parameters in `AppConfig`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sim_speed` | 100.0 | Simulation speed multiplier |
| `wheel_base` | 30.0 | Distance between wheels (pixels) |
| `v_max` | 1.0 | Maximum normalized speed |
| `painter_size` | (1280, 720) | Canvas size |

## Project Structure

```
src/micromvp/
├── core/
│   ├── car.py           # Independent car agent
│   ├── coordinator.py   # Multi-car orchestration
│   ├── models.py        # Data models
│   ├── ddr.py           # Differential drive kinematics
│   └── patterns.py      # Circle, figure-8 patterns
├── env/
│   ├── base.py          # Environment ABC
│   ├── sim_env.py       # Simulation
│   └── real_env.py      # Real hardware
├── io/
│   ├── aruco_observer.py
│   └── wifi_sink.py
├── ui/
│   ├── qt_app.py
│   └── qt_canvas.py
└── main.py
```

## Notes

- UI is built with PyQt6
- ZMQ message format: `<id> x0 y0 x1 y1 x2 y2 x3 y3`
- Hardware support isolated in `micromvp/io`
- WiFi: SSID `microMVP_AP`, password `12345678`
- Demo mode does not require hardware

Since the workspace would vary between different environment, the workspace size should be provided by the enviroment. And the GUI would construct the canvas accordingly.