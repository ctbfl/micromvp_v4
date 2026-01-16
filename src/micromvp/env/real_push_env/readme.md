# Real Push Environment

Real-world environment for the 1st Spring St robot setup.

## Overview

This module provides hardware interface for:
- **Observation**: ArUco marker detection via camera
- **Action**: UDP commands to robot wheel motors

## Workspace Setup

The workspace uses ArUco markers for pose estimation:
- **Workspace markers**: 5x5 ArUco dictionary, placed at fixed positions on table
- **Robot markers**: 4x4 ArUco dictionary, mounted on top of each robot
- **Workspace size**: 40cm x 60cm

### Workspace Marker Layout

```
(0,56)---[9]------------------[11]---(36,56)
  |                                    |
  |           [5] (18,49)              |
  |                                    |
  |   [1] (10,36)    [3] (26,36)       |
  |                                    |
(0,28)---[4]------------------[6]---(36,28)
  |                                    |
  |   [0] (10,20)    [2] (26,20)       |
  |                                    |
  |           [7] (18,7)               |
  |                                    |
(0,0)----[8]------------------[10]---(36,0)
```

Each marker is 4cm x 4cm.

## Quick Start

```python
from micromvp.env import RealPushEnv, RealPushConfig

# Configure environment
config = RealPushConfig(
    robot_endpoints=[
        (1, "10.0.0.100", 9001),  # Robot 1
        (2, "10.0.0.101", 9001),  # Robot 2
    ],
    warmup_frames=30,  # Use warmup for stable workspace pose
)

# Create and start environment
env = RealPushEnv(config)
env.start()

# Main loop
while running:
    observations = env.observe()
    actions = compute_actions(observations)
    env.apply_actions(actions)

# Cleanup
env.close()
```

## Configuration

### RealPushConfig

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_endpoints` | `[]` | List of (robot_id, ip, port) tuples |
| `camera_device` | `0` | Camera device index |
| `camera_resolution` | `"720p"` | Resolution: "480p", "720p", "1080p" |
| `calibration_file` | `...camera.yaml` | Camera calibration file path |
| `warmup_frames` | `30` | Frames for warmup (0=dynamic mode) |
| `show_preview` | `True` | Show camera preview window |
| `send_hz` | `50.0` | Action command frequency |
| `invert_right_wheel` | `True` | Invert right wheel direction |

## Observation Modes

### Warmup Mode (Recommended)

When `warmup_frames > 0`, the observer:
1. Collects N frames during startup
2. Selects the frame with best workspace marker detection
3. Fixes the workspace pose for subsequent frames
4. Only detects robot markers during runtime (faster)

### Dynamic Mode

When `warmup_frames = 0`:
- Workspace pose is re-estimated every frame
- More robust to camera movement
- Higher computational cost

## Robot Protocol

### UDP Command Format

```
Packet: struct.pack("<Hhh", seq, left_milli, right_milli)
- seq: uint16, sequence number (wraps at 65535)
- left_milli: int16, left wheel speed (-1000 to 1000)
- right_milli: int16, right wheel speed (-1000 to 1000)
```

Speed values are normalized:
- `-1000` = full reverse
- `0` = stop
- `1000` = full forward

## Components

### ArucoObserver

Standalone camera observer for robot pose detection.

```python
from micromvp.env.real_push_env import ArucoObserver, ObserverConfig

config = ObserverConfig(
    camera_device=0,
    warmup_frames=30,
)
observer = ArucoObserver(config)
observer.start()

# Get observations
obs = observer.get_observations()
for car_id, car_obs in obs.items():
    print(f"Car {car_id}: ({car_obs.x_cm}, {car_obs.y_cm}) @ {car_obs.yaw_deg}°")

observer.stop()
```

### UDPActionSender

Standalone action sender for robot control.

```python
from micromvp.env.real_push_env import UDPActionSender, UDPActionConfig, RobotEndpoint
from micromvp.core.models import Action

config = UDPActionConfig(
    endpoints=[RobotEndpoint(1, "10.0.0.100", 9001)],
    send_hz=50.0,
)
sender = UDPActionSender(config)
sender.start()

# Send action
sender.set_action(1, Action(left_speed=0.5, right_speed=0.5))

# Emergency stop
sender.stop_all()

sender.stop()
```

## Example

See `examples/test_real_env_keyboard.py` for a complete keyboard control example.

```bash
# Control robot 1 at IP 10.0.0.100
python examples/test_real_env_keyboard.py --robots "1:10.0.0.100"

# Control multiple robots
python examples/test_real_env_keyboard.py --robots "1:10.0.0.100,2:10.0.0.101"

# Use dynamic mode (no warmup)
python examples/test_real_env_keyboard.py --robots "1:10.0.0.100" --warmup 0
```

## Dependencies

- OpenCV with ArUco module (`opencv-contrib-python`)
- PyYAML (for camera calibration)
- NumPy

## Files

```
real_push_env/
├── __init__.py          # Module exports
├── real_push_env.py     # Main RealPushEnv class
├── observer.py          # ArUco camera observer
├── udp_action.py        # UDP action sender
└── readme.md            # This file
```
