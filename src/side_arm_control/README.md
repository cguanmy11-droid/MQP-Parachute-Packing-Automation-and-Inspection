# Side Arm Control

High-level control for the ESP32-based side arm gantry system. Converts mm coordinates to motor commands, provides ROS 2 action/service interfaces, and handles visualization.

## Hardware

- **X axis**: Stepper 2 (horizontal belt rails)
- **Y axis**: Stepper 1 (vertical lead screw)
- **Z axis**: DC motor (depth, timed movement)
- **Hook rotation**: Servo motor

## Nodes

| Node | Description |
|------|-------------|
| `coordinate_node` | Core motion controller - converts mm coordinates to motor commands |
| `side_arm_interface_node` | High-level hook operations (insert, visual servo, release) |
| `side_arm_visualizer` | RViz marker and TF publisher for hook position |
| `side_arm_joint_state_publisher` | Publishes joint states for URDF visualization |
| `manual_jog` | Interactive terminal control for manual positioning |

## Actions

| Action | Namespace | Description |
|--------|-----------|-------------|
| `move_to_coordinate` | `/side_arm_*/` | Move to XYZ with progress feedback |
| `insert_hook` | `/side_arm_*/` | Multi-stage hook insertion |
| `visual_servo` | `/side_arm_*/` | Visual servoing to align with loop |

## Services

| Service | Namespace | Description |
|---------|-----------|-------------|
| `move_to_position` | `/side_arm_*/` | Blocking move to XYZ position |
| `rotate_hook` | `/side_arm_*/` | Rotate hook servo |

## Topics

**Publishes:**
- `/side_arm_*/parsed_state` (SideArmState) - Position in mm, limit switches, homing status
- `/side_arm_*/command` (String) - Motor commands to ESP32
- `/side_arm_*/status` (HookStatus) - Hook state
- `side_arm_marker` (Marker) - RViz hook visualization

**Subscribes:**
- `/side_arm_*/state` (String) - Raw JSON from ESP32
- `/side_arm_*/yolo/centers` - Loop detections for visual servo

## Configuration

Config files in `config/`:
- `side_arm_left.yaml` - Left arm (V1, homes to X=0)
- `side_arm_right.yaml` - Right arm (V2, homes to X=340, inverted axes)
- `side_arm_v1.yaml` / `side_arm_v2.yaml` - Hardware-specific configs

## Running

```bash
# Manual jog (interactive terminal)
ros2 run side_arm_control manual_jog

# Move to position via service
ros2 service call /side_arm_left/move_to_position parachute_interfaces/srv/MoveToPosition \
  "{x_mm: 100.0, y_mm: 50.0, z_mm: 20.0, speed_scale: 0.5}"

# Home the arm
ros2 topic pub --once /side_arm_left/command std_msgs/String "data: 'HOME_ALL'"

# Check arm state
ros2 topic echo /side_arm_left/parsed_state
```

## Coordinate System

```
Looking at the parachute bag from above:

    Y+ (up, vertical lead screw)
    ^
    |
    |    Hook travels along X (horizontal belt)
    |    ═══════════════════════════►
    |                               X+
    |
    └─────────────────────────────────

    Z+ extends toward the bag (DC motor depth)
```
