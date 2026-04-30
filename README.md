# Parachute Packing Automation and Inspection

**WPI Major Qualifying Project (MQP) 2025-2026**

Automated parachute line packing system using coordinated robotics, computer vision, and a ROS 2 state machine.

## System Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     Parachute Packing System                            │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   ┌──────────────┐    ┌──────────────┐    ┌──────────────┐             │
│   │  Top Camera  │    │ Side Cameras │    │   Operator   │             │
│   │  (YOLO cls)  │    │  (YOLO det)  │    │     GUI      │             │
│   └──────┬───────┘    └──────┬───────┘    └──────┬───────┘             │
│          │                   │                   │                      │
│          └─────────┬─────────┘                   │                      │
│                    ▼                             │                      │
│          ┌─────────────────┐                     │                      │
│          │    Perception   │◄────────────────────┘                      │
│          │  Loop Detection │                                            │
│          │  Target Select  │                                            │
│          └────────┬────────┘                                            │
│                   │                                                     │
│                   ▼                                                     │
│          ┌─────────────────┐                                            │
│          │   Coordinator   │                                            │
│          │  State Machine  │                                            │
│          └───────┬─────────┘                                            │
│                  │                                                      │
│        ┌─────────┼─────────┐                                            │
│        ▼         ▼         ▼                                            │
│   ┌─────────┐ ┌─────────┐ ┌─────────┐                                   │
│   │  Left   │ │  Main   │ │  Right  │                                   │
│   │Side Arm │ │  Arm    │ │Side Arm │                                   │
│   │  (V1)   │ │ (WX200) │ │  (V2)   │                                   │
│   └─────────┘ └─────────┘ └─────────┘                                   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

| Component | Hardware | Purpose |
|-----------|----------|---------|
| **Main Arm** | WidowX-200 (Interbotix) | Stows parachute lines around hooks |
| **Left Side Arm** | ESP32 + 3-axis gantry (V1) | Inserts hook through loops (positive Y) |
| **Right Side Arm** | ESP32 + 3-axis gantry (V2) | Inserts hook through loops (negative Y) |
| **Top Camera** | USB camera + YOLO | Detects loop state (stowed/unstowed) |
| **Side Cameras** | USB cameras + YOLO | Tracks loop positions for visual servoing |

---

## Prerequisites

- **OS**: Ubuntu 22.04
- **ROS 2**: Humble
- **Python**: 3.10+
- **Hardware** (for non-simulation):
  - WidowX-200 robot arm
  - ESP32-based gantry controllers (1-2 units)
  - USB cameras (1-3 units)

---

## Installation

### 1. Clone the Repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/YOUR_ORG/MQP-Parachute-Packing-Automation-and-Inspection.git
cd MQP-Parachute-Packing-Automation-and-Inspection
```

### 2. Install Interbotix Dependencies

```bash
# Import Interbotix repositories
vcs import src < interbotix.repos

# Or manually clone (if vcs not available):
# cd src
# git clone -b humble https://github.com/Interbotix/interbotix_ros_core.git
# git clone -b humble https://github.com/Interbotix/interbotix_ros_manipulators.git
# git clone -b humble https://github.com/Interbotix/interbotix_ros_toolboxes.git
```

### 3. Install Python Dependencies

```bash
pip install -r requirements.txt

# For YOLO detection (top camera):
pip install ultralytics

# For GUI:
pip install PyQt5
```

### 4. Install ROS 2 Dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Quick Start

### Full System (Simulation)

Both side arms + main arm with simulated vision:

```bash
ros2 launch parachute_coordinator full_system.launch.py sim:=true vision_test:=true
```

Start the stowing sequence:
```bash
ros2 topic pub --once /stow/command std_msgs/String "data: start"
```

### Full System (Hardware)

```bash
ros2 launch parachute_coordinator full_system.launch.py \
    sim:=false \
    use_real_camera:=true \
    enable_top_cam:=true \ 
    left_camera_index:=/dev/video8 \ 
    right_camera_index:=/dev/video6 \
    top_cam_device:=/dev/video2 \
```

Note: you likely need to specify the correct ports for the cameras and ESP32 for the side arms. 
It is possible to set environment variables to bypass this step but in order to find the correct ports 
initially, you can run the following, and update the index values above accordingly or in the launch file.

```bash
sudo apt update
sudo apt install v4l-utils
v4l2-ctl --list-devices
# Rerun this last command with and without the cameras plugged in to determine which camera is which 
```


### Single Side Arm Testing

```bash
# Left arm only, simulation
ros2 launch parachute_coordinator full_system.launch.py \
    sim:=true enable_right:=false vision_test:=true

# Right arm only, hardware
ros2 launch parachute_coordinator full_system.launch.py \
    sim:=false enable_left:=false
```

### Operator GUI

```bash
ros2 run parachute_gui operator_console
```

---

## Launch Files

| Launch File | Description |
|-------------|-------------|
| `full_system.launch.py` | Complete system: dual arms, main arm, vision, state machine |
| `state_machine_demo.launch.py` | Single side arm + state machine (legacy) |
| `full_stow_demo.launch.py` | Single-loop stow demonstration |
| `dual_arm_test.launch.py` | Dual arm testing without coordinator |
| `dual_side_arm.launch.py` | Both side arms, no main arm |

### Key Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `sim` | `true` | Simulation mode (no hardware) |
| `enable_main_arm` | `true` | Enable WidowX-200 |
| `enable_left` | `true` | Enable left side arm (V1) |
| `enable_right` | `true` | Enable right side arm (V2) |
| `vision_test` | `false` | Use simulated loop positions |
| `use_real_camera` | `false` | Enable side cameras with YOLO |
| `enable_top_cam` | `false` | Enable top camera for loop state |
| `enable_coordinator` | `true` | Enable state machine |

---

## Control Commands

After launching, control the system via `/stow/command`:

```bash
# Start stowing
ros2 topic pub --once /stow/command std_msgs/String "data: start"

# Pause/Resume
ros2 topic pub --once /stow/command std_msgs/String "data: pause"
ros2 topic pub --once /stow/command std_msgs/String "data: resume"

# Error recovery
ros2 topic pub --once /stow/command std_msgs/String "data: retry"
ros2 topic pub --once /stow/command std_msgs/String "data: skip"
ros2 topic pub --once /stow/command std_msgs/String "data: abort"

# Change motion pattern
ros2 topic pub --once /stow/command std_msgs/String "data: pattern:square_stow"
```

---

## Package Structure

```
src/
├── parachute_coordinator/     # State machine and system orchestration
├── parachute_perception/      # Loop detection, target selection, sensor fusion
├── parachute_interfaces/      # ROS 2 messages, services, actions
├── parachute_gui/             # Operator console (PyQt5)
├── main_arm_control/          # WidowX-200 control and motion planning
├── side_arm_control/          # Side arm interface and visualization
├── side_arm_motor_control/    # ESP32 serial bridge
├── yolo_detect_ros/           # Side camera YOLO detection
├── top_cam_loop_state/        # Top camera loop state classification
├── top_cam_yolo/              # YOLO training for top camera
└── widowx_custom_perception/  # Color segmentation (legacy)
```

---

## Hardware Configuration

### Serial Ports

Side arms connect via USB serial. Default ports:
- Left arm: `/dev/ttyUSB0`
- Right arm: `/dev/ttyUSB1`

Override with launch arguments:
```bash
ros2 launch parachute_coordinator full_system.launch.py \
    left_serial_port:=/dev/ttyACM0
```

Note: Using the USB hub bought by the 2025-2026 MQP, there are some particularities to the connections.
Based on the serial bandwith of the USB hub, only connect 1 camera to the hub at a time, it can also hold the 
connection to the ESP32s and the Main arm port. With the processor management, it can be done but sometimes causes
latency issues, so if you run into issues, double check your port bandwith. 

### Camera Devices

| Camera | Default Device | Launch Argument |
|--------|----------------|-----------------|
| Left side | `/dev/video8` | `left_camera_index` |
| Right side | `/dev/video4` | `right_camera_index` |
| Top | `/dev/video6` | `top_cam_device` |

---

## Documentation

| Document | Description |
|----------|-------------|
| [docs/commands.md](docs/commands.md) | Complete launch command reference |
| [docs/SYSTEM_CATALOG.md](docs/SYSTEM_CATALOG.md) | All nodes, topics, and services |
| [docs/STATE_MACHINE_PLAN.md](docs/STATE_MACHINE_PLAN.md) | Coordinator state machine design |
| [docs/SIDE_ARM_MOTION_PLAN.md](docs/SIDE_ARM_MOTION_PLAN.md) | Side arm motion planning |
| [docs/ARCHITECTURE_EN.md](docs/ARCHITECTURE_EN.md) | Detailed system architecture |
| [src/parachute_gui/README.md](src/parachute_gui/README.md) | Operator GUI usage |

---

## Dependencies

### requirements.txt
```
numpy==1.26.4
opencv-python==4.9.0.80
transforms3d
scipy
vcstool
```

### interbotix.repos

Interbotix ROS 2 packages for WidowX-200 control:
- `interbotix_ros_core`
- `interbotix_ros_manipulators`
- `interbotix_ros_toolboxes`
- `moveit_visual_tools`

Usage:
```bash
vcs import src < interbotix.repos
```

---

## Troubleshooting

### Rebuild after changes
```bash
colcon build --packages-select parachute_coordinator side_arm_control
source install/setup.bash
```

### Check running nodes
```bash
ros2 node list
```

### View TF tree
```bash
ros2 run tf2_tools view_frames
```

### Check coordinator state
```bash
ros2 topic echo /coordinator/state
```

---

## License

Worcester Polytechnic Institute - Major Qualifying Project

## Team

MQP 2025-2026 - Parachute Packing Automation and Inspection

