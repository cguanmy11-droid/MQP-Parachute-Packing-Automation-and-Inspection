# Launch Commands for running different nodes and tests

### Test System Coordination
ros2 launch parachute_coordinator test_system.launch.py


## Option A: Real Robot
### Terminal 1: Robot + Interface
ros2 launch main_arm_control arm_interface.launch.py

### Terminal 2 (optional): Xbox controller
ros2 run joy joy_node
ros2 run main_arm_control xbox_arm_controller

## Option B: Simulation
### Terminal 1: Sim + Interface
ros2 launch main_arm_control arm_simulation.launch.py

### Terminal 2 (optional): Xbox controller  
ros2 run joy joy_node
ros2 run main_arm_control xbox_arm_controller

## RViz should open automatically showing the robot
# Option C: Test Mode (no hardware)
bashros2 run main_arm_control main_arm_interface_node --ros-args -p test_mode:=true

### For running the robot arm with simulation
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx200
ros2 run main_arm_control main_arm_interface_node --ros-args -p test_mode:=false




#####################
For running using the dual arm launch and in simulation

# Rebuild first
  cd ~/coding_projects/MQP-Parachute-Packing-Automation-and-Inspection
  colcon build --packages-select parachute_coordinator
  source install/setup.bash

  # Basic: Both arms on hardware, RViz enabled
  ros2 launch parachute_coordinator dual_arm_test.launch.py

  # With Xbox controller for main arm
  ros2 launch parachute_coordinator dual_arm_test.launch.py enable_teleop:=true

  # Main arm in simulation, side arm on hardware
  ros2 launch parachute_coordinator dual_arm_test.launch.py main_arm_sim:=true

  # Both in simulation/test mode
  ros2 launch parachute_coordinator dual_arm_test.launch.py main_arm_sim:=true side_arm_test_mode:=true

  # Side arm only (for side arm testing without main arm)
  ros2 launch parachute_coordinator dual_arm_test.launch.py enable_main_arm:=false

  # Main arm only (for main arm testing without side arm)
  ros2 launch parachute_coordinator dual_arm_test.launch.py enable_side_arm:=false

  # Custom serial port for side arm
  ros2 launch parachute_coordinator dual_arm_test.launch.py serial_port:=/dev/ttyACM0

  ---
  Launch Arguments Reference

  | Argument             | Default      | Description                  |
  |----------------------|--------------|------------------------------|
  | enable_main_arm      | true         | Enable WX200 main arm        |
  | main_arm_sim         | false        | Main arm simulation mode     |
  | enable_teleop        | false        | Xbox controller for main arm |
  | controller_type      | xboxone      | Controller type              |
  | robot_model          | wx200        | Main arm model               |
  | enable_side_arm      | true         | Enable side arm              |
  | side_arm_test_mode   | false        | Side arm simulated movements |
  | serial_port          | /dev/ttyUSB0 | ESP32 serial port            |
  | enable_visualization | true         | Side arm RViz marker         |
  | use_rviz             | true         | Launch RViz                  |

  ---
  Nodes Launched

  Main Arm (when enabled):
  - interbotix_xsarm_control - Base arm control
  - main_arm_interface_node - High-level interface
  - main_arm_planner_node - Motion planning
  - main_arm_teleop_node - Xbox control (if enable_teleop:=true)

  Side Arm (when enabled):
  - side_arm_serial_bridge - ESP32 communication
  - side_arm_coordinate_node - mm to motor commands
  - side_arm_interface_node - High-level actions/services
  - side_arm_visualizer - RViz marker for hook position

  ---
  Manual Jog (Run Separately)

  The manual jog is interactive and should be run in a separate terminal after launching:

  # Terminal 1: Launch the system
  ros2 launch parachute_coordinator dual_arm_test.launch.py

  # Terminal 2: Run manual jog for side arm
  ros2 run side_arm_control manual_jog

  # Send Coordinate


---

# Full System Launch Commands

The full system includes: main arm (WX200), side arms, vision, and state machine coordinator.

## Quick Reference

| Launch File | Use Case |
|-------------|----------|
| `full_system.launch.py` | Complete system with state machine (production/demo) |
| `dual_side_arm.launch.py` | Both side arms only, no main arm |
| `dual_arm_test.launch.py` | Single side arm + main arm testing |

---

## Full System (State Machine + All Arms)

### Simulation with both side arms and vision test loops
```bash
ros2 launch parachute_coordinator full_system.launch.py sim:=true vision_test:=true
```

### Simulation with RIGHT arm only (V2 - homes to X=340)
```bash
ros2 launch parachute_coordinator full_system.launch.py sim:=true enable_left:=false enable_right:=true vision_test:=true
```

### Simulation with LEFT arm only (V1 - homes to X=0)
```bash
ros2 launch parachute_coordinator full_system.launch.py sim:=true enable_left:=true enable_right:=false vision_test:=true
```

### Hardware mode (all systems real)
```bash
ros2 launch parachute_coordinator full_system.launch.py sim:=false
```

### Start the stowing sequence (after launch)
```bash
ros2 topic pub --once /stow/command std_msgs/String "data: start"
```

### Pause/Resume stowing
```bash
ros2 topic pub --once /stow/command std_msgs/String "data: pause"
ros2 topic pub --once /stow/command std_msgs/String "data: resume"
```

### Full System Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| sim | true | Simulation mode (no hardware) |
| enable_main_arm | true | Enable WX200 main arm |
| enable_left | true | Enable left side arm (V1) |
| enable_right | true | Enable right side arm (V2) |
| vision_test | false | Use simulated loop targets |
| enable_coordinator | true | Enable state machine |

---

## Dual Side Arm Only (No Main Arm)

### Both arms in simulation
```bash
ros2 launch parachute_coordinator dual_side_arm.launch.py sim:=true
```

### Right arm only (V2)
```bash
ros2 launch parachute_coordinator dual_side_arm.launch.py sim:=true enable_left:=false
```

### Left arm only (V1)
```bash
ros2 launch parachute_coordinator dual_side_arm.launch.py sim:=true enable_right:=false
```

### Hardware mode
```bash
ros2 launch parachute_coordinator dual_side_arm.launch.py sim:=false
```

---

## Single Side Arm Testing (dual_arm_test)

### Side arm simulation only (no main arm)
```bash
ros2 launch parachute_coordinator dual_arm_test.launch.py side_arm_sim:=true enable_main_arm:=false
```

### Side arm with V2 config
```bash
ros2 launch parachute_coordinator dual_arm_test.launch.py side_arm_sim:=true enable_main_arm:=false arm_config:=side_arm_right.yaml
```

### Side arm with vision test
```bash
ros2 launch parachute_coordinator dual_arm_test.launch.py side_arm_sim:=true enable_main_arm:=false vision_test_mode:=true
```

---

## Side Arm Configurations

| Config File | Arm | Homing Position | Notes |
|-------------|-----|-----------------|-------|
| side_arm_v1.yaml | V1 | X=0, Y=0, Z=0 | Original gantry |
| side_arm_left.yaml | Left (V1) | X=0, Y=0, Z=0 | Positive Y side |
| side_arm_v2.yaml | V2 | X=340, Y=0, Z=0 | Framed gantry, inverted X/Z |
| side_arm_right.yaml | Right (V2) | X=340, Y=0, Z=0 | Negative Y side |

---

## Manual Side Arm Control

### Move to position (simulation or hardware)
```bash
ros2 service call /side_arm_right/move_to_position parachute_interfaces/srv/MoveToPosition \
  "{x_mm: 100.0, y_mm: 50.0, z_mm: 20.0, speed_scale: 0.5}"
```

### Home the arm
```bash
ros2 topic pub --once /side_arm_right/command std_msgs/String "data: HOME_ALL"
```

### Check arm state
```bash
ros2 topic echo /side_arm_right/parsed_state
```

---

## Troubleshooting

### Rebuild after changes
```bash
cd ~/coding_projects/MQP-Parachute-Packing-Automation-and-Inspection
colcon build --packages-select side_arm_control parachute_coordinator
source install/setup.bash
```

### Check if nodes are running
```bash
ros2 node list
```

### View TF tree
```bash
ros2 run tf2_tools view_frames
```
