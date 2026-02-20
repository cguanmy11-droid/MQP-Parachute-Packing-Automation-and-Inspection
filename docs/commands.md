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
