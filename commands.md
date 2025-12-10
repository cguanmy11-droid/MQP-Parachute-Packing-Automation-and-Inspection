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