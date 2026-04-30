# Main Arm Control

Controls the WidowX-200 robot arm for parachute line stowing, including motion planning, gripper control, and teleoperation.

## Nodes

| Node | Description |
|------|-------------|
| `main_arm_interface_node` | Low-level arm interface and command bridge |
| `main_arm_planner_node` | End-effector pose planner and waypoint sequence executor |
| `gripper_control_node` | Gripper open/close with load sensing |
| `main_arm_teleop_node` | Starts/stops Interbotix joystick teleoperation |
| `xbox_arm_controller_node` | Xbox-based incremental end-effector control |
| `motor_health_monitor` | Monitors motor temperatures and load |
| `test_kinematics` | Kinematics testing utility |

## Coordinate System

The WidowX-200 uses a right-handed base frame:

- `+X`: forward from the robot base
- `+Y`: left when facing the robot
- `+Z`: upward

```text
         Z (up)
         |
         |
         |_______ Y (left when facing robot)
        /
       /
      X (forward from robot base)

Top View:

        Y (left)
        ^
        |
        |
        |______> X (forward)
       /
      /
    base

Side View:

    Z (up)
    ^
    |     gripper
    |      /
    |     /
    |    /_____ end-effector
    |
    |________> X (forward)
   base
```

## `main_arm_human_move_for_test`

`main_arm_human_move_for_test.py` is a live monitoring and hand-guided testing tool.

### What it does

- Prints all current joint angles in radians and degrees
- Prints the current end-effector position `(x, y, z)` in the base frame
- Prints the current end-effector orientation `(roll, pitch, yaw)`
- Prints the end-effector pitch relative to the ground
- Prints the gripper finger position and opening width
- Publishes the same live state to ROS 2 topics
- Optionally disables torque so the arm can be moved by hand

### Run

```bash
ros2 run main_arm_control main_arm_human_move_for_test
```

### Run with torque disabled for hand-guided pose teaching

```bash
ros2 run main_arm_control main_arm_human_move_for_test --ros-args -p torque_off_arm:=true -p torque_off_gripper:=true
```

### Published topics

- `/main_arm/human_move/joint_state`
- `/main_arm/human_move/ee_pose`
- `/main_arm/human_move/ee_pitch_to_ground_deg`
- `/main_arm/human_move/gripper_opening_mm`
- `/main_arm/human_move/summary`

## `main_arm_planner_node`

`main_arm_planner.py` accepts direct point/pose commands and also runs higher-level sequences.

### Main input topics

- `/main_arm/target_point` (`geometry_msgs/Point`)
- `/main_arm/target_pose` (`geometry_msgs/Pose`)
- `/main_arm/target_joint_angles` (`sensor_msgs/JointState`)
- `/main_arm/run_auto_sequence` (`std_msgs/Empty`)
- `/main_arm/hole_center_sequence` (`geometry_msgs/Pose`, alias of left-hole sequence)
- `/main_arm/left_hole_center_sequence` (`geometry_msgs/Pose`)
- `/main_arm/right_hole_center_sequence` (`geometry_msgs/Pose`)

### Planner status output

- `/main_arm/planner_status`
- `/main_arm/current_joint_angles`

## Hole-Center Waypoint Sequences

The planner contains two waypoint sets:

- `LEFT_HOLE_WAYPOINTS`
- `RIGHT_HOLE_WAYPOINTS`

Each waypoint is defined **in the hole-center frame**, not directly in the robot base frame.

### Hole-center frame definition

When you publish a `geometry_msgs/Pose` to `/main_arm/left_hole_center_sequence` or `/main_arm/right_hole_center_sequence`:

- `position` is the hole-center origin expressed in the robot base frame
- `orientation` defines the hole-center frame axes expressed in the robot base frame
- each waypoint offset `(dx, dy, dz, droll, dpitch, dyaw)` is then applied relative to that local hole-center frame

In other words:

- the hole center is the local origin
- the quaternion you publish defines the local frame orientation
- the planner transforms the stored waypoint offsets from the hole-center frame back into the robot base frame before execution

### Motion behavior

For each stored waypoint, the planner now executes the motion in two stages:

1. Move the end-effector to the target position first
2. Rotate to the target orientation second

This avoids moving position and orientation at the same time during the hole sequence.

### Right-hole example

Use the following command to run the right-hole waypoint sequence with an example hole-center pose:

```bash
ros2 topic pub --once /main_arm/right_hole_center_sequence geometry_msgs/Pose "{position: {x: 0.2090, y: -0.1580, z: 0.0604}, orientation: {x: 0.021, y: 0.659, z: -0.019, w: 0.752}}"
```

### Left-hole example

The command format is identical for the left hole. Publish the hole-center pose to the left-hole topic:

```bash
ros2 topic pub --once /main_arm/left_hole_center_sequence geometry_msgs/Pose "{position: {x: 0.2565, y: 0.1160, z: 0.060}, orientation: {x: 0.02055928805249521, y: 0.6625853327516731, z: 0.0184659342163154, w: 0.7484764537182501}}"
```


## Auto Sequence / Pick Line Bundle Function

The main-arm “pick line bundle” behavior is implemented in `main_arm_planner.py` as the planner auto sequence:

- callback entry: `auto_sequence_callback()`
- worker function: `run_auto_sequence()`

This sequence performs the following steps:

1. Go to sleep pose
2. Go to home pose
3. Open the gripper to the configured opening
4. Move to the configured target end-effector pose
5. Close the gripper to grasp the bundle
6. Move to the configured return pose

### Trigger from ROS 2

```bash
ros2 topic pub -1 /main_arm/run_auto_sequence std_msgs/Empty "{}"
```

This is the recommended way to call the pick-line-bundle function from the command line.

## Notes

- For the hole-center sequence topics, all input poses must be expressed in the main-arm base frame.
- The hole-center quaternion is important because it defines the local frame used to interpret the stored waypoint offsets.
- On this 5-DOF arm, yaw is not commanded independently in the same way as roll and pitch; the effective yaw is coupled to the arm geometry and base rotation.