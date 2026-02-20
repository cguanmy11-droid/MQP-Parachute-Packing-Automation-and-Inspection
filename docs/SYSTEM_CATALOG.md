# Parachute Packing Automation - System Catalog

Complete catalog of packages, nodes, and launch files.

---

## Packages Overview

| Package | Purpose |
|---------|---------|
| **side_arm_control** | Side arm (hook) manipulation - 3-axis XYZ + servo |
| **side_arm_motor_control_bridge** | ESP32 serial communication bridge |
| **main_arm_control** | WidowX-200 arm control for line stowing |
| **parachute_coordinator** | System orchestration and sequencing |
| **parachute_perception** | Vision pipeline - loop detection and targeting |
| **parachute_interfaces** | Shared ROS2 messages, services, actions |

---

## SIDE_ARM_CONTROL

### Nodes

#### `coordinate_node`
**Purpose**: Core motion controller - converts mm coordinates to motor commands.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Pub | `/side_arm/parsed_state` | SideArmState | Position in mm (x, y, z) |
| Pub | `/side_arm/command` | String | Motor commands to ESP32 |
| Sub | `/side_arm/state` | String | Raw JSON from ESP32 |

| Interface | Name | Description |
|-----------|------|-------------|
| Action | `/side_arm/move_to_coordinate` | Move to XYZ with progress feedback |
| Service | `/side_arm/move_to_position` | Blocking move to position |

**Coordinate Mapping**:
- X axis = Stepper 2 (horizontal belt)
- Y axis = Stepper 1 (vertical lead screw)
- Z axis = DC motor (depth, timed movement)

---

#### `side_arm_interface_node`
**Purpose**: High-level hook insertion operations.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Pub | `/side_arm/status` | HookStatus | Current hook state |
| Sub | `/side_arm/parsed_state` | SideArmState | Position tracking |

| Interface | Name | Description |
|-----------|------|-------------|
| Action | `/side_arm/insert_hook` | 4-stage insertion (approach→align→insert→verify) |
| Service | `/side_arm/rotate_hook` | Rotate hook by angle |

---

#### `side_arm_visualizer`
**Purpose**: RViz visualization - publishes hook marker and TF frames.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Pub | `side_arm_marker` | Marker | Hook mesh visualization |
| Pub | TF: `side_arm_hook` | - | Hook position frame |
| Pub | TF: `camera_frame` | - | Camera position (child of hook) |
| Sub | `/side_arm/parsed_state` | SideArmState | Position data |
| Sub | `/side_arm/state` | String | Servo angle |

---

#### `side_arm_joint_state_publisher`
**Purpose**: Bridges state to URDF joint states for robot_state_publisher.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Pub | `/side_arm/joint_states` | JointState | URDF joint positions |
| Sub | `/side_arm/parsed_state` | SideArmState | Position in mm |

**Joint Mapping**: joint_x, joint_y, joint_z (prismatic, meters), joint_servo (revolute, radians)

---

#### `manual_jog`
**Purpose**: Keyboard control for calibration/testing.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Pub | `/side_arm/command` | String | Motor commands |
| Sub | `/side_arm/state` | String | State feedback |

**Keyboard Controls**:
- `w/s` - Vertical (Y)
- `a/d` - Horizontal (X)
- `q/e` - Depth (Z)
- `z/c` - Servo rotation
- `h` - Home all axes
- `x` - Emergency stop

---

## SIDE_ARM_MOTOR_CONTROL_BRIDGE

### Nodes

#### `serial_bridge`
**Purpose**: Lowest-level ESP32 serial communication.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Pub | `/side_arm/state` | String | Raw JSON: `STATE {"l1":..,"s1":..,"dc":..}` |
| Sub | `/side_arm/command` | String | Commands to ESP32 |

| Interface | Name | Description |
|-----------|------|-------------|
| Service | `side_arm/request_state` | Force state update |

**Parameters**: `serial_port` (/dev/ttyUSB0), `baud_rate` (115200)

---

## MAIN_ARM_CONTROL

### Nodes

#### `main_arm_interface_node`
**Purpose**: Low-level WidowX-200 control interface.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Pub | `/main_arm/status` | ArmStatus | Detailed status |
| Pub | `/main_arm/current_pose` | Pose | End-effector pose |
| Sub | `/main_arm/pose_command` | String | Named poses (home, sleep) |
| Sub | `/main_arm/gripper_command` | String | open/close |

| Interface | Name | Description |
|-----------|------|-------------|
| Action | `/main_arm/execute_trajectory` | Execute planned trajectory |

---

#### `main_arm_planner_node`
**Purpose**: Motion planning with IK solving.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Pub | `/main_arm/planner_status` | String | Planning status |
| Sub | `/main_arm/target_point` | Point | Cartesian target (x,y,z) |
| Sub | `/main_arm/target_pose` | Pose | Full pose target |

**Workspace**: X[0.1-0.45], Y[-0.3-0.3], Z[0.05-0.4] meters

---

#### `main_arm_teleop_node`
**Purpose**: Xbox controller teleoperation wrapper.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Pub | `/main_arm/teleop_status` | String | active/inactive |
| Sub | `/main_arm/enable_teleop` | Bool | Enable/disable |

---

## PARACHUTE_PERCEPTION

### Nodes

#### `loop_detector_node`
**Purpose**: Detects loops from YOLO feed or publishes test loops.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Pub | `/detected_loops` | DetectedLoops | Detected loop array |
| Sub | `/yolo/centers` | PoseArray | YOLO detections |

---

#### `target_selector_node`
**Purpose**: Selects next target loop (default: rightmost).

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Pub | `/loop_positions` | DetectedLoops | Current loops |
| Sub | `/detected_loops` | DetectedLoops | Incoming detections |

| Interface | Name | Description |
|-----------|------|-------------|
| Service | `/request_next_target` | Returns next target loop |

---

#### `loop_visualizer_node`
**Purpose**: RViz visualization of detected loops.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Pub | `/detected_loop_markers` | MarkerArray | Sphere markers |
| Sub | `/detected_loops` | DetectedLoops | Loops to visualize |

---

#### `loop_ground_truth_node`
**Purpose**: Maintains ground truth positions for simulation.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Pub | `/loop_ground_truth` | LoopGroundTruth | Actual positions |
| Pub | `/loop_ground_truth_markers` | MarkerArray | RViz markers |

---

#### `detection_simulator_node`
**Purpose**: Simulates camera detection based on ground truth + camera pose.

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Pub | `/detected_loops` | DetectedLoops | Simulated detections |
| Sub | `/loop_ground_truth` | LoopGroundTruth | Ground truth |

---

## PARACHUTE_COORDINATOR

### Nodes

#### `packing_coordinator_node`
**Purpose**: Orchestrates full packing sequence.

**Sequence**: Target selection → Hook insertion → Line stowing → Hook rotation → Repeat

| Interface | Name | Package |
|-----------|------|---------|
| Calls Service | `/request_next_target` | parachute_perception |
| Calls Action | `/side_arm/insert_hook` | side_arm_control |
| Calls Action | `/main_arm/execute_trajectory` | main_arm_control |
| Calls Service | `/side_arm/rotate_hook` | side_arm_control |

---

## Launch Files

### `dual_arm_test.launch.py` (PRIMARY)
**Package**: parachute_coordinator

**Purpose**: Complete digital twin launch with both arms and visualization.

**Nodes Launched**:
| Group | Nodes |
|-------|-------|
| Main Arm | interbotix_xsarm_control, main_arm_interface_node, main_arm_teleop_node |
| Side Arm | serial_bridge, coordinate_node, side_arm_interface_node, side_arm_visualizer |
| Side Arm URDF | side_arm_robot_state_publisher, side_arm_joint_state_publisher |
| Digital Twin | frame_state_publisher, static TF publishers |
| Perception | loop_ground_truth_node, loop_visualizer_node, detection_simulator_node |

**Key Arguments**:
| Argument | Default | Description |
|----------|---------|-------------|
| `enable_main_arm` | true | Enable WidowX-200 |
| `main_arm_sim` | false | Simulate main arm |
| `enable_side_arm` | true | Enable side arm |
| `side_arm_test_mode` | false | Simulate side arm motion |
| `use_rviz` | true | Launch RViz |
| `enable_teleop` | false | Xbox controller |
| `use_joint_sliders` | false | Manual URDF sliders |
| `vision_test_mode` | false | Simulated detections |

**Example Usage**:
```bash
# Full simulation (no hardware)
ros2 launch parachute_coordinator dual_arm_test.launch.py \
    main_arm_sim:=true side_arm_test_mode:=true

# Side arm only (hardware)
ros2 launch parachute_coordinator dual_arm_test.launch.py \
    enable_main_arm:=false

# With Xbox teleop
ros2 launch parachute_coordinator dual_arm_test.launch.py \
    enable_teleop:=true
```

---

### Other Launch Files

| Launch File | Package | Purpose |
|-------------|---------|---------|
| `side_arm_full.launch.py` | side_arm_control | Side arm only |
| `main_arm.launch.py` | main_arm_control | Main arm with planner |
| `arm_interface.launch.py` | main_arm_control | Minimal main arm |
| `side_arm_serial.launch.py` | side_arm_motor_control | Serial bridge only |
| `test_system.launch.py` | parachute_coordinator | Full test mode |
| `demo_system.launch.py` | parachute_coordinator | Demo with perception |

---

## Custom Interfaces (parachute_interfaces)

### Messages
| Message | Fields | Purpose |
|---------|--------|---------|
| `SideArmState` | x_mm, y_mm, z_mm, is_homed, limits | Parsed position |
| `DetectedLoop` | loop_id, pose, confidence | Single loop |
| `DetectedLoops` | header, loops[] | Loop array |
| `HookStatus` | state, position, angle | Hook state |
| `ArmStatus` | state, is_moving, error | Arm state |

### Services
| Service | Request | Response | Purpose |
|---------|---------|----------|---------|
| `MoveToPosition` | x_mm, y_mm, z_mm | success, message | Blocking move |
| `RotateHook` | angle_degrees | success, final_angle | Rotate hook |
| `RequestNextTarget` | - | target_available, target_loop | Get next loop |

### Actions
| Action | Goal | Feedback | Result |
|--------|------|----------|--------|
| `MoveToCoordinate` | x,y,z,speed | progress, current_pos | success, final_pos |
| `InsertHook` | target_loop | state, progress | success, time |
| `ExecuteTrajectory` | trajectory | progress | success |

---

## Data Flow Diagrams

### Side Arm Control Flow
```
ESP32 Hardware
      ↓
serial_bridge (/side_arm/state)
      ↓
coordinate_node (/side_arm/parsed_state)
      ↓
├─→ side_arm_interface_node (actions/services)
├─→ side_arm_visualizer (RViz marker + TF)
└─→ side_arm_joint_state_publisher (URDF joints)
      ↓
robot_state_publisher (URDF TF tree)
      ↓
RViz
```

### Perception Flow
```
Camera/YOLO (or ground_truth_node in sim)
      ↓
loop_detector_node (/detected_loops)
      ↓
├─→ target_selector_node (/request_next_target)
└─→ loop_visualizer_node (RViz markers)
```

### Coordination Flow
```
packing_coordinator_node
      │
      ├─→ /request_next_target (perception)
      ├─→ /side_arm/insert_hook (side arm)
      ├─→ /main_arm/execute_trajectory (main arm)
      └─→ /side_arm/rotate_hook (side arm)
```
