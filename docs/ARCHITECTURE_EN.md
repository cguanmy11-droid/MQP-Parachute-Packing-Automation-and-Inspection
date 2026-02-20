\### WidowX-200 Perception and Control System Architecture Details (Based on Existing Interbotix ROS2 Packages)

This document is oriented towards actual deployment, detailing which components can be directly reused, which need modification on existing code, and which need to be newly created and introduce third-party dependencies; it also provides in-depth text-only explanations of key concepts (Waypoint generation, Trajectory following, Gripper partial open/close, ESP32 motor/sensor control). All symbol names, topics, and class names are annotated according to the current state of the project.

---

### 1. Objectives and Scope

- Connect external cameras and perform color segmentation, convert segmented targets into Waypoints, then control the WidowX-200 robotic arm to reach and execute grasping (including gripper partial open/close).
- RGB images from the second camera go through YOLO network inference, and detection results are sent back to ESP32 via micro-ROS.
- Use ESP32 to control stepper motors, DC motors, servos, and read limit sensors; ROS2 side handles command publishing and status subscription.

---

### 2. Existing Reusable Components, Modification Points, and New Additions

#### 2.1 Directly Reusable (Existing)
- Robotic arm driver and high-level control interfaces (existing):
  - `interbotix_xs_sdk` node (driver layer, complete topic/service interfaces), path: `src/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk/`
    - Command topics subscribed (all existing):
      - `/<robot_name>/commands/joint_group` (group commands)
      - `/<robot_name>/commands/joint_single` (single joint commands)
      - `/<robot_name>/commands/joint_trajectory` (trajectory commands)
    - Services (partial list): `/<robot_name>/torque_enable`, `/<robot_name>/reboot_motors`, `/<robot_name>/get_robot_info`, `/<robot_name>/set_operating_modes`, etc.
  - High-level Python API: `interbotix_xs_modules` (`InterbotixManipulatorXS`, `InterbotixGripperXSInterface`, etc.), path: `src/interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_modules/`
    - Key capabilities: end-effector pose setting, Cartesian small-segment trajectories, joint group commands, gripper force/open-close control, etc.

- Point cloud perception and camera-arm calibration (existing, optional):
  - Point cloud pipeline and clustering interface: `interbotix_perception_modules` (`InterbotixPointCloudInterface`), path: `src/interbotix_ros_toolboxes/interbotix_perception_toolbox/interbotix_perception_modules/`
    - Capabilities: get cluster positions via service, control filter parameters, start/stop pipeline (refer to `get_cluster_positions`).
  - AprilTag and camera→arm coordinate calibration: `interbotix_perception_modules/armtag.py`.
  - Quick integration launch (RealSense example): `interbotix_xsarm_perception/launch/xsarm_perception.launch.py` launches RealSense, point cloud filtering, ArmTag, and static TF tools.

- Perception messages (existing, reusable):
  - `interbotix_perception_msgs`: `ClusterInfo.msg`, `ClusterInfoArray.srv`, `FilterParams.srv`, etc.

#### 2.2 Existing Content Recommended for "Parameter-only Modification"
- Point cloud filter parameter file: `interbotix_xsarm_perception/config/filter_params.yaml`
  - Adjust `voxel_leaf_size`, `crop_box`, `plane_segmentation`, `cluster_tolerance/size`, etc. to adapt to your desktop/camera setup.
- Launch parameters of `xsarm_perception.launch.py` (no code changes needed):
  - `cloud_topic`: Match your depth/point cloud topic; change to corresponding topic for non-RealSense devices.
  - `camera_color_topic`, `camera_info_topic`: Point to your color stream.
  - `ref_frame`, `arm_base_frame`, `arm_tag_frame`: Keep consistent with URDF according to field naming.

#### 2.3 New Additions (New packages/nodes/third-party for this project)
- New package: `widowx_custom_perception` (custom)
  - New nodes:
    - `color_segmentation_node`: Subscribe to external camera color images, perform color segmentation, generate target image centroid and category (see Chapter 3 for details).
    - `arm_waypoint_controller_node`: Convert segmentation/clustering results to robotic arm Waypoints and send them to `interbotix_xs_sdk` (see Chapters 4 and 5 for details).
    - `yolo_detection_node`: Subscribe to second camera, execute YOLO inference, publish detection results (see Chapter 6 for details).
    - `yolo_to_esp32_bridge_node`: Convert YOLO results into command semantics consumable by ESP32 and publish (see Chapter 7 for details).
  - Third-party dependencies (runtime only): OpenCV (color segmentation), YOLO inference framework (e.g., `ultralytics`/PyTorch).

- New package: `widowx_esp32_interface` (custom)
  - New node: `esp32_motor_controller_node`: Subscribe to bridge commands, convert them to micro-ROS/ESP32 control topics; also subscribe to limit/status (see Chapter 8 for details).
  - Runtime dependency: `micro_ros_agent` (existing package, not in this repository).

- New interface package (optional): `widowx_custom_msgs`
  - If directly reusing `interbotix_perception_msgs/ClusterInfo` and standard messages meets requirements, this can be omitted; if more semantic messages for YOLO/ESP32 are needed, it is recommended to define separately (see Chapter 7 "Message Semantics").

---

### 3. Camera and Perception Layer (Color Segmentation and Point Cloud)

#### 3.1 External Camera Integration
- Option A (recommended, with depth): Use any depth camera → publish `sensor_msgs/PointCloud2` and color/camera info; directly obtain clusters and 3D positions through `interbotix_perception_modules` point cloud pipeline (no need to write triangulation).
- Option B (RGB only): Rely only on color images for color segmentation, 3D position needs extrapolation:
  - If the desktop is a known plane, combine `camera_info` with known plane Z to estimate landing point via monocular back-projection (adaptation error, requires calibration).
  - Or combine ArUco/AprilTag markers placed on the desktop, estimate approximate pose via plane homography; accuracy depends on calibration quality.

#### 3.2 Color Segmentation (`color_segmentation_node`, new)
- Input: `/camera1/image_raw`, `/camera1/camera_info`
- Processing: HSV space threshold segmentation → connected component extraction → centroid (pixel coordinates), area, average color.
- Output (choose one of two):
  - If using point cloud approach: Only as "color category hint", 3D position still uses `InterbotixPointCloudInterface.get_cluster_positions` output (sorted/aligned by `arm_base_frame`).
  - If RGB only: Publish custom `SegmentationResult` (or reuse `ClusterInfo`'s `color` field semantics, add image centroid and estimated depth, recommend custom message to avoid ambiguity).

#### 3.3 Point Cloud Clustering (existing)
- Use `InterbotixPointCloudInterface` to get cluster 3D positions, core capabilities:
  - Poll cluster results via `ClusterInfoArray` service;
  - Support `ref_frame` specification (e.g., robotic arm `/<robot_name>/base_link`), and sort according to `sort_axis`/`reverse`;
  - When `is_parallel` is `true`, returns "top surface" position, beneficial for placement/grasping height control.
- Existing demo reference: `interbotix_xsarm_perception/demos/color_sorter.py` (example completes camera→base coordinate alignment via `armtag`, then drives grasping using point cloud clustering results).

#### 3.4 Camera→Arm Coordinate Alignment (existing)
- Use `InterbotixArmTagInterface` (`armtag.py`):
  - Read `camera_color_topic`/`camera_info_topic`, detect AprilTag on arm, compute static transform from `ref_frame` (e.g., `camera_color_optical_frame`) to `/<robot_name>/base_link`;
  - Can work with `interbotix_tf_tools` static TF storage to persist the transform.

---

### 4. Waypoint Generation (What it is, How to produce, Constraints and Best Practices)

#### 4.1 Concept Definition
- Waypoint (path point) is an element in the desired pose sequence of the robotic arm end-effector in the robot base coordinate system (`/<robot_name>/base_link`), usually containing position (x,y,z) and orientation (roll,pitch,yaw) components. Waypoint is not equivalent to trajectory sampling points, but is often used as keyframes for generating trajectories.

#### 4.2 Generation Process (Color/Point Cloud to Waypoint)
1) Get target 3D position (recommend point cloud clustering `position`, or estimate via RGB+geometric assumptions).
2) Apply task-level offsets:
   - Approach point: Above target `z = z_object + z_clearance`;
   - Grasp point: `z = z_object + z_grasp_offset`;
   - Lift point: After grasping, return to `z_clearance`.
3) Orientation strategy:
   - For arms with fewer than 6 DoF, `yaw` is automatically derived from `atan2(y, x)`, `y` and `yaw` need to maintain constraints;
   - When grasping, often maintain fixed `pitch` (e.g., gripper vertically downward `pitch≈0.5`).
4) Optional: Safety/kinematics validation (joint limits, singular poses, obstacle avoidance).

#### 4.3 Waypoint Assembly and Output
- Waypoint sequence is usually: [Home/Sleep → Approach → Grasp → Lift → Place → Retreat].
- If using `InterbotixManipulatorXS`: Each Waypoint can obtain inverse kinematics solution via "end-effector pose command" planning interface (planning only, not immediately executed), used to concatenate into final trajectory (see Chapter 5).

---

### 5. Trajectory Following (What it is, How to execute, Relationship with MoveIt)

#### 5.1 Concept Definition
- Trajectory is a time-parameterized continuous motion plan in joint space or task space. Following refers to executing the trajectory point by point based on time reference, ensuring the end-effector or joints complete motion at desired velocity/acceleration.

#### 5.2 Two Common Implementation Paths (both can be based on existing interfaces)
- Path A: High-level API directly pushes "Cartesian small-segment trajectory" (existing)
  - Use `InterbotixArmXSInterface.set_ee_cartesian_trajectory`:
    - Internally discretizes into small-step Waypoints, gradually calls IK planning, and after success uniformly publishes one `JointTrajectoryCommand` via `/<robot_name>/commands/joint_trajectory`.
    - Suitable for short-distance linear interpolation needs, simple configuration, no MoveIt required.

- Path B: Combine Waypoints → Assemble `JointTrajectory` → Send (existing)
  - For each Waypoint, use `set_ee_pose_components` for "planning only" to obtain joint points (not immediately executed), then concatenate these joint points with timestamps into `trajectory_msgs/JointTrajectory`, finally send once via `/<robot_name>/commands/joint_trajectory` (more controllable).

#### 5.3 MoveIt Path (optional)
- Through `interbotix_xs_ros_control` combined with MoveIt to complete collision avoidance and complex trajectories, interface still consistent with `/<robot_name>/commands/joint_trajectory`, but trajectory generated by MoveIt's `joint_trajectory_controller`.

#### 5.4 Time Parameters and Dynamic Characteristics
- X-Series default "time-based trajectory": `moving_time` and `accel_time` control single-segment motion duration and acceleration/deceleration time (`accel_time <= moving_time/2`).
- The first point of the trajectory usually uses current joint state as starting point, needs to be filled before publishing.

---

### 6. Gripper Partial Open/Close (What it is, How to control, Force and Limits)

#### 6.1 Concept Definition
- "Partial open/close" means the gripper is not fully closed/fully open, but reaches an intermediate state with specified force or target opening, used for gripping objects of different materials/sizes.

#### 6.2 Existing Control Capabilities
- `InterbotixGripperXSInterface` supports:
  - Force/pressure setting (in PWM/current mode, approximate control of gripping force via "force");
  - Small open/close for a period of time (achieve partial open/close effect via "effort value + time");
  - Safe stop: Automatically stops based on fingertip position and soft limits, avoiding over-driving.

#### 6.3 Typical Strategy
- Set pressure range (e.g., 0.0–1.0 ratio) to match different materials;
- In "in-position + hold" scenarios, first close with smaller force, monitor position, then briefly increase;
- In "partial open" scenarios, drive briefly in open direction with small effort, achieve approximate opening with time window.

---

### 7. Second Camera YOLO → ESP32 Bridge

#### 7.1 Processing Chain
- `yolo_detection_node` (new):
  - Input: `/camera2/image_raw`;
  - Processing: YOLO inference, output detection category, confidence, pixel box center/size;
  - Output: `/yolo/detections` (recommend custom `YoloDetectionArray`, containing timestamp and multi-target array).

- `yolo_to_esp32_bridge_node` (new):
  - Subscribe: `/yolo/detections`;
  - Reduction: Convert detection results to device action semantics (e.g., "drive stepper motor scanning when tracking certain target");
  - Publish: `/esp32/detection_commands` (recommend `DetectionCommand`, containing target category, priority, desired action code, etc.).

#### 7.2 Message Semantics (if creating new `widowx_custom_msgs`)
- `YoloDetection`: `class_name`, `confidence`, `bbox_center` (pixels), `bbox_size`, optional `track_id`.
- `YoloDetectionArray`: `Header`+`YoloDetection[]`.
- `DetectionCommand`: `Header`, `detected_object`, `position` (nullable/pixel/camera coordinate), `action_code` (custom protocol enumeration).

(Note: If simplification is sufficient, can also reuse `std_msgs/String` to carry JSON, but not conducive to maintainability and type checking.)

---

### 8. ESP32 Side Motor and Sensor Control (Classes and Responsibilities, Limit Strategy, micro-ROS Channel)

#### 8.1 ROS2 Side Node (`esp32_motor_controller_node`, new)
- Responsibility division (class perspective, text-only):
  - StepperController (stepper motor control class, ROS2 side):
    - Subscribe to action semantics from upstream (e.g., `DetectionCommand`).
    - Convert "target steps/speed/direction/microstep/acceleration-deceleration" to downstream topic (e.g., `/esp32/stepper_cmd`).
    - Maintain state: whether in homing/busy/error.
  - DCMotorController (DC motor control class, ROS2 side):
    - Publish "target PWM/direction/duration" to `/esp32/dc_motor_cmd`.
    - Optional closed-loop: Adjust duty cycle based on feedback (encoder/limit) if firmware supports.
  - ServoController (servo control class, ROS2 side):
    - Encapsulate "target angle/pulse width" as `/esp32/servo_cmd` (can use `std_msgs/Int16MultiArray` or custom).
  - LimitSwitchMonitor (limit monitoring class, ROS2 side):
    - Subscribe to `/esp32/limit_switch_status`, notify controllers to interrupt/homing when triggered.

#### 8.2 micro-ROS/ESP32 Side (firmware responsibilities, text-only)
- micro-ROS node should:
  - Subscribe: `/esp32/stepper_cmd`, `/esp32/dc_motor_cmd`, `/esp32/servo_cmd`;
  - Publish: `/esp32/limit_switch_status`, optional `/esp32/feedback` (containing running status, error code, current position, etc.).
  - Stepper motor firmware class (illustrative):
    - Parse commands (motor ID, steps, speed, direction, acceleration curve).
    - Execute acceleration-deceleration curve, handle homing logic (move slowly in negative direction until limit triggered→zero→move away from limit slightly).
  - DC motor firmware class:
    - Parse commands (motor ID, PWM, direction, duration).
    - If equipped with encoder/current detection, can report load/speed estimation.
  - Servo firmware class:
    - Parse angle/pulse width array, periodically refresh PWM.
  - Limit firmware class:
    - Poll or interrupt report status; when triggered, immediately publish status and optionally set "emergency stop/homing" flag.

#### 8.3 Limit/Homing Strategy (recommended)
- After power-on, first execute: All Steppers enter homing: move at safe speed in negative direction→trigger limit→reverse exit certain steps→zero.
- If any limit triggers during operation:
  - Immediately stop related axis;
  - Report error code and current status;
  - Wait for upper layer to issue "reset/continue" command.

---

### 9. Data Flow and Node Topology (Text Version)

1) Camera 1 (RGBD/or RGB):
   - If RGBD: `/camera1/*` → `interbotix_perception_modules` point cloud pipeline → (service) `get_cluster_positions` → target 3D point → `arm_waypoint_controller_node`.
   - If RGB: `/camera1/image_raw` → `color_segmentation_node` → target image centroid/color → (combine calibration/plane model) → 3D point → `arm_waypoint_controller_node`.
2) `arm_waypoint_controller_node`:
   - Generate Waypoint sequence;
   - Send via `/<robot_name>/commands/joint_trajectory` or high-level API;
   - Control gripper (partial open/close).
3) Camera 2 (RGB):
   - `/camera2/image_raw` → `yolo_detection_node` → `/yolo/detections` → `yolo_to_esp32_bridge_node` → `/esp32/detection_commands`.
4) `esp32_motor_controller_node`:
   - Subscribe `/esp32/detection_commands` → generate `/esp32/stepper_cmd`, `/esp32/dc_motor_cmd`, `/esp32/servo_cmd`;
   - Subscribe `/esp32/limit_switch_status` for protection/homing.
5) `micro_ros_agent` ↔ ESP32 (serial/UDP):
   - Responsible for ROS2↔micro-ROS bridging.

---

### 10. Launch and Parameters (Recommendations)

- Robotic arm and perception (including RealSense and point cloud):
  - Directly use existing: `ros2 launch interbotix_xsarm_perception xsarm_perception.launch.py robot_model:=wx200`, and override as needed: `cloud_topic`, `camera_color_topic`, `camera_info_topic`, etc.
- New perception and control:
  - In `widowx_custom_perception` provide combined Launch, merge above nodes (color segmentation, Waypoint control, YOLO, bridge) with `xsarm_perception.launch.py`;
  - In `widowx_esp32_interface` provide ESP32 side Launch (nodes only);
  - Independently launch `micro_ros_agent` (e.g., `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0`).

---

### 11. Clear Annotation: Existing/New/Modification List (Quick Index)

- Existing reuse (no code changes needed):
  - Robotic arm driver and interfaces: `interbotix_xs_sdk`, `interbotix_xs_modules` (`arm.py`, `gripper.py`).
  - Point cloud and calibration: `interbotix_perception_modules` (`pointcloud.py`, `armtag.py`).
  - Perception messages: `interbotix_perception_msgs`.
  - Combined launch: `interbotix_xsarm_perception/launch/xsarm_perception.launch.py`.

- Existing content requiring parameter modification only (non-code):
  - `interbotix_xsarm_perception/config/filter_params.yaml` (scene filter parameters).
  - Parameters of `xsarm_perception.launch.py`: `cloud_topic`, `camera_*_topic`, `ref_frame`, etc.

- New additions (implemented in this project):
  - `widowx_custom_perception` package: `color_segmentation_node`, `arm_waypoint_controller_node`, `yolo_detection_node`, `yolo_to_esp32_bridge_node`;
  - `widowx_esp32_interface` package: `esp32_motor_controller_node`;
  - (Optional) `widowx_custom_msgs` define YOLO/ESP32 related messages.

---

### 12. Quality and Debugging Recommendations

- Layer-by-layer integration: First camera and point cloud, then Waypoint to single-point movement, then trajectory, then gripper, finally connect complete grasping flow.
- Frame system verification: Ensure after `armtag` or static TF is published, camera frame and `/<robot_name>/base_link` are correctly aligned in `rviz`.
- Safety boundaries: Set `z_clearance` before grasping and maximum tilt angle limits;
- Limit protection: When ROS2 side encounters `/esp32/limit_switch_status` trigger, immediately interrupt related axis commands;
- Performance: Separate YOLO inference and color segmentation into different nodes to avoid blocking grasping main chain;
- Parameterization: Make color thresholds, grasping height, speed, gripper pressure, etc. into ROS parameters for easy runtime adjustment.

---

### 13. Summary

Without modifying Interbotix stack core code, this solution achieves through "parameterizing existing perception/driver capabilities + creating small amount of glue and bridge nodes":
- Target extraction based on point cloud or RGB → Waypoint generation → Trajectory following → Gripper partial open/close;
- Second camera YOLO inference → micro-ROS/ESP32 executes peripheral control;
- Key interfaces all reuse existing topics/services/classes, new parts only handle scenario logic and protocol encapsulation, overall maintainability and portability are good.

---

## Appendix A: Visualized System Architecture and Data Flow Diagrams

### A.1 Overall System Architecture Diagram (ASCII)

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                        Overall System Architecture                              │
└─────────────────────────────────────────────────────────────────────────────────┘

╔═════════════════════════════════════════════════════════════════════════════════╗
║                      Camera and Perception Layer                                ║
╚═════════════════════════════════════════════════════════════════════════════════╝

┌──────────────────┐              ┌──────────────────┐
│  Camera 1        │              │  Camera 2        │
│  (External USB)  │              │  (External USB)  │
│  【Hardware】      │              │  【Hardware】      │
└────────┬─────────┘              └────────┬─────────┘
         │                                  │
         │ /camera1/image_raw               │ /camera2/image_raw
         │ /camera1/camera_info             │ /camera2/camera_info
         │ (sensor_msgs/Image)              │ (sensor_msgs/Image)
         ▼                                  ▼
┌──────────────────────────┐      ┌──────────────────────────┐
│  camera1_driver_node     │      │  camera2_driver_node     │
│  【Available Existing】   │      │  【Available Existing】   │
│  usb_cam / v4l2_camera   │      │  usb_cam / v4l2_camera   │
│                          │      │                          │
│  Location: Standard ROS2 │      │  Location: Standard ROS2 │
│  Package                  │      │  Package                  │
└──────────┬───────────────┘      └──────────┬───────────────┘
           │                                  │
           │ /camera1/image_raw               │ /camera2/image_raw
           │                                  │
           ▼                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│               widowx_custom_perception (New Package)                │
│  Location: /home/my11/interbotix_ws/src/widowx_custom_perception/  │
└─────────────────────────────────────────────────────────────────────┘
           │                                  │
           ▼                                  ▼
┌──────────────────────────┐      ┌──────────────────────────┐
│  color_segmentation_node │      │  yolo_detection_node     │
│  【New Node】             │      │  【New Node】             │
│                          │      │                          │
│  Functions:              │      │  Functions:               │
│  - Subscribe camera1 img │      │  - Subscribe camera2 img │
│  - HSV color segment     │      │  - YOLO inference        │
│  - Compute centroid      │      │  - Extract bbox info     │
│  - Publish seg results   │      │  - Publish detections    │
└──────────┬───────────────┘      └──────────┬───────────────┘
           │                                  │
           │ /segmentation/results            │ /yolo/detections
           │ (widowx_custom_msgs/             │ (widowx_custom_msgs/
           │  SegmentationResult)             │  YoloDetectionArray)
           │                                  │
           ▼                                  ▼

╔═════════════════════════════════════════════════════════════════════════════════╗
║                          Task Control Layer                                     ║
╚═════════════════════════════════════════════════════════════════════════════════╝

┌──────────────────────────────────────────────────────────────────────┐
│               widowx_custom_perception (New Package)                 │
│  Location: /home/my11/interbotix_ws/src/widowx_custom_perception/   │
└──────────────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────┐
│  arm_waypoint_controller_node 【New Node】                     │
│                                                               │
│  Functions:                                                   │
│  - Subscribe color seg results                                │
│  - Convert 2D pixel to 3D world coords (use camera_info)      │
│  - Generate arm waypoints                                     │
│  - Call arm control API                                       │
│  - Control gripper open/close                                 │
│                                                               │
│  Dependencies:                                                 │
│  - InterbotixManipulatorXS (Existing API)                    │
│  - tf2 transforms                                             │
└────────┬──────────────────────┬───────────────────────────────┘
         │                      │
         │ Subscribe:           │ Publish/Call:
         │ /segmentation/       │ /wx200/commands/joint_group
         │ results              │ /wx200/commands/joint_single
         │                      │ (interbotix_xs_msgs)
         │                      │
         │                      ▼
         │              ┌────────────────────────────────────┐
         │              │  interbotix_xs_modules 【Existing】│
         │              │  InterbotixManipulatorXS           │
         │              │                                    │
         │              │  Location: src/interbotix_ros_toolbox│
         │              │  es/interbotix_xs_toolbox/         │
         │              │  interbotix_xs_modules/             │
         │              │  interbotix_xs_modules/xs_robot/    │
         │              │  arm.py                            │
         │              │  gripper.py                        │
         │              └────────┬───────────────────────────┘
         │                       │
         │                       │ /wx200/commands/*
         │                       ▼
         │              ┌────────────────────────────────────┐
         │              │  xs_sdk node 【Existing】           │
         │              │  Low-level Driver                  │
         │              │                                    │
         │              │  Location: src/interbotix_ros_core/│
         │              │  interbotix_ros_xseries/          │
         │              │  interbotix_xs_sdk/                │
         │              └────────┬───────────────────────────┘
         │                       │
         │                       │ U2D2/Serial Communication
         │                       ▼
         │              ┌────────────────────────────────────┐
         │              │  WidowX-200 Arm 【Hardware】        │
         │              │  Dynamixel Servos                 │
         │              └────────────────────────────────────┘
         │
         ▼

┌──────────────────────────────────────────────────────────────────────┐
│               widowx_custom_perception (New Package)                 │
└──────────────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────┐
│  yolo_to_esp32_bridge_node 【New Node】                       │
│                                                               │
│  Functions:                                                   │
│  - Subscribe YOLO detections                                  │
│  - Process and encapsulate detection info                     │
│  - Publish to ESP32-specific topic                            │
│  - Optional: Add filtering, tracking logic                    │
└────────┬──────────────────────────────────────────────────────┘
         │
         │ /esp32/detection_commands
         │ (widowx_custom_msgs/DetectionCommand)
         ▼

╔═════════════════════════════════════════════════════════════════════════════════╗
║                      ESP32 Communication and Control Layer                      ║
╚═════════════════════════════════════════════════════════════════════════════════╝

┌──────────────────────────────────────────────────────────────────────┐
│               widowx_esp32_interface (New Package)                  │
│  Location: /home/my11/interbotix_ws/src/widowx_esp32_interface/    │
└──────────────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────┐
│  esp32_motor_controller_node 【New Node】                      │
│                                                               │
│  Functions:                                                   │
│  - Subscribe detection commands                               │
│  - Publish stepper motor control commands                    │
│  - Publish DC motor control commands                         │
│  - Publish servo control commands                            │
│  - Subscribe limit sensor status                             │
│                                                               │
│  Publish Topics:                                              │
│  - /esp32/stepper_cmd (widowx_custom_msgs/StepperCommand)    │
│  - /esp32/dc_motor_cmd (widowx_custom_msgs/DCMotorCommand)  │
│  - /esp32/servo_cmd (std_msgs/Int16MultiArray)               │
│                                                               │
│  Subscribe Topics:                                            │
│  - /esp32/limit_switch_status                                │
│  - /esp32/detection_commands                                 │
└────────┬────────────────────────────┬─────────────────────────┘
         │                            │
         │ Publish Commands            │ Subscribe Feedback
         ▼                            ▼
┌──────────────────────────────────────────────────────────────┐
│  micro_ros_agent 【Available Existing】                      │
│  ROS2 ←→ micro-ROS Bridge                                    │
│                                                              │
│  Launch Command:                                             │
│  ros2 run micro_ros_agent micro_ros_agent serial            │
│      --dev /dev/ttyUSB0                                      │
│                                                              │
│  Location: Standard micro-ros Package                        │
└────────┬─────────────────────────────────────────────────────┘
         │
         │ Serial Communication (Serial/UART)
         ▼
┌──────────────────────────────────────────────────────────────┐
│  ESP32 with micro-ROS 【Hardware】                           │
│                                                              │
│  Firmware Functions:                                         │
│  - Subscribe ROS2 topics                                     │
│  - Control Stepper Motor (Stepper Driver)                   │
│  - Control DC Motor (Motor Driver like L298N)                │
│  - Control Servo (PWM Output)                                │
│  - Read Limit Sensors (GPIO Input)                           │
│  - Publish Sensor Status to ROS2                             │
└────────┬─────────────────────────────────────────────────────┘
         │
         │ GPIO/PWM/Driver Interface
         ▼
┌──────────────────────────────────────────────────────────────┐
│  Peripheral Hardware                                         │
│  - Stepper Motor + Driver (e.g., A4988/DRV8825)            │
│  - DC Motor + Driver (e.g., L298N)                           │
│  - Servo Motors (PWM Control)                                │
│  - Limit Sensors (Limit Switches)                            │
└──────────────────────────────────────────────────────────────┘
```

---

### A.2 Detailed Node Description Table

| Node Name | Package | Status | Location/Description |
|---------|---------|------|-----------|
| **camera1_driver_node** | usb_cam/v4l2_camera | Existing | Standard ROS2 camera driver package |
| **camera2_driver_node** | usb_cam/v4l2_camera | Existing | Standard ROS2 camera driver package |
| **color_segmentation_node** | widowx_custom_perception | **New** | `/src/widowx_custom_perception/scripts/color_segmentation_node.py` |
| **yolo_detection_node** | widowx_custom_perception | **New** | `/src/widowx_custom_perception/scripts/yolo_detection_node.py` |
| **arm_waypoint_controller_node** | widowx_custom_perception | **New** | `/src/widowx_custom_perception/scripts/arm_waypoint_controller_node.py` |
| **yolo_to_esp32_bridge_node** | widowx_custom_perception | **New** | `/src/widowx_custom_perception/scripts/yolo_to_esp32_bridge_node.py` |
| **esp32_motor_controller_node** | widowx_esp32_interface | **New** | `/src/widowx_esp32_interface/scripts/esp32_motor_controller_node.py` |
| **micro_ros_agent** | micro_ros_agent | Existing | Standard micro-ROS package |
| **xs_sdk** | interbotix_xs_sdk | Existing | Robotic arm low-level driver |
| **InterbotixManipulatorXS** | interbotix_xs_modules | Existing(API) | Python API class, not standalone node |

---

### A.3 Custom Message Definitions (`widowx_custom_msgs`)

#### A.3.1 SegmentationResult.msg
```
# Color segmentation result
std_msgs/Header header
string color_name                  # Color name
geometry_msgs/Point centroid       # Centroid coordinates (pixels)
float32 area                       # Region area
sensor_msgs/CameraInfo camera_info # Camera information
```

#### A.3.2 YoloDetection.msg
```
# Single YOLO detection result
string class_name
float32 confidence
geometry_msgs/Point bbox_center
int32 bbox_width
int32 bbox_height
```

#### A.3.3 YoloDetectionArray.msg
```
# YOLO detection result array
std_msgs/Header header
YoloDetection[] detections
```

#### A.3.4 DetectionCommand.msg
```
# Detection command sent to ESP32
std_msgs/Header header
string detected_object
geometry_msgs/Point position
int32 action_code  # Action code (custom)
```

#### A.3.5 StepperCommand.msg
```
# Stepper motor control
int32 motor_id
int32 steps
int32 speed
int32 direction  # 1=forward, -1=reverse
```

#### A.3.6 DCMotorCommand.msg
```
# DC motor control
int32 motor_id
int32 pwm_value  # 0-255
int32 direction  # 1=forward, -1=reverse
```

#### A.3.7 LimitSwitchStatus.msg
```
# Limit sensor status
std_msgs/Header header
bool[] switch_states  # Sensor status array
```

---

### A.4 File Modification List (Detailed Version)

| File Category | Location | Operation Type | Description |
|---------|------|---------|------|
| **No modification to existing files** | - | - | Design completely implemented through new packages and nodes, utilizing existing API interfaces |
| **Optional optimization** | `/config/camera_calibration/` | New | If precise coordinate conversion is needed, add calibration parameters for camera1 and camera2 |
| **Optional optimization** | `interbotix_xsarm_perception/config/filter_params.yaml` | Parameter adjustment | Adjust point cloud filter parameters according to actual scene |

---

### A.5 System Launch Process

#### A.5.1 Main Launch File Structure
Location: `/src/widowx_custom_perception/launch/widowx_custom_system.launch.py`

Launch order:
1. Robotic arm driver (`interbotix_xsarm_control`)
2. Dual camera drivers (`usb_cam` or `v4l2_camera`)
3. Color segmentation node
4. YOLO detection node
5. Arm waypoint controller
6. YOLO to ESP32 bridge
7. ESP32 motor controller

#### A.5.2 Launch Commands

**Terminal 1: Launch micro-ros agent**
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

**Terminal 2: Launch complete system**
```bash
ros2 launch widowx_custom_perception widowx_custom_system.launch.py robot_model:=wx200
```

**Optional: Launch only robotic arm and perception (including RealSense)**
```bash
ros2 launch interbotix_xsarm_perception xsarm_perception.launch.py \
    robot_model:=wx200 \
    cloud_topic:=/camera1/depth/color/points \
    camera_color_topic:=/camera1/color/image_raw \
    camera_info_topic:=/camera1/color/camera_info
```

---

### A.6 Coordinate Transformation Details

#### A.6.1 Camera1 → Robotic Arm Base Coordinate System Transformation

**Method A: Use Existing ArmTag Calibration (Recommended)**
- Tool: `InterbotixArmTagInterface`
- Location: `src/interbotix_ros_toolboxes/interbotix_perception_toolbox/interbotix_perception_modules/interbotix_perception_modules/armtag.py`
- Process:
  1. Attach AprilTag on robotic arm (position needs to be defined in URDF)
  2. Run calibration process to obtain `camera_color_optical_frame` → `/<robot_name>/base_link` transform
  3. Use `interbotix_tf_tools` static TF storage tool to persist transform

**Method B: Manual Calibration**
- Use `tf2_ros.StaticTransformBroadcaster` to publish static transform
- Need to measure or calculate relative pose between camera and robotic arm through multi-point calibration

**Method C: Use Point Cloud Direct Output (Simplest)**
- If using depth camera, `InterbotixPointCloudInterface.get_cluster_positions` can directly output target position in `/<robot_name>/base_link` coordinate system
- Prerequisite: TF chain from camera→base is established

---

### A.7 Technology Stack and Dependencies

| Component | Recommended Technology | Installation Method |
|-----|---------|---------|
| **Color Segmentation** | OpenCV (cv2) | `pip install opencv-python` |
| **YOLO Inference** | YOLOv8 (ultralytics) | `pip install ultralytics` |
| **YOLO Inference (Alternative)** | YOLOv5 (PyTorch) | `pip install torch torchvision` |
| **Coordinate Transformation** | tf2_ros + tf2_geometry_msgs | Included by default in ROS2 |
| **ESP32 Firmware** | micro_ros_arduino | Arduino Library Manager |
| **ESP32 Firmware (Alternative)** | micro-ROS for ESP-IDF | ESP-IDF Component |
| **Camera Driver** | usb_cam | `sudo apt install ros-${ROS_DISTRO}-usb-cam` |
| **Camera Driver (Alternative)** | v4l2_camera | `sudo apt install ros-${ROS_DISTRO}-v4l2-camera` |
| **micro-ROS Agent** | micro_ros_agent | `sudo apt install ros-${ROS_DISTRO}-micro-ros-agent` |

---

### A.8 Development Priorities and Milestones

#### Phase 1: Camera Driver + Color Segmentation (1-2 days)
- [ ] Configure external USB camera driver
- [ ] Implement `color_segmentation_node`
- [ ] Test color segmentation effect, publish results to topic
- [ ] Verify: Visualize segmentation results in RViz

#### Phase 2: Robotic Arm Control Integration (2-3 days)
- [ ] Implement `arm_waypoint_controller_node`
- [ ] Integrate `InterbotixManipulatorXS` API
- [ ] Test single-point movement (based on fixed coordinates)
- [ ] Test Waypoint sequence execution
- [ ] Test Gripper partial open/close
- [ ] Verify: Robotic arm can move and grasp according to color segmentation results

#### Phase 3: YOLO Detection (2-3 days)
- [ ] Configure second USB camera
- [ ] Implement `yolo_detection_node`
- [ ] Load YOLO model and test inference performance
- [ ] Publish detection results to topic
- [ ] Verify: Detection boxes visible in RViz or image window

#### Phase 4: ESP32 micro-ROS Integration (3-4 days)
- [ ] Configure ESP32 development environment (Arduino or ESP-IDF)
- [ ] Write micro-ROS firmware (subscribe/publish topics)
- [ ] Implement stepper motor, DC motor, servo control logic
- [ ] Implement limit sensor reading and reporting
- [ ] Implement `yolo_to_esp32_bridge_node`
- [ ] Implement `esp32_motor_controller_node`
- [ ] Verify: ROS2 commands can correctly control ESP32 peripherals

#### Phase 5: System Integration and Testing (2-3 days)
- [ ] Create unified launch file
- [ ] End-to-end test: Color segmentation→Robotic arm grasping
- [ ] End-to-end test: YOLO detection→ESP32 response
- [ ] Performance optimization (reduce latency, increase frame rate)
- [ ] Write user documentation and troubleshooting guide

---

### A.9 Key Interface Quick Reference

#### Robotic Arm Control API (Existing)
```python
# InterbotixManipulatorXS
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

bot = InterbotixManipulatorXS(robot_model='wx200', robot_name='wx200')

# Set end-effector pose
bot.arm.set_ee_pose_components(x=0.3, y=0.0, z=0.2, pitch=0.5)

# Cartesian trajectory
bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.05)

# Gripper control
bot.gripper.grasp(delay=1.0)         # Grasp
bot.gripper.release(delay=1.0)       # Release
bot.gripper.set_pressure(0.5)         # Set pressure (0.0-1.0)

# Sleep/Home pose
bot.arm.go_to_sleep_pose()
bot.arm.go_to_home_pose()
```

#### Point Cloud Clustering API (Existing)
```python
# InterbotixPointCloudInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface

pcl = InterbotixPointCloudInterface(node_inf=global_node)

# Get cluster positions
success, clusters = pcl.get_cluster_positions(
    ref_frame='wx200/base_link',
    sort_axis='y',
    reverse=True
)

# clusters = [{'position': [x,y,z], 'color': [r,g,b], ...}, ...]
```

#### Coordinate Calibration API (Existing)
```python
# InterbotixArmTagInterface
from interbotix_perception_modules.armtag import InterbotixArmTagInterface

armtag = InterbotixArmTagInterface(
    ref_frame='camera_color_optical_frame',
    arm_tag_frame='wx200/ar_tag_link',
    arm_base_frame='wx200/base_link',
    node_inf=global_node
)

# Execute calibration
armtag.find_ref_to_arm_base_transform()
```

---

### A.10 Troubleshooting Checklist

#### Robotic Arm Cannot Connect
- [ ] Check if U2D2 is plugged in and recognized (`ls /dev/ttyUSB*`)
- [ ] Check if servo power is on
- [ ] Check if udev rules are configured (refer to `99-interbotix-udev.rules`)
- [ ] Try restarting `xs_sdk` node

#### Camera Cannot Be Recognized
- [ ] Check USB connection (`ls /dev/video*`)
- [ ] Test camera (`v4l2-ctl --list-devices`)
- [ ] Confirm device path in launch file is correct
- [ ] Check camera driver parameters (whether resolution, frame rate are supported)

#### ESP32 Cannot Communicate
- [ ] Check serial connection (`ls /dev/ttyUSB*` or `/dev/ttyACM*`)
- [ ] Confirm `micro_ros_agent` is running
- [ ] Check if ESP32 firmware is correctly flashed
- [ ] Check serial monitor to confirm micro-ROS initialization success
- [ ] Try different baud rate (default 115200)

#### Coordinate Transformation Inaccurate
- [ ] Check if TF tree is complete (`ros2 run tf2_tools view_frames`)
- [ ] Visualize coordinate system alignment in RViz
- [ ] Re-execute ArmTag calibration
- [ ] Check if camera calibration parameters are loaded

#### YOLO Inference Slow
- [ ] Reduce input image resolution
- [ ] Use smaller YOLO model (e.g., YOLOv8n)
- [ ] Enable GPU acceleration (install CUDA version PyTorch)
- [ ] Reduce inference frequency (frame skipping)

---

**Document Version**: v1.0  
**Last Updated**: 2025-11-07  
**Maintainer**: Project Team

