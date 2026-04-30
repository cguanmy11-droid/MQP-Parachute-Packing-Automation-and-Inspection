# Automated Parachute Line Stowing System - State Machine Documentation

## System Overview

This system automates the process of stowing parachute suspension lines using a **dual-arm robotic system** with **computer vision guidance**. The system uses:

- **Main Arm (WX200)**: A 5-DOF robotic arm for manipulating parachute lines
- **Side Arm (Custom 3-axis gantry)**: A hook mechanism for capturing and guiding lines through loops
- **YOLO-based Vision System**: Real-time detection of parachute loops using a USB camera
- **ROS 2 State Machine**: Event-driven coordinator orchestrating the entire stowing sequence

---

## State Machine Architecture

### State Diagram

```
                              ┌─────────────────────────────────────────┐
                              │                                         │
                              ▼                                         │
    ┌──────────┐  start   ┌──────────┐  homed   ┌──────────┐           │
    │   IDLE   │ ───────► │  HOMING  │ ───────► │ AT_LOOP  │ ◄─────────┤
    └──────────┘          └──────────┘          └──────────┘           │
         ▲                      │                    │                  │
         │                      │ fail          positioned              │
         │                      ▼                    │                  │
         │                ┌──────────┐               ▼                  │
         │ abort          │  ERROR   │ ◄────── ┌──────────┐             │
         │◄───────────────│          │         │  INSERT  │─┐           │
         │                └──────────┘         └──────────┘ │ collision │
         │                      │                    │      │ (retry)   │
         │                  retry│               inserted   └───────────┘
         │                      │                    │
         │                      ▼                    ▼
         │               (returns to           ┌──────────┐
         │                error source)        │ HANDOFF  │
         │                                     └──────────┘
         │                                          │
         │                                   trajectory_complete
         │                                          │
         │                                          ▼
         │                                     ┌──────────┐
         │                                     │ RETRACT  │
         │                                     └──────────┘
         │                                          │
         │                                      retracted
         │                                          │
         │                                          ▼
         │                                     ┌──────────┐    loops_remaining
         │              ┌──────────┐           │ RELEASE  │ ──────────────────►
         └───────────── │ COMPLETE │ ◄──────── └──────────┘
                        └──────────┘  all_complete
```

### States Description

| State | Description | Key Actions |
|-------|-------------|-------------|
| **IDLE** | System ready, waiting for operator | Arms homed, ready for start command |
| **HOMING** | Initializing side arm position | Sends HOME_ALL, captures loop positions |
| **AT_LOOP** | Targeting next loop | Calls `/request_next_target` service, positions arms |
| **INSERT** | Hook insertion | Vision servo centering, hook through loop |
| **HANDOFF** | Dual-arm coordination | Rotates hook 90°, executes stow trajectory |
| **RETRACT** | Hook withdrawal | Rotates hook again, retracts through loop |
| **RELEASE** | Cycle completion | Verifies stow quality, increments counter |
| **COMPLETE** | All loops stowed | Success state, can restart |
| **ERROR** | Fault handling | Halts motion, awaits operator recovery |

---

## Key Components

### ROS 2 Nodes

| Node | Package | Purpose |
|------|---------|---------|
| `packing_coordinator_node` | parachute_coordinator | State machine controller |
| `target_selector_node` | parachute_perception | Loop selection and ordering |
| `yolo_detector` | yolo_detect_ros | Real-time loop detection |
| `camera_to_3d_node` | parachute_perception | Pixel to 3D coordinate conversion |
| `loop_visualizer_node` | parachute_perception | RViz visualization |
| `main_arm_interface_node` | main_arm_control | WX200 arm control |
| `side_arm_interface_node` | side_arm_control | Gantry hook control |
| `side_arm_coordinate_node` | side_arm_control | Motor command translation |

### Topic Flow

```
USB Camera
    │
    ▼
┌──────────────┐    /yolo/centers    ┌────────────────┐   /detected_loops   ┌───────────────────┐
│ yolo_detector│ ─────────────────► │ camera_to_3d   │ ─────────────────► │ target_selector   │
└──────────────┘   (pixel coords)   └────────────────┘   (3D world coords) └───────────────────┘
                                                                                     │
                                                                    /request_next_target (service)
                                                                                     │
                                                                                     ▼
                                                                           ┌─────────────────────┐
                                                                           │ packing_coordinator │
                                                                           └─────────────────────┘
                                                                                     │
                                          ┌──────────────────────────────────────────┼──────────────┐
                                          │                                          │              │
                                          ▼                                          ▼              ▼
                                 /side_arm/insert_hook                    /main_arm/execute   /stow/status
                                       (action)                             _trajectory
                                                                              (action)
```

### Services

| Service | Type | Provider | Purpose |
|---------|------|----------|---------|
| `/request_next_target` | RequestNextTarget | target_selector_node | Get next loop to stow |
| `/capture_loops` | CaptureLoops | target_selector_node | Snapshot loop positions |
| `/side_arm/rotate_hook` | RotateHook | side_arm_interface_node | Rotate hook mechanism |
| `/side_arm/move_to_position` | MoveToPosition | side_arm_interface_node | Direct position move |

### Actions

| Action | Type | Provider | Purpose |
|--------|------|----------|---------|
| `/side_arm/insert_hook` | InsertHook | side_arm_interface_node | Hook insertion sequence |
| `/main_arm/execute_trajectory` | ExecuteTrajectory | main_arm_interface_node | Execute stow trajectory |
| `/side_arm/move_to_coordinate` | MoveToCoordinate | coordinate_node | Position control with feedback |

---

## Vision Pipeline

### YOLO Loop Detection

1. **Camera Input**: USB camera captures frames at 30 FPS
2. **YOLO Inference**: Custom-trained YOLOv8 model detects parachute loops
3. **Pixel Centers**: Detection centers published as pixel coordinates
4. **3D Conversion**: Using camera intrinsics and assumed depth, convert to 3D
5. **TF Transform**: Transform from camera_frame to world frame

### Camera Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_index` | 4 | USB camera device index |
| `image_width` | 640 | Image width in pixels |
| `image_height` | 480 | Image height in pixels |
| `camera_fov` | 80.0 | Horizontal field of view (degrees) |
| `assumed_depth` | 0.22 | Distance from camera to loop plane (meters) |
| `conf_threshold` | 0.5 | YOLO confidence threshold |

### Target Selection Strategies

| Strategy | Description |
|----------|-------------|
| `leftmost` | Process loops from left to right |
| `rightmost` | Process loops from right to left |
| `nearest` | Process closest loop first |

---

## Coordinate System

### Frame Hierarchy

```
world (base frame)
    │
    ├── wx200/base_link (main arm base)
    │       └── ... (arm links)
    │
    ├── framemodel_root (physical frame structure)
    │
    └── side_arm_origin (gantry origin)
            └── y_carriage_link
                    └── camera_frame (attached to Y carriage)
```

### Side Arm Coordinate Mapping

The side arm uses a custom coordinate transformation:

| Axis | Direction | Range | Notes |
|------|-----------|-------|-------|
| X | Inverted (SA X+ = World X-) | 0-300mm | Horizontal rails |
| Y | Normal | 0-50mm | Vertical lead screw |
| Z | Normal | 0-180mm | Depth lead screw |

**Hook Offset Calibration** (position when arm is homed):
- Hook X: 350mm in world frame
- Hook Y: 180mm in world frame
- Hook Z: -10mm in world frame

**Conversion Formula**:
```
arm_x = hook_offset_x - world_x  (inverted)
arm_y = world_y - hook_offset_y  (normal)
arm_z = world_z - hook_offset_z  (normal)
```

---

## Motion Patterns

The system supports configurable stow trajectories defined as motion patterns:

### Built-in Patterns

| Pattern | Description |
|---------|-------------|
| `square_stow` | Square motion path for stowing |
| `recorded_stow` | Pre-recorded optimal trajectory |
| `direct` | Straight-line movement |

### Pattern Configuration

Patterns can be defined in JSON files with waypoints:
```json
{
  "name": "square_stow",
  "speed_factor": 0.5,
  "waypoints": [
    {"x": 0.0, "y": 0.0, "z": 0.0},
    {"x": 0.05, "y": 0.0, "z": -0.02},
    ...
  ]
}
```

---

## Launch Files

### State Machine with Real Camera

```bash
ros2 launch parachute_coordinator state_machine_demo.launch.py \
    use_real_camera:=true
```

### Key Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `main_arm_sim` | false | Run main arm in simulation |
| `side_arm_sim` | false | Run side arm in simulation |
| `use_real_camera` | false | Use USB camera + YOLO |
| `vision_test_mode` | false | Use simulated loop detections |
| `use_test_loops` | false | Use hardcoded test positions |
| `camera_index` | 4 | Camera device index |
| `selection_strategy` | leftmost | Loop ordering strategy |
| `stow_pattern` | square_stow | Motion pattern name |
| `expected_loop_count` | 0 | Expected loops (0 = auto) |

### Common Launch Configurations

```bash
# Full hardware with real camera
ros2 launch parachute_coordinator state_machine_demo.launch.py \
    use_real_camera:=true

# Simulation mode (no hardware)
ros2 launch parachute_coordinator state_machine_demo.launch.py \
    main_arm_sim:=true side_arm_sim:=true vision_test_mode:=true

# Hardware with simulated vision
ros2 launch parachute_coordinator state_machine_demo.launch.py \
    vision_test_mode:=true

# Side arm only testing
ros2 launch parachute_coordinator dual_arm_test.launch.py \
    enable_main_arm:=false use_real_camera:=true
```

---

## Operator Commands

Commands are sent via `/stow/command` topic:

```bash
# Start the stowing sequence
ros2 topic pub --once /stow/command std_msgs/String "data: start"

# Check current status
ros2 topic pub --once /stow/command std_msgs/String "data: status"

# Emergency stop and reset
ros2 topic pub --once /stow/command std_msgs/String "data: stop"

# Change motion pattern
ros2 topic pub --once /stow/command std_msgs/String "data: pattern:recorded_stow"

# Error recovery
ros2 topic pub --once /stow/command std_msgs/String "data: retry"
ros2 topic pub --once /stow/command std_msgs/String "data: skip"
ros2 topic pub --once /stow/command std_msgs/String "data: abort"
```

---

## Error Handling

### Error Sources

| Error | Source State | Recovery Options |
|-------|--------------|------------------|
| `homing_failed` | HOMING | Check hardware, retry |
| `vision_failure` | AT_LOOP | Check camera, lighting |
| `ik_failure` | AT_LOOP | Adjust target position |
| `collision` | INSERT | Auto-retry with offset |
| `max_retries` | INSERT | Skip or manual intervention |
| `trajectory_failure` | HANDOFF | Check IK, waypoints |
| `excessive_force` | RETRACT | Check for jams |

### Recovery Flow

1. Error occurs → System enters ERROR state
2. All motion halted, diagnostics logged
3. Operator chooses:
   - `retry`: Return to error source state
   - `skip`: Skip current loop, try next
   - `abort`: Return to IDLE

---

## System Performance

### Timing (Typical Values)

| Phase | Duration |
|-------|----------|
| Homing | 10-30 seconds |
| Target acquisition | <1 second |
| Hook insertion | 3-5 seconds |
| Vision servo centering | 1-3 seconds |
| Stow trajectory | 5-10 seconds |
| Hook retraction | 2-4 seconds |
| **Total per loop** | **~20-45 seconds** |

### Vision Specifications

| Metric | Value |
|--------|-------|
| Detection rate | 30 FPS |
| YOLO confidence | >50% |
| Position accuracy | ±5mm |
| Depth assumption | 220mm |

---

## Design Principles

1. **Event-driven transitions**: State changes happen because actions complete, not timers
2. **Thin coordinator**: Send high-level goals, let arm nodes handle details
3. **Single source of truth**: One detected loop drives both arms
4. **Fail safely**: Every action has timeout, every failure goes to ERROR
5. **Operator in the loop**: ERROR state requires human decision to continue

---

## References

- MQP Paper: Figure 6 (State Machine Diagram)
- ROS 2 Humble Documentation
- YOLOv8 Ultralytics Documentation
- Interbotix WX200 Documentation
