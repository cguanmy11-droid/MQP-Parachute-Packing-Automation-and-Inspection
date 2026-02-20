# Loop Visualization System - Architecture Plan

This document describes the planned architecture for integrating YOLO loop detection with the digital twin visualization, supporting both real camera operation and simulation/test modes.

## Overview

The system must support two operational modes:
1. **Real Camera Mode**: YOLO detects loops in camera frame → transform to world frame for visualization
2. **Simulation Mode**: Ground truth loops exist in world frame → simulate what camera would detect

In both cases, the camera is mounted on the side arm and moves with it.

---

## Current Architecture (Simple)

```
┌────────────────────┐     ┌─────────────────┐     ┌──────────────────┐
│test_loop_publisher │────▶│  /detected_loops │────▶│ loop_visualizer  │
│  (hardcoded)       │     │  (camera_frame)  │     │  (camera_frame)  │
└────────────────────┘     └─────────────────┘     └──────────────────┘
                                                            │
                                                            ▼
                                                         RViz
```

**Problems:**
- Camera frame is static (doesn't move with side arm)
- No ground truth loop positions
- Visualization in camera_frame, not world frame
- Can't simulate realistic detection based on camera position

---

## New Architecture

### TF Tree (Updated)

```
world
└── wx200/base_link
    ├── framemodel_root
    │   ├── framemodel (packing frame mesh)
    │   └── parachute_bag (bag mesh)
    │
    ├── side_arm_origin (static TF)
    │   └── side_arm_hook (from side_arm_visualizer, dynamic position)
    │       └── camera_frame (fixed offset from hook)
    │
    └── loop_ground_truth (virtual frame for loop spawning)
        └── (loop markers visualized here)
```

### Node Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           GROUND TRUTH LAYER                                │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    loop_ground_truth_node                            │   │
│  │  - Maintains list of loop positions in world frame                   │   │
│  │  - Publishes /loop_ground_truth (LoopGroundTruth msg)               │   │
│  │  - Publishes /loop_ground_truth_markers (MarkerArray) for RViz      │   │
│  │  - Configurable: static positions, random, or from file             │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      │ /loop_ground_truth
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           DETECTION LAYER                                   │
│                                                                             │
│   ┌─────────────────────────┐         ┌─────────────────────────┐          │
│   │   SIMULATION MODE       │         │    REAL CAMERA MODE     │          │
│   │                         │         │                         │          │
│   │ detection_simulator_node│         │  yolo_detector_node     │          │
│   │  - Subscribes to        │         │  - Real camera input    │          │
│   │    /loop_ground_truth   │         │  - YOLO inference       │          │
│   │  - Looks up TF:         │         │  - Outputs pixel coords │          │
│   │    world → camera_frame │         │                         │          │
│   │  - Checks camera FOV    │         │         │               │          │
│   │  - Simulates detection  │         │         ▼               │          │
│   │    noise/confidence     │         │  loop_detector_node     │          │
│   │                         │         │  - Converts pixels to   │          │
│   │                         │         │    camera_frame coords  │          │
│   └───────────┬─────────────┘         └───────────┬─────────────┘          │
│               │                                   │                         │
│               └───────────────┬───────────────────┘                         │
│                               │                                             │
│                               ▼                                             │
│                      /detected_loops                                        │
│                      (DetectedLoops, camera_frame)                          │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         VISUALIZATION LAYER                                 │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      loop_visualizer_node                            │   │
│  │  - Subscribes to /detected_loops (camera_frame)                      │   │
│  │  - Looks up TF: camera_frame → world                                 │   │
│  │  - Transforms detections to world frame                              │   │
│  │  - Publishes /detected_loop_markers (MarkerArray, world frame)       │   │
│  │  - Different color from ground truth to show detection vs reality    │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
                                    RViz
                        (shows both ground truth and detections)
```

---

## Node Specifications

### 1. loop_ground_truth_node

**Purpose**: Maintain and publish the "real" loop positions in the world frame.

**Subscriptions**: None (or optional service to add/remove loops)

**Publications**:
| Topic | Type | Frame | Description |
|-------|------|-------|-------------|
| `/loop_ground_truth` | `LoopGroundTruth` (new msg) | world | Loop positions for other nodes |
| `/loop_ground_truth_markers` | `MarkerArray` | world | Visualization of actual loops |

**Parameters**:
```yaml
loop_positions:  # List of [x, y, z] in world frame
  - [0.35, -0.05, 0.10]
  - [0.40, -0.05, 0.08]
  - [0.45, -0.05, 0.12]
pattern: "static"  # static, random, grid, file
frame_id: "world"
marker_color: [0.2, 0.2, 0.8, 0.8]  # Blue, semi-transparent
marker_scale: 0.02
```

**Behavior**:
- On startup, initialize loop positions based on pattern parameter
- Publish ground truth at steady rate (e.g., 10 Hz)
- Ground truth markers rendered as wireframe or semi-transparent to distinguish from detections

---

### 2. detection_simulator_node

**Purpose**: Simulate what the camera would detect based on ground truth loops and camera position.

**Subscriptions**:
| Topic | Type | Description |
|-------|------|-------------|
| `/loop_ground_truth` | `LoopGroundTruth` | Actual loop positions |

**Publications**:
| Topic | Type | Frame | Description |
|-------|------|-------|-------------|
| `/detected_loops` | `DetectedLoops` | camera_frame | Simulated detections |

**TF Lookups**:
- `world` → `camera_frame` (to transform loop positions into camera view)

**Parameters**:
```yaml
camera_fov_horizontal: 60.0  # degrees
camera_fov_vertical: 45.0    # degrees
max_detection_range: 0.5     # meters
min_detection_range: 0.05    # meters
detection_noise_stddev: 0.005  # meters, position noise
confidence_base: 0.90
confidence_noise: 0.05
publish_rate: 5.0  # Hz
```

**Behavior**:
1. Receive ground truth loop positions (world frame)
2. For each loop:
   a. Look up TF: world → camera_frame
   b. Transform loop position to camera_frame
   c. Check if loop is within camera FOV and range
   d. If visible, add to detections with simulated noise and confidence
3. Publish DetectedLoops message in camera_frame

**Visibility Check** (in camera_frame, camera looks along +Z):
```python
def is_in_fov(point_camera_frame):
    x, y, z = point_camera_frame
    if z < min_range or z > max_range:
        return False
    angle_h = math.atan2(x, z)
    angle_v = math.atan2(y, z)
    return (abs(angle_h) < fov_h/2) and (abs(angle_v) < fov_v/2)
```

---

### 3. loop_visualizer_node (Modified)

**Purpose**: Transform detected loops from camera_frame to world frame for visualization.

**Subscriptions**:
| Topic | Type | Description |
|-------|------|-------------|
| `/detected_loops` | `DetectedLoops` | Detections in camera_frame |

**Publications**:
| Topic | Type | Frame | Description |
|-------|------|-------|-------------|
| `/detected_loop_markers` | `MarkerArray` | world | Visualization of detections |

**TF Lookups**:
- `camera_frame` → `world` (to transform detections for display)

**Parameters**:
```yaml
frame_id: "world"  # Output frame for markers
marker_color: [0.2, 0.8, 0.2, 1.0]  # Green, solid
selected_color: [1.0, 0.2, 0.2, 1.0]  # Red for target
marker_scale: 0.015
show_detection_rays: true  # Optional: show ray from camera to detection
```

**Behavior**:
1. Receive DetectedLoops (camera_frame)
2. For each detection:
   a. Look up TF: camera_frame → world
   b. Transform detection position to world frame
   c. Create marker at world position
3. Highlight rightmost detection (target)
4. Publish MarkerArray in world frame

---

### 4. camera_frame TF Publisher

**Option A: Static offset from side_arm_hook**

Add to `side_arm_visualizer.py` - publish camera_frame as child of hook position.

**Option B: Dedicated node**

New node that subscribes to side arm state and publishes camera_frame TF.

**Transform Definition**:
```yaml
# Camera mounted on hook, looking outward
parent_frame: "side_arm_origin"
child_frame: "camera_frame"
# Offset from side arm origin (adjust based on actual mounting)
translation: [x_offset, y_offset, z_offset]  # To be calibrated
# Rotation to align camera axes:
#   Camera convention: +Z forward (into scene), +X right, +Y down
#   Robot convention: varies
rotation_rpy: [roll, pitch, yaw]  # To be calibrated
```

---

## Message Definitions

### New Message: LoopGroundTruth

**File**: `parachute_interfaces/msg/LoopGroundTruth.msg`

```
# Ground truth loop positions for simulation

std_msgs/Header header
geometry_msgs/Point[] positions    # Loop centers in header.frame_id
int32 count                        # Number of loops
```

---

## Data Flow Diagrams

### Simulation Mode

```
                    loop_ground_truth_node
                            │
            ┌───────────────┴───────────────┐
            │                               │
            ▼                               ▼
    /loop_ground_truth              /loop_ground_truth_markers
    (LoopGroundTruth)                   (MarkerArray)
            │                               │
            ▼                               │
  detection_simulator_node                  │
            │                               │
            │  TF lookup:                   │
            │  world → camera_frame         │
            │                               │
            ▼                               │
    /detected_loops                         │
    (DetectedLoops, camera_frame)           │
            │                               │
            ▼                               │
    loop_visualizer_node                    │
            │                               │
            │  TF lookup:                   │
            │  camera_frame → world         │
            │                               │
            ▼                               ▼
    /detected_loop_markers ─────────────▶ RViz
    (MarkerArray, world)              (shows both)
```

### Real Camera Mode

```
        USB Camera
            │
            ▼
    yolo_detector_node
            │
            ▼
      /yolo/centers
      (PoseArray, pixels)
            │
            ▼
    loop_detector_node
            │
            │  Convert pixels → camera_frame coords
            │
            ▼
    /detected_loops
    (DetectedLoops, camera_frame)
            │
            ▼
    loop_visualizer_node
            │
            │  TF lookup:
            │  camera_frame → world
            │
            ▼
    /detected_loop_markers ─────────────▶ RViz
    (MarkerArray, world)
```

---

## TF Relationships

### Dynamic TF Chain for Camera

```
world (static)
  │
  └──▶ wx200/base_link (from robot_state_publisher)
         │
         └──▶ side_arm_origin (static TF)
                │
                └──▶ side_arm_position (dynamic, from side_arm state)
                       │
                       └──▶ camera_frame (static offset from hook)
```

**Key Insight**: The `camera_frame` position in world coordinates changes as the side arm moves. This is handled automatically by TF if we set up the chain correctly.

### Required TF Publishers

| Publisher | Parent | Child | Type |
|-----------|--------|-------|------|
| `side_arm_static_tf` | `wx200/base_link` | `side_arm_origin` | Static |
| `side_arm_visualizer` | `side_arm_origin` | `side_arm_hook` | Dynamic |
| `camera_tf_publisher` | `side_arm_hook` | `camera_frame` | Static |

---

## Implementation Steps

### Phase 1: Fix Camera Frame TF

1. Modify `side_arm_visualizer.py` to publish `side_arm_hook` TF (not just marker)
2. Add static TF from `side_arm_hook` to `camera_frame`
3. Verify in RViz that camera_frame moves with side arm

### Phase 2: Ground Truth Node

1. Create `LoopGroundTruth.msg` in parachute_interfaces
2. Create `loop_ground_truth_node.py`
3. Test: verify blue loop markers appear at fixed world positions

### Phase 3: Detection Simulator

1. Create `detection_simulator_node.py`
2. Implement TF lookup and FOV checking
3. Test: verify detections only appear when loops are in camera view

### Phase 4: Modify Visualizer

1. Update `loop_visualizer_node.py` to transform camera_frame → world
2. Test: verify green detection markers align with blue ground truth when camera points at loops

### Phase 5: Integration & Calibration

1. Calibrate camera_frame offset/rotation relative to hook
2. Tune FOV parameters to match real camera
3. Test full pipeline with side arm movement

---

## Launch Configuration

### Simulation Mode
```bash
ros2 launch parachute_coordinator dual_arm_test.launch.py \
    vision_test_mode:=true \
    enable_loop_visualization:=true
```

Launches:
- `loop_ground_truth_node`
- `detection_simulator_node`
- `loop_visualizer_node`
- Camera frame TF chain

### Real Camera Mode
```bash
ros2 launch parachute_coordinator dual_arm_test.launch.py \
    vision_test_mode:=false \
    enable_loop_visualization:=true
```

Launches:
- `yolo_detector_node`
- `loop_detector_node`
- `loop_visualizer_node`
- Camera frame TF chain

---

## RViz Visualization

| Marker Topic | Color | Meaning |
|--------------|-------|---------|
| `/loop_ground_truth_markers` | Blue, wireframe | Actual loop positions (simulation only) |
| `/detected_loop_markers` | Green, solid | Where system thinks loops are |
| `/detected_loop_markers` (selected) | Red, solid | Current target loop |

**Visual Debugging**:
- If green markers don't align with blue → detection/TF issue
- If green markers only appear when camera faces loops → FOV working correctly
- If green markers jitter → noise simulation working

---

## File Changes Summary

| File | Action | Description |
|------|--------|-------------|
| `parachute_interfaces/msg/LoopGroundTruth.msg` | Create | New message type |
| `parachute_perception/loop_ground_truth_node.py` | Create | Ground truth publisher |
| `parachute_perception/detection_simulator_node.py` | Create | Simulates camera detection |
| `parachute_perception/loop_visualizer_node.py` | Modify | Add TF transform to world |
| `parachute_perception/test_loop_publisher_node.py` | Remove/Replace | Replaced by above nodes |
| `side_arm_control/side_arm_visualizer.py` | Modify | Publish TF for hook position |
| `parachute_coordinator/launch/dual_arm_test.launch.py` | Modify | Add new nodes, camera TF |
| `main_arm_control/config/simulation.rviz` | Modify | Add ground truth markers display |

---

## Open Questions / Decisions Needed

1. **Camera mounting**: Exact offset and orientation of camera relative to hook tip?
2. **Camera FOV**: What are the actual horizontal/vertical FOV values?
3. **Detection range**: Min/max distance at which loops can be detected?
4. **Coordinate conventions**: Which way does camera +Z point? (typically into scene)
5. **Should ground truth be visible in real camera mode?** (for debugging alignment)
