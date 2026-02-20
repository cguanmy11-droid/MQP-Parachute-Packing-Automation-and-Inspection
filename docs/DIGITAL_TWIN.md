# Digital Twin Visualization System

This document describes the RViz-based digital twin visualization system for the parachute packing automation project.

## Overview

The digital twin provides a real-time 3D visualization of:
- **Main Arm**: WidowX-200 robot arm (Interbotix XS Series)
- **Side Arm**: Custom hook mechanism for loop manipulation
- **Packing Frame**: Aluminum frame structure with parachute bag
- **Detected Loops**: YOLO-detected suspension loops (planned)

## Architecture

```
                    ┌─────────────────────────────────────────────────┐
                    │                    RViz                         │
                    │  ┌─────────────┐  ┌─────────────┐  ┌─────────┐ │
                    │  │ RobotModel  │  │ RobotModel  │  │ Markers │ │
                    │  │ (main arm)  │  │  (frame)    │  │ (hook)  │ │
                    │  └──────▲──────┘  └──────▲──────┘  └────▲────┘ │
                    └─────────┼───────────────┼───────────────┼──────┘
                              │               │               │
          ┌───────────────────┴───┐   ┌───────┴───────┐   ┌──┴──────────────┐
          │   robot_description   │   │/frame_description│  │/side_arm_marker │
          │      (topic)          │   │    (topic)       │  │    (topic)      │
          └───────────────────────┘   └─────────────────-┘  └─────────────────┘
                    ▲                         ▲                     ▲
                    │                         │                     │
     ┌──────────────┴──────────────┐  ┌───────┴───────┐  ┌─────────┴─────────┐
     │  Interbotix xsarm_control   │  │robot_state_pub│  │side_arm_visualizer│
     │  (robot_state_publisher)    │  │   (frame)     │  │     (node)        │
     └─────────────────────────────┘  └───────────────┘  └─────────────────-─┘
                                                                  ▲
                                                                  │
                                                    ┌─────────────┴─────────────┐
                                                    │  /side_arm/parsed_state   │
                                                    │     (SideArmState)        │
                                                    └───────────────────────────┘
```

## Components

### 1. Main Arm (WidowX-200)

**Package**: `main_arm_control`

The main arm visualization uses the Interbotix SDK's built-in TF broadcasting:

| Component | Details |
|-----------|---------|
| URDF | Provided by Interbotix SDK |
| TF Root | `world` → `wx200/base_link` |
| End Effector | `wx200/ee_gripper_link` |
| Topic | `robot_description` |

### 2. Packing Frame Assembly

**Package**: `main_arm_control`

| File | Description | Color |
|------|-------------|-------|
| `meshes/framemodel.stl` | Aluminum packing frame (515 KB) | Silver (0.8, 0.8, 0.85) |
| `meshes/parachute_bag.stl` | Parachute storage bag (2.25 MB) | Green (0.2, 0.4, 0.2) |

**URDF**: `urdf/framemodel.urdf`

**TF Transform** (relative to `wx200/base_link`):
- Position: (0, -0.15, -0.22) meters
- Orientation: RPY (1.5708, 0, -1.5708) radians

### 3. Side Arm Hook

**Package**: `side_arm_control`

**Visualizer Node**: `side_arm_visualizer.py`

The hook is rendered as an RViz Marker (MESH_RESOURCE) rather than URDF because its position is dynamically controlled.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `roll` | 3.1416 | Base orientation roll (radians) |
| `pitch` | 1.5708 | Base orientation pitch (radians) |
| `yaw` | 0.0 | Base orientation yaw (radians) |
| `offset_x` | 0.0 | Mesh origin offset X (meters) |
| `offset_y` | 0.05 | Mesh origin offset Y (meters) |
| `offset_z` | 0.0 | Mesh origin offset Z (meters) |
| `scale` | 0.001 | Mesh scale (mm to meters) |
| `servo_axis` | 'yaw' | Axis for servo rotation |
| `servo_scale` | 0.002 | Radians per microsecond |
| `test_mode` | false | Use test position instead of live data |

**Color Coding**:
- Gray (0.3, 0.3, 0.3): Homed state
- Orange (1.0, 0.5, 0.0): Not homed

**Input Topics**:
- `/side_arm/parsed_state` (SideArmState): Position in mm
- `/side_arm/state` (String): Raw JSON with servo angle

**Output Topic**:
- `side_arm_marker` (visualization_msgs/Marker)

## TF Tree

```
world
└── wx200/base_link
    ├── wx200/shoulder_link
    │   └── ... (arm kinematic chain)
    │       └── wx200/ee_gripper_link
    ├── framemodel_root
    │   ├── framemodel (mesh)
    │   └── parachute_bag (mesh)
    └── side_arm_origin
        └── (hook marker rendered here)
```

## Launch Files

### Full System (Dual Arm Test)

```bash
ros2 launch parachute_coordinator dual_arm_test.launch.py
```

**Arguments**:
| Argument | Default | Description |
|----------|---------|-------------|
| `enable_main_arm` | true | Enable main arm control |
| `main_arm_sim` | false | Use simulated main arm |
| `enable_side_arm` | true | Enable side arm control |
| `enable_visualization` | true | Enable RViz markers |
| `use_rviz` | true | Launch RViz |
| `enable_teleop` | false | Enable Xbox controller |

### Simulation Only

```bash
ros2 launch main_arm_control arm_simulation.launch.py
```

## RViz Configuration

**File**: `src/main_arm_control/config/simulation.rviz`

**Displays**:
1. Grid - XY reference plane
2. RobotModel (main arm) - From `robot_description`
3. TF - Coordinate frame axes
4. RobotModel (frame) - From `/frame_description`
5. Marker (side arm) - From `side_arm_marker`

**Fixed Frame**: `world`

## Data Flow

### Main Arm
```
Hardware/Simulation → Interbotix SDK → TF Broadcaster → RViz
```

### Side Arm
```
ESP32 → Serial Bridge → /side_arm/state
                     → Coordinate Node → /side_arm/parsed_state
                                      → Visualizer → /side_arm_marker → RViz
```

### Frame Assembly
```
Static URDF → robot_state_publisher → /frame_description → RViz
Static TF → /tf_static → RViz
```

## Test Mode

The side arm visualizer supports test mode for development without hardware:

```bash
# Launch with test mode enabled
ros2 launch parachute_coordinator dual_arm_test.launch.py side_arm_test_mode:=true
```

Or set parameters directly:
```bash
ros2 param set /side_arm_visualizer test_mode true
ros2 param set /side_arm_visualizer test_x 0.15
ros2 param set /side_arm_visualizer test_y 0.10
ros2 param set /side_arm_visualizer test_z 0.05
```

## Coordinate Systems

| Frame | Origin | X | Y | Z |
|-------|--------|---|---|---|
| `world` | Fixed | Forward | Left | Up |
| `wx200/base_link` | Robot base | Forward | Left | Up |
| `side_arm_origin` | Hook home | Along rail | Perpendicular | Vertical |
| `camera_frame` | Camera lens | Right | Down | Into scene |

## Adding New Visualizations

To add new markers (e.g., detected loops), follow the pattern in `side_arm_visualizer.py`:

1. Create a ROS 2 node that subscribes to the data source
2. Publish `visualization_msgs/Marker` or `MarkerArray`
3. Set appropriate `frame_id` (typically `camera_frame` or `world`)
4. Add the marker topic to RViz displays

See the **Loop Visualizer Plan** section below for the planned YOLO detection visualization.

---

## Loop Visualization (YOLO Detections)

Visualizes detected suspension loops from the YOLO vision system as 3D markers in RViz, with a test mode that publishes simulated detections when the camera is not connected.

### Message Flow

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  YOLO Detector  │────▶│  Loop Detector  │────▶│ Target Selector │
│                 │     │                 │     │                 │
│ /yolo/centers   │     │/detected_loops  │     │/request_next_   │
│ (PoseArray)     │     │(DetectedLoops)  │     │  target (srv)   │
└─────────────────┘     └─────────────────┘     └─────────────────┘
                                │
           ┌────────────────────┤
           │                    ▼
           │            ┌─────────────────┐
           │            │ Loop Visualizer │
           │            │                 │
           │            │/loop_markers    │
           │            │(MarkerArray)    │
           │            └─────────────────┘
           │                    │
           ▼                    ▼
┌─────────────────────────────────────────┐
│                  RViz                    │
│  Grid overlay + Loop sphere markers      │
└─────────────────────────────────────────┘

Test Mode (no camera):
┌─────────────────────┐
│ Test Loop Publisher │──▶ /detected_loops
└─────────────────────┘
```

### Loop Visualizer Node

**File**: `src/parachute_perception/parachute_perception/loop_visualizer_node.py`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `marker_scale` | 0.015 | Loop marker radius (meters) |
| `marker_color_r/g/b/a` | 0.2/0.8/0.2/1.0 | Default loop color (green) |
| `selected_color_r/g/b` | 1.0/0.2/0.2 | Target loop color (red) |
| `grid_enabled` | true | Show camera frame reference grid |
| `grid_size_x` | 0.4 | Grid width (meters) |
| `grid_size_y` | 0.3 | Grid height (meters) |
| `grid_cells_x` | 16 | Grid cells in X direction |
| `grid_cells_y` | 12 | Grid cells in Y direction |
| `frame_id` | 'camera_frame' | TF frame for visualization |

**Features**:
- Sphere markers for each detected loop
- Text labels showing loop ID and confidence
- Rightmost loop highlighted in red (target)
- Reference grid in camera frame
- Auto-cleanup when no detections received

### Test Loop Publisher Node

**File**: `src/parachute_perception/parachute_perception/test_loop_publisher_node.py`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `publish_rate` | 2.0 | Publishing rate (Hz) |
| `num_loops` | 5 | Number of simulated loops |
| `pattern` | 'random' | Pattern: random, grid, line, static |
| `x_min` / `x_max` | 0.05 / 0.35 | X coordinate bounds (meters) |
| `y_fixed` | -0.11 | Fixed Y depth (meters) |
| `z_min` / `z_max` | 0.02 / 0.15 | Z coordinate bounds (meters) |
| `movement` | false | Animate loop positions |
| `movement_speed` | 0.02 | Animation speed (m/s) |
| `base_confidence` | 0.85 | Base confidence value |
| `frame_id` | 'camera_frame' | TF frame for coordinates |

**Patterns**:
- `random`: Scattered loops within bounds (default)
- `grid`: Regular grid pattern for calibration
- `line`: Horizontal line of loops
- `static`: Fixed predefined positions

### Camera Frame TF

The `camera_frame` is positioned relative to `wx200/base_link`:
- Position: (0.3, 0.0, 0.2) meters
- Orientation: Pitch 90 degrees (looking down)

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `vision_test_mode` | false | Use simulated loop detections |
| `enable_loop_visualization` | true | Enable loop visualization in RViz |

### Usage

```bash
# With real camera (full system)
ros2 launch parachute_coordinator dual_arm_test.launch.py

# With simulated loops (no camera required)
ros2 launch parachute_coordinator dual_arm_test.launch.py vision_test_mode:=true

# Standalone test publisher with grid pattern
ros2 run parachute_perception test_loop_publisher_node --ros-args -p pattern:=grid -p num_loops:=9

# Standalone visualizer
ros2 run parachute_perception loop_visualizer_node

# Animated loops for demo
ros2 run parachute_perception test_loop_publisher_node --ros-args -p movement:=true -p pattern:=random
```

### RViz Display Configuration

The `simulation.rviz` config includes:
- `LoopMarkers`: MarkerArray display subscribing to `/loop_markers`
- `SideArmMarker`: Marker display for `/side_arm_marker`
- TF frames for `camera_frame` and `side_arm_origin`
