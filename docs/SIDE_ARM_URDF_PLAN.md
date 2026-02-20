# Side Arm URDF Plan

This document outlines the plan to create a URDF model for the side arm gantry system, enabling full visualization and simulation in RViz without hardware.

## Current State

The side arm is currently visualized using:
- A **marker** (hook.stl mesh) published by `side_arm_visualizer.py`
- A **dynamic TF** (`side_arm_hook`) that moves based on `/side_arm/parsed_state`
- No joint-based URDF model

**Limitations:**
- Cannot use RViz joint sliders to move the arm
- No visual representation of the gantry structure (rails, carriages)
- Simulation requires running the coordinate_node and visualizer

---

## Mechanical Structure

The side arm is a **Cartesian gantry manipulator** with 4 DOF:

```
                    Y-axis (vertical, lead screw)
                    ▲
                    │
                    │   ┌─────────────────────┐
                    │   │     X-carriage      │◄──── X-axis (horizontal, belt)
                    │   │  ┌───────────────┐  │
                    │   │  │  Z-slider     │  │
                    │   │  │  ┌─────────┐  │  │
                    │   │  │  │  Hook   │  │  │◄──── Z-axis (depth, DC motor)
                    │   │  │  │ (servo) │  │  │
                    │   │  │  └─────────┘  │  │
                    │   │  └───────────────┘  │
                    │   └─────────────────────┘
                    │
    ────────────────┼────────────────────────────► X-axis
                    │
                    │ (into page: Z-axis)
```

### Movement Ranges

| Joint | Type | Axis | Range | Notes |
|-------|------|------|-------|-------|
| `joint_x` | Prismatic | X | 0 - 300 mm | Horizontal, belt drive |
| `joint_y` | Prismatic | Y | 0 - 200 mm | Vertical, lead screw |
| `joint_z` | Prismatic | Z | 0 - 150 mm | Depth, DC motor |
| `joint_servo` | Revolute | Z-rot | ±0.5 rad (~±30°) | Hook rotation |

---

## Proposed URDF Structure

### Link Hierarchy

```
side_arm_base (fixed to side_arm_origin)
    │
    └── x_rail_link (visual: horizontal rail)
        │
        └── x_carriage_link (prismatic joint_x)
            │
            └── y_rail_link (visual: vertical rail)
                │
                └── y_carriage_link (prismatic joint_y)
                    │
                    └── z_rail_link (visual: depth rail/slider)
                        │
                        └── z_carriage_link (prismatic joint_z)
                            │
                            └── servo_link (revolute joint_servo)
                                │
                                └── hook_link (visual: hook.stl)
                                    │
                                    └── camera_link (fixed offset)
```

### Joint Definitions

```xml
<!-- X-axis: Horizontal movement -->
<joint name="joint_x" type="prismatic">
  <parent link="x_rail_link"/>
  <child link="x_carriage_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="0.0" upper="0.3" effort="10" velocity="0.1"/>
</joint>

<!-- Y-axis: Vertical movement -->
<joint name="joint_y" type="prismatic">
  <parent link="y_rail_link"/>
  <child link="y_carriage_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0.0" upper="0.2" effort="10" velocity="0.1"/>
</joint>

<!-- Z-axis: Depth movement -->
<joint name="joint_z" type="prismatic">
  <parent link="z_rail_link"/>
  <child link="z_carriage_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0.0" upper="0.15" effort="10" velocity="0.05"/>
</joint>

<!-- Servo: Hook rotation -->
<joint name="joint_servo" type="revolute">
  <parent link="z_carriage_link"/>
  <child link="servo_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.5" upper="0.5" effort="1" velocity="1.0"/>
</joint>
```

---

## Visual Components

### Option A: Simple Geometry (Recommended for MVP)

Use basic shapes (boxes, cylinders) to represent the gantry:

| Link | Visual | Dimensions | Color |
|------|--------|------------|-------|
| `x_rail_link` | Box | 0.35 x 0.02 x 0.02 m | Gray |
| `x_carriage_link` | Box | 0.04 x 0.04 x 0.02 m | Blue |
| `y_rail_link` | Box | 0.02 x 0.25 x 0.02 m | Gray |
| `y_carriage_link` | Box | 0.03 x 0.03 x 0.02 m | Green |
| `z_rail_link` | Box | 0.02 x 0.02 x 0.18 m | Gray |
| `z_carriage_link` | Box | 0.02 x 0.02 x 0.03 m | Orange |
| `hook_link` | Mesh | hook.stl | Dark gray |

### Option B: Detailed Meshes (Future)

Create CAD models for each component and export as STL:
- Rail extrusions
- Carriage blocks
- Motor housings
- Servo mount

---

## Implementation Plan

### Phase 1: Create Basic URDF

1. **Create URDF file**: `src/side_arm_control/urdf/side_arm.urdf`
2. **Define links** with simple box geometries for rails and carriages
3. **Define joints** with correct axes and limits
4. **Include hook mesh** at the end effector
5. **Test in RViz** with `joint_state_publisher_gui`

### Phase 2: Joint State Bridge

Create a node that bridges between side arm state and URDF joints:

**Option A: Modify existing `side_arm_visualizer.py`**
- Add joint_state publisher alongside TF/marker
- Publish `sensor_msgs/JointState` with joint_x, joint_y, joint_z, joint_servo

**Option B: Create new `side_arm_joint_state_publisher.py`**
- Subscribe to `/side_arm/parsed_state`
- Publish `/side_arm/joint_states`

```python
# Mapping from SideArmState to JointState
joint_state.name = ['joint_x', 'joint_y', 'joint_z', 'joint_servo']
joint_state.position = [
    msg.x_mm / 1000.0,      # Convert mm to m
    msg.y_mm / 1000.0,
    msg.z_mm / 1000.0,
    servo_angle_rad         # From servo offset calculation
]
```

### Phase 3: Launch Integration

Update `dual_arm_test.launch.py`:

```python
# Side arm URDF
side_arm_urdf_path = os.path.join(
    get_package_share_directory('side_arm_control'),
    'urdf', 'side_arm.urdf'
)
with open(side_arm_urdf_path, 'r') as f:
    side_arm_urdf = f.read()

# Side arm state publisher
side_arm_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='side_arm_state_publisher',
    namespace='side_arm',
    parameters=[{'robot_description': side_arm_urdf}],
)

# Joint state publisher (GUI for testing, or bridge for hardware)
side_arm_joint_gui = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='side_arm_joint_gui',
    namespace='side_arm',
    condition=IfCondition(LaunchConfiguration('side_arm_test_mode'))
)
```

### Phase 4: RViz Configuration

Add to `simulation.rviz`:
- RobotModel display for side arm URDF
- Topic: `/side_arm/robot_description`

### Phase 5: Deprecate Old Visualizer (Optional)

Once URDF is working:
- Remove marker publishing from `side_arm_visualizer.py`
- Keep only TF publishing (or move TF to robot_state_publisher)
- The URDF automatically provides the hook visual

---

## File Changes Summary

| File | Action | Description |
|------|--------|-------------|
| `side_arm_control/urdf/side_arm.urdf` | Create | URDF with prismatic/revolute joints |
| `side_arm_control/side_arm_joint_state_pub.py` | Create | Bridge state → joint_states |
| `side_arm_control/setup.py` | Modify | Add new node entry point |
| `side_arm_control/CMakeLists.txt` or setup.cfg | Modify | Install URDF files |
| `parachute_coordinator/launch/dual_arm_test.launch.py` | Modify | Add URDF loading and state publisher |
| `main_arm_control/config/simulation.rviz` | Modify | Add side arm RobotModel display |

---

## Testing Checklist

- [ ] URDF loads without errors (`check_urdf side_arm.urdf`)
- [ ] RViz displays side arm model
- [ ] Joint sliders move the model correctly (joint_state_publisher_gui)
- [ ] Axes directions match real hardware (X=horizontal, Y=vertical, Z=depth)
- [ ] Joint limits prevent invalid positions
- [ ] Hook mesh appears at correct position/orientation
- [ ] camera_link TF aligns with camera_frame from visualizer
- [ ] State bridge correctly maps hardware state to joints

---

## Coordinate System Notes

**From exploration of existing code:**

- X positive → stepper2 moves right (belt)
- Y positive → stepper1 moves down (lead screw)
- Z positive → hook extends/moves into loop (DC motor)
- Servo positive → counterclockwise rotation

**URDF conventions:**
- All units in meters (convert from mm)
- Rotations in radians
- Follow REP-103 (X forward, Y left, Z up) where practical

**Alignment with side_arm_origin TF:**
The existing static TF places `side_arm_origin` relative to `wx200/base_link`:
```
xyz: 0.4, 0.19, -0.03
rpy: 1.5708, 0, 3.1416  (roll=90°, yaw=180°)
```

The URDF base should align with this frame.

---

## Open Questions

1. Should we create simplified STL meshes for the rails/carriages, or use boxes?
2. Should the joint_state_publisher_gui always be available, or only in test mode?
3. Do we need collision geometry for the URDF? (Not needed for visualization-only)
4. Should camera_link be part of URDF or stay as separate TF from visualizer?
