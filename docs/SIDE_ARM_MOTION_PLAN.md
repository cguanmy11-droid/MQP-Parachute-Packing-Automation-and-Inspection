# Side Arm Motion Plan

This document describes the unified motion system for the side arm URDF in RViz.

## Implemented Architecture (Option A)

The coordinate_node now supports three modes:

```
┌─────────────────────────────────────────────────────────────────┐
│              HARDWARE ONLY (simulation_mode=false)              │
│                                                                 │
│  ESP32 → serial_bridge → /side_arm/state                       │
│                        ↓                                        │
│                  coordinate_node (HW mode)                      │
│                        ↓                                        │
│              /side_arm/parsed_state                             │
│                        ↓                                        │
│              joint_state_publisher → joint_states → URDF       │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│            SIMULATION ONLY (simulation_mode=true, no serial)    │
│                                                                 │
│  Service call: /side_arm/move_to_position                      │
│                        ↓                                        │
│              coordinate_node (SIM mode)                         │
│                        ↓ (simulates motion internally)          │
│              /side_arm/parsed_state                             │
│                        ↓                                        │
│              joint_state_publisher → joint_states → URDF       │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│        HARDWARE + SIMULATION (simulation_mode=true + serial)    │
│                                                                 │
│  Service call → coordinate_node (SIM+HW mode)                  │
│                        ↓                                        │
│        ┌───────────────┴───────────────┐                       │
│        ↓                               ↓                        │
│  Simulated motion              Hardware commands                │
│  (internal state)              (via serial_bridge)              │
│        ↓                               ↓                        │
│  /side_arm/parsed_state        ESP32 executes                  │
│        ↓                                                        │
│  joint_state_publisher → URDF mirrors real-time                │
└─────────────────────────────────────────────────────────────────┘
```

## Usage

### Hardware Only (Real Testing)
```bash
ros2 launch parachute_coordinator dual_arm_test.launch.py \
    enable_side_arm:=true \
    side_arm_test_mode:=false
```
- URDF mirrors actual hardware position
- Requires ESP32 connected via serial

### Simulation Only (No Hardware)
```bash
ros2 launch parachute_coordinator dual_arm_test.launch.py \
    enable_side_arm:=true \
    side_arm_test_mode:=true \
    enable_main_arm:=false
```
- URDF moves without hardware
- Useful for development and testing

### Hardware + Simulation (Visual Debugging)
```bash
ros2 launch parachute_coordinator dual_arm_test.launch.py \
    enable_side_arm:=true \
    side_arm_test_mode:=true
```
- Both hardware AND simulation run together
- URDF shows what the system "sees" in real-time
- Useful for debugging during real tests

### Test Commands
```bash
# Move side arm (works in any mode)
ros2 service call /side_arm/move_to_position \
    parachute_interfaces/srv/MoveToPosition \
    "{x_mm: 100.0, y_mm: 50.0, z_mm: 20.0, speed_scale: 0.5}"
```

## Implementation Details

### coordinate_node.py Changes
- Added `simulation_mode` parameter
- Added `sim_speed_mm_per_sec` parameter (default 50.0 mm/s)
- Added `_update_simulated_position()` method
- Modified service callbacks to:
  - Start simulated motion when `simulation_mode=true`
  - Send hardware commands if hardware is connected OR not in simulation mode
- Publishes `/side_arm/parsed_state` in all modes

### joint_state_publisher.py
- Subscribes to `/side_arm/parsed_state`
- Converts mm positions to joint positions
- Publishes `/side_arm/joint_states` for robot_state_publisher

### dual_arm_test.launch.py
- `side_arm_test_mode` maps to `simulation_mode` parameter
- Serial bridge only launches when `side_arm_test_mode=false`

## File Changes Summary

| File | Changes |
|------|---------|
| `coordinate_node.py` | Added simulation mode with internal position interpolation |
| `side_arm_joint_state_publisher.py` | Simplified to just subscribe to parsed_state |
| `dual_arm_test.launch.py` | Pass `simulation_mode` parameter to coordinate_node |
