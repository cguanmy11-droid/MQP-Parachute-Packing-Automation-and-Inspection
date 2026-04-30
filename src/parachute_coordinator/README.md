# Parachute Coordinator

State machine coordinator for the automated parachute line stowing system. Orchestrates dual robotic arms through vision-guided manipulation.

## Nodes

| Node | Description |
|------|-------------|
| `packing_coordinator_node` | Main state machine - orchestrates stowing sequence |
| `packing_demo_node` | Demo node for basic testing |
| `full_stow_demo_node` | Single-loop stow demonstration |
| `loop_visit_test_node` | Test node for visiting loops without stowing |
| `packing_coordinator_legacy_node` | Legacy single-arm coordinator |

## State Machine

States (matching paper Figure 6):

| State | Description |
|-------|-------------|
| `IDLE` | System ready, arms homed |
| `AT_LOOP` | Vision-guided positioning at next loop |
| `INSERT` | Hook insertion with collision detection |
| `HANDOFF` | Dual-arm line transfer |
| `RETRACT` | Hook withdrawal with line seating |
| `RELEASE` | Cycle completion and verification |
| `COMPLETE` | All loops stowed |
| `ERROR` | Fault handling with operator recovery |

## Control Commands

Control via `/stow/command` topic:

```bash
# Start the sequence
ros2 topic pub --once /stow/command std_msgs/String "data: start"

# Pause/Resume
ros2 topic pub --once /stow/command std_msgs/String "data: pause"
ros2 topic pub --once /stow/command std_msgs/String "data: resume"

# Change motion pattern
ros2 topic pub --once /stow/command std_msgs/String "data: pattern:square_stow"

# Error recovery (in ERROR state)
ros2 topic pub --once /stow/command std_msgs/String "data: retry"
ros2 topic pub --once /stow/command std_msgs/String "data: skip"
ros2 topic pub --once /stow/command std_msgs/String "data: abort"
```

## Topics

**Publishes:**
- `/coordinator/state` (String) - Current state name
- `/coordinator/error` (String) - Error messages
- `/stow/progress` - Stowing progress

**Subscribes:**
- `/stow/command` (String) - Control commands

## Launch Files

| Launch File | Description |
|-------------|-------------|
| `full_system.launch.py` | Complete system with all arms and vision |
| `state_machine_demo.launch.py` | Single arm + state machine |
| `full_stow_demo.launch.py` | Single-loop demonstration |
| `dual_arm_test.launch.py` | Dual arm testing without coordinator |
| `dual_side_arm.launch.py` | Both side arms, no main arm |

## Configuration

- `config/stow_transitions.yaml` - State machine transition rules
- `config/motion_patterns/*.json` - Motion pattern definitions

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `test_mode` | `false` | Simulate actions without hardware |
| `stow_pattern` | `recorded_stow` | Motion pattern for stowing |
| `action_timeout` | `30.0` | Timeout for action calls (seconds) |
| `expected_loop_count` | `0` | Expected loops (0 = skip verification) |
| `enable_left_arm` | `true` | Enable left side arm |
| `enable_right_arm` | `true` | Enable right side arm |
