# Parachute Operator GUI Package

Operator console for semi-autonomous parachute packing. Provides real-time state visualization, loop detection display, and sequence control.

## Nodes
- `operator_console` - Main GUI node

## Topics
- Subscribes: `/coordinator/state`, `/coordinator/error`, `/detected_loops`, `/target_loop`
- Publishes: `/stow/command`, `/joystick_enabled`

## Controls
- **State diagram** - Live visualization of coordinator state machine
- **Loop panel** - Detected loops with confidence and position, current target highlighted
- **Sequence** - Start, Pause/Resume, and error recovery (Retry, Skip, Abort)
- **Joystick toggle** - Hand off main arm control to Xbox controller

## Running
```bash
ros2 run parachute_gui operator_console
```

Runs standalone alongside the main stack. Requires `packing_coordinator_node` to publish `/coordinator/state` and `/coordinator/error`.