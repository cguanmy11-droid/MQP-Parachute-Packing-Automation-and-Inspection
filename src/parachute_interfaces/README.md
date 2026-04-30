# Parachute Interfaces

Custom ROS 2 messages, services, and actions for the parachute packing system.

## Messages

| Message | Description |
|---------|-------------|
| `DetectedLoop.msg` | Single loop with pose and confidence |
| `DetectedLoops.msg` | Array of detected loops |
| `LoopState.msg` | Loop with position-based ID and stow state (fully/partial/not/unknown) |
| `LoopStateArray.msg` | Array of loop states from top camera |
| `LoopGroundTruth.msg` | Ground truth loop positions for simulation |
| `SideArmState.msg` | Side arm position in mm, limit switches, homing status |
| `HookStatus.msg` | Side arm hook state and position |
| `ArmStatus.msg` | Main arm position and state |

## Services

| Service | Description |
|---------|-------------|
| `RequestNextTarget.srv` | Get next loop to target from perception |
| `MoveToPosition.srv` | Move side arm to absolute XYZ position (mm) |
| `MoveToWorldPose.srv` | Move side arm hook to world frame pose |
| `RotateHook.srv` | Rotate hook servo by specified angle |
| `PlanToHook.srv` | Plan main arm trajectory relative to hook |
| `VerifyHookInsertion.srv` | Verify hook went through loop |
| `VerifyHookPosition.srv` | Verify hook is at expected position |
| `CaptureLoops.srv` | Trigger top camera to capture and classify loops |
| `GetCalibratedLoops.srv` | Get calibrated loop positions |

## Actions

| Action | Description |
|--------|-------------|
| `InsertHook.action` | Multi-stage hook insertion (approach, align, insert, verify) |
| `MoveToCoordinate.action` | Move side arm to coordinate with progress feedback |
| `VisualServo.action` | Visual servoing to align hook with loop |
| `ReleaseHook.action` | Release hook and retract |
| `ExecuteTrajectory.action` | Execute main arm stowing motion with feedback |

## Building

```bash
colcon build --packages-select parachute_interfaces
source install/setup.bash
```

## Usage

```python
from parachute_interfaces.msg import DetectedLoop, SideArmState, LoopState
from parachute_interfaces.srv import RequestNextTarget, MoveToPosition
from parachute_interfaces.action import InsertHook, MoveToCoordinate
```
