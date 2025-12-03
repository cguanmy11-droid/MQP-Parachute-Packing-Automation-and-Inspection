# Parachute Interfaces

Custom ROS 2 messages, services, and actions for parachute packing system.

## Messages
- `DetectedLoop.msg` - Single loop pose
- `DetectedLoops.msg` - Array of detected loops
- `HookStatus.msg` - Side arm hook state
- `ArmStatus.msg` - Arm position and state

## Services
- `VerifyHookInsertion.srv`
- `RequestNextTarget.srv`
- `PlanToHook.srv`

## Actions
- `InsertHook.action`
- `ExecuteTrajectory.action`