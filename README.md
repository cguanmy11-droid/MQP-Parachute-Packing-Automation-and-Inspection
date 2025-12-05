# MQP-Parachute-Packing-Automation-and-Inspection

Parachute Packing Automation System - Architecture Overview
Version: v1.0 (In Development)
Last Updated: 2024-12-02
Project: WPI MQP - Parachute Packing Automation and Inspection

1. System Overview
This system automates the packing of parachutes using a coordinated robotic setup consisting of:

Main Arm: WidowX-200 robotic manipulator for parachute line stowing
Side Arm: ESP32-controlled hook mechanism for loop insertion
Perception: Camera-based system for loop detection and positioning
Coordination: ROS 2 state machine orchestrating the complete packing sequence


2. Package Structure
2.1 Existing/Reusable Packages

interbotix_xs_sdk: WidowX-200 low-level driver (existing, no modifications)
interbotix_xs_modules: High-level Python API for arm control (existing, no modifications)

2.2 New Custom Packages
Package NamePurposeStatusparachute_interfacesCustom ROS 2 messages, services, and actions✅ Implementedparachute_perceptionLoop detection and target selection🔄 In Developmentside_arm_controlESP32 hook arm interface and control🔄 In Developmentmain_arm_controlWidowX-200 motion planning and execution🔄 In Developmentparachute_coordinatorHigh-level state machine and orchestration🔄 In Development

3. System Architecture (ASCII Diagram)
┌─────────────────────────────────────────────────────────────────┐
│                    Parachute Packing System                     │
└─────────────────────────────────────────────────────────────────┘

╔═══════════════════════════════════════════════════════════════════╗
║                      Perception Layer                             ║
╚═══════════════════════════════════════════════════════════════════╝

┌──────────────────┐
│  Camera          │
│  【Hardware】     │
└────────┬─────────┘
         │ /camera/image_raw
         ▼
┌───────────────────────────────────────────────────────┐
│  parachute_perception Package                         │
│  Location: src/parachute_perception/                  │
└───────────────────────────────────────────────────────┘
         │
    ┌────┴────┐
    ▼         ▼
┌─────────────────┐    ┌──────────────────────┐
│ loop_detector   │    │ target_selector      │
│ _node           │───▶│ _node                │
│ 【New】          │    │ 【New】               │
│                 │    │                      │
│ Detects loops   │    │ Selects rightmost    │
│ in camera feed  │    │ loop as target       │
└─────────────────┘    └──────────┬───────────┘
         │                        │
         │ /detected_loops        │ Service: /request_next_target
         │                        │
         └────────────────────────┘

╔═══════════════════════════════════════════════════════════════════╗
║                    Coordination Layer                             ║
╚═══════════════════════════════════════════════════════════════════╝

┌───────────────────────────────────────────────────────┐
│  parachute_coordinator Package                        │
│  Location: src/parachute_coordinator/                 │
└───────────────────────────────────────────────────────┘
         │
         ▼
┌────────────────────────────────────────┐
│  packing_coordinator_node 【New】       │
│                                        │
│  State Machine:                        │
│  1. Request target loop                │
│  2. Insert hook                        │
│  3. Rotate hook (90°)                  │
│  4. Execute stowing trajectory         │
│  5. Rotate hook (90°)            │
│  6. Retract hook                       │
└────┬──────────────────┬────────────────┘
     │                  │
     │ Calls Services   │ Calls Actions
     ▼                  ▼

╔═══════════════════════════════════════════════════════════════════╗
║                      Control Layer                                ║
╚═══════════════════════════════════════════════════════════════════╝

┌─────────────────────────────┐  ┌─────────────────────────────┐
│  side_arm_control Package   │  │  main_arm_control Package   │
│  Location: src/side_arm_     │  │  Location: src/main_arm_    │
│  control/                    │  │  control/                   │
└─────────────────────────────┘  └─────────────────────────────┘
         │                                  │
    ┌────┴────┐                        ┌────┴────┐
    ▼         ▼                        ▼         ▼
┌──────────┐ ┌──────────┐    ┌──────────┐ ┌──────────┐
│ side_arm │ │ hook_    │    │ main_arm │ │ main_arm │
│ interface│ │ verifi-  │    │ interface│ │ planner  │
│ _node    │ │ cation   │    │ _node    │ │ _node    │
│ 【New】   │ │ _node    │    │ 【New】   │ │ 【New】   │
│          │ │ 【New】   │    │          │ │          │
│ ESP32↔   │ │          │    │ WidowX   │ │ Motion   │
│ ROS2     │ │ Verifies │    │ control  │ │ planning │
│ bridge   │ │ insertion│    │          │ │          │
└─────┬────┘ └──────────┘    └─────┬────┘ └──────────┘
      │                            │
      │ Actions:                   │ Actions:
      │ /side_arm/insert_hook      │ /main_arm/execute_trajectory
      │ Services:                  │
      │ /side_arm/rotate_hook      │
      │ /side_arm/retract_hook     │
      ▼                            ▼
┌──────────────┐            ┌──────────────┐
│  ESP32       │            │  WidowX-200  │
│  【Hardware】 │            │  【Hardware】 │
│  + Hook Arm  │            │  Robotic Arm │
└──────────────┘            └──────────────┘

4. Custom Interfaces (parachute_interfaces)
4.1 Messages
MessagePurposeDetectedLoop.msgSingle loop with pose and confidenceDetectedLoops.msgArray of detected loopsHookStatus.msgSide arm hook state and positionArmStatus.msgMain arm state and current pose
4.2 Services
ServicePurposeRequestNextTarget.srvRequest next target loop from perceptionVerifyHookInsertion.srvVerify hook successfully insertedPlanToHook.srvPlan trajectory relative to hook positionRotateHook.srvRotate hook by specified angle
4.3 Actions
ActionPurposeInsertHook.actionInsert hook through target loop (with feedback)ExecuteTrajectory.actionExecute main arm stowing motion (with feedback)

5. Key Nodes
5.1 Perception Nodes

loop_detector_node: Detects parachute loops, publishes positions
target_selector_node: Selects rightmost loop as next target
hook_verification_node: Verifies successful hook insertion

5.2 Control Nodes

side_arm_interface_node: Controls ESP32 hook arm, provides action servers
main_arm_interface_node: Low-level WidowX-200 control interface
main_arm_planner_node: Motion planning for stowing trajectories

5.3 Coordinator Node

packing_coordinator_node: State machine orchestrating complete packing sequence


6. Data Flow
Camera → loop_detector → target_selector → [Service Call]
                                                ↓
                        packing_coordinator ← [Response]
                                ↓
                          [Action Call]
                                ↓
                    ┌───────────┴───────────┐
                    ▼                       ▼
            side_arm_interface      main_arm_interface
                    ↓                       ↓
                ESP32 Hook              WidowX-200

7. Packing Sequence

Detect & Select: Perception identifies loops, selects rightmost
Insert Hook: Side arm inserts hook through target loop
Verify: Confirm hook insertion successful
Rotate Pre-Stow: Rotate hook 90° for line clearance
Stow Line: Main arm executes stowing trajectory around hook
Rotate Post-Stow: Rotate hook back -90°
Retract: Retract hook to clear for next loop
Repeat: Continue until all loops processed


8. Development Status
✅ Completed

Package structure created
Custom interfaces defined and built
Basic node skeletons implemented
Test mode with dummy data functional
Launch file for system integration

🔄 In Progress

Camera integration and loop detection algorithm
ESP32 firmware and micro-ROS interface
WidowX-200 motion planning integration
Full sequence testing and validation

📋 Planned

Computer vision optimization
Trajectory refinement
Safety interlocks and error recovery
Performance optimization
Documentation and user guides


9. Build and Launch
Build System
bash# Build all packages
colcon build

# Build specific package
colcon build --packages-select parachute_interfaces

# Source workspace
source install/setup.bash
Launch System
bash# Launch complete test system
ros2 launch parachute_coordinator test_system.launch.py

# Launch individual nodes
ros2 run parachute_perception loop_detector_node
ros2 run side_arm_control side_arm_interface_node
ros2 run main_arm_control main_arm_interface_node
ros2 run parachute_coordinator packing_coordinator_node
```

---

## 10. Dependencies

### ROS 2 Packages
- `rclpy` - Python client library
- `geometry_msgs` - Pose and point messages
- `sensor_msgs` - Camera and image messages
- `std_msgs` - Standard message types

### Hardware
- WidowX-200 Robot Arm (Interbotix)
- ESP32 Development Board
- USB Camera(s)
- Stepper motor + driver for hook arm
- Limit switches

### External Libraries
- OpenCV (cv_bridge) - Image processing
- NumPy - Numerical operations
- (Future) PyTorch/TensorFlow - Deep learning for perception

---

## 11. File Locations
```
parachute_packing_ws/
├── src/
│   ├── parachute_interfaces/       # Custom messages/services/actions
│   ├── parachute_perception/       # Loop detection and selection
│   ├── side_arm_control/           # ESP32 hook arm control
│   ├── main_arm_control/           # WidowX-200 control
│   └── parachute_coordinator/      # High-level orchestration
├── build/                          # Build artifacts
├── install/                        # Installed packages
└── log/                           # Build logs

12. Future Enhancements

Integration with MoveIt for advanced motion planning
Multi-camera perception for improved accuracy
Machine learning for loop detection robustness
Real-time trajectory adaptation
Web-based monitoring interface
Automated error recovery strategies


Institution: Worcester Polytechnic Institute
Project Type: Major Qualifying Project (MQP)
Academic Year: 2025-2026