# Parachute Perception

Loop detection, target selection, and sensor fusion for the parachute packing system.

## Nodes

### Core Detection & Selection
| Node | Description |
|------|-------------|
| `target_selector_node` | Selects next loop to target (leftmost/rightmost/nearest strategy) |
| `loop_detector_node` | Base loop detection from camera feed |
| `camera_to_3d_node` | Converts 2D pixel detections (YOLO) to 3D positions |

### Sensor Fusion
| Node | Description |
|------|-------------|
| `loop_fusion_node` | Fuses side camera positions with top camera stow states |
| `loop_ground_truth_node` | Publishes known loop positions for simulation/testing |
| `detection_simulator_node` | Simulates loop detections based on ground truth |

### Visualization & Testing
| Node | Description |
|------|-------------|
| `loop_visualizer_node` | RViz markers for detected loops and targets |
| `loop_calibration_node` | Calibrate camera-to-world transformations |
| `test_loop_publisher_node` | Publish test loop data for development |

### Verification
| Node | Description |
|------|-------------|
| `hook_verification_node` | Verifies hook insertion through loop |

## Topics

**Publishes:**
- `/detected_loops` - 3D loop positions (DetectedLoops)
- `/target_loop` - Currently selected target (DetectedLoop)
- `/fused_loop_states` - Unified loop states from sensor fusion (LoopStateArray)
- `/detected_loop_markers` - RViz visualization markers

**Subscribes:**
- `/yolo/centers` - Pixel coordinates from YOLO detector
- `/side_arm_left/detected_loops` - Left camera detections
- `/side_arm_right/detected_loops` - Right camera detections
- `/top_cam/loop_states` - Top camera stow classifications
- `/loop_ground_truth` - Ground truth positions

## Services

- `/request_next_target` - Get next loop to target
- `/capture_loops` - Trigger loop capture from top camera

## Data Flow

```
Side Cameras (YOLO)          Top Camera (YOLO + classifier)
        |                              |
        v                              v
  camera_to_3d_node             top_cam_loop_state
        |                              |
        v                              v
   DetectedLoops                 LoopStateArray
        |                              |
        └──────────┬───────────────────┘
                   v
            loop_fusion_node
                   |
                   v
            LoopStateArray (fused)
                   |
                   v
          target_selector_node
                   |
                   v
            /request_next_target → Coordinator
```

## Running

```bash
# Target selector with test loops
ros2 run parachute_perception target_selector_node --ros-args -p use_test_loops:=true

# Loop visualizer
ros2 run parachute_perception loop_visualizer_node

# Ground truth for simulation
ros2 run parachute_perception loop_ground_truth_node
```
