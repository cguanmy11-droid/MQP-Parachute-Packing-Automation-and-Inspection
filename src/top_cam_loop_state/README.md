# Top Camera Loop State

Top-down camera node for loop detection and stow state classification. Uses YOLO detection + classification to determine which loops are stowed (fully/partial/not).

## Overview

Camera orientation:
- Top half of image → Left (L) side of parachute
- Bottom half of image → Right (R) side of parachute

ID assignment:
- Top half (cy <= height/2), sorted left-to-right: L1, L2, L3...
- Bottom half (cy > height/2), sorted left-to-right: R1, R2, R3...

## Nodes

| Node | Description |
|------|-------------|
| `loop_state_node` | YOLO detection + classification, publishes loop states |

## Topics

**Publishes:**
- `/top_cam/loop_states` (LoopStateArray) - Loop IDs with stow states and positions
- `/top_cam/image` (Image) - Annotated camera feed with detections

## Services

- `/top_cam/capture` (Trigger) - Trigger single capture (when continuous_mode=false)
- `/top_cam/enable` (SetBool) - Enable/disable processing

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_index` | `/dev/video6` | Camera device path |
| `det_weights` | (see launch) | YOLO detection weights |
| `cls_weights` | (see launch) | YOLO classification weights |
| `conf_threshold` | `0.35` | Detection confidence threshold |
| `iou_threshold` | `0.45` | NMS IoU threshold |
| `frame_rate` | `30.0` | Camera frame rate |
| `display` | `false` | Show OpenCV window |
| `publish_image` | `true` | Publish annotated image |
| `continuous_mode` | `false` | Continuous vs on-demand capture |

## Loop States

| State | Color | Description |
|-------|-------|-------------|
| `fully` | Green | Loop fully stowed |
| `partial` | Yellow | Loop partially stowed |
| `not` | Red | Loop not stowed |
| `unknown` | Gray | Classification failed |

## Dependencies

```bash
pip install ultralytics opencv-python numpy
```

## Running

```bash
# Via launch file (recommended)
ros2 launch top_cam_loop_state top_cam_loop_state.launch.py

# With display
ros2 launch top_cam_loop_state top_cam_loop_state.launch.py display:=true

# Trigger capture (when continuous_mode=false)
ros2 service call /top_cam/capture std_srvs/srv/Trigger
```

## Weights

Detection and classification weights are specified via launch arguments. Default paths:
- Detection: `top_cam_yolo/runs/detect/.../best.pt`
- Classification: `top_cam_yolo/runs/classify/.../best.pt`

For training new weights, see `top_cam_yolo` package.

## Output Format

Each `LoopState` in the array contains:
- `loop_id` - Position-based ID (e.g., "L1", "R3")
- `state` - Stow state ("fully", "partial", "not", "unknown")
- `confidence` - Classification confidence (0.0-1.0)
- `center_x`, `center_y` - Normalized image coordinates
- `world_position` - 3D position (computed by fusion node)
