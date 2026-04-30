# YOLO Detect ROS

ROS 2 wrapper for running YOLOv8 detection on USB cameras. Used for side camera loop detection.

## Nodes

| Node | Description |
|------|-------------|
| `yolo_detector` | Real-time YOLO detection on camera feed, publishes loop centers |

## Topics

**Publishes:**
- `yolo/centers` (PoseArray) - Detected loop centers in pixel coordinates
- `yolo/image` (Image) - Annotated camera image with detections

## Services

- `yolo/enable` (SetBool) - Enable/disable detection processing

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_index` | `/dev/video4` | Camera device path |
| `weights_path` | `runs/yolo26m_hole2_100/weights/best.pt` | YOLO weights file |
| `conf_threshold` | `0.5` | Detection confidence threshold |
| `iou_threshold` | `0.5` | NMS IoU threshold |
| `frame_rate` | `30.0` | Camera capture frame rate |
| `camera_frame_id` | `camera_frame` | TF frame for detections |
| `publish_image` | `true` | Publish annotated image |
| `display` | `true` | Show OpenCV window |

## Dependencies

```bash
pip install ultralytics opencv-python numpy
```

## Running

```bash
# Basic usage
ros2 run yolo_detect_ros yolo_detector

# With parameters
ros2 run yolo_detect_ros yolo_detector --ros-args \
    -p camera_index:=/dev/video4 \
    -p conf_threshold:=0.5 \
    -p display:=true

# Via launch file
ros2 launch yolo_detect_ros yolo_detect.launch.py
```

## Output Format

The `yolo/centers` topic publishes a `PoseArray` where each pose contains:
- `position.x` - Pixel X coordinate (horizontal)
- `position.y` - Pixel Y coordinate (vertical)
- `position.z` - Not used (0)

These pixel coordinates are converted to 3D positions by `camera_to_3d_node` in `parachute_perception`.

## Weights

Pre-trained weights are included at:
```
runs/yolo26m_hole2_100/weights/best.pt
```

For training new weights, see `top_cam_yolo` package.
