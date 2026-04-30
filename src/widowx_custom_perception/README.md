# WidowX Custom Perception

> **LEGACY PACKAGE**: This package was used for early color segmentation experiments. The active perception pipeline now uses `yolo_detect_ros` and `top_cam_loop_state` for YOLO-based loop detection.

---

Custom perception node package for the WidowX-200 robot arm system.

## Features

### Color Segmentation Node

Uses HSV color space to detect and segment square targets, extracting four corner points and calculating the average position.

**Subscribes:**
- `/camera1/image_raw` (sensor_msgs/Image) - Input image
- `/camera1/camera_info` (sensor_msgs/CameraInfo) - Camera calibration info

**Publishes:**
- `/segmentation/results` (widowx_custom_msgs/SegmentationResult) - Segmentation results

## Installation

### 1. Build the message package

```bash
colcon build --packages-select widowx_custom_msgs
source install/setup.bash
```

### 2. Build the perception package

```bash
colcon build --packages-select widowx_custom_perception
source install/setup.bash
```

### 3. Install dependencies

```bash
pip3 install opencv-python numpy
```

## Usage

### Quick Start

```bash
# Launch color segmentation node
ros2 launch widowx_custom_perception color_segmentation.launch.py
```

### Using Custom Parameters

```bash
ros2 launch widowx_custom_perception color_segmentation.launch.py \
    config_file:=/path/to/your/params.yaml
```

### Run Node Directly

```bash
ros2 run widowx_custom_perception color_segmentation_node \
    --ros-args --params-file src/widowx_custom_perception/config/color_segmentation_params.yaml
```

## Parameters

### HSV Color Range Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `hsv_lower_h` | int | 0 | Hue lower bound (0-179) |
| `hsv_lower_s` | int | 100 | Saturation lower bound (0-255) |
| `hsv_lower_v` | int | 100 | Value lower bound (0-255) |
| `hsv_upper_h` | int | 10 | Hue upper bound (0-179) |
| `hsv_upper_s` | int | 255 | Saturation upper bound (0-255) |
| `hsv_upper_v` | int | 255 | Value upper bound (0-255) |

**Common HSV Ranges by Color:**

| Color | H Range | S Range | V Range |
|-------|---------|---------|---------|
| Red (low) | 0-10 | 100-255 | 100-255 |
| Red (high) | 170-179 | 100-255 | 100-255 |
| Orange | 10-25 | 100-255 | 100-255 |
| Yellow | 20-40 | 100-255 | 100-255 |
| Green | 40-80 | 50-255 | 50-255 |
| Blue | 100-130 | 100-255 | 100-255 |
| Purple | 130-160 | 50-255 | 50-255 |

### Morphological Operation Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `morph_kernel_size` | int | 5 | Morphological kernel size (must be odd) |
| `morph_iterations` | int | 2 | Number of morphological iterations |

### Contour Filtering Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_area` | float | 500.0 | Minimum area (pixels) |
| `max_area` | float | 50000.0 | Maximum area (pixels) |
| `min_aspect_ratio` | float | 0.7 | Minimum aspect ratio |
| `max_aspect_ratio` | float | 1.3 | Maximum aspect ratio (1.0 = square) |

### Corner Detection Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `approx_epsilon_factor` | float | 0.02 | Contour approximation precision (0.01-0.05) |

### Other Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enable_debug_view` | bool | true | Show debug window |
| `color_name` | string | 'red' | Target color name |
| `image_topic` | string | '/camera1/image_raw' | Image topic |
| `camera_info_topic` | string | '/camera1/camera_info' | Camera info topic |
| `result_topic` | string | '/segmentation/results' | Result publish topic |

## Parameter Tuning Guide

### 1. Lighting Adjustments

**Bright environment:**
- Increase `hsv_lower_v` (150-200)
- May need to increase `hsv_lower_s` (150-200)

**Low light environment:**
- Decrease `hsv_lower_v` (50-80)
- Decrease `hsv_lower_s` (50-80)

### 2. Target Size Adjustments

**Target too small:**
- Decrease `min_area` (100-300)
- Reduce `morph_iterations` (1)

**Target too large or noisy:**
- Increase `max_area` (100000+)
- Increase `morph_iterations` (3-4)

### 3. Shape Requirements

**Rectangular targets:**
- Expand aspect ratio range: `min_aspect_ratio: 0.5`, `max_aspect_ratio: 2.0`

**Strict squares:**
- Narrow aspect ratio range: `min_aspect_ratio: 0.9`, `max_aspect_ratio: 1.1`

### 4. Corner Detection Precision

**Corners inaccurate:**
- Decrease `approx_epsilon_factor` (0.01)

**Corners fitting to noise:**
- Increase `approx_epsilon_factor` (0.03-0.05)

## Live Parameter Adjustment

Use `rqt_reconfigure` for real-time parameter tuning:

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

Or via command line:

```bash
# Adjust HSV hue upper bound
ros2 param set /color_segmentation_node hsv_upper_h 15

# Adjust minimum area
ros2 param set /color_segmentation_node min_area 1000.0

# Toggle debug view
ros2 param set /color_segmentation_node enable_debug_view false
```

## Output Message Format

```
std_msgs/Header header
string color_name              # Color name
geometry_msgs/Point centroid   # Centroid coordinates
geometry_msgs/Point corner_average  # Average of four corners
geometry_msgs/Point[4] corners # Four corners [TL, TR, BR, BL]
float32 area                   # Area
float32 confidence             # Confidence
sensor_msgs/CameraInfo camera_info  # Camera info
```

## Testing & Debugging

### 1. View published results

```bash
ros2 topic echo /segmentation/results
```

### 2. Check image subscription

```bash
ros2 topic hz /camera1/image_raw
```

### 3. Visualization

The node displays two windows (if `enable_debug_view: true`):
- **Original**: Annotated original image (green contours, blue corners, red centroid, yellow corner average)
- **Mask**: Binary mask after HSV segmentation

## Troubleshooting

### Problem: No contours detected

**Possible causes:**
1. Incorrect HSV range
2. Target too small or too large
3. Poor lighting conditions

**Solutions:**
1. Use HSV color picker tool to adjust range
2. Adjust `min_area` and `max_area`
3. Adjust `hsv_lower_v` and `hsv_upper_v`

### Problem: Inaccurate corner detection

**Possible causes:**
1. Blurry target edges
2. Inappropriate approximation precision

**Solutions:**
1. Increase `morph_iterations` to smooth edges
2. Adjust `approx_epsilon_factor`

### Problem: Multiple contours detected

**Possible causes:**
1. Similar colors in background
2. Insufficient morphological operations

**Solutions:**
1. Narrow HSV range
2. Increase `morph_iterations`
3. Adjust `min_area` to filter small contours

## Developer Info

- Package: `widowx_custom_perception`
- Node: `color_segmentation_node`
- Language: Python 3
- ROS Version: ROS 2 Humble
- Dependencies: OpenCV, NumPy, cv_bridge

## License

BSD-3-Clause
