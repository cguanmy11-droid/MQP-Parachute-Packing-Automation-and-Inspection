# WidowX Custom Perception

自定义感知节点包，用于 WidowX-200 机械臂系统。

## 功能

### 1. Color Segmentation Node (颜色分割节点)

使用 HSV 颜色空间检测和分割方形目标，提取四个角点并计算平均位置。

**订阅:**
- `/camera1/image_raw` (sensor_msgs/Image) - 输入图像
- `/camera1/camera_info` (sensor_msgs/CameraInfo) - 相机标定信息

**发布:**
- `/segmentation/results` (widowx_custom_msgs/SegmentationResult) - 分割结果

## 安装

### 1. 构建消息包

```bash
cd /home/my11/interbotix_ws
colcon build --packages-select widowx_custom_msgs
source install/setup.bash
```

### 2. 构建感知包

```bash
colcon build --packages-select widowx_custom_perception
source install/setup.bash
```

### 3. 安装依赖

```bash
pip3 install opencv-python
pip3 install numpy
```

## 使用方法

### 快速启动

```bash
# 启动颜色分割节点
ros2 launch widowx_custom_perception color_segmentation.launch.py
```

### 使用自定义参数

```bash
ros2 launch widowx_custom_perception color_segmentation.launch.py \
    config_file:=/path/to/your/params.yaml
```

### 直接运行节点

```bash
ros2 run widowx_custom_perception color_segmentation_node \
    --ros-args --params-file src/widowx_custom_perception/config/color_segmentation_params.yaml
```

## 可调参数详解

### HSV 颜色范围参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `hsv_lower_h` | int | 0 | 色调下限 (0-179) |
| `hsv_lower_s` | int | 100 | 饱和度下限 (0-255) |
| `hsv_lower_v` | int | 100 | 明度下限 (0-255) |
| `hsv_upper_h` | int | 10 | 色调上限 (0-179) |
| `hsv_upper_s` | int | 255 | 饱和度上限 (0-255) |
| `hsv_upper_v` | int | 255 | 明度上限 (0-255) |

**常见颜色的 HSV 范围:**

| 颜色 | H范围 | S范围 | V范围 |
|------|-------|-------|-------|
| 红色(低) | 0-10 | 100-255 | 100-255 |
| 红色(高) | 170-179 | 100-255 | 100-255 |
| 橙色 | 10-25 | 100-255 | 100-255 |
| 黄色 | 20-40 | 100-255 | 100-255 |
| 绿色 | 40-80 | 50-255 | 50-255 |
| 蓝色 | 100-130 | 100-255 | 100-255 |
| 紫色 | 130-160 | 50-255 | 50-255 |

### 形态学操作参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `morph_kernel_size` | int | 5 | 形态学核大小（必须为奇数）|
| `morph_iterations` | int | 2 | 形态学操作迭代次数 |

### 轮廓筛选参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `min_area` | float | 500.0 | 最小面积（像素）|
| `max_area` | float | 50000.0 | 最大面积（像素）|
| `min_aspect_ratio` | float | 0.7 | 最小宽高比 |
| `max_aspect_ratio` | float | 1.3 | 最大宽高比（1.0为正方形）|

### 角点检测参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `approx_epsilon_factor` | float | 0.02 | 轮廓逼近精度因子 (0.01-0.05) |

### 其他参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enable_debug_view` | bool | true | 是否显示调试窗口 |
| `color_name` | string | 'red' | 目标颜色名称 |
| `image_topic` | string | '/camera1/image_raw' | 图像话题 |
| `camera_info_topic` | string | '/camera1/camera_info' | 相机信息话题 |
| `result_topic` | string | '/segmentation/results' | 结果发布话题 |

## 参数调优指南

### 1. 光照条件调整

**强光环境:**
- 提高 `hsv_lower_v` (150-200)
- 可能需要提高 `hsv_lower_s` (150-200)

**弱光环境:**
- 降低 `hsv_lower_v` (50-80)
- 降低 `hsv_lower_s` (50-80)

### 2. 目标大小调整

**目标太小:**
- 降低 `min_area` (100-300)
- 减少 `morph_iterations` (1)

**目标太大或有噪点:**
- 提高 `max_area` (100000+)
- 增加 `morph_iterations` (3-4)

### 3. 形状要求调整

**长方形目标:**
- 扩大宽高比范围: `min_aspect_ratio: 0.5`, `max_aspect_ratio: 2.0`

**严格正方形:**
- 缩小宽高比范围: `min_aspect_ratio: 0.9`, `max_aspect_ratio: 1.1`

### 4. 角点检测精度

**角点不准确:**
- 减小 `approx_epsilon_factor` (0.01)

**角点过于贴合噪点:**
- 增大 `approx_epsilon_factor` (0.03-0.05)

## 实时参数调整

使用 `rqt_reconfigure` 可以实时调整参数:

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

或使用命令行:

```bash
# 调整 HSV 色调上限
ros2 param set /color_segmentation_node hsv_upper_h 15

# 调整最小面积
ros2 param set /color_segmentation_node min_area 1000.0

# 开启/关闭调试视图
ros2 param set /color_segmentation_node enable_debug_view false
```

## 输出消息格式

```
std_msgs/Header header
string color_name              # 颜色名称
geometry_msgs/Point centroid   # 质心坐标
geometry_msgs/Point corner_average  # 四角平均位置
geometry_msgs/Point[4] corners # 四个角点 [TL, TR, BR, BL]
float32 area                   # 面积
float32 confidence             # 置信度
sensor_msgs/CameraInfo camera_info  # 相机信息
```

## 测试与调试

### 1. 查看发布的结果

```bash
ros2 topic echo /segmentation/results
```

### 2. 检查图像订阅

```bash
ros2 topic hz /camera1/image_raw
```

### 3. 可视化

节点会自动显示两个窗口（如果 `enable_debug_view: true`）:
- **Original**: 带标注的原始图像（绿色轮廓、蓝色角点、红色质心、黄色角点平均）
- **Mask**: HSV 分割后的二值掩码

## 故障排查

### 问题: 没有检测到轮廓

**可能原因:**
1. HSV 范围不正确
2. 目标太小或太大
3. 光照条件不佳

**解决方法:**
1. 使用 HSV 颜色拾取工具调整范围
2. 调整 `min_area` 和 `max_area`
3. 调整 `hsv_lower_v` 和 `hsv_upper_v`

### 问题: 角点检测不准确

**可能原因:**
1. 目标边缘模糊
2. 逼近精度不合适

**解决方法:**
1. 增加 `morph_iterations` 进行边缘平滑
2. 调整 `approx_epsilon_factor`

### 问题: 多个轮廓被检测

**可能原因:**
1. 背景中有相似颜色
2. 形态学操作不足

**解决方法:**
1. 缩小 HSV 范围
2. 增加 `morph_iterations`
3. 调整 `min_area` 过滤小轮廓

## 开发者信息

- Package: `widowx_custom_perception`
- Node: `color_segmentation_node`
- Language: Python 3
- ROS Version: ROS 2 Humble/Foxy/Iron
- Dependencies: OpenCV, NumPy, cv_bridge

## License

BSD-3-Clause

