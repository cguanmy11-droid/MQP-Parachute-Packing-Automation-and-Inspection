# Color Segmentation Node 安装与使用指南

## 📦 已创建的文件结构

```
/home/my11/interbotix_ws/src/
├── widowx_custom_msgs/                    # 自定义消息包
│   ├── msg/
│   │   └── SegmentationResult.msg        # 分割结果消息定义
│   ├── CMakeLists.txt
│   └── package.xml
│
└── widowx_custom_perception/              # 感知节点包
    ├── widowx_custom_perception/
    │   ├── __init__.py
    │   └── color_segmentation_node.py    # 核心节点实现
    ├── scripts/
    │   └── color_segmentation_node.py    # 节点脚本
    ├── config/
    │   └── color_segmentation_params.yaml # 参数配置文件
    ├── launch/
    │   └── color_segmentation.launch.py  # 启动文件
    ├── resource/
    │   └── widowx_custom_perception
    ├── setup.py
    ├── setup.cfg
    ├── package.xml
    └── README.md                          # 详细文档
```

## ⚠️ Setuptools 冲突问题解决

当前遇到的 `AssertionError: /usr/lib/python3.10/distutils/core.py` 问题是由于本地安装的setuptools与系统版本冲突导致的。

### 解决方案 1: 使用系统setuptools (推荐)

```bash
# 临时移除本地setuptools
cd /home/my11/interbotix_ws
mv ~/.local/lib/python3.10/site-packages/setuptools ~/.local/lib/python3.10/site-packages/setuptools.backup
mv ~/.local/lib/python3.10/site-packages/_distutils_hack ~/.local/lib/python3.10/site-packages/_distutils_hack.backup

# 编译
colcon build --packages-select widowx_custom_msgs
source install/setup.bash
colcon build --packages-select widowx_custom_perception
source install/setup.bash

# 恢复setuptools (如果需要)
mv ~/.local/lib/python3.10/site-packages/setuptools.backup ~/.local/lib/python3.10/site-packages/setuptools
mv ~/.local/lib/python3.10/site-packages/_distutils_hack.backup ~/.local/lib/python3.10/site-packages/_distutils_hack
```

### 解决方案 2: 使用虚拟环境

```bash
# 创建虚拟环境
python3 -m venv ~/ros2_venv
source ~/ros2_venv/bin/activate

# 安装必要依赖
pip install opencv-python numpy

# 编译
cd /home/my11/interbotix_ws
colcon build --packages-select widowx_custom_msgs widowx_custom_perception
source install/setup.bash
```

### 解决方案 3: 使用Docker (最干净)

```bash
# 使用ROS2 Humble Docker镜像
docker run -it --rm \
    -v /home/my11/interbotix_ws:/workspace \
    ros:humble \
    bash -c "cd /workspace && colcon build --packages-select widowx_custom_msgs widowx_custom_perception"
```

## 🚀 编译步骤

### 1. 编译消息包

```bash
cd /home/my11/interbotix_ws
colcon build --packages-select widowx_custom_msgs
source install/setup.bash
```

### 2. 编译感知包

```bash
colcon build --packages-select widowx_custom_perception
source install/setup.bash
```

### 3. 验证编译

```bash
# 检查消息是否生成
ros2 interface show widowx_custom_msgs/msg/SegmentationResult

# 检查节点是否可执行
ros2 run widowx_custom_perception color_segmentation_node --help
```

## 📷 使用方法

### 方式 1: 使用 Launch 文件 (推荐)

```bash
# 使用默认参数
ros2 launch widowx_custom_perception color_segmentation.launch.py

# 使用自定义参数文件
ros2 launch widowx_custom_perception color_segmentation.launch.py \
    config_file:=/path/to/custom_params.yaml
```

### 方式 2: 直接运行节点

```bash
ros2 run widowx_custom_perception color_segmentation_node \
    --ros-args --params-file \
    src/widowx_custom_perception/config/color_segmentation_params.yaml
```

### 方式 3: 运行时设置参数

```bash
ros2 run widowx_custom_perception color_segmentation_node \
    --ros-args \
    -p hsv_lower_h:=100 \
    -p hsv_upper_h:=130 \
    -p hsv_lower_s:=100 \
    -p hsv_upper_s:=255 \
    -p color_name:=blue \
    -p enable_debug_view:=true
```

## 🎨 预设颜色配置

### 红色检测

```yaml
hsv_lower_h: 0
hsv_lower_s: 100
hsv_lower_v: 100
hsv_upper_h: 10
hsv_upper_s: 255
hsv_upper_v: 255
color_name: 'red'
```

### 蓝色检测

```yaml
hsv_lower_h: 100
hsv_lower_s: 100
hsv_lower_v: 100
hsv_upper_h: 130
hsv_upper_s: 255
hsv_upper_v: 255
color_name: 'blue'
```

### 绿色检测

```yaml
hsv_lower_h: 40
hsv_lower_s: 50
hsv_lower_v: 50
hsv_upper_h: 80
hsv_upper_s: 255
hsv_upper_v: 255
color_name: 'green'
```

### 黄色检测

```yaml
hsv_lower_h: 20
hsv_lower_s: 100
hsv_lower_v: 100
hsv_upper_h: 40
hsv_upper_s: 255
hsv_upper_v: 255
color_name: 'yellow'
```

## 🛠️ 实时参数调整

### 查看当前参数

```bash
ros2 param list /color_segmentation_node
```

### 修改参数

```bash
# 调整HSV范围
ros2 param set /color_segmentation_node hsv_lower_h 40
ros2 param set /color_segmentation_node hsv_upper_h 80

# 调整面积范围
ros2 param set /color_segmentation_node min_area 1000.0
ros2 param set /color_segmentation_node max_area 100000.0

# 开启/关闭调试窗口
ros2 param set /color_segmentation_node enable_debug_view false
```

### 保存当前参数

```bash
ros2 param dump /color_segmentation_node > my_tuned_params.yaml
```

## 📊 测试与调试

### 1. 测试模拟相机 (如果没有真实相机)

```bash
# 安装 usb_cam
sudo apt install ros-humble-usb-cam

# 启动USB相机
ros2 run usb_cam usb_cam_node_exe --ros-args \
    -p video_device:=/dev/video0 \
    -p image_width:=640 \
    -p image_height:=480 \
    -p framerate:=30.0 \
    -r __ns:=/camera1 \
    -r /camera1/usb_cam_node_exe/image_raw:=/camera1/image_raw \
    -r /camera1/usb_cam_node_exe/camera_info:=/camera1/camera_info
```

### 2. 查看发布的结果

```bash
ros2 topic echo /segmentation/results
```

### 3. 监控话题频率

```bash
ros2 topic hz /segmentation/results
```

### 4. 可视化图像

```bash
# 安装 rqt_image_view
sudo apt install ros-humble-rqt-image-view

# 查看原始图像
ros2 run rqt_image_view rqt_image_view /camera1/image_raw
```

## 📝 SegmentationResult 消息格式

```
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id

string color_name              # 检测到的颜色名称

geometry_msgs/Point centroid   # 质心坐标 (像素)
  float64 x
  float64 y
  float64 z

geometry_msgs/Point corner_average  # 四角平均位置 (像素)
  float64 x
  float64 y
  float64 z

geometry_msgs/Point[4] corners # 四个角点 [TL, TR, BR, BL]
  [0] top-left
  [1] top-right
  [2] bottom-right
  [3] bottom-left

float32 area                   # 面积 (平方像素)
float32 confidence             # 置信度 (0.0-1.0)

sensor_msgs/CameraInfo camera_info  # 相机标定信息
```

## 🎯 常见使用场景

### 场景 1: 检测红色方形物体

```bash
ros2 run widowx_custom_perception color_segmentation_node \
    --ros-args \
    -p hsv_lower_h:=0 -p hsv_upper_h:=10 \
    -p hsv_lower_s:=100 -p hsv_upper_s:=255 \
    -p hsv_lower_v:=100 -p hsv_upper_v:=255 \
    -p min_area:=500.0 -p max_area:=50000.0 \
    -p color_name:=red \
    -p enable_debug_view:=true
```

### 场景 2: 检测蓝色大物体

```bash
ros2 run widowx_custom_perception color_segmentation_node \
    --ros-args \
    -p hsv_lower_h:=100 -p hsv_upper_h:=130 \
    -p min_area:=10000.0 \
    -p color_name:=blue
```

### 场景 3: 低光环境检测

```bash
ros2 run widowx_custom_perception color_segmentation_node \
    --ros-args \
    -p hsv_lower_v:=50 \
    -p hsv_upper_v:=200 \
    -p morph_iterations:=3
```

## 🐛 故障排查

### 问题 1: 没有检测到轮廓

**现象**: 终端显示 "No contours found"

**解决**:
1. 检查HSV范围是否正确
2. 查看调试窗口中的 Mask 图像
3. 降低 `min_area` 或提高 `max_area`
4. 调整光照条件

### 问题 2: 检测到多个对象

**现象**: 结果不稳定，频繁跳变

**解决**:
1. 缩小HSV范围
2. 增大 `min_area`
3. 增加 `morph_iterations`
4. 调整 `min_aspect_ratio` 和 `max_aspect_ratio`

### 问题 3: 角点位置不准确

**现象**: 角点偏离实际位置

**解决**:
1. 调整 `approx_epsilon_factor` (0.01-0.05)
2. 增加 `morph_iterations` 平滑边缘
3. 提高图像分辨率

### 问题 4: 节点无法启动

**现象**: `ModuleNotFoundError: No module named 'cv_bridge'`

**解决**:
```bash
sudo apt install ros-humble-cv-bridge python3-opencv
pip3 install opencv-python numpy
```

## 📖 核心代码说明

### 颜色分割核心流程

```python
# 1. BGR → HSV
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# 2. 创建掩码
mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

# 3. 形态学操作 (去噪+填充)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

# 4. 查找轮廓
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 5. 筛选轮廓 (面积+宽高比)
for contour in contours:
    area = cv2.contourArea(contour)
    aspect_ratio = w / h
    if min_area <= area <= max_area and min_ratio <= aspect_ratio <= max_ratio:
        valid_contours.append(contour)

# 6. 提取角点
corners = find_square_corners(largest_contour)
corner_avg = corners.mean(axis=0)

# 7. 发布结果
result_msg.corner_average.x = corner_avg[0]
result_msg.corner_average.y = corner_avg[1]
```

## 🔗 相关资源

- [OpenCV HSV 颜色空间参考](https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html)
- [ROS2 参数教程](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [cv_bridge 文档](http://wiki.ros.org/cv_bridge)
- [ARCHITECTURE.md](./ARCHITECTURE.md) - 完整系统架构文档

## ✅ 快速测试清单

- [ ] 消息包编译成功
- [ ] 感知包编译成功
- [ ] 节点可以启动
- [ ] 可以订阅到相机图像
- [ ] 调试窗口正常显示
- [ ] 可以检测到目标轮廓
- [ ] 角点位置准确
- [ ] 结果消息正常发布
- [ ] 参数可以实时调整

## 📧 支持

如遇到问题，请检查:
1. ROS2 环境是否正确 source
2. 相机驱动是否正常
3. OpenCV 是否正确安装
4. 参数配置是否合理

---

**版本**: v1.0  
**最后更新**: 2025-11-09  
**ROS2 版本**: Humble  

