#!/usr/bin/env python3
"""
颜色分割节点 - 使用HSV颜色空间分割方形目标并提取角点
订阅: /camera1/image_raw, /camera1/camera_info
发布: /segmentation/results (SegmentationResult)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from widowx_custom_msgs.msg import SegmentationResult
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorSegmentationNode(Node):
    """颜色分割节点 - 检测并分割彩色方形目标"""

    def __init__(self):
        super().__init__('color_segmentation_node')

        # 声明可调参数 - HSV颜色范围
        self.declare_parameter('hsv_lower_h', 0)      # 色调下限 (0-179)
        self.declare_parameter('hsv_lower_s', 100)    # 饱和度下限 (0-255)
        self.declare_parameter('hsv_lower_v', 100)    # 明度下限 (0-255)
        self.declare_parameter('hsv_upper_h', 10)     # 色调上限 (0-179)
        self.declare_parameter('hsv_upper_s', 255)    # 饱和度上限 (0-255)
        self.declare_parameter('hsv_upper_v', 255)    # 明度上限 (0-255)

        # 形态学操作参数
        self.declare_parameter('morph_kernel_size', 5)      # 形态学核大小
        self.declare_parameter('morph_iterations', 2)       # 形态学操作迭代次数

        # 轮廓筛选参数
        self.declare_parameter('min_area', 500.0)           # 最小面积 (像素)
        self.declare_parameter('max_area', 50000.0)         # 最大面积 (像素)
        self.declare_parameter('min_aspect_ratio', 0.7)     # 最小宽高比
        self.declare_parameter('max_aspect_ratio', 1.3)     # 最大宽高比 (接近正方形)

        # 角点检测参数
        self.declare_parameter('approx_epsilon_factor', 0.02)  # 轮廓逼近精度因子

        # 可视化参数
        self.declare_parameter('enable_debug_view', True)   # 是否显示调试窗口
        self.declare_parameter('color_name', 'red')         # 目标颜色名称

        # 话题名称参数
        self.declare_parameter('image_topic', '/camera1/image_raw')
        self.declare_parameter('camera_info_topic', '/camera1/camera_info')
        self.declare_parameter('result_topic', '/segmentation/results')

        # CV Bridge
        self.bridge = CvBridge()

        # 相机信息缓存
        self.camera_info = None

        # 获取话题名称
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        result_topic = self.get_parameter('result_topic').value

        # 创建订阅者
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10
        )

        # 创建发布者
        self.result_pub = self.create_publisher(
            SegmentationResult,
            result_topic,
            10
        )

        self.get_logger().info('Color Segmentation Node initialized')
        self.get_logger().info(f'Subscribing to: {image_topic}')
        self.get_logger().info(f'Publishing to: {result_topic}')

    def camera_info_callback(self, msg):
        """相机信息回调"""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('Camera info received')

    def get_hsv_range(self):
        """获取HSV颜色范围"""
        lower = np.array([
            self.get_parameter('hsv_lower_h').value,
            self.get_parameter('hsv_lower_s').value,
            self.get_parameter('hsv_lower_v').value
        ])
        upper = np.array([
            self.get_parameter('hsv_upper_h').value,
            self.get_parameter('hsv_upper_s').value,
            self.get_parameter('hsv_upper_v').value
        ])
        return lower, upper

    def find_square_corners(self, contour):
        """
        从轮廓中提取四个角点
        使用多边形逼近找到四边形的四个顶点
        """
        epsilon = self.get_parameter('approx_epsilon_factor').value * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # 如果逼近结果是四边形，直接返回
        if len(approx) == 4:
            return approx.reshape(4, 2)

        # 否则，使用凸包和极值点方法
        hull = cv2.convexHull(contour)

        # 找到最小外接矩形的四个角点
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        return box

    def order_corners(self, corners):
        """
        排序角点: [top-left, top-right, bottom-right, bottom-left]
        """
        # 计算中心点
        center = corners.mean(axis=0)

        # 按照角度排序
        angles = np.arctan2(corners[:, 1] - center[1], corners[:, 0] - center[0])
        sorted_indices = np.argsort(angles)
        sorted_corners = corners[sorted_indices]

        # 找到最上方的点作为起点
        top_idx = np.argmin(sorted_corners[:, 1])

        # 重新排列使top-left在第一位
        ordered = np.roll(sorted_corners, -top_idx, axis=0)

        return ordered

    def image_callback(self, msg):
        """图像回调 - 主处理函数"""
        try:
            # 将ROS图像转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 转换到HSV颜色空间
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # 获取HSV阈值
            lower_hsv, upper_hsv = self.get_hsv_range()

            # 创建掩码
            mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

            # 形态学操作 - 去噪和填充
            kernel_size = self.get_parameter('morph_kernel_size').value
            iterations = self.get_parameter('morph_iterations').value
            kernel = np.ones((kernel_size, kernel_size), np.uint8)

            # 先开运算去除噪点，再闭运算填充空洞
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=iterations)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=iterations)

            # 查找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) == 0:
                self.get_logger().debug('No contours found')
                return

            # 筛选轮廓 - 面积和宽高比
            min_area = self.get_parameter('min_area').value
            max_area = self.get_parameter('max_area').value
            min_ratio = self.get_parameter('min_aspect_ratio').value
            max_ratio = self.get_parameter('max_aspect_ratio').value

            valid_contours = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if min_area <= area <= max_area:
                    # 检查宽高比
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = float(w) / h if h > 0 else 0

                    if min_ratio <= aspect_ratio <= max_ratio:
                        valid_contours.append(contour)

            if len(valid_contours) == 0:
                self.get_logger().debug('No valid square contours found')
                return

            # 选择最大的有效轮廓
            largest_contour = max(valid_contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            # 计算质心
            M = cv2.moments(largest_contour)
            if M['m00'] == 0:
                return

            cx = M['m10'] / M['m00']
            cy = M['m01'] / M['m00']

            # 提取四个角点
            corners = self.find_square_corners(largest_contour)
            corners = self.order_corners(corners)

            # 计算四个角点的平均位置
            corner_avg = corners.mean(axis=0)

            # 创建并发布结果消息
            result_msg = SegmentationResult()
            result_msg.header = msg.header
            result_msg.color_name = self.get_parameter('color_name').value

            # 质心
            result_msg.centroid.x = float(cx)
            result_msg.centroid.y = float(cy)
            result_msg.centroid.z = 0.0

            # 角点平均位置
            result_msg.corner_average.x = float(corner_avg[0])
            result_msg.corner_average.y = float(corner_avg[1])
            result_msg.corner_average.z = 0.0

            # 四个角点
            for i in range(4):
                corner_point = Point()
                corner_point.x = float(corners[i][0])
                corner_point.y = float(corners[i][1])
                corner_point.z = 0.0
                result_msg.corners[i] = corner_point

            result_msg.area = float(area)
            result_msg.confidence = 0.95  # 可根据实际情况调整

            # 添加相机信息
            if self.camera_info is not None:
                result_msg.camera_info = self.camera_info

            # 发布结果
            self.result_pub.publish(result_msg)

            self.get_logger().info(
                f'Detected square: centroid=({cx:.1f}, {cy:.1f}), '
                f'corner_avg=({corner_avg[0]:.1f}, {corner_avg[1]:.1f}), '
                f'area={area:.1f}'
            )

            # 可视化（如果启用）
            if self.get_parameter('enable_debug_view').value:
                self.visualize(cv_image, mask, largest_contour, corners, (cx, cy), corner_avg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def visualize(self, image, mask, contour, corners, centroid, corner_avg):
        """可视化调试"""
        # 创建副本
        vis_image = image.copy()
        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # 绘制轮廓
        cv2.drawContours(vis_image, [contour], -1, (0, 255, 0), 2)

        # 绘制角点
        for i, corner in enumerate(corners):
            cv2.circle(vis_image, tuple(corner.astype(int)), 8, (255, 0, 0), -1)
            cv2.putText(vis_image, str(i), tuple(corner.astype(int)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # 绘制质心
        cv2.circle(vis_image, (int(centroid[0]), int(centroid[1])), 5, (0, 0, 255), -1)
        cv2.putText(vis_image, 'Centroid', (int(centroid[0]) + 10, int(centroid[1])),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 绘制角点平均位置
        cv2.circle(vis_image, (int(corner_avg[0]), int(corner_avg[1])), 5, (255, 255, 0), -1)
        cv2.putText(vis_image, 'Corner Avg', (int(corner_avg[0]) + 10, int(corner_avg[1])),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # 显示
        cv2.imshow('Original', vis_image)
        cv2.imshow('Mask', mask_color)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ColorSegmentationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

