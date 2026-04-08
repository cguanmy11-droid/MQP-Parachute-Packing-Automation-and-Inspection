"""
ROS 2 节点：使用 Ultralytics YOLOv8 读取 USB 摄像头并输出检测框中心。
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import Image

try:
    from ultralytics import YOLO
except ImportError as exc:  # pragma: no cover
    raise RuntimeError(
        '未找到 ultralytics 包，请先在当前 ROS 2 环境中运行: pip install ultralytics'
    ) from exc


class YoloDetectorNode(Node):
    """基于 YOLOv8 的实时检测节点。"""

    def __init__(self) -> None:
        super().__init__('yolo_detector')
        # self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_index', '/dev/video4')
        self.declare_parameter('weights_path', 'runs/yolo26m_hole2_100/weights/best.pt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.5)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('camera_frame_id', 'camera_frame')
        self.declare_parameter('centers_topic', 'yolo/centers')
        self.declare_parameter('image_topic', 'yolo/image')
        self.declare_parameter('publish_image', True)
        self.declare_parameter('display', True)

        # self.camera_index = int(self.get_parameter('camera_index').get_parameter_value().integer_value)
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().string_value
        weights_arg = self.get_parameter('weights_path').get_parameter_value().string_value
        self.weights_path = self._resolve_weights_path(weights_arg)
        self.conf_threshold = float(self.get_parameter('conf_threshold').get_parameter_value().double_value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').get_parameter_value().double_value)
        self.frame_rate = float(self.get_parameter('frame_rate').get_parameter_value().double_value)
        self.frame_rate = self.frame_rate if self.frame_rate > 0 else 30.0
        self.camera_frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
        self.display = bool(self.get_parameter('display').get_parameter_value().bool_value)
        self.publish_image = bool(self.get_parameter('publish_image').get_parameter_value().bool_value)
        centers_topic = self.get_parameter('centers_topic').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.device = self._select_device()
        self.get_logger().info(
            f'加载权重: {self.weights_path} (device={self.device})'
        )
        self.model = YOLO(str(self.weights_path))
        self.model.to(self.device)

        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f'无法打开摄像头索引 {self.camera_index}')

        self.publisher = self.create_publisher(PoseArray, centers_topic, 10)
        self.image_publisher = self.create_publisher(Image, image_topic, 10)
        self.window_name = 'YOLO Detection'
        if self.display:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(timer_period, self._process_frame)
        self.get_logger().info('YOLO 检测节点已启动，按 ESC 可在窗口中退出。')

    def _select_device(self) -> str:
        try:
            import torch  # type: ignore
            return 'cuda:0' if torch.cuda.is_available() else 'cpu'
        except Exception:
            return 'cpu'

    def _cv2_to_imgmsg(self, cv_image: np.ndarray) -> Image:
        """Convert OpenCV image to ROS Image message (without cv_bridge)."""
        msg = Image()
        msg.height = cv_image.shape[0]
        msg.width = cv_image.shape[1]
        if len(cv_image.shape) == 3:
            msg.encoding = 'bgr8'
            msg.step = cv_image.shape[1] * 3
        else:
            msg.encoding = 'mono8'
            msg.step = cv_image.shape[1]
        msg.data = cv_image.tobytes()
        return msg

    def _resolve_weights_path(self, raw_path: str) -> Path:
        """依次尝试绝对路径、share 目录、源码目录。"""
        if not raw_path:
            raw_path = 'runs/yolov8_labelme2/weights/best.pt'

        candidate = Path(raw_path).expanduser()
        if candidate.exists():
            return candidate

        try:
            share_dir = Path(get_package_share_directory('yolo_detect_ros'))
            candidate = (share_dir / raw_path).resolve()
            if candidate.exists():
                return candidate
        except PackageNotFoundError:
            pass

        source_dir = Path(__file__).resolve().parents[1]
        candidate = (source_dir / raw_path).resolve()
        if candidate.exists():
            return candidate

        raise FileNotFoundError(f'未找到指定权重文件: {raw_path}')

    def _process_frame(self) -> None:
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('读取摄像头失败，等待下一帧')
            return

        results = self.model.predict(
            source=frame,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            device=self.device,
            verbose=False,
            stream=False,
        )
        result = results[0]
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = self.camera_frame_id

        if result.boxes is not None and result.boxes.xyxy is not None:
            boxes = result.boxes.xyxy.cpu().numpy()
            for (x1, y1, x2, y2) in boxes:
                pose = Pose()
                pose.position.x = float((x1 + x2) / 2.0)
                pose.position.y = float((y1 + y2) / 2.0)
                pose.position.z = 0.0
                pose.orientation.w = 1.0
                pose_array.poses.append(pose)

        self.publisher.publish(pose_array)
        self.get_logger().debug(f'发布 {len(pose_array.poses)} 个检测中心')

        # Get annotated frame for display/publishing
        annotated = result.plot()

        # Publish annotated image to ROS2
        if self.publish_image:
            img_msg = self._cv2_to_imgmsg(annotated)
            img_msg.header.stamp = pose_array.header.stamp
            img_msg.header.frame_id = self.camera_frame_id
            self.image_publisher.publish(img_msg)

        if self.display:
            cv2.imshow(self.window_name, annotated)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                self.get_logger().info('收到 ESC，准备关闭节点。')
                rclpy.shutdown()

    def destroy_node(self) -> bool:
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        if self.display:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[YoloDetectorNode] = None
    try:
        node = YoloDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


