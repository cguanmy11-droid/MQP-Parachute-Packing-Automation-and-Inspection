from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg_share = Path(get_package_share_directory('yolo_detect_ros'))
    bundled_weights = pkg_share / 'runs' / 'yolov8_labelme2' / 'weights' / 'best.pt'
    default_weights = str(bundled_weights) if bundled_weights.exists() else 'runs/yolov8_labelme2/weights/best.pt'

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'camera_index',
                default_value='0',
                description='USB 摄像头索引 (与 OpenCV VideoCapture 对应)',
            ),
            DeclareLaunchArgument(
                'weights_path',
                default_value=default_weights,
                description='YOLOv8 权重文件路径 (可填绝对路径或相对包 share 的路径)',
            ),
            DeclareLaunchArgument(
                'conf_threshold',
                default_value='0.5',
                description='目标置信度阈值',
            ),
            DeclareLaunchArgument(
                'iou_threshold',
                default_value='0.5',
                description='NMS IOU 阈值',
            ),
            DeclareLaunchArgument(
                'frame_rate',
                default_value='30.0',
                description='处理帧率 (Hz)',
            ),
            DeclareLaunchArgument(
                'display',
                default_value='true',
                description='是否弹出 OpenCV 窗口显示检测结果',
            ),
            Node(
                package='yolo_detect_ros',
                executable='yolo_detector',
                name='yolo_detector',
                output='screen',
                parameters=[
                    {
                        'camera_index': LaunchConfiguration('camera_index'),
                        'weights_path': LaunchConfiguration('weights_path'),
                        'conf_threshold': LaunchConfiguration('conf_threshold'),
                        'iou_threshold': LaunchConfiguration('iou_threshold'),
                        'frame_rate': LaunchConfiguration('frame_rate'),
                        'display': LaunchConfiguration('display'),
                    }
                ],
            ),
        ]
    )



