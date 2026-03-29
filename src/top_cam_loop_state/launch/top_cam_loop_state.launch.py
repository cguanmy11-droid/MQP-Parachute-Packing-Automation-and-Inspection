import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Ensure user site-packages are visible (ROS 2 disables them by default)
_user_site = os.path.expanduser("~/.local/lib/python3.12/site-packages")
_extra_path = _user_site if os.path.isdir(_user_site) else ""


def generate_launch_description():
    existing_pythonpath = os.environ.get("PYTHONPATH", "")
    merged_pythonpath = f"{_extra_path}:{existing_pythonpath}" if _extra_path else existing_pythonpath

    return LaunchDescription([
        SetEnvironmentVariable("PYTHONPATH", merged_pythonpath),
        DeclareLaunchArgument("camera_index",  default_value="0",    description="USB camera index"),
        DeclareLaunchArgument("det_weights",   default_value="",     description="Path to YOLO detection weights (.pt)"),
        DeclareLaunchArgument("cls_weights",   default_value="",     description="Path to YOLO classification weights (.pt)"),
        DeclareLaunchArgument("conf_threshold",default_value="0.35", description="Detection confidence threshold"),
        DeclareLaunchArgument("iou_threshold", default_value="0.45", description="NMS IoU threshold"),
        DeclareLaunchArgument("frame_rate",    default_value="30.0", description="Processing frame rate (Hz)"),
        DeclareLaunchArgument("display",       default_value="true", description="Show OpenCV window"),

        Node(
            package="top_cam_loop_state",
            executable="loop_state_node",
            name="top_cam_loop_state",
            output="screen",
            parameters=[{
                "camera_index":   LaunchConfiguration("camera_index"),
                "det_weights":    LaunchConfiguration("det_weights"),
                "cls_weights":    LaunchConfiguration("cls_weights"),
                "conf_threshold": LaunchConfiguration("conf_threshold"),
                "iou_threshold":  LaunchConfiguration("iou_threshold"),
                "frame_rate":     LaunchConfiguration("frame_rate"),
                "display":        LaunchConfiguration("display"),
            }],
        ),
    ])
