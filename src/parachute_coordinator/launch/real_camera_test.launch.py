#!/usr/bin/env python3
"""
Real Camera Test Launch File

Launches the perception system with a real USB camera for loop detection testing.
Includes YOLO detector, pixel-to-3D conversion, and visualization.

Usage:
    # Basic usage with defaults
    ros2 launch parachute_coordinator real_camera_test.launch.py

    # Specify camera index
    ros2 launch parachute_coordinator real_camera_test.launch.py camera_index:=1

    # Adjust depth assumption (meters from camera to loop plane)
    ros2 launch parachute_coordinator real_camera_test.launch.py assumed_depth:=0.15

    # With full dual arm system
    ros2 launch parachute_coordinator dual_arm_test.launch.py &
    ros2 launch parachute_coordinator real_camera_test.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ==================== ARGUMENTS ====================

    # Camera settings
    camera_index_arg = DeclareLaunchArgument(
        'camera_index', default_value='4',
        description='USB camera index (0, 1, 2, etc.)'
    )
    display_arg = DeclareLaunchArgument(
        'display', default_value='true',
        description='Show YOLO detection window'
    )

    # Camera intrinsics
    image_width_arg = DeclareLaunchArgument(
        'image_width', default_value='640',
        description='Camera image width in pixels'
    )
    image_height_arg = DeclareLaunchArgument(
        'image_height', default_value='480',
        description='Camera image height in pixels'
    )
    camera_fov_arg = DeclareLaunchArgument(
        'camera_fov', default_value='60.0',
        description='Camera horizontal field of view in degrees'
    )

    # Depth assumption for pixel-to-3D conversion
    assumed_depth_arg = DeclareLaunchArgument(
        'assumed_depth', default_value='0.12',
        description='Assumed depth from camera to loop plane (meters)'
    )

    # YOLO settings
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold', default_value='0.5',
        description='YOLO confidence threshold'
    )

    # ==================== YOLO DETECTOR ====================

    yolo_detector = Node(
        package='yolo_detect_ros',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'camera_index': LaunchConfiguration('camera_index'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'iou_threshold': 0.5,
            'frame_rate': 30.0,
            'camera_frame_id': 'camera_frame',
            'centers_topic': '/yolo/centers',
            'display': LaunchConfiguration('display'),
        }]
    )

    # ==================== PIXEL TO 3D CONVERTER ====================

    camera_to_3d = Node(
        package='parachute_perception',
        executable='camera_to_3d_node',
        name='camera_to_3d_node',
        output='screen',
        parameters=[{
            # Camera intrinsics
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'camera_fov_horizontal': LaunchConfiguration('camera_fov'),
            # Depth assumption
            'assumed_depth': LaunchConfiguration('assumed_depth'),
            # Topics
            'input_topic': '/yolo/centers',
            'output_topic': '/detected_loops',
            'camera_frame_id': 'camera_frame',
            # Detection confidence
            'base_confidence': 0.85,
        }]
    )

    # ==================== VISUALIZATION ====================
    # NOTE: loop_visualizer is NOT included here because dual_arm_test.launch.py
    # already starts it. Running this launch file adds only the camera pipeline.
    # If running standalone (without dual_arm_test), uncomment the visualizer below.

    # Standalone mode flag
    standalone_arg = DeclareLaunchArgument(
        'standalone', default_value='false',
        description='Set true if running without dual_arm_test (starts visualizer)'
    )

    # Only start visualizer in standalone mode
    from launch.conditions import IfCondition
    loop_visualizer = Node(
        package='parachute_perception',
        executable='loop_visualizer_node',
        name='loop_visualizer_node',
        output='screen',
        parameters=[{
            'input_frame_id': 'camera_frame',
            'output_frame_id': 'world',
            'marker_scale': 0.015,
            'grid_enabled': True,
            'grid_size_x': 0.3,
            'grid_size_y': 0.2,
            'grid_offset_z': LaunchConfiguration('assumed_depth'),
        }],
        condition=IfCondition(LaunchConfiguration('standalone'))
    )

    # ==================== RVIZ ====================

    # Note: Uses the existing dual_arm config if available
    # For standalone testing, you can view /detected_loop_markers and /camera_fov_grid
    # Add these displays in RViz:
    #   - MarkerArray: /detected_loop_markers
    #   - MarkerArray: /camera_fov_grid
    #   - TF (to see camera_frame position)

    # ==================== LAUNCH ====================

    return LaunchDescription([
        # Arguments
        camera_index_arg,
        display_arg,
        image_width_arg,
        image_height_arg,
        camera_fov_arg,
        assumed_depth_arg,
        conf_threshold_arg,
        standalone_arg,

        # Nodes
        yolo_detector,
        camera_to_3d,
        loop_visualizer,  # Only runs if standalone:=true
    ])
