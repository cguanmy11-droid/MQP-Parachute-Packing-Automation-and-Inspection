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

    # With calibration enabled (for capturing ground truth loop positions)
    ros2 launch parachute_coordinator dual_arm_test.launch.py vision_test_mode:=false &
    ros2 launch parachute_coordinator real_camera_test.launch.py enable_calibration:=true

    # Then trigger calibration (homes arm, moves to x=180mm, collects, returns home):
    ros2 service call /calibrate_loops std_srvs/srv/Trigger

    # Calibration sequence:
    #   1. HOME_ALL - homes all 3 axes via limit switches
    #   2. Move to calibration position (x=180mm by default)
    #   3. Collect detections for N seconds, transform to world frame
    #   4. Cluster and filter detections
    #   5. Return to home position
    #   6. Save results to /tmp/loop_calibration/latest.json

    # Calibration parameters can be adjusted:
    ros2 launch parachute_coordinator real_camera_test.launch.py \\
        enable_calibration:=true \\
        calibration_duration:=10.0 \\
        spatial_tolerance:=0.01 \\
        min_detection_count:=20 \\
        calibration_x_mm:=200.0

    # Load previously saved calibration:
    ros2 service call /load_calibration std_srvs/srv/Trigger
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
        'camera_fov', default_value='80.0',
        description='Camera horizontal field of view in degrees'
    )

    # Depth assumption for pixel-to-3D conversion
    assumed_depth_arg = DeclareLaunchArgument(
        'assumed_depth', default_value='0.22',
        description='Assumed depth from camera to loop plane (meters)'
    )

    # YOLO settings
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold', default_value='0.5',
        description='YOLO confidence threshold'
    )

    # Calibration settings
    enable_calibration_arg = DeclareLaunchArgument(
        'enable_calibration', default_value='false',
        description='Enable loop calibration node (call /calibrate_loops service to start)'
    )
    calibration_duration_arg = DeclareLaunchArgument(
        'calibration_duration', default_value='30.0',
        description='Max duration to collect detections (will finish early when all loops verified)'
    )
    spatial_tolerance_arg = DeclareLaunchArgument(
        'spatial_tolerance', default_value='0.008',
        description='Distance threshold to group detections into same loop (meters)'
    )
    min_detection_count_arg = DeclareLaunchArgument(
        'min_detection_count', default_value='15',
        description='Minimum detections required for a valid loop'
    )
    calibration_x_mm_arg = DeclareLaunchArgument(
        'calibration_x_mm', default_value='50.0',
        description='X position (mm) to move to during calibration for best view of loops'
    )
    min_loops_expected_arg = DeclareLaunchArgument(
        'min_loops_expected', default_value='1',
        description='Minimum number of loops to detect before early termination is allowed'
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

    # ==================== CALIBRATION NODE ====================

    loop_calibration = Node(
        package='parachute_perception',
        executable='loop_calibration_node',
        name='loop_calibration_node',
        output='screen',
        parameters=[{
            'collection_duration': LaunchConfiguration('calibration_duration'),
            'spatial_tolerance': LaunchConfiguration('spatial_tolerance'),
            'min_detection_count': LaunchConfiguration('min_detection_count'),
            'calibration_x_mm': LaunchConfiguration('calibration_x_mm'),
            'calibration_y_mm': 0.0,
            'calibration_z_mm': 0.0,
            'home_before_calibration': True,
            'return_home_after': True,
            'early_termination': True,  # Finish early when all loops hit threshold
            'min_loops_expected': LaunchConfiguration('min_loops_expected'),
            'stable_time': 1.0,  # Seconds with stable loop count before early termination
            'homing_timeout': 60.0,  # Max seconds to wait for is_homed
            'collect_on_return': True,  # Also collect detections while returning home
            'camera_frame_id': 'camera_frame',
            'world_frame_id': 'world',
            'save_to_file': True,
            'save_directory': '/tmp/loop_calibration',
            'loop_radius': 0.015,
            'publish_rate': 10.0,
        }],
        condition=IfCondition(LaunchConfiguration('enable_calibration'))
    )

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
        enable_calibration_arg,
        calibration_duration_arg,
        spatial_tolerance_arg,
        min_detection_count_arg,
        calibration_x_mm_arg,
        min_loops_expected_arg,

        # Nodes
        yolo_detector,
        camera_to_3d,
        loop_visualizer,  # Only runs if standalone:=true
        loop_calibration,  # Only runs if enable_calibration:=true
    ])
