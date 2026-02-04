#!/usr/bin/env python3
"""
Dual Arm Test Launch File

Launches both side arm and main arm systems for integrated testing
with digital twin visualization in RViz.

Usage:
    # Basic hardware mode (both arms)
    ros2 launch parachute_coordinator dual_arm_test.launch.py

    # With Xbox controller for main arm
    ros2 launch parachute_coordinator dual_arm_test.launch.py enable_teleop:=true

    # Main arm in simulation, side arm on hardware
    ros2 launch parachute_coordinator dual_arm_test.launch.py main_arm_sim:=true

    # Side arm only (no main arm)
    ros2 launch parachute_coordinator dual_arm_test.launch.py enable_main_arm:=false

    # Main arm only (no side arm)
    ros2 launch parachute_coordinator dual_arm_test.launch.py enable_side_arm:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Load frame URDF
    frame_urdf_path = os.path.join(
        get_package_share_directory('main_arm_control'),
        'urdf',
        'framemodel.urdf'
    )

    with open(frame_urdf_path, 'r') as f:
        frame_urdf = f.read()

    # ==================== LAUNCH ARGUMENTS ====================

    # Main arm arguments
    enable_main_arm_arg = DeclareLaunchArgument(
        'enable_main_arm',
        default_value='true',
        description='Enable main arm (WX200) control'
    )

    main_arm_sim_arg = DeclareLaunchArgument(
        'main_arm_sim',
        default_value='false',
        description='Run main arm in simulation mode'
    )

    enable_teleop_arg = DeclareLaunchArgument(
        'enable_teleop',
        default_value='false',
        description='Enable Xbox controller teleoperation for main arm'
    )

    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='xboxone',
        description='Controller type (xboxone, ps4, etc.)'
    )

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='wx200',
        description='Main arm robot model'
    )

    # Side arm arguments
    enable_side_arm_arg = DeclareLaunchArgument(
        'enable_side_arm',
        default_value='true',
        description='Enable side arm control'
    )

    side_arm_test_mode_arg = DeclareLaunchArgument(
        'side_arm_test_mode',
        default_value='false',
        description='Run side arm in test mode (simulated movements)'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for side arm ESP32 connection'
    )

    # Visualization arguments
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable side arm RViz visualization marker'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for main arm visualization'
    )

    # Vision/perception arguments
    vision_test_mode_arg = DeclareLaunchArgument(
        'vision_test_mode',
        default_value='false',
        description='Use simulated loop detections instead of camera'
    )

    enable_loop_visualization_arg = DeclareLaunchArgument(
        'enable_loop_visualization',
        default_value='true',
        description='Enable loop detection visualization in RViz'
    )

    # ==================== MAIN ARM NODES ====================

    # Interbotix arm control (hardware or sim)
    main_arm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': LaunchConfiguration('robot_model'),
            'use_sim': LaunchConfiguration('main_arm_sim'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_main_arm'))
    )

    # Main arm interface node
    main_arm_interface = Node(
        package='main_arm_control',
        executable='main_arm_interface_node',
        name='main_arm_interface_node',
        output='screen',
        parameters=[{
            'robot_model': LaunchConfiguration('robot_model'),
            'robot_name': LaunchConfiguration('robot_model'),
            'use_sim': LaunchConfiguration('main_arm_sim'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_main_arm'))
    )

    # Main arm teleop node (optional)
    main_arm_teleop = Node(
        package='main_arm_control',
        executable='main_arm_teleop_node',
        name='main_arm_teleop_node',
        output='screen',
        parameters=[{
            'robot_model': LaunchConfiguration('robot_model'),
            'controller_type': LaunchConfiguration('controller_type'),
            'auto_start': True,
        }],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('enable_main_arm'), "' == 'true' and '",
                LaunchConfiguration('enable_teleop'), "' == 'true'"
            ])
        )
    )

    # Joy node for Xbox controller
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        condition=IfCondition(LaunchConfiguration('enable_teleop'))
    )

    # ==================== SIDE ARM NODES ====================

    # Serial bridge node (communicates with ESP32)
    side_arm_serial_bridge = Node(
        package='side_arm_motor_control_bridge',
        executable='serial_bridge',
        name='side_arm_serial_bridge',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 115200,
            'command_topic': '/side_arm/command',
            'state_topic': '/side_arm/state',
            'state_request_hz': 10.0,
            'auto_request_state': True,
        }],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('enable_side_arm'), "' == 'true' and '",
                LaunchConfiguration('side_arm_test_mode'), "' == 'false'"
            ])
        )
    )

    # Coordinate node (converts mm to motor commands)
    side_arm_coordinate = Node(
        package='side_arm_control',
        executable='coordinate_node',
        name='side_arm_coordinate_node',
        parameters=[{
            'steps_per_mm_horizontal': 300.0,
            'steps_per_mm_vertical': 100.0,
            'dc_mm_per_second': 4.0,
            'dc_speed_percent': 50,
            'default_speed_horizontal': 1200.0,
            'default_speed_vertical': 500.0,
            'max_x_mm': 300.0,
            'max_y_mm': 200.0,
            'max_z_mm': 150.0,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_side_arm'))
    )

    # Side arm interface node (high-level actions/services)
    side_arm_interface = Node(
        package='side_arm_control',
        executable='side_arm_interface_node',
        name='side_arm_interface_node',
        parameters=[{
            'test_mode': LaunchConfiguration('side_arm_test_mode'),
            'approach_offset_z': 50.0,
            'insert_depth_z': 30.0,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_side_arm'))
    )

    #  Side arm visualizer (RViz marker + TF for hook and camera)
    side_arm_visualizer = Node(
        package='side_arm_control',
        executable='side_arm_visualizer',
        name='side_arm_visualizer',
        output='screen',
        parameters=[{
            # Hook mesh orientation
            'roll': -1.5708,
            'pitch': 0.0,
            'yaw': -1.5708,
            # Hook mesh offset
            'offset_x': -0.01,
            'offset_y': 0.009,
            'offset_z': 0.07,
            'scale': 0.001,
            # Servo rotation
            'servo_axis': 'pitch',
            'servo_scale': 0.001,
            # Test mode
            'test_mode': LaunchConfiguration('side_arm_test_mode'),
            'test_x': 0.0,
            'test_y': 0.0,
            'test_z': 0.0,
            # TF publishing
            'publish_hook_tf': True,
            'hook_frame_id': 'side_arm_hook',
            # Camera frame (child of hook) - adjust these to calibrate camera position
            'publish_camera_tf': True,
            'camera_frame_id': 'camera_frame',
            'camera_offset_x': 0.0,    # Forward from hook
            'camera_offset_y': 0.0,    # Left/right from hook
            'camera_offset_z': 0.05,   # Distance from hook tip
            'camera_roll': 0.0,        # Camera orientation relative to hook
            'camera_pitch': 0.0,
            'camera_yaw': 0.0,
        }],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('enable_side_arm'), "' == 'true' and '",
                LaunchConfiguration('enable_visualization'), "' == 'true'"
            ])
        )
    )

    # ==================== DIGITAL TWIN LAUNCH ====================

    # Frame state publisher
    frame_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='frame_state_publisher',
        parameters=[{'robot_description': frame_urdf}],
        remappings=[('robot_description', 'frame_description')],
        condition=IfCondition(LaunchConfiguration('enable_main_arm'))
    )

    # Frame TF (position relative to robot base)
    frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='frame_static_tf',
        arguments=[
            '--x', '0', '--y', '-0.15', '--z', '-0.22',
            '--roll', '1.5708', '--pitch', '0', '--yaw', '-1.5708',
            '--frame-id', 'wx200/base_link',
            '--child-frame-id', 'framemodel_root'
        ],
        condition=IfCondition(LaunchConfiguration('enable_main_arm'))
    )

    # Side arm origin TF
    side_arm_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='side_arm_static_tf',
        arguments=[
            '--x', '0.4', '--y', '0.19', '--z', '-0.03',
            '--roll', '1.5708', '--pitch', '0', '--yaw', '3.1416',
            '--frame-id', 'wx200/base_link',
            '--child-frame-id', 'side_arm_origin'
        ],
        condition=IfCondition(LaunchConfiguration('enable_side_arm'))
    )

    # NOTE: camera_frame TF is now published dynamically by side_arm_visualizer
    # as a child of side_arm_hook, so it moves with the side arm

    # ==================== PERCEPTION VISUALIZATION NODES ====================

    # Loop visualizer (subscribes to /detected_loops, publishes /loop_markers)
    loop_visualizer = Node(
        package='parachute_perception',
        executable='loop_visualizer_node',
        name='loop_visualizer_node',
        output='screen',
        parameters=[{
            'marker_scale': 0.015,
            'grid_enabled': True,
            'grid_size_x': 0.4,
            'grid_size_y': 0.3,
            'frame_id': 'camera_frame',
        }],
        condition=IfCondition(LaunchConfiguration('enable_loop_visualization'))
    )

    # Test loop publisher (publishes simulated /detected_loops when camera not available)
    test_loop_publisher = Node(
        package='parachute_perception',
        executable='test_loop_publisher_node',
        name='test_loop_publisher_node',
        output='screen',
        parameters=[{
            'publish_rate': 2.0,
            'num_loops': 5,
            'pattern': 'random',
            'x_min': 0.05,
            'x_max': 0.35,
            'y_fixed': -0.11,
            'z_min': 0.02,
            'z_max': 0.15,
            'movement': False,
            'frame_id': 'camera_frame',
        }],
        condition=IfCondition(LaunchConfiguration('vision_test_mode'))
    )

    # ==================== LAUNCH DESCRIPTION ====================

    return LaunchDescription([
        # Arguments
        enable_main_arm_arg,
        main_arm_sim_arg,
        enable_teleop_arg,
        controller_type_arg,
        robot_model_arg,
        enable_side_arm_arg,
        side_arm_test_mode_arg,
        serial_port_arg,
        enable_visualization_arg,
        use_rviz_arg,
        vision_test_mode_arg,
        enable_loop_visualization_arg,
        joy_node,

        # Main arm nodes
        main_arm_control_launch,
        main_arm_interface,
        main_arm_teleop,

        # Side arm nodes
        side_arm_serial_bridge,
        side_arm_coordinate,
        side_arm_interface,
        side_arm_visualizer,

        # Digital twin visualization
        frame_state_publisher,
        frame_tf,
        side_arm_tf,
        # camera_frame is now published by side_arm_visualizer

        # Perception visualization
        loop_visualizer,
        test_loop_publisher,
    ])
