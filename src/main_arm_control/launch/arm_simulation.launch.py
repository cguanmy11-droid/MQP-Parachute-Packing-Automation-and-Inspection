#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Path to RViz config
    rviz_config = os.path.join(
        get_package_share_directory('main_arm_control'),
        'config',
        'simulation.rviz'
    )
    
    # Launch xs_sdk with RViz config
    xsarm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': 'wx200',
            'use_rviz': 'true',
            'use_sim': 'true',
            'rviz_config': rviz_config,  # Add this line
        }.items()
    )
    
    # Your interface node in sim mode
    main_arm_interface = Node(
        package='main_arm_control',
        executable='main_arm_interface_node',
        name='main_arm_interface',
        output='screen',
        parameters=[{
            'test_mode': False,
            'use_sim': True,
            'robot_model': 'wx200',
            'robot_name': 'wx200',
        }]
    )
    
    # ---- Packing Frame Visualization ----
    # Load the frame URDF
    frame_urdf_path = os.path.join(
        get_package_share_directory('main_arm_control'),
        'urdf',
        'framemodel.urdf'
    )
    
    with open(frame_urdf_path, 'r') as f:
        frame_urdf = f.read()
    
    # Publish the frame model
    frame_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='frame_state_publisher',
        parameters=[{'robot_description': frame_urdf}],
        remappings=[
            ('robot_description', 'frame_description')  # Publish to different topic
        ]
    )
    
    # Connect frame to robot's TF tree
    frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='frame_static_tf',
        arguments=[
            '--x', '0',
            '--y', '-0.15', 
            '--z', '-0.22',      # Adjust: frame is below robot base
            '--roll', '1.5708',
            '--pitch', '0',
            '--yaw', '-1.5708',
            '--frame-id', 'wx200/base_link',
            '--child-frame-id', 'framemodel_root'
        ]
    )

    # Side arm visualizer (from side_arm_control package)
    side_arm_visualizer = Node(
        package='side_arm_control',
        executable='side_arm_visualizer',
        name='side_arm_visualizer',
        output='screen',
    )

    # Side arm origin transform
    side_arm_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='side_arm_static_tf',
        arguments=[
            '--x', '0.35',
            '--y', '0.25',
            '--z', '-0.05',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'wx200/base_link',
            '--child-frame-id', 'side_arm_origin'
        ]
    )
    
    return LaunchDescription([
        xsarm_control_launch,
        main_arm_interface,
        frame_state_publisher,
        frame_tf,
        side_arm_tf,
        side_arm_visualizer,
    ])