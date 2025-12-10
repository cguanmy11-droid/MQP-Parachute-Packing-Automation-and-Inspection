#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Include the xsarm_control launch file (starts xs_sdk)
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
            'use_rviz': 'false',  # We'll launch RViz separately if needed
        }.items()
    )
    
    # Your main arm interface node
    main_arm_interface = Node(
        package='main_arm_control',
        executable='main_arm_interface_node',
        name='main_arm_interface',
        output='screen',
        parameters=[{
            'test_mode': False,
            'robot_model': 'wx200',
            'robot_name': 'wx200',
            'moving_time': '1',
            'accel_time': '1',
        }]
    )
    
    return LaunchDescription([
        xsarm_control_launch,
        main_arm_interface,
    ])