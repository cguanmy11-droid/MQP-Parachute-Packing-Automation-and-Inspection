#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Launch xs_sdk with use_sim_time and fake hardware
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
            'use_sim': 'true',  # This launches fake hardware
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
    
    return LaunchDescription([
        xsarm_control_launch,
        main_arm_interface,
    ])