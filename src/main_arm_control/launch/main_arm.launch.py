#!/usr/bin/env python3
"""
Main Arm Control Launch File
Launches interface, planner, and optionally teleoperation nodes
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='wx200',
        description='Robot model'
    )
    
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation'
    )
    
    enable_teleop_arg = DeclareLaunchArgument(
        'enable_teleop',
        default_value='false',
        description='Enable Xbox teleoperation at startup'
    )
    
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='xboxone',
        description='Controller type (xboxone, ps4, etc.)'
    )
    
    # Base arm control
    arm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': LaunchConfiguration('robot_model'),
            'use_sim': LaunchConfiguration('use_sim'),
        }.items()
    )
    
    # Interface node
    interface_node = Node(
        package='main_arm_control',
        executable='main_arm_interface_node',
        name='main_arm_interface_node',
        output='screen',
        parameters=[{
            'robot_model': LaunchConfiguration('robot_model'),
            'robot_name': LaunchConfiguration('robot_model'),
        }]
    )
    
    # Planner node
    planner_node = Node(
        package='main_arm_control',
        executable='main_arm_planner_node',
        name='main_arm_planner_node',
        output='screen',
        parameters=[{
            'robot_model': LaunchConfiguration('robot_model'),
            'robot_name': LaunchConfiguration('robot_model'),
        }]
    )
    
    # Teleoperation node (optional)
    teleop_node = Node(
        package='main_arm_control',
        executable='main_arm_teleop_node',
        name='main_arm_teleop_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_teleop')),
        parameters=[{
            'robot_model': LaunchConfiguration('robot_model'),
            'controller_type': LaunchConfiguration('controller_type'),
            'auto_start': True,
        }]
    )
    
    return LaunchDescription([
        robot_model_arg,
        use_sim_arg,
        enable_teleop_arg,
        controller_type_arg,
        arm_control_launch,
        interface_node,
        planner_node,
        teleop_node
    ])
