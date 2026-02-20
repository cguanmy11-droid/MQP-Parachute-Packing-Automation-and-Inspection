#!/usr/bin/env python3
"""
Full Stow Demo Launch File

Launches the complete system and runs a demo stowing sequence
with a predefined loop position.

Usage:
    # Full demo with hardware
    ros2 launch parachute_coordinator full_stow_demo.launch.py

    # Demo with simulation (no hardware)
    ros2 launch parachute_coordinator full_stow_demo.launch.py \
        main_arm_sim:=true side_arm_sim:=true

    # Custom target position
    ros2 launch parachute_coordinator full_stow_demo.launch.py \
        target_x:=0.40 target_y:=0.10 target_z:=0.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ==================== ARGUMENTS ====================

    # Target/hook position for the demo (anchor point for motion patterns)
    target_x_arg = DeclareLaunchArgument(
        'target_x', default_value='0.30',
        description='Target/hook X position (meters, world frame)'
    )
    target_y_arg = DeclareLaunchArgument(
        'target_y', default_value='0.04',
        description='Target/hook Y position (meters, world frame)'
    )
    target_z_arg = DeclareLaunchArgument(
        'target_z', default_value='0.022',
        description='Target/hook Z position (meters, world frame)'
    )

    # Side arm position (in side arm frame, mm)
    side_arm_x_arg = DeclareLaunchArgument(
        'side_arm_x_mm', default_value='82.0',
        description='Side arm X position (mm)'
    )
    side_arm_y_arg = DeclareLaunchArgument(
        'side_arm_y_mm', default_value='27.0',
        description='Side arm Y position (mm)'
    )
    side_arm_z_arg = DeclareLaunchArgument(
        'side_arm_z_mm', default_value='140.0',
        description='Side arm Z position (mm)'
    )

    # Motion pattern
    stow_pattern_arg = DeclareLaunchArgument(
        'stow_pattern', default_value='recorded_stow',
        description='Motion pattern for stowing (stow_arc, approach_vertical, etc.)'
    )

    # Hardware/simulation modes
    main_arm_sim_arg = DeclareLaunchArgument(
        'main_arm_sim', default_value='false',
        description='Run main arm in simulation mode'
    )
    side_arm_sim_arg = DeclareLaunchArgument(
        'side_arm_sim', default_value='false',
        description='Run side arm in pure simulation mode (no serial bridge)'
    )
    side_arm_test_mode_arg = DeclareLaunchArgument(
        'side_arm_test_mode', default_value='false',
        description='Run side arm in test/simulation mode (with hardware)'
    )

    # Demo control
    auto_start_arg = DeclareLaunchArgument(
        'auto_start', default_value='true',
        description='Start demo automatically'
    )
    start_delay_arg = DeclareLaunchArgument(
        'start_delay', default_value='0.0',
        description='Delay before starting demo (seconds)'
    )

    # ==================== INCLUDE DUAL ARM SYSTEM ====================

    dual_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('parachute_coordinator'),
                'launch',
                'dual_arm_test.launch.py'
            ])
        ]),
        launch_arguments={
            'enable_main_arm': 'true',
            'main_arm_sim': LaunchConfiguration('main_arm_sim'),
            'enable_side_arm': 'true',
            'side_arm_sim': LaunchConfiguration('side_arm_sim'),
            'side_arm_test_mode': LaunchConfiguration('side_arm_test_mode'),
            'enable_teleop': 'false',
            'use_rviz': 'true',
            'vision_test_mode': 'false',  # Don't need vision for demo
        }.items()
    )

    # ==================== DEMO NODE ====================

    demo_node = Node(
        package='parachute_coordinator',
        executable='full_stow_demo_node',
        name='full_stow_demo_node',
        output='screen',
        parameters=[{
            'target_x': LaunchConfiguration('target_x'),
            'target_y': LaunchConfiguration('target_y'),
            'target_z': LaunchConfiguration('target_z'),
            'side_arm_x_mm': LaunchConfiguration('side_arm_x_mm'),
            'side_arm_y_mm': LaunchConfiguration('side_arm_y_mm'),
            'side_arm_z_mm': LaunchConfiguration('side_arm_z_mm'),
            'stow_pattern': LaunchConfiguration('stow_pattern'),
            'auto_start': LaunchConfiguration('auto_start'),
            'start_delay': LaunchConfiguration('start_delay'),
            'step_delay': 2.0,
        }]
    )

    # ==================== LAUNCH ====================

    return LaunchDescription([
        # Arguments
        target_x_arg,
        target_y_arg,
        target_z_arg,
        side_arm_x_arg,
        side_arm_y_arg,
        side_arm_z_arg,
        stow_pattern_arg,
        main_arm_sim_arg,
        side_arm_sim_arg,
        side_arm_test_mode_arg,
        auto_start_arg,
        start_delay_arg,

        # Launch dual arm system
        dual_arm_launch,

        # Launch demo node (with delay to let system start)
        TimerAction(
            period=3.0,
            actions=[demo_node]
        ),
    ])
