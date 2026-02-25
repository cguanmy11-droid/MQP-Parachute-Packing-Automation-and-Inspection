#!/usr/bin/env python3
"""
State Machine Demo Launch File

Launches the full state machine coordinator with perception-in-the-loop:
- Dual arm system (main arm + side arm)
- Target selector node (provides /request_next_target service)
- Packing coordinator node (state machine)
- RViz visualization

This is the complete system for automated parachute line stowing.

Usage:
    # Full simulation mode (no hardware):
    ros2 launch parachute_coordinator state_machine_demo.launch.py \
        main_arm_sim:=true side_arm_sim:=true

    # Hardware mode with test loops (no camera needed):
    ros2 launch parachute_coordinator state_machine_demo.launch.py \
        use_test_loops:=true

    # Full hardware with simulated vision:
    ros2 launch parachute_coordinator state_machine_demo.launch.py \
        vision_test_mode:=true

    # Start the stowing sequence:
    ros2 topic pub --once /stow/command std_msgs/String "data: start"

    # Check status:
    ros2 topic pub --once /stow/command std_msgs/String "data: status"

    # Change motion pattern:
    ros2 topic pub --once /stow/command std_msgs/String "data: pattern:recorded_stow"

    # Error recovery commands:
    ros2 topic pub --once /stow/command std_msgs/String "data: retry"
    ros2 topic pub --once /stow/command std_msgs/String "data: skip"
    ros2 topic pub --once /stow/command std_msgs/String "data: abort"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ==================== ARGUMENTS ====================

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
        description='Run side arm in test mode (simulated movements alongside hardware)'
    )

    # Vision/perception modes
    vision_test_mode_arg = DeclareLaunchArgument(
        'vision_test_mode', default_value='false',
        description='Use simulated loop detections (ground truth + detection simulator)'
    )
    use_test_loops_arg = DeclareLaunchArgument(
        'use_test_loops', default_value='false',
        description='Use hardcoded test loop positions in target selector (no detection needed)'
    )

    # Target selection strategy
    selection_strategy_arg = DeclareLaunchArgument(
        'selection_strategy', default_value='rightmost',
        description='Loop selection strategy: rightmost, leftmost, or nearest'
    )

    # Motion pattern
    stow_pattern_arg = DeclareLaunchArgument(
        'stow_pattern', default_value='recorded_stow',
        description='Motion pattern for stowing trajectory'
    )

    # Coordinator settings
    action_timeout_arg = DeclareLaunchArgument(
        'action_timeout', default_value='30.0',
        description='Timeout for action calls (seconds)'
    )

    # Visualization
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz visualization'
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
            'use_rviz': LaunchConfiguration('use_rviz'),
            'vision_test_mode': LaunchConfiguration('vision_test_mode'),
            'enable_loop_visualization': 'true',
        }.items()
    )

    # ==================== PERCEPTION NODES ====================

    # Target selector - provides /request_next_target service
    # Called by packing_coordinator_node in AT_LOOP state
    # Delayed slightly to allow TF tree to be established
    target_selector_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='parachute_perception',
            executable='target_selector_node',
            name='target_selector_node',
            output='screen',
            parameters=[{
                'use_test_loops': LaunchConfiguration('use_test_loops'),
                'selection_strategy': LaunchConfiguration('selection_strategy'),
                'stow_proximity_threshold': 0.01,  # meters
            }]
        )]
    )

    # ==================== COORDINATOR NODE ====================

    # State machine coordinator - orchestrates the stowing sequence
    # Delayed start to allow arm systems to initialize
    packing_coordinator_node = Node(
        package='parachute_coordinator',
        executable='packing_coordinator_node',
        name='packing_coordinator_node',
        output='screen',
        parameters=[{
            'test_mode': False,
            'stow_pattern': LaunchConfiguration('stow_pattern'),
            'action_timeout': LaunchConfiguration('action_timeout'),
        }]
    )

    # Wrap coordinator in TimerAction for delayed start
    delayed_coordinator = TimerAction(
        period=3.0,  # Wait for arms to initialize
        actions=[packing_coordinator_node]
    )

    # ==================== LAUNCH DESCRIPTION ====================

    return LaunchDescription([
        # Arguments
        main_arm_sim_arg,
        side_arm_sim_arg,
        side_arm_test_mode_arg,
        vision_test_mode_arg,
        use_test_loops_arg,
        selection_strategy_arg,
        stow_pattern_arg,
        action_timeout_arg,
        use_rviz_arg,

        # Dual arm system (includes RViz, TFs, visualization)
        dual_arm_launch,

        # Perception - target selector
        target_selector_node,

        # Coordinator (delayed start)
        delayed_coordinator,
    ])
