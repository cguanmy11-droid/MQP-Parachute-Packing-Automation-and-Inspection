#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Perception nodes
        Node(
            package='parachute_perception',
            executable='loop_detector_node',
            name='loop_detector',
            output='log',
            parameters=[{'test_mode': False}]
        ),
        Node(
            package='parachute_perception',
            executable='target_selector_node',
            name='target_selector',
            output='log',
            parameters=[{'test_mode': True}]
        ),
        # Yolo Detect Package
        Node(
            package='yolo_detect_ros',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen'
        ),
        
        # Side arm control
        # Node(
        #     package='side_arm_control',
        #     executable='side_arm_interface_node',
        #     name='side_arm_interface',
        #     output='screen',
        #     parameters=[{'test_mode': True}]
        # ),
        
        # Main arm control
        Node(
            package='main_arm_control',
            executable='main_arm_interface_node',
            name='main_arm_interface',
            output='screen',
            parameters=[{'test_mode': True}]
        ),
        # Coordinator (starts after a delay to let others initialize)
        Node(
            package='parachute_coordinator',
            executable='packing_demo_node',
            name='packing_demo',
            output='screen',
            parameters=[{'test_mode': True, 'start_delay': 3.0}]
        ),
    ])