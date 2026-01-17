#!/usr/bin/env python3
"""
Full Side Arm Launch File

Launches all nodes needed for side arm control:
1. Serial bridge (communicates with ESP32)
2. Coordinate node (converts mm to motor commands)
3. Interface node (high-level actions/services)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='false',
        description='Run in test mode (simulated movements)'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for ESP32 connection'
    )

    # Serial bridge node
    serial_bridge = Node(
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
    )

    # Coordinate node
    coordinate_node = Node(
        package='side_arm_control',
        executable='coordinate_node',
        name='side_arm_coordinate_node',
        parameters=[{
            'steps_per_mm_horizontal': 80.0,   # Stepper2 - belt drive (tune this)
            'steps_per_mm_vertical': 200.0,    # Stepper1 - lead screw (tune this)
            'dc_mm_per_second': 10.0,          # DC motor travel rate (tune this)
            'dc_speed_percent': 50,
            'default_speed_horizontal': 800.0,
            'default_speed_vertical': 400.0,
            'max_x_mm': 300.0,
            'max_y_mm': 200.0,
            'max_z_mm': 150.0,
        }],
        output='screen',
    )

    # Interface node
    interface_node = Node(
        package='side_arm_control',
        executable='side_arm_interface_node',
        name='side_arm_interface_node',
        parameters=[{
            'test_mode': LaunchConfiguration('test_mode'),
            'approach_offset_z': 50.0,
            'insert_depth_z': 30.0,
        }],
        output='screen',
    )

    return LaunchDescription([
        test_mode_arg,
        serial_port_arg,
        serial_bridge,
        coordinate_node,
        interface_node,
    ])
