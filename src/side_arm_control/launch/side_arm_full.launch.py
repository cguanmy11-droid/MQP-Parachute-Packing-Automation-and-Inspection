#!/usr/bin/env python3
"""
Full Side Arm Launch File

Launches all nodes needed for side arm control:
1. Serial bridge (communicates with ESP32)
2. Coordinate node (converts mm to motor commands)
3. Interface node (high-level actions/services)

Usage:
    # Default (v1 config):
    ros2 launch side_arm_control side_arm_full.launch.py

    # With specific config:
    ros2 launch side_arm_control side_arm_full.launch.py arm_config:=side_arm_v2.yaml

    # With environment variable:
    SIDE_ARM_CONFIG=/path/to/config.yaml ros2 launch side_arm_control side_arm_full.launch.py
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_side_arm_config(config_file: str) -> dict:
    """Load side arm configuration from YAML file."""
    with open(config_file, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # =============================================================================
    # LOAD CONFIGURATION
    # =============================================================================
    pkg_share = get_package_share_directory('side_arm_control')
    config_dir = os.path.join(pkg_share, 'config')
    default_config = os.path.join(config_dir, 'side_arm_v1.yaml')

    # Load config from environment variable or default
    config_file = os.environ.get('SIDE_ARM_CONFIG', default_config)
    config = load_side_arm_config(config_file)

    # Extract config sections
    serial_config = config.get('serial_bridge', {})
    coord_config = config.get('coordinate_node', {})
    workspace_config = config.get('workspace', {})
    interface_config = config.get('interface_node', {})

    # =============================================================================
    # LAUNCH ARGUMENTS
    # =============================================================================
    arm_config_arg = DeclareLaunchArgument(
        'arm_config',
        default_value=config_file,
        description='Path to side arm YAML config file'
    )

    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='false',
        description='Run in test mode (simulated movements)'
    )

    # Serial port: env var > config file > default
    serial_port_default = os.environ.get(
        'SIDE_ARM_PORT',
        serial_config.get('serial_port', '/dev/ttyUSB0')
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value=serial_port_default,
        description='Serial port for ESP32 connection'
    )

    # =============================================================================
    # NODES
    # =============================================================================

    # Serial bridge node
    serial_bridge = Node(
        package='side_arm_motor_control_bridge',
        executable='serial_bridge',
        name='side_arm_serial_bridge',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': serial_config.get('baud_rate', 115200),
            'command_topic': '/side_arm/command',
            'state_topic': '/side_arm/state',
            'state_request_hz': serial_config.get('state_request_hz', 10.0),
            'auto_request_state': serial_config.get('auto_request_state', True),
        }],
        output='screen',
    )

    # Coordinate node
    coordinate_node = Node(
        package='side_arm_control',
        executable='coordinate_node',
        name='side_arm_coordinate_node',
        parameters=[{
            'steps_per_mm_horizontal': coord_config.get('steps_per_mm_horizontal', 300.0),
            'steps_per_mm_vertical': coord_config.get('steps_per_mm_vertical', 100.0),
            'dc_mm_per_second': coord_config.get('dc_mm_per_second', 4.0),
            'dc_speed_percent': coord_config.get('dc_speed_percent', 50),
            'default_speed_horizontal': coord_config.get('default_speed_horizontal', 1200.0),
            'default_speed_vertical': coord_config.get('default_speed_vertical', 500.0),
            'max_x_mm': workspace_config.get('max_x_mm', 300.0),
            'max_y_mm': workspace_config.get('max_y_mm', 200.0),
            'max_z_mm': workspace_config.get('max_z_mm', 150.0),
            # Homing position (V1 defaults to 0, V2 homes to max_x)
            'home_x_mm': coord_config.get('home_x_mm', 0.0),
            'home_y_mm': coord_config.get('home_y_mm', 0.0),
            'home_z_mm': coord_config.get('home_z_mm', 0.0),
            # Position invert flags (V2 inverts X and Z)
            'position_invert_x': coord_config.get('position_invert_x', False),
            'position_invert_y': coord_config.get('position_invert_y', False),
            'position_invert_z': coord_config.get('position_invert_z', False),
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
            'approach_offset_z': interface_config.get('approach_offset_z', 50.0),
            'insert_depth_z': interface_config.get('insert_depth_z', 30.0),
            'hook_offset_x_mm': interface_config.get('hook_offset_x_mm', 350.0),
            'hook_offset_y_mm': interface_config.get('hook_offset_y_mm', 180.0),
            'hook_offset_z_mm': interface_config.get('hook_offset_z_mm', -10.0),
            'invert_x': interface_config.get('invert_x', True),
            'invert_y': interface_config.get('invert_y', False),
            'invert_z': interface_config.get('invert_z', False),
            'enable_vision_servo': interface_config.get('enable_vision_servo', True),
            'servo_kp_x': interface_config.get('servo_kp_x', 1.2),
            'servo_deadband_px': interface_config.get('servo_deadband_px', 5.0),
            'servo_timeout_sec': interface_config.get('servo_timeout_sec', 10.0),
            'servo_min_speed': interface_config.get('servo_min_speed', 400),
            'servo_max_speed': interface_config.get('servo_max_speed', 1100),
            'image_width_px': interface_config.get('image_width_px', 640),
        }],
        output='screen',
    )

    return LaunchDescription([
        arm_config_arg,
        test_mode_arg,
        serial_port_arg,
        serial_bridge,
        coordinate_node,
        interface_node,
    ])
