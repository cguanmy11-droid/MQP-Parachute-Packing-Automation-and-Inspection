#!/usr/bin/env python3
"""
Dual Side Arm Launch File

Launches both left (V1) and right (V2) side arms with their respective
configurations, namespaces, and TF prefixes.

Usage:
    # Both arms in simulation
    ros2 launch parachute_coordinator dual_side_arm.launch.py sim:=true

    # Both arms on hardware
    ros2 launch parachute_coordinator dual_side_arm.launch.py

    # Left arm only
    ros2 launch parachute_coordinator dual_side_arm.launch.py enable_right:=false

    # Right arm only
    ros2 launch parachute_coordinator dual_side_arm.launch.py enable_left:=false
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
import xacro


def load_config(config_file: str) -> dict:
    """Load configuration from YAML file."""
    with open(config_file, 'r') as f:
        return yaml.safe_load(f)


def create_arm_nodes(context, arm_name: str, config_path: str, sim_mode: bool):
    """Create all nodes for a single arm."""
    config = load_config(config_path)

    # Extract config sections
    identity = config.get('side_arm', {})
    serial_cfg = config.get('serial_bridge', {})
    coord_cfg = config.get('coordinate_node', {})
    workspace_cfg = config.get('workspace', {})
    interface_cfg = config.get('interface_node', {})
    viz_cfg = config.get('visualizer', {})
    joint_cfg = config.get('joint_state_publisher', {})
    tf_cfg = config.get('tf_transform', {})

    frame_prefix = identity.get('frame_prefix', '')
    namespace = identity.get('namespace', f'side_arm_{arm_name}')
    urdf_file = identity.get('urdf_file', 'side_arm.urdf.xacro')

    # Load and process URDF
    urdf_path = os.path.join(
        get_package_share_directory('side_arm_control'),
        'urdf',
        urdf_file
    )
    robot_description = xacro.process_file(
        urdf_path,
        mappings={'prefix': frame_prefix}
    ).toxml()

    # Frame name with prefix
    side_arm_frame = f"{frame_prefix}side_arm_origin"

    nodes = []

    # Serial bridge (hardware mode only)
    if not sim_mode:
        nodes.append(Node(
            package='side_arm_motor_control_bridge',
            executable='serial_bridge',
            name='serial_bridge',
            namespace=namespace,
            parameters=[{
                'serial_port': serial_cfg.get('serial_port', '/dev/ttyUSB0'),
                'baud_rate': serial_cfg.get('baud_rate', 115200),
                'command_topic': 'command',  # Relative topic
                'state_topic': 'state',      # Relative topic
                'state_request_hz': serial_cfg.get('state_request_hz', 10.0),
                'auto_request_state': serial_cfg.get('auto_request_state', True),
            }],
            output='screen',
        ))

    # Coordinate node
    nodes.append(Node(
        package='side_arm_control',
        executable='coordinate_node',
        name='coordinate_node',
        namespace=namespace,
        parameters=[{
            'simulation_mode': sim_mode,
            'sim_speed_mm_per_sec': coord_cfg.get('sim_speed_mm_per_sec', 50.0),
            'steps_per_mm_horizontal': coord_cfg.get('steps_per_mm_horizontal', 300.0),
            'steps_per_mm_vertical': coord_cfg.get('steps_per_mm_vertical', 100.0),
            'dc_mm_per_second': coord_cfg.get('dc_mm_per_second', 4.0),
            'dc_speed_percent': coord_cfg.get('dc_speed_percent', 50),
            'default_speed_horizontal': coord_cfg.get('default_speed_horizontal', 1200.0),
            'default_speed_vertical': coord_cfg.get('default_speed_vertical', 500.0),
            'max_x_mm': workspace_cfg.get('max_x_mm', 300.0),
            'max_y_mm': workspace_cfg.get('max_y_mm', 200.0),
            'max_z_mm': workspace_cfg.get('max_z_mm', 150.0),
        }],
        output='screen',
    ))

    # Interface node
    nodes.append(Node(
        package='side_arm_control',
        executable='side_arm_interface_node',
        name='interface_node',
        namespace=namespace,
        parameters=[{
            'test_mode': sim_mode,
            'side_arm_frame': side_arm_frame,
            'approach_offset_z': interface_cfg.get('approach_offset_z', 50.0),
            'insert_depth_z': interface_cfg.get('insert_depth_z', 30.0),
            'hook_offset_x_mm': interface_cfg.get('hook_offset_x_mm', 10.0),
            'hook_offset_y_mm': interface_cfg.get('hook_offset_y_mm', 210.0),
            'hook_offset_z_mm': interface_cfg.get('hook_offset_z_mm', -156.0),
            'invert_x': interface_cfg.get('invert_x', False),
            'invert_y': interface_cfg.get('invert_y', False),
            'invert_z': interface_cfg.get('invert_z', False),
            'enable_vision_servo': interface_cfg.get('enable_vision_servo', True),
            'servo_kp_x': interface_cfg.get('servo_kp_x', 1.2),
            'servo_deadband_px': interface_cfg.get('servo_deadband_px', 5.0),
            'servo_timeout_sec': interface_cfg.get('servo_timeout_sec', 10.0),
            'servo_min_speed': interface_cfg.get('servo_min_speed', 400),
            'servo_max_speed': interface_cfg.get('servo_max_speed', 1100),
            'image_width_px': interface_cfg.get('image_width_px', 640),
        }],
        output='screen',
    ))

    # Visualizer
    nodes.append(Node(
        package='side_arm_control',
        executable='side_arm_visualizer',
        name='visualizer',
        namespace=namespace,
        parameters=[{
            'roll': viz_cfg.get('roll', -1.5708),
            'pitch': viz_cfg.get('pitch', 0.0),
            'yaw': viz_cfg.get('yaw', -1.5708),
            'offset_x': viz_cfg.get('offset_x', -0.01),
            'offset_y': viz_cfg.get('offset_y', 0.009),
            'offset_z': viz_cfg.get('offset_z', 0.07),
            'scale': viz_cfg.get('scale', 0.001),
            'servo_axis': viz_cfg.get('servo_axis', 'pitch'),
            'servo_scale': viz_cfg.get('servo_scale', 0.001),
            'test_mode': sim_mode,
            'publish_hook_tf': viz_cfg.get('publish_hook_tf', False),
            'hook_frame_id': viz_cfg.get('hook_frame_id', f'{frame_prefix}side_arm_hook'),
            'publish_camera_tf': viz_cfg.get('publish_camera_tf', False),
            'camera_frame_id': viz_cfg.get('camera_frame_id', f'{frame_prefix}camera_frame'),
        }],
        output='screen',
    ))

    # Robot state publisher (URDF)
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[{'robot_description': robot_description}],
        remappings=[
            (f'/{namespace}/tf', '/tf'),
            (f'/{namespace}/tf_static', '/tf_static'),
        ],
    ))

    # Joint state publisher
    nodes.append(Node(
        package='side_arm_control',
        executable='side_arm_joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[{
            'joint_prefix': frame_prefix,
            'servo_scale': joint_cfg.get('servo_scale', 0.001),
            'publish_rate': joint_cfg.get('publish_rate', 50.0),
        }],
        output='screen',
    ))

    # Static TF for arm origin
    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{arm_name}_arm_tf',
        arguments=[
            '--x', str(tf_cfg.get('x', 0.0)),
            '--y', str(tf_cfg.get('y', 0.0)),
            '--z', str(tf_cfg.get('z', 0.0)),
            '--roll', str(tf_cfg.get('roll', 0.0)),
            '--pitch', str(tf_cfg.get('pitch', 0.0)),
            '--yaw', str(tf_cfg.get('yaw', 0.0)),
            '--frame-id', tf_cfg.get('parent_frame', 'world'),
            '--child-frame-id', side_arm_frame,
        ],
    ))

    return nodes


def launch_setup(context, *args, **kwargs):
    """Setup function called at launch time."""
    sim_mode = LaunchConfiguration('sim').perform(context).lower() == 'true'
    enable_left = LaunchConfiguration('enable_left').perform(context).lower() == 'true'
    enable_right = LaunchConfiguration('enable_right').perform(context).lower() == 'true'

    config_dir = os.path.join(
        get_package_share_directory('side_arm_control'),
        'config'
    )

    nodes = []

    # RViz
    rviz_config = os.path.join(
        get_package_share_directory('parachute_coordinator'),
        'config',
        'dual_arm_both.rviz'
    )
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'warn'],
        output='screen',
    ))

    # World frame (fallback if no main arm)
    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'wx200/base_link',
        ],
    ))

    # Left arm nodes
    if enable_left:
        left_config = os.path.join(config_dir, 'side_arm_left.yaml')
        nodes.extend(create_arm_nodes(context, 'left', left_config, sim_mode))

    # Right arm nodes
    if enable_right:
        right_config = os.path.join(config_dir, 'side_arm_right.yaml')
        nodes.extend(create_arm_nodes(context, 'right', right_config, sim_mode))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='true',
            description='Run in simulation mode (no hardware)'
        ),
        DeclareLaunchArgument(
            'enable_left',
            default_value='true',
            description='Enable left arm (V1)'
        ),
        DeclareLaunchArgument(
            'enable_right',
            default_value='true',
            description='Enable right arm (V2)'
        ),
        OpaqueFunction(function=launch_setup),
    ])
