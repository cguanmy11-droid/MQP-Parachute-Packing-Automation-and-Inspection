#!/usr/bin/env python3
"""
Full System Launch File

Launches the complete parachute packing system:
- Main arm (WX200)
- Both side arms (left V1, right V2)
- Frame model
- Vision/perception nodes
- State machine coordinator (dual arm alternating)
- Target selector

Usage:
    # Full system in simulation with test loops
    ros2 launch parachute_coordinator full_system.launch.py sim:=true vision_test:=true

    # Full system on hardware
    ros2 launch parachute_coordinator full_system.launch.py

    # With vision test mode (simulated loops)
    ros2 launch parachute_coordinator full_system.launch.py sim:=true vision_test:=true

    # Start the stowing sequence (both arms alternate):
    ros2 topic pub --once /stow/command std_msgs/String "data: start"

    # Pause/resume:
    ros2 topic pub --once /stow/command std_msgs/String "data: pause"
    ros2 topic pub --once /stow/command std_msgs/String "data: resume"
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def load_config(config_file: str) -> dict:
    """Load configuration from YAML file."""
    with open(config_file, 'r') as f:
        return yaml.safe_load(f)


def create_side_arm_nodes(arm_name: str, config_path: str, sim_mode: bool):
    """Create all nodes for a single side arm."""
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
                'command_topic': 'command',
                'state_topic': 'state',
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
            # Homing position (V1 defaults to 0, V2 homes to max_x)
            'home_x_mm': coord_cfg.get('home_x_mm', 0.0),
            'home_y_mm': coord_cfg.get('home_y_mm', 0.0),
            'home_z_mm': coord_cfg.get('home_z_mm', 0.0),
            # Position invert flags (V2 inverts X and Z)
            'position_invert_x': coord_cfg.get('position_invert_x', False),
            'position_invert_y': coord_cfg.get('position_invert_y', False),
            'position_invert_z': coord_cfg.get('position_invert_z', False),
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
    enable_main_arm = LaunchConfiguration('enable_main_arm').perform(context).lower() == 'true'
    enable_left = LaunchConfiguration('enable_left').perform(context).lower() == 'true'
    enable_right = LaunchConfiguration('enable_right').perform(context).lower() == 'true'
    vision_test = LaunchConfiguration('vision_test').perform(context).lower() == 'true'
    enable_coordinator = LaunchConfiguration('enable_coordinator').perform(context).lower() == 'true'

    config_dir = os.path.join(
        get_package_share_directory('side_arm_control'),
        'config'
    )

    nodes = []

    # ==================== RVIZ ====================
    rviz_config = os.path.join(
        get_package_share_directory('parachute_coordinator'),
        'config',
        'dual_arm_both.rviz'
    )
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    ))

    # ==================== FRAME MODEL ====================
    frame_urdf_path = os.path.join(
        get_package_share_directory('main_arm_control'),
        'urdf',
        'framemodel.urdf'
    )
    with open(frame_urdf_path, 'r') as f:
        frame_urdf = f.read()

    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='frame_state_publisher',
        parameters=[{'robot_description': frame_urdf}],
        remappings=[('robot_description', 'frame_description')],
    ))

    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='frame_tf',
        arguments=[
            '--x', '0', '--y', '-0.15', '--z', '-0.22',
            '--roll', '1.5708', '--pitch', '0', '--yaw', '-1.5708',
            '--frame-id', 'world',
            '--child-frame-id', 'framemodel_root',
        ],
    ))

    # ==================== MAIN ARM ====================
    if enable_main_arm:
        # Include interbotix arm control
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('interbotix_xsarm_control'),
                    'launch',
                    'xsarm_control.launch.py'
                ])
            ]),
            launch_arguments={
                'robot_model': 'wx200',
                'use_sim': str(sim_mode).lower(),
                'use_rviz': 'false',
            }.items(),
        ))

        # Main arm interface
        nodes.append(Node(
            package='main_arm_control',
            executable='main_arm_interface_node',
            name='main_arm_interface_node',
            output='screen',
            parameters=[{
                'robot_model': 'wx200',
                'robot_name': 'wx200',
                'use_sim': sim_mode,
                'test_mode': False,
            }],
        ))

        # Gripper control
        nodes.append(Node(
            package='main_arm_control',
            executable='gripper_control_node',
            name='gripper_control_node',
            output='screen',
            parameters=[{
                'robot_name': 'wx200',
                'grip_pwm': -200,
                'release_pwm': 200,
                'load_threshold': 40.0,
                'settle_time': 0.5,
                'test_mode': False,
            }],
        ))
    else:
        # Fallback world frame if no main arm
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

    # ==================== SIDE ARMS ====================
    if enable_left:
        left_config = os.path.join(config_dir, 'side_arm_left.yaml')
        nodes.extend(create_side_arm_nodes('left', left_config, sim_mode))

    if enable_right:
        right_config = os.path.join(config_dir, 'side_arm_right.yaml')
        nodes.extend(create_side_arm_nodes('right', right_config, sim_mode))

    # ==================== VISION NODES ====================
    if vision_test:
        # Loop ground truth (simulated loops on both sides)
        # Left side (positive Y ~0.15) - 5 loops for left arm
        # Right side (negative Y ~-0.15) - 5 loops for right arm
        static_loop_positions = [
            # Left side loops (positive Y, for left arm) - straight line
            0.25, 0.15, -0.015,  # Loop 0
            0.29, 0.15, -0.015,  # Loop 1
            0.33, 0.15, -0.015,  # Loop 2
            0.37, 0.15, -0.015,  # Loop 3
            0.40, 0.15, -0.015,  # Loop 4
            # Right side loops (negative Y, for right arm) - straight line
            0.25, -0.15, -0.015, # Loop 5
            0.29, -0.15, -0.015, # Loop 6
            0.33, -0.15, -0.015, # Loop 7
            0.37, -0.15, -0.015, # Loop 8
            0.40, -0.15, -0.015, # Loop 9
        ]

        nodes.append(Node(
            package='parachute_perception',
            executable='loop_ground_truth_node',
            name='loop_ground_truth_node',
            output='screen',
            parameters=[{
                'frame_id': 'world',
                'publish_rate': 10.0,
                'pattern': 'static',
                'num_loops': 10,
                'loop_radius': 0.015,
                'static_positions': static_loop_positions,
                'marker_color_r': 0.2,
                'marker_color_g': 0.4,
                'marker_color_b': 0.9,
                'marker_color_a': 0.7,
            }],
        ))

        # Detection simulator
        nodes.append(Node(
            package='parachute_perception',
            executable='detection_simulator_node',
            name='detection_simulator_node',
            output='screen',
            parameters=[{
                'camera_frame_id': 'camera_frame',
                'world_frame_id': 'world',
                'camera_fov_horizontal': 80.0,
                'camera_fov_vertical': 80.0,
                'max_detection_range': 1.0,
                'min_detection_range': 0.01,
                'detection_noise_stddev': 0.00003,
                'confidence_base': 0.90,
                'publish_rate': 5.0,
                'debug_bypass_fov': True,
            }],
        ))

    # Loop visualizer (always on)
    nodes.append(Node(
        package='parachute_perception',
        executable='loop_visualizer_node',
        name='loop_visualizer_node',
        output='screen',
        parameters=[{
            'marker_scale': 0.015,
            'input_frame_id': 'camera_frame',
            'output_frame_id': 'world',
            'grid_enabled': True,
            'grid_size_x': 0.4,
            'grid_size_y': 0.3,
            'grid_offset_z': 0.15,
        }],
    ))

    # ==================== TARGET SELECTOR & COORDINATOR ====================
    if enable_coordinator:
        # Provides /request_next_target service - delayed to allow TF tree setup
        nodes.append(TimerAction(
            period=2.0,
            actions=[Node(
                package='parachute_perception',
                executable='target_selector_node',
                name='target_selector_node',
                output='screen',
                parameters=[{
                    'use_test_loops': vision_test,
                    'selection_strategy': 'leftmost',
                    'stow_proximity_threshold': 0.01,
                }]
            )]
        ))

        # State machine - orchestrates dual arm alternating stow sequence
        # Delayed start to allow arm systems to initialize
        nodes.append(TimerAction(
            period=3.0,
            actions=[Node(
                package='parachute_coordinator',
                executable='packing_coordinator_node',
                name='packing_coordinator_node',
                output='screen',
                parameters=[{
                    'test_mode': False,
                    'stow_pattern': 'square_stow',
                    'action_timeout': 30.0,
                    'expected_loop_count': 0,
                    'enable_left_arm': enable_left,
                    'enable_right_arm': enable_right,
                }]
            )]
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='true',
            description='Run in simulation mode'
        ),
        DeclareLaunchArgument(
            'enable_main_arm',
            default_value='true',
            description='Enable main arm (WX200)'
        ),
        DeclareLaunchArgument(
            'enable_left',
            default_value='true',
            description='Enable left side arm (V1)'
        ),
        DeclareLaunchArgument(
            'enable_right',
            default_value='true',
            description='Enable right side arm (V2)'
        ),
        DeclareLaunchArgument(
            'vision_test',
            default_value='false',
            description='Enable vision test mode (simulated loops)'
        ),
        DeclareLaunchArgument(
            'enable_coordinator',
            default_value='true',
            description='Enable state machine coordinator (set false for manual testing)'
        ),
        OpaqueFunction(function=launch_setup),
    ])
