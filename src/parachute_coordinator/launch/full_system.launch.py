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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, SetEnvironmentVariable
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
        ros_arguments=['--log-level', 'warn'],
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

    # Read camera config from each arm's yaml
    left_config = load_config(os.path.join(config_dir, 'side_arm_left.yaml'))
    right_config = load_config(os.path.join(config_dir, 'side_arm_right.yaml'))
    left_camera_frame = left_config.get('visualizer', {}).get('camera_frame_id', 'left_camera_frame')
    right_camera_frame = right_config.get('visualizer', {}).get('camera_frame_id', 'right_camera_frame')

    # Read vision args
    vision_test = LaunchConfiguration('vision_test').perform(context).lower() == 'true'
    enable_ground_truth = LaunchConfiguration('enable_ground_truth').perform(context).lower() == 'true'
    use_real_camera = LaunchConfiguration('use_real_camera').perform(context).lower() == 'true'
    enable_side_cam = LaunchConfiguration('enable_side_cam').perform(context).lower() == 'true'
    enable_top_cam = LaunchConfiguration('enable_top_cam').perform(context).lower() == 'true'

    # ---- Simulated ground truth loops (both sides) ----
    static_loop_positions = [
        0.25,  0.15, -0.015,  # Left loops 0-4
        0.29,  0.15, -0.015,
        0.33,  0.15, -0.015,
        0.37,  0.15, -0.015,
        0.40,  0.15, -0.015,
        0.25, -0.15, -0.015,  # Right loops 5-9
        0.29, -0.15, -0.015,
        0.33, -0.15, -0.015,
        0.37, -0.15, -0.015,
        0.40, -0.15, -0.015,
    ]

    if enable_ground_truth:
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

    if vision_test and not use_real_camera:
        # One detection simulator per arm (each uses its own camera frame)
        for cam_frame, arm_ns in [
            (left_camera_frame, 'side_arm_left'),
            (right_camera_frame, 'side_arm_right'),
        ]:
            nodes.append(Node(
                package='parachute_perception',
                executable='detection_simulator_node',
                name=f'detection_simulator_{arm_ns}',
                output='screen',
                parameters=[{
                    'camera_frame_id': cam_frame,
                    'world_frame_id': 'world',
                    'camera_fov_horizontal': 80.0,
                    'camera_fov_vertical': 80.0,
                    'max_detection_range': 1.0,
                    'min_detection_range': 0.01,
                    'detection_noise_stddev': 0.00003,
                    'confidence_base': 0.90,
                    'publish_rate': 5.0,
                    'debug_bypass_fov': True,
                    'output_topic': f'/{arm_ns}/detected_loops',
                }],
            ))

    # ---- Loop visualizer (always on) ----
    for arm_ns in ['side_arm_left', 'side_arm_right']:
        nodes.append(Node(
            package='parachute_perception',
            executable='loop_visualizer_node',
            name=f'loop_visualizer_{arm_ns}',
            output='screen',
            parameters=[{
                'marker_scale': 0.015,
                'input_topic': f'/{arm_ns}/detected_loops',
                'target_topic': f'/{arm_ns}/target_loop',
                'output_frame_id': 'world',
                'grid_enabled': True,
                'grid_size_x': 0.4,
                'grid_size_y': 0.3,
                'grid_offset_z': 0.15,
                'markers_topic': f'/{arm_ns}/detected_loop_markers',
                'grid_topic': f'/{arm_ns}/camera_fov_grid',
            }],
        ))

    # ---- Real cameras (one per arm) ----
    if use_real_camera:
        camera_configs = []
        if enable_left:
            camera_configs.append(('left_camera_index', left_camera_frame, 'side_arm_left'))
        if enable_right:
            camera_configs.append(('right_camera_index', right_camera_frame, 'side_arm_right'))

        for cam_idx_arg, cam_frame, arm_ns in camera_configs:
            nodes.append(Node(
                package='yolo_detect_ros',
                executable='yolo_detector',
                name=f'yolo_detector_{arm_ns}',
                output='screen',
                parameters=[{
                    'camera_index': LaunchConfiguration(cam_idx_arg),
                    'conf_threshold': 0.5,
                    'iou_threshold': 0.5,
                    'frame_rate': 30.0,
                    'camera_frame_id': cam_frame,
                    'centers_topic': f'/{arm_ns}/yolo/centers',
                    'image_topic': f'/{arm_ns}/yolo/image',
                    'publish_image': True,
                    'display': LaunchConfiguration('camera_display'),
                }],
            ))
            nodes.append(Node(
                package='parachute_perception',
                executable='camera_to_3d_node',
                name=f'camera_to_3d_{arm_ns}',
                output='screen',
                parameters=[{
                    'image_width': 640,
                    'image_height': 480,
                    'camera_fov_horizontal': 80.0,
                    'assumed_depth': LaunchConfiguration('assumed_depth'),
                    'input_topic': f'/{arm_ns}/yolo/centers',
                    'output_topic': f'/{arm_ns}/detected_loops',
                    'camera_frame_id': cam_frame,
                    'base_confidence': 0.85,
                }],
            ))

    # ---- Top camera ----
    if enable_top_cam:
        _user_site = os.path.expanduser('~/.local/lib/python3.12/site-packages')
        _extra_path = _user_site if os.path.isdir(_user_site) else ''
        _merged = f"{_extra_path}:{os.environ.get('PYTHONPATH', '')}" if _extra_path else os.environ.get('PYTHONPATH', '')
        nodes.append(SetEnvironmentVariable('PYTHONPATH', _merged))
        nodes.append(TimerAction(
            period=5.0,  # wait 5 seconds for side cameras to claim their devices
            actions=[Node(
                package='top_cam_loop_state',
                executable='loop_state_node',
                name='top_cam_loop_state',
                output='screen',
                parameters=[{
                    'camera_index': LaunchConfiguration('top_cam_device'),
                    'det_weights': LaunchConfiguration('top_cam_det_weights'),
                    'cls_weights': LaunchConfiguration('top_cam_cls_weights'),
                    'conf_threshold': 0.35,
                    'iou_threshold': 0.45,
                    'frame_rate': 30.0,
                    'display': LaunchConfiguration('top_cam_display'),
                    'publish_image': True,
                    'image_topic': '/top_cam/image',
                }],
            )],
        ))
    
    # ---- Camera Loop Fusion Node ----
    if enable_top_cam and use_real_camera:
        nodes.append(Node(
            package='parachute_perception',
            executable='loop_fusion_node',
            name='loop_fusion_node',
            output='screen',
            parameters=[{
                'publish_rate': 10.0,
                'max_match_distance': 0.03,  # meters
                'left_topic': '/side_arm_left/detected_loops',
                'right_topic': '/side_arm_right/detected_loops',
                'top_cam_topic': '/top_cam/loop_states',
                'ground_truth_topic': '/loop_ground_truth',
                'output_topic': '/fused_loop_states',
                'num_loops_per_side': 9,
            }]
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
                }],
                remappings=[
                    ('/detected_loops', '/side_arm_right/detected_loops'),
                    ('/target_loop', '/side_arm_right/target_loop'),
                ]
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
            'enable_ground_truth',
            default_value='true',
            description='Enable use of ground truth node for sensor fusion'
        ),
        DeclareLaunchArgument(
            'enable_coordinator',
            default_value='true',
            description='Enable state machine coordinator (set false for manual testing)'
        ),
        # Adding vision launch args
        DeclareLaunchArgument('use_real_camera', default_value='false',
            description='Use real USB cameras with YOLO detection'),
        DeclareLaunchArgument('left_camera_index', default_value='/dev/video8',
            description='Left arm camera device index'),
        DeclareLaunchArgument('right_camera_index', default_value='/dev/video4',
            description='Right arm camera device index'),
        DeclareLaunchArgument('camera_display', default_value='false',
            description='Show YOLO detection windows'),
        DeclareLaunchArgument('assumed_depth', default_value='0.22',
            description='Assumed depth camera to loop plane (meters)'),
        DeclareLaunchArgument('enable_side_cam', default_value='false',
            description='Enable side camera pipeline'),
        DeclareLaunchArgument('enable_top_cam', default_value='false',
            description='Enable top camera loop state detection'),
        DeclareLaunchArgument('top_cam_device', default_value='/dev/video6',
            description='Top camera device path'),
        DeclareLaunchArgument('top_cam_det_weights',
            default_value=os.path.join(os.path.expanduser('~'),
                'MQP-Parachute-Packing-Automation-and-Inspection/src/top_cam_yolo/runs/detect/runs/detect/yolo26m_holes_all_aug/weights/best.pt'),
            description='Top camera detection weights'),
        DeclareLaunchArgument('top_cam_cls_weights',
            default_value=os.path.join(os.path.expanduser('~'),
                'MQP-Parachute-Packing-Automation-and-Inspection/src/top_cam_yolo/runs/classify/runs/classify/yolo26m_cls_custom_aug/weights/best.pt'),
            description='Top camera classification weights'),
        DeclareLaunchArgument('top_cam_display', default_value='false',
            description='Show top camera window'),
        OpaqueFunction(function=launch_setup),
    ])
