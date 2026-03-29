#!/usr/bin/env python3
"""
Dual Arm Test Launch File

Launches both side arm and main arm systems for integrated testing
with digital twin visualization in RViz.

Usage:
    # Basic hardware mode (both arms)
    ros2 launch parachute_coordinator dual_arm_test.launch.py

    # With Xbox controller for main arm
    ros2 launch parachute_coordinator dual_arm_test.launch.py enable_teleop:=true

    # Main arm in simulation, side arm on hardware
    ros2 launch parachute_coordinator dual_arm_test.launch.py main_arm_sim:=true

    # Side arm only (no main arm)
    ros2 launch parachute_coordinator dual_arm_test.launch.py enable_main_arm:=false

    # Main arm only (no side arm)
    ros2 launch parachute_coordinator dual_arm_test.launch.py enable_side_arm:=false

    # Side arm pure simulation mode (no serial bridge, URDF moves):
    ros2 launch parachute_coordinator dual_arm_test.launch.py \\
        side_arm_sim:=true enable_main_arm:=false

    # Side arm hardware + simulation (URDF mirrors AND simulates motion)
    # Useful for seeing what the system sees during real tests
    ros2 launch parachute_coordinator dual_arm_test.launch.py \\
        side_arm_test_mode:=true

Side Arm Motion Modes:
    - side_arm_sim=false, side_arm_test_mode=false: Hardware only. Serial bridge required.
    - side_arm_sim=true: Pure simulation. No serial bridge, URDF moves via simulated commands.
    - side_arm_sim=false, side_arm_test_mode=true: Hybrid. Hardware + simulation run together.

Test with simulated vision (loops in RViz):
    ros2 launch parachute_coordinator dual_arm_test.launch.py \\
        side_arm_test_mode:=true vision_test_mode:=true enable_main_arm:=false

Test with real camera (YOLO detection):
    ros2 launch parachute_coordinator dual_arm_test.launch.py \\
        use_real_camera:=true camera_index:=4

Move side arm in simulation:
    ros2 service call /side_arm/move_to_position \\
        parachute_interfaces/srv/MoveToPosition \\
        \"{x_mm: 100.0, y_mm: 50.0, z_mm: 20.0, speed_scale: 0.5}\"
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_side_arm_config(config_file: str) -> dict:
    """Load side arm configuration from YAML file."""
    with open(config_file, 'r') as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    """Setup function called at launch time when arguments are resolved."""
    # Load custom RViz config
    rviz_config_path = os.path.join(
        get_package_share_directory('parachute_coordinator'),
        'config',
        'dual_arm.rviz'
    )

    # Load frame URDF (optional — only needed when main arm is enabled)
    frame_urdf = ''
    try:
        frame_urdf_path = os.path.join(
            get_package_share_directory('main_arm_control'),
            'urdf',
            'framemodel.urdf'
        )
        with open(frame_urdf_path, 'r') as f:
            frame_urdf = f.read()
    except Exception:
        pass  # main_arm_control not built — main arm nodes will be skipped

    # =============================================================================
    # SIDE ARM CONFIGURATION
    # =============================================================================
    # Get the arm_config argument value (resolved at launch time)
    arm_config_file = LaunchConfiguration('arm_config').perform(context)

    # If it's just a filename (not a full path), look in the config directory
    if not os.path.isabs(arm_config_file) and not os.path.exists(arm_config_file):
        side_arm_config_dir = os.path.join(
            get_package_share_directory('side_arm_control'),
            'config'
        )
        arm_config_file = os.path.join(side_arm_config_dir, arm_config_file)

    print(f"[dual_arm_test] Loading side arm config: {arm_config_file}")
    side_arm_config = load_side_arm_config(arm_config_file)

    # Extract URDF/xacro filename and frame prefix from config
    side_arm_identity = side_arm_config.get('side_arm', {})
    urdf_filename = side_arm_identity.get('urdf_file', 'side_arm.urdf.xacro')
    frame_prefix = side_arm_identity.get('frame_prefix', '')
    arm_namespace = side_arm_identity.get('namespace', 'side_arm')

    side_arm_urdf_path = os.path.join(
        get_package_share_directory('side_arm_control'),
        'urdf',
        urdf_filename
    )

    # Process xacro with prefix parameter
    if urdf_filename.endswith('.xacro'):
        import xacro
        side_arm_urdf = xacro.process_file(
            side_arm_urdf_path,
            mappings={'prefix': frame_prefix}
        ).toxml()
    else:
        with open(side_arm_urdf_path, 'r') as f:
            side_arm_urdf = f.read()

    # Extract config sections for easier access
    serial_config = side_arm_config.get('serial_bridge', {})
    coord_config = side_arm_config.get('coordinate_node', {})
    workspace_config = side_arm_config.get('workspace', {})
    interface_config = side_arm_config.get('interface_node', {})
    viz_config = side_arm_config.get('visualizer', {})
    joint_pub_config = side_arm_config.get('joint_state_publisher', {})
    tf_config = side_arm_config.get('tf_transform', {})

    # Camera frame name from config (supports prefixed frames like left_camera_frame)
    camera_frame_id = viz_config.get('camera_frame_id', 'camera_frame')
    print(f"[dual_arm_test] Using camera frame: {camera_frame_id}")

    # Side camera perception arguments
    enable_side_cam_arg = DeclareLaunchArgument(
        'enable_side_cam',
        default_value='false',
        description='Enable side camera YOLO detection pipeline'
    )

    side_cam_device_arg = DeclareLaunchArgument(
        'side_cam_device',
        default_value='0',
        description='Side camera device index'
    )

    side_cam_conf_arg = DeclareLaunchArgument(
        'side_cam_conf',
        default_value='0.5',
        description='Side camera YOLO confidence threshold'
    )

    side_cam_depth_arg = DeclareLaunchArgument(
        'side_cam_depth',
        default_value='0.22',
        description='Assumed depth from side camera to loop plane (meters)'
    )

    # Top camera loop state arguments
    enable_top_cam_arg = DeclareLaunchArgument(
        'enable_top_cam',
        default_value='false',
        description='Enable top camera YOLO loop state detection'
    )

    top_cam_device_arg = DeclareLaunchArgument(
        'top_cam_device',
        default_value='/dev/video4',
        description='Top camera device path'
    )

    top_cam_det_weights_arg = DeclareLaunchArgument(
        'top_cam_det_weights',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'MQP_ws/MQP-Parachute-Packing-Automation-and-Inspection/src/top_cam_yolo/runs/detect/runs/detect/yolo26m_holes_all_aug/weights/best.pt'
        ),
        description='Top camera YOLO detection weights'
    )

    top_cam_cls_weights_arg = DeclareLaunchArgument(
        'top_cam_cls_weights',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'MQP_ws/MQP-Parachute-Packing-Automation-and-Inspection/src/top_cam_yolo/runs/classify/runs/classify/yolo26m_cls_custom_aug/weights/best.pt'
        ),
        description='Top camera YOLO classification weights'
    )

    # ==================== MAIN ARM NODES ====================

    # Interbotix arm control (hardware or sim)
    # NOTE: use_rviz is set to false here - we launch our own RViz with custom config
    main_arm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': LaunchConfiguration('robot_model'),
            'use_sim': LaunchConfiguration('main_arm_sim'),
            'use_rviz': 'false',  # We launch our own RViz with custom config
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_main_arm'))
    )

    # Custom RViz with dual arm config (includes trajectory waypoints visualization)
    # Always launch RViz (previously had condition that wasn't working with IncludeLaunchDescription)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
    )

    # Main arm interface node
    # NOTE: test_mode=true means robot is NOT initialized (just logs commands)
    #       use_sim=true means robot IS initialized but controls simulation
    main_arm_interface = Node(
        package='main_arm_control',
        executable='main_arm_interface_node',
        name='main_arm_interface_node',
        output='screen',
        parameters=[{
            'robot_model': LaunchConfiguration('robot_model'),
            'robot_name': LaunchConfiguration('robot_model'),
            'use_sim': LaunchConfiguration('main_arm_sim'),
            'test_mode': False,  # Always false so robot is initialized (sim or hardware)
        }],
        condition=IfCondition(LaunchConfiguration('enable_main_arm'))
    )

    # Main arm planner node (motion planning with IK solving)
    main_arm_planner = Node(
        package='main_arm_control',
        executable='main_arm_planner_node',
        name='main_arm_planner_node',
        output='screen',
        parameters=[{
            'robot_model': LaunchConfiguration('robot_model'),
            'robot_name': LaunchConfiguration('robot_model'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_main_arm'))
    )

    gripper_control = Node(
        package='main_arm_control',
        executable='gripper_control_node',
        name='gripper_control_node',
        output='screen',
        parameters=[{
            'robot_name': LaunchConfiguration('robot_model'),
            'grip_pwm': -200,
            'release_pwm': 200,
            'load_threshold': 40.0,
            'settle_time': 0.5,
            'test_mode': False,
        }],
        condition=IfCondition(LaunchConfiguration('enable_main_arm'))
    )

    motor_health_monitor = Node(
        package='main_arm_control',
        executable='motor_health_monitor',
        name='motor_health_monitor',
        output='screen',
        parameters=[{
            'robot_name': LaunchConfiguration('robot_model'),
            'monitor_rate': 0.2,
            'load_warn_threshold': 75.0,
            'load_critical_threshold': 85.0,
            'auto_clear': False, # set True to auto recover from stalls
        }],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('enable_main_arm'), "' == 'true' and '",
                LaunchConfiguration('main_arm_sim'), "' != 'true' and '", 
                LaunchConfiguration('enable_monitor'), "' == 'true'",
            ])
        )
    )

    # Main arm teleop node (optional)
    # main_arm_teleop = Node(
    #     package='main_arm_control',
    #     executable='main_arm_teleop_node',
    #     name='main_arm_teleop_node',
    #     output='screen',
    #     parameters=[{
    #         'robot_model': LaunchConfiguration('robot_model'),
    #         'controller_type': LaunchConfiguration('controller_type'),
    #         'auto_start': True,
    #     }],
    #     condition=IfCondition(
    #         PythonExpression([
    #             "'", LaunchConfiguration('enable_main_arm'), "' == 'true' and '",
    #             LaunchConfiguration('enable_teleop'), "' == 'true'"
    #         ])
    #     )
    # )

    main_arm_xbox = Node(
        package='main_arm_control',
        executable='xbox_arm_controller_node',
        name='xbox_arm_controller_node',
        output='screen',
        condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('enable_main_arm'), "' == 'true' and '",
                LaunchConfiguration('enable_teleop'), "' == 'true'"
            ])
        )
    )

    # Joy node for Xbox controller
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('enable_main_arm'), "' == 'true' and '",
                LaunchConfiguration('enable_teleop'), "' == 'true'"
            ])
        )
    )

    # ==================== SIDE ARM NODES ====================

    # Serial bridge node (communicates with ESP32)
    # Only launches when NOT in simulation mode (side_arm_sim=false)
    # In hybrid mode (side_arm_test_mode=true), both hardware and simulation run
    side_arm_serial_bridge = Node(
        package='side_arm_motor_control_bridge',
        executable='serial_bridge',
        name='side_arm_serial_bridge',
        namespace=arm_namespace,  # Use namespace from config
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': serial_config.get('baud_rate', 115200),
            'command_topic': 'command',  # Relative topic for namespace support
            'state_topic': 'state',      # Relative topic for namespace support
            'state_request_hz': serial_config.get('state_request_hz', 10.0),
            'auto_request_state': serial_config.get('auto_request_state', True),
        }],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('enable_side_arm'), "' == 'true' and '",
                LaunchConfiguration('side_arm_sim'), "' == 'false'"
            ])
        )
    )

    # Determine if simulation mode should be enabled (either side_arm_sim or side_arm_test_mode)
    side_arm_simulation_enabled = PythonExpression([
        "'", LaunchConfiguration('side_arm_sim'), "' == 'true' or '",
        LaunchConfiguration('side_arm_test_mode'), "' == 'true'"
    ])

    # Coordinate node (converts mm to motor commands)
    # In simulation/test mode, runs simulation. Hardware commands are also sent if serial bridge is running.
    side_arm_coordinate = Node(
        package='side_arm_control',
        executable='coordinate_node',
        name='side_arm_coordinate_node',
        namespace=arm_namespace,  # Use namespace from config
        parameters=[{
            'simulation_mode': side_arm_simulation_enabled,  # Enable simulation when sim or test mode
            'sim_speed_mm_per_sec': coord_config.get('sim_speed_mm_per_sec', 50.0),
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
        condition=IfCondition(LaunchConfiguration('enable_side_arm'))
    )

    # Side arm interface node (high-level actions/services)
    # Construct the frame name with prefix
    side_arm_frame_name = f"{frame_prefix}side_arm_origin"

    side_arm_interface = Node(
        package='side_arm_control',
        executable='side_arm_interface_node',
        name='side_arm_interface_node',
        namespace=arm_namespace,  # Use namespace from config
        parameters=[{
            'test_mode': side_arm_simulation_enabled,
            'side_arm_frame': side_arm_frame_name,  # TF frame name with prefix
            'approach_offset_z': interface_config.get('approach_offset_z', 50.0),
            'insert_depth_z': interface_config.get('insert_depth_z', 30.0),
            'hook_offset_x_mm': interface_config.get('hook_offset_x_mm', 10.0),
            'hook_offset_y_mm': interface_config.get('hook_offset_y_mm', 210.0),
            'hook_offset_z_mm': interface_config.get('hook_offset_z_mm', -156.0),
            'invert_x': interface_config.get('invert_x', False),
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
        condition=IfCondition(LaunchConfiguration('enable_side_arm'))
    )

    #  Side arm visualizer (RViz marker + TF for hook and camera)
    side_arm_visualizer = Node(
        package='side_arm_control',
        executable='side_arm_visualizer',
        name='side_arm_visualizer',
        namespace=arm_namespace,  # Use namespace from config
        output='screen',
        parameters=[{
            # Hook mesh orientation (from config)
            'roll': viz_config.get('roll', -1.5708),
            'pitch': viz_config.get('pitch', 0.0),
            'yaw': viz_config.get('yaw', -1.5708),
            # Hook mesh offset
            'offset_x': viz_config.get('offset_x', -0.01),
            'offset_y': viz_config.get('offset_y', 0.009),
            'offset_z': viz_config.get('offset_z', 0.07),
            'scale': viz_config.get('scale', 0.001),
            # Servo rotation
            'servo_axis': viz_config.get('servo_axis', 'pitch'),
            'servo_scale': viz_config.get('servo_scale', 0.001),
            # Test mode (enabled in sim or test mode)
            'test_mode': side_arm_simulation_enabled,
            'test_x': 0.0,
            'test_y': 0.0,
            'test_z': 0.0,
            # TF publishing
            'publish_hook_tf': viz_config.get('publish_hook_tf', False),
            'hook_frame_id': viz_config.get('hook_frame_id', 'side_arm_hook'),
            # Camera frame is now published by URDF robot_state_publisher (attached to y_carriage)
            # Set to False to avoid TF conflict with URDF's camera_frame
            'publish_camera_tf': viz_config.get('publish_camera_tf', False),
            'camera_frame_id': viz_config.get('camera_frame_id', 'camera_frame'),
            'camera_offset_x': viz_config.get('camera_offset_x', 0.0),
            'camera_offset_y': viz_config.get('camera_offset_y', 0.0),
            'camera_offset_z': viz_config.get('camera_offset_z', 0.05),
            # Rotate camera to look toward the ground truth loops
            'camera_roll': viz_config.get('camera_roll', 0.0),
            'camera_pitch': viz_config.get('camera_pitch', 3.1416),
            'camera_yaw': viz_config.get('camera_yaw', 0.0),
        }],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('enable_side_arm'), "' == 'true' and '",
                LaunchConfiguration('enable_visualization'), "' == 'true'"
            ])
        )
    )

    # ==================== SIDE ARM URDF ====================

    # Side arm robot state publisher (publishes URDF to /{namespace}/robot_description)
    side_arm_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='side_arm_robot_state_publisher',
        namespace=arm_namespace,
        parameters=[{'robot_description': side_arm_urdf}],
        remappings=[
            (f'/{arm_namespace}/tf', '/tf'),
            (f'/{arm_namespace}/tf_static', '/tf_static'),
        ],
        condition=IfCondition(LaunchConfiguration('enable_side_arm'))
    )

    # Joint state publisher GUI (for manual slider control in test mode)
    side_arm_joint_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='side_arm_joint_gui',
        namespace=arm_namespace,
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('enable_side_arm'), "' == 'true' and '",
                LaunchConfiguration('use_joint_sliders'), "' == 'true'"
            ])
        )
    )

    # Joint state bridge (converts hardware state to joint states)
    side_arm_joint_bridge = Node(
        package='side_arm_control',
        executable='side_arm_joint_state_publisher',
        name='side_arm_joint_state_publisher',
        namespace=arm_namespace,
        parameters=[{
            'joint_prefix': frame_prefix,  # Prefix for joint names
            'servo_scale': joint_pub_config.get('servo_scale', 0.001),
            'publish_rate': joint_pub_config.get('publish_rate', 50.0),
            'test_mode': side_arm_simulation_enabled,
            'test_x': joint_pub_config.get('test_x', 0.15),
            'test_y': joint_pub_config.get('test_y', 0.10),
            'test_z': joint_pub_config.get('test_z', 0.05),
            'test_servo': joint_pub_config.get('test_servo', 0.0),
        }],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('enable_side_arm'), "' == 'true' and '",
                LaunchConfiguration('use_joint_sliders'), "' == 'false'"
            ])
        )
    )

    # ==================== DIGITAL TWIN LAUNCH ====================

    # Frame state publisher
    frame_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='frame_state_publisher',
        parameters=[{'robot_description': frame_urdf}],
        remappings=[('robot_description', 'frame_description')],
        condition=IfCondition(LaunchConfiguration('enable_main_arm'))
    )

    # Frame TF (position relative to robot base)
    frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='frame_static_tf',
        arguments=[
            '--x', '0', '--y', '-0.15', '--z', '-0.22',
            '--roll', '1.5708', '--pitch', '0', '--yaw', '-1.5708',
            '--frame-id', 'world',
            '--child-frame-id', 'framemodel_root'
        ],
        condition=IfCondition(LaunchConfiguration('enable_main_arm'))
    )

    # Fallback: world -> wx200/base_link TF when main arm is disabled
    # This allows the side arm TF chain to work without the main arm
    world_to_base_fallback_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_fallback_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'wx200/base_link'
        ],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('enable_main_arm'), "' == 'false'"
            ])
        )
    )

    # Side arm origin TF (position from config)
    # Uses frame_prefix for the child frame name
    side_arm_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='side_arm_static_tf',
        arguments=[
            '--x', str(tf_config.get('x', 0.37)),
            '--y', str(tf_config.get('y', 0.41)),
            '--z', str(tf_config.get('z', -0.26)),
            '--roll', str(tf_config.get('roll', 1.5708)),
            '--pitch', str(tf_config.get('pitch', 0)),
            '--yaw', str(tf_config.get('yaw', 3.1416)),
            '--frame-id', tf_config.get('parent_frame', 'world'),
            '--child-frame-id', side_arm_frame_name  # Uses prefix from config
        ],
        condition=IfCondition(LaunchConfiguration('enable_side_arm'))
    )

    # NOTE: camera_frame TF is now published by URDF robot_state_publisher
    # (attached to y_carriage_link, so it moves with X/Y but not Z)

    # ==================== PERCEPTION VISUALIZATION NODES ====================

    # Loop ground truth (publishes actual loop positions in world frame)
    # Dual-sided: 5 loops on left (positive Y), 5 loops on right (negative Y)
    static_loop_positions = [
        # Left side loops (positive Y ~0.15, for left arm) - straight line
        0.25, 0.15, -0.015,  # Loop 0
        0.29, 0.15, -0.015,  # Loop 1
        0.33, 0.15, -0.015,  # Loop 2
        0.37, 0.15, -0.015,  # Loop 3
        0.40, 0.15, -0.015,  # Loop 4
        # Right side loops (negative Y ~-0.15, for right arm) - straight line
        0.25, -0.15, -0.015, # Loop 5
        0.29, -0.15, -0.015, # Loop 6
        0.33, -0.15, -0.015, # Loop 7
        0.37, -0.15, -0.015, # Loop 8
        0.40, -0.15, -0.015, # Loop 9
    ]

    loop_ground_truth = Node(
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
            # Marker visualization (blue for ground truth)
            'marker_color_r': 0.2,
            'marker_color_g': 0.4,
            'marker_color_b': 0.9,
            'marker_color_a': 0.7,
        }],
        condition=IfCondition(LaunchConfiguration('vision_test_mode'))
    )

    # Loop visualizer (subscribes to /detected_loops, publishes /detected_loop_markers)
    # Transforms detections from camera_frame to world for display
    loop_visualizer = Node(
        package='parachute_perception',
        executable='loop_visualizer_node',
        name='loop_visualizer_node',
        output='screen',
        parameters=[{
            'marker_scale': 0.015,
            'input_frame_id': camera_frame_id,  # Frame detections arrive in (from config)
            'output_frame_id': 'world',         # Frame to publish markers in
            'grid_enabled': True,
            'grid_size_x': 0.4,
            'grid_size_y': 0.3,
            'grid_offset_z': 0.15,  # Grid distance in front of camera
        }],
        condition=IfCondition(LaunchConfiguration('enable_loop_visualization'))
    )

    # Detection simulator (simulates YOLO detection based on ground truth and camera pose)
    # Transforms ground truth loops from world to camera_frame, checks FOV visibility
    detection_simulator = Node(
        package='parachute_perception',
        executable='detection_simulator_node',
        name='detection_simulator_node',
        output='screen',
        parameters=[{
            'camera_frame_id': camera_frame_id,  # From config
            'world_frame_id': 'world',
            # Camera FOV - set wide for testing, narrow later to match real camera
            'camera_fov_horizontal': 80.0,  # degrees
            'camera_fov_vertical': 80.0,    # degrees
            'max_detection_range': 1.0,      # meters (extended for testing)
            'min_detection_range': 0.01,     # meters
            # Detection simulation
            'detection_noise_stddev': 0.00003,  # meters
            'confidence_base': 0.90,
            'confidence_noise': 0.0001,
            'false_negative_rate': 0.0,
            'publish_rate': 5.0,
            # Debug - enable to bypass FOV checks and see what's happening
            'debug_bypass_fov': True,   # TEMP: Enable to detect loops even if camera wrong way
            'debug_verbose': False,      # Show detailed logging
        }],
        condition=IfCondition(LaunchConfiguration('vision_test_mode'))
    )

    # ==================== SIDE CAMERA PERCEPTION ====================

    side_cam_yolo = Node(
        package='yolo_detect_ros',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'camera_index': LaunchConfiguration('side_cam_device'),
            'conf_threshold': LaunchConfiguration('side_cam_conf'),
            'iou_threshold': 0.5,
            'frame_rate': 30.0,
            'camera_frame_id': 'camera_frame',
            'centers_topic': '/yolo/centers',
            'display': True,
        }],
        condition=IfCondition(LaunchConfiguration('enable_side_cam'))
    )

    side_cam_to_3d = Node(
        package='parachute_perception',
        executable='camera_to_3d_node',
        name='camera_to_3d_node',
        output='screen',
        parameters=[{
            'image_width': 640,
            'image_height': 480,
            'camera_fov_horizontal': 80.0,
            'assumed_depth': LaunchConfiguration('side_cam_depth'),
            'input_topic': '/yolo/centers',
            'output_topic': '/detected_loops',
            'camera_frame_id': 'camera_frame',
            'base_confidence': 0.85,
        }],
        condition=IfCondition(LaunchConfiguration('enable_side_cam'))
    )

    # ==================== TOP CAMERA LOOP STATE ====================

    # Ensure user site-packages are visible for ultralytics
    _user_site = os.path.expanduser("~/.local/lib/python3.12/site-packages")
    _extra_path = _user_site if os.path.isdir(_user_site) else ""
    _existing_pythonpath = os.environ.get("PYTHONPATH", "")
    _merged_pythonpath = f"{_extra_path}:{_existing_pythonpath}" if _extra_path else _existing_pythonpath

    top_cam_env = SetEnvironmentVariable("PYTHONPATH", _merged_pythonpath)

    top_cam_loop_state = Node(
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
            'display': True,
        }],
        condition=IfCondition(LaunchConfiguration('enable_top_cam'))
    )

    # ==================== REAL CAMERA NODES ====================
    # YOLO detector (real USB camera)
    yolo_detector = Node(
        package='yolo_detect_ros',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'camera_index': LaunchConfiguration('camera_index'),
            'conf_threshold': 0.5,
            'iou_threshold': 0.5,
            'frame_rate': 30.0,
            'camera_frame_id': camera_frame_id,  # From config
            'centers_topic': '/yolo/centers',
            'display': LaunchConfiguration('camera_display'),
        }],
        condition=IfCondition(LaunchConfiguration('use_real_camera'))
    )

    # Pixel to 3D converter (converts YOLO pixel detections to 3D world coordinates)
    camera_to_3d = Node(
        package='parachute_perception',
        executable='camera_to_3d_node',
        name='camera_to_3d_node',
        output='screen',
        parameters=[{
            # Camera intrinsics
            'image_width': 640,
            'image_height': 480,
            'camera_fov_horizontal': 80.0,
            # Depth assumption
            'assumed_depth': LaunchConfiguration('assumed_depth'),
            # Topics
            'input_topic': '/yolo/centers',
            'output_topic': '/detected_loops',
            'camera_frame_id': camera_frame_id,  # From config
            # Detection confidence
            'base_confidence': 0.85,
        }],
        condition=IfCondition(LaunchConfiguration('use_real_camera'))
    )

    # ==================== RETURN NODES ====================
    # Return list of nodes (arguments are declared in generate_launch_description)

    return [
        enable_side_cam_arg,
        side_cam_device_arg,
        side_cam_conf_arg,
        side_cam_depth_arg,
        enable_top_cam_arg,
        top_cam_device_arg,
        top_cam_det_weights_arg,
        top_cam_cls_weights_arg,
        top_cam_env,
        joy_node,

        # Main arm nodes
        main_arm_control_launch,
        main_arm_interface,
        gripper_control,
        motor_health_monitor,
        # main_arm_planner,  # Disabled - main_arm_interface handles target_point with pitch
        # main_arm_teleop,
        main_arm_xbox,

        # RViz with custom config
        rviz_node,

        # Side arm nodes
        side_arm_serial_bridge,
        side_arm_coordinate,
        side_arm_interface,
        side_arm_visualizer,

        # Side arm URDF visualization
        side_arm_state_publisher,
        side_arm_joint_gui,
        side_arm_joint_bridge,

        # Digital twin visualization
        frame_state_publisher,
        frame_tf,
        world_to_base_fallback_tf,
        side_arm_tf,
        # camera_frame is published by URDF robot_state_publisher (side_arm)

        # Perception visualization
        loop_ground_truth,
        loop_visualizer,
        detection_simulator,

        # Side camera perception
        side_cam_yolo,
        side_cam_to_3d,

        # Top camera loop state
        top_cam_loop_state,

        # Real camera nodes
        yolo_detector,
        camera_to_3d,
    ]


def generate_launch_description():
    """Generate launch description with arguments and OpaqueFunction for config loading."""

    # Default config paths
    side_arm_config_dir = os.path.join(
        get_package_share_directory('side_arm_control'),
        'config'
    )
    default_arm_config = os.environ.get(
        'SIDE_ARM_CONFIG',
        os.path.join(side_arm_config_dir, 'side_arm_v1.yaml')
    )

    return LaunchDescription([
        # ==================== LAUNCH ARGUMENTS ====================

        # Main arm arguments
        DeclareLaunchArgument(
            'enable_main_arm',
            default_value='true',
            description='Enable main arm (WX200) control'
        ),
        DeclareLaunchArgument(
            'main_arm_sim',
            default_value='false',
            description='Run main arm in simulation mode'
        ),
        DeclareLaunchArgument(
            'enable_teleop',
            default_value='false',
            description='Enable Xbox controller teleoperation for main arm'
        ),
        DeclareLaunchArgument(
            'controller_type',
            default_value='xboxone',
            description='Controller type (xboxone, ps4, etc.)'
        ),
        DeclareLaunchArgument(
            'robot_model',
            default_value='wx200',
            description='Main arm robot model'
        ),
        DeclareLaunchArgument(
            'enable_monitor',
            default_value='false',
            description='Enable Motor Monitoring for Main Arm'
        ),

        # Side arm arguments
        DeclareLaunchArgument(
            'enable_side_arm',
            default_value='true',
            description='Enable side arm control'
        ),
        DeclareLaunchArgument(
            'side_arm_sim',
            default_value='false',
            description='Run side arm in pure simulation mode (no serial bridge)'
        ),
        DeclareLaunchArgument(
            'side_arm_test_mode',
            default_value='false',
            description='Run side arm in test mode (simulated movements alongside hardware)'
        ),
        DeclareLaunchArgument(
            'arm_config',
            default_value=default_arm_config,
            description='Path or filename of side arm YAML config (e.g., side_arm_v2.yaml)'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value=os.environ.get('SIDE_ARM_PORT', '/dev/ttyUSB0'),
            description='Serial port for side arm ESP32 connection (or set SIDE_ARM_PORT env var)'
        ),
        DeclareLaunchArgument(
            'use_joint_sliders',
            default_value='false',
            description='Use joint_state_publisher_gui for manual side arm control via sliders'
        ),

        # Visualization arguments
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='true',
            description='Enable side arm RViz visualization marker'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for main arm visualization'
        ),

        # Vision/perception arguments
        DeclareLaunchArgument(
            'vision_test_mode',
            default_value='false',
            description='Use simulated loop detections instead of camera'
        ),
        DeclareLaunchArgument(
            'use_real_camera',
            default_value='false',
            description='Use real USB camera with YOLO detection'
        ),
        DeclareLaunchArgument(
            'camera_index',
            default_value='4',
            description='USB camera index for YOLO detector'
        ),
        DeclareLaunchArgument(
            'camera_display',
            default_value='true',
            description='Show YOLO detection window'
        ),
        DeclareLaunchArgument(
            'assumed_depth',
            default_value='0.22',
            description='Assumed depth from camera to loop plane (meters)'
        ),
        DeclareLaunchArgument(
            'enable_loop_visualization',
            default_value='true',
            description='Enable loop detection visualization in RViz'
        ),

        # Use OpaqueFunction to load config at launch time
        OpaqueFunction(function=launch_setup),
    ])
