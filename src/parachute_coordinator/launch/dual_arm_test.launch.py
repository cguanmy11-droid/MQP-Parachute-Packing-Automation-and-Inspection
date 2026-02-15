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

Move side arm in simulation:
    ros2 service call /side_arm/move_to_position \\
        parachute_interfaces/srv/MoveToPosition \\
        \"{x_mm: 100.0, y_mm: 50.0, z_mm: 20.0, speed_scale: 0.5}\"
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Load frame URDF
    frame_urdf_path = os.path.join(
        get_package_share_directory('main_arm_control'),
        'urdf',
        'framemodel.urdf'
    )

    with open(frame_urdf_path, 'r') as f:
        frame_urdf = f.read()

    # Load side arm URDF
    side_arm_urdf_path = os.path.join(
        get_package_share_directory('side_arm_control'),
        'urdf',
        'side_arm.urdf'
    )

    with open(side_arm_urdf_path, 'r') as f:
        side_arm_urdf = f.read()

    # ==================== LAUNCH ARGUMENTS ====================

    # Main arm arguments
    enable_main_arm_arg = DeclareLaunchArgument(
        'enable_main_arm',
        default_value='true',
        description='Enable main arm (WX200) control'
    )

    main_arm_sim_arg = DeclareLaunchArgument(
        'main_arm_sim',
        default_value='false',
        description='Run main arm in simulation mode'
    )

    enable_teleop_arg = DeclareLaunchArgument(
        'enable_teleop',
        default_value='false',
        description='Enable Xbox controller teleoperation for main arm'
    )

    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='xboxone',
        description='Controller type (xboxone, ps4, etc.)'
    )

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='wx200',
        description='Main arm robot model'
    )

    # Side arm arguments
    enable_side_arm_arg = DeclareLaunchArgument(
        'enable_side_arm',
        default_value='true',
        description='Enable side arm control'
    )

    side_arm_sim_arg = DeclareLaunchArgument(
        'side_arm_sim',
        default_value='false',
        description='Run side arm in pure simulation mode (no serial bridge)'
    )

    side_arm_test_mode_arg = DeclareLaunchArgument(
        'side_arm_test_mode',
        default_value='false',
        description='Run side arm in test mode (simulated movements alongside hardware)'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for side arm ESP32 connection'
    )

    use_joint_sliders_arg = DeclareLaunchArgument(
        'use_joint_sliders',
        default_value='false',
        description='Use joint_state_publisher_gui for manual side arm control via sliders'
    )

    # Visualization arguments
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable side arm RViz visualization marker'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for main arm visualization'
    )

    # Vision/perception arguments
    vision_test_mode_arg = DeclareLaunchArgument(
        'vision_test_mode',
        default_value='false',
        description='Use simulated loop detections instead of camera'
    )

    enable_loop_visualization_arg = DeclareLaunchArgument(
        'enable_loop_visualization',
        default_value='true',
        description='Enable loop detection visualization in RViz'
    )

    # ==================== MAIN ARM NODES ====================

    # Interbotix arm control (hardware or sim)
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
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_main_arm'))
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

    # Main arm teleop node (optional)
    main_arm_teleop = Node(
        package='main_arm_control',
        executable='main_arm_teleop_node',
        name='main_arm_teleop_node',
        output='screen',
        parameters=[{
            'robot_model': LaunchConfiguration('robot_model'),
            'controller_type': LaunchConfiguration('controller_type'),
            'auto_start': True,
        }],
        condition=IfCondition(
            PythonExpression([
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
        parameters=[{
            'device_id': 0,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        condition=IfCondition(LaunchConfiguration('enable_teleop'))
    )

    # ==================== SIDE ARM NODES ====================

    # Serial bridge node (communicates with ESP32)
    # Only launches when NOT in simulation mode (side_arm_sim=false)
    # In hybrid mode (side_arm_test_mode=true), both hardware and simulation run
    side_arm_serial_bridge = Node(
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
        parameters=[{
            'simulation_mode': side_arm_simulation_enabled,  # Enable simulation when sim or test mode
            'sim_speed_mm_per_sec': 50.0,  # Simulation motion speed
            'steps_per_mm_horizontal': 300.0,
            'steps_per_mm_vertical': 100.0,
            'dc_mm_per_second': 4.0,
            'dc_speed_percent': 50,
            'default_speed_horizontal': 1200.0,
            'default_speed_vertical': 500.0,
            'max_x_mm': 300.0,
            'max_y_mm': 200.0,
            'max_z_mm': 150.0,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_side_arm'))
    )

    # Side arm interface node (high-level actions/services)
    side_arm_interface = Node(
        package='side_arm_control',
        executable='side_arm_interface_node',
        name='side_arm_interface_node',
        parameters=[{
            'test_mode': side_arm_simulation_enabled,
            'approach_offset_z': 50.0,
            'insert_depth_z': 30.0,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_side_arm'))
    )

    #  Side arm visualizer (RViz marker + TF for hook and camera)
    side_arm_visualizer = Node(
        package='side_arm_control',
        executable='side_arm_visualizer',
        name='side_arm_visualizer',
        output='screen',
        parameters=[{
            # Hook mesh orientation
            'roll': -1.5708,
            'pitch': 0.0,
            'yaw': -1.5708,
            # Hook mesh offset
            'offset_x': -0.01,
            'offset_y': 0.009,
            'offset_z': 0.07,
            'scale': 0.001,
            # Servo rotation
            'servo_axis': 'pitch',
            'servo_scale': 0.001,
            # Test mode (enabled in sim or test mode)
            'test_mode': side_arm_simulation_enabled,
            'test_x': 0.0,
            'test_y': 0.0,
            'test_z': 0.0,
            # TF publishing
            'publish_hook_tf': True,
            'hook_frame_id': 'side_arm_hook',
            # Camera frame is now published by URDF robot_state_publisher (attached to y_carriage)
            # Set to False to avoid TF conflict with URDF's camera_frame
            'publish_camera_tf': False,
            'camera_frame_id': 'camera_frame',
            'camera_offset_x': 0.0,    # Not used when publish_camera_tf is False
            'camera_offset_y': 0.0,
            'camera_offset_z': 0.05,
            # Rotate camera to look toward the ground truth loops
            # These values need tuning based on actual setup
            'camera_roll': 0.0,
            'camera_pitch': 3.1416,    # 180 degrees - flip forward direction
            'camera_yaw': 0.0,
        }],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('enable_side_arm'), "' == 'true' and '",
                LaunchConfiguration('enable_visualization'), "' == 'true'"
            ])
        )
    )

    # ==================== SIDE ARM URDF ====================

    # Side arm robot state publisher (publishes URDF to /side_arm/robot_description)
    side_arm_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='side_arm_robot_state_publisher',
        namespace='side_arm',
        parameters=[{'robot_description': side_arm_urdf}],
        condition=IfCondition(LaunchConfiguration('enable_side_arm'))
    )

    # Joint state publisher GUI (for manual slider control in test mode)
    side_arm_joint_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='side_arm_joint_gui',
        namespace='side_arm',
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
        namespace='side_arm',
        parameters=[{
            'servo_scale': 0.001,
            'publish_rate': 50.0,
            'test_mode': side_arm_simulation_enabled,
            'test_x': 0.15,
            'test_y': 0.10,
            'test_z': 0.05,
            'test_servo': 0.0,
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
            '--frame-id', 'wx200/base_link',
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

    # Side arm origin TF
    side_arm_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='side_arm_static_tf',
        arguments=[
            '--x', '0.4', '--y', '0.19', '--z', '-0.03',
            '--roll', '1.5708', '--pitch', '0', '--yaw', '3.1416',
            '--frame-id', 'wx200/base_link',
            '--child-frame-id', 'side_arm_origin'
        ],
        condition=IfCondition(LaunchConfiguration('enable_side_arm'))
    )

    # NOTE: camera_frame TF is now published by URDF robot_state_publisher
    # (attached to y_carriage_link, so it moves with X/Y but not Z)

    # ==================== PERCEPTION VISUALIZATION NODES ====================

    # Loop ground truth (publishes actual loop positions in world frame)
    loop_ground_truth = Node(
        package='parachute_perception',
        executable='loop_ground_truth_node',
        name='loop_ground_truth_node',
        output='screen',
        parameters=[{
            'frame_id': 'world',
            'publish_rate': 10.0,
            'pattern': 'line',  # static, random, grid, line
            'num_loops': 5,
            'loop_radius': 0.015,
            # Position bounds for random/grid/line patterns
            'x_min': 0.25,
            'x_max': 0.40,
            'y_min': 0.148,
            'y_max': 0.152,
            'z_min': -0.04,
            'z_max': 0.01,
            # Static positions: [x0,y0,z0, x1,y1,z1, ...] - adjust to match physical setup
            'static_positions': [
                0.35, -0.05, -0.02,
                0.38, -0.05, 0.00,
                0.41, -0.05, -0.01,
                0.44, -0.05, 0.01,
                0.47, -0.05, -0.02,
            ],
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
            'input_frame_id': 'camera_frame',  # Frame detections arrive in
            'output_frame_id': 'world',        # Frame to publish markers in
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
            'camera_frame_id': 'camera_frame',
            'world_frame_id': 'world',
            # Camera FOV - set wide for testing, narrow later to match real camera
            'camera_fov_horizontal': 120.0,  # degrees (wide for testing)
            'camera_fov_vertical': 120.0,    # degrees (wide for testing)
            'max_detection_range': 1.0,      # meters (extended for testing)
            'min_detection_range': 0.01,     # meters
            # Detection simulation
            'detection_noise_stddev': 0.003,  # meters
            'confidence_base': 0.90,
            'confidence_noise': 0.05,
            'false_negative_rate': 0.0,
            'publish_rate': 5.0,
            # Debug - enable to bypass FOV checks and see what's happening
            'debug_bypass_fov': True,   # TEMP: Enable to detect loops even if camera wrong way
            'debug_verbose': True,      # Show detailed logging
        }],
        condition=IfCondition(LaunchConfiguration('vision_test_mode'))
    )

    # ==================== LAUNCH DESCRIPTION ====================

    return LaunchDescription([
        # Arguments
        enable_main_arm_arg,
        main_arm_sim_arg,
        enable_teleop_arg,
        controller_type_arg,
        robot_model_arg,
        enable_side_arm_arg,
        side_arm_sim_arg,
        side_arm_test_mode_arg,
        serial_port_arg,
        use_joint_sliders_arg,
        enable_visualization_arg,
        use_rviz_arg,
        vision_test_mode_arg,
        enable_loop_visualization_arg,
        joy_node,

        # Main arm nodes
        main_arm_control_launch,
        main_arm_interface,
        main_arm_planner,
        main_arm_teleop,

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
    ])
