#!/usr/bin/env python3
"""
Simple Main Arm Interface Node 
Provides action server for trajectory execution
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from parachute_interfaces.action import ExecuteTrajectory
from parachute_interfaces.msg import ArmStatus
from geometry_msgs.msg import Pose, Twist, PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import modern_robotics as mr
from std_msgs.msg import String, Float32
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import time
from interbotix_xs_msgs.msg import JointSingleCommand

class MainArmInterfaceNode(Node):
    def __init__(self):
        super().__init__('main_arm_interface_node')

        # Declare parameters
        self.declare_parameter('test_mode', True) # Set to False for real robot
        self.declare_parameter('use_sim', False)
        self.declare_parameter('robot_model', 'wx200')
        self.declare_parameter('robot_name', 'wx200')
        self.declare_parameter('moving_time', 2.0)  # Time for each movement
        self.declare_parameter('accel_time', 0.5)   # Acceleration time

        # Joystick control timing - must be fast for responsive feel
        self.declare_parameter('joy_moving_time', 0.08)
        self.declare_parameter('joy_accel_time', 0.04)
        self.joy_moving_time = self.get_parameter('joy_moving_time').value
        self.joy_accel_time = self.get_parameter('joy_accel_time').value

        # Latest joystick command (non-blocking storage)
        self.latest_ee_increment = None
        self.joy_active = False

        # Handle parameters as either bool or string (from LaunchConfiguration)
        test_mode_val = self.get_parameter('test_mode').value
        if isinstance(test_mode_val, bool):
            self.test_mode = test_mode_val
        else:
            self.test_mode = str(test_mode_val).lower() == 'true'

        use_sim_val = self.get_parameter('use_sim').value
        if isinstance(use_sim_val, bool):
            self.use_sim = use_sim_val
        else:
            self.use_sim = str(use_sim_val).lower() == 'true'
        robot_model = self.get_parameter('robot_model').value
        robot_name = self.get_parameter('robot_name').value
        moving_time = self.get_parameter('moving_time').value
        accel_time = self.get_parameter('accel_time').value
        self.joy_moving_time = self.get_parameter('joy_moving_time').value
        self.joy_accel_time = self.get_parameter('joy_accel_time').value
        
        self.bot = None

        # Initialize robot (only if not in test mode)
        if not self.test_mode:
            # Initialize robot for hardware and simulation
            self.get_logger().info(f'Initializing {robot_model} arm...')
            try:
                self.bot = InterbotixManipulatorXS(
                    robot_model=robot_model,
                    robot_name=robot_name,
                    moving_time=moving_time,
                    accel_time=accel_time,
                )
                if self.use_sim:
                    self.get_logger().info('Running in SIMULATION MODE')
                    self.get_logger().info('Robot controlled via RViz simulation')
                else: 
                    self.get_logger().info('Robot arm initialized successfully')
                
                # Now initialize using the Interbotix Node instead
                # super().__init__('main_arm_interface_node')
                
                # Go to home pose on startup
                self.bot.arm.go_to_home_pose()
                # Gripper position control
                self.declare_parameter('gripper_step', 0.003)    # increment per X/Y press
                self.declare_parameter('gripper_open', 0.037)    # fully open position
                self.declare_parameter('gripper_closed', 0.015)  # fully closed position

                self.gripper_pwm = 0          # current PWM (0 = stopped)
                self.gripper_pwm_max = 350    # max close force
                self.gripper_pwm_step = 50    # step per X/Y press

                self.current_pose_name = 'home'
            except Exception as e:
                self.get_logger().error(f'Failed to initialize robot: {e}')
                self.bot = None
        else:
            self.get_logger().info('Running in TEST MODE - no robot initialization')
            # super().__init__('main_arm_interface_node')
        

        # Action server for trajectory execution
        self.action_server = ActionServer(self, ExecuteTrajectory, '/main_arm/execute_trajectory', self.execute_trajectory_callback)
        
        # Subscribers for simple pose and gripper commands
        self.pose_cmd_sub = self.create_subscription(String, '/main_arm/pose_command', self.pose_command_callback, 10)
        self.pose_cmd_sub = self.create_subscription(String, '/main_arm/gripper_command', self.gripper_command_callback, 10)
        # self.gripper_effort_sub = self.create_subscription(Float32, '/main_arm/gripper_effort', self.gripper_effort_callback, 10)
        self.ee_increment_sub = self.create_subscription(Twist, '/main_arm/ee_increment', self.ee_increment_callback, 10)
        # Add this subscriber for testing positions
        self.test_position_sub = self.create_subscription(Pose, '/main_arm/test_position', self.test_position_callback, 10)
        # Timer to process joystick commands - runs faster than the Xbox publishes
        # so we never miss a command. Only processes if a new command is waiting.
        self.joy_timer = self.create_timer(0.05, self.process_joy_increment)

        # Publisher for arm and pose status
        self.status_publisher = self.create_publisher(ArmStatus, '/main_arm/status', 10)
        self.simple_status_publisher = self.create_publisher(String, '/main_arm/simple_status', 10)
        self.pose_publisher = self.create_publisher(Pose, '/main_arm/current_pose', 10)

        # Publisher for trajectory waypoints visualization (MarkerArray with LINE_STRIP + spheres)
        self.waypoints_marker_pub = self.create_publisher(MarkerArray, '/main_arm/trajectory_markers', 10)
        
        # Timer to publish status
        self.timer = self.create_timer(1.0, self.publish_status)
        self.gripper_pwm_pub = self.create_publisher(JointSingleCommand, '/wx200/commands/joint_single', 10)

        # State tracking
        self.current_state = ArmStatus.STATE_IDLE
        self.current_pose_name = 'home' if self.bot else 'unknown'
        self.gripper_effort = 0.5  # Default gripper effort (0.0-1.0)
        self.is_moving = False
        self.error_message = ""
        
        self.get_logger().info('Main Arm Interface Node initialized')
        
    def publish_status(self):
        """Publish current arm status"""
        msg = ArmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.current_state
        msg.is_moving = self.is_moving or (self.current_state == ArmStatus.STATE_EXECUTING)
        msg.goal_reached = (self.current_state == ArmStatus.STATE_SUCCESS)
        msg.error_message = self.error_message
        
        # Get current pose if robot is available
        if self.bot is not None:
            try:
                current_ee_pose = self.bot.arm.get_ee_pose()
                pose_msg = Pose()
                pose_msg.position.x = current_ee_pose[0, 3]
                pose_msg.position.y = current_ee_pose[1, 3]
                pose_msg.position.z = current_ee_pose[2, 3]
                msg.current_pose = pose_msg
                
                # Also publish to separate topic
                self.pose_publisher.publish(pose_msg)
            except:
                msg.current_pose = Pose()
        else:
            msg.current_pose = Pose()
        
        self.status_publisher.publish(msg)

        # Also publish simple status for other nodes
        simple_status = String()
        if self.is_moving:
            simple_status.data = f'moving_to_{self.current_pose_name}'
        else:
            simple_status.data = f'idle_at_{self.current_pose_name}'
        self.simple_status_publisher.publish(simple_status)
        
    def execute_trajectory_callback(self, goal_handle):
        """Action callback - simulate trajectory execution"""
        num_waypoints = len(goal_handle.request.waypoints)
        self.get_logger().info(f'Executing trajectory with {num_waypoints} waypoints')

        # Publish waypoints as MarkerArray for RViz visualization (LINE_STRIP + spheres)
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # LINE_STRIP connecting all waypoints
        line_marker = Marker()
        line_marker.header.stamp = stamp
        line_marker.header.frame_id = 'world'
        line_marker.ns = 'trajectory_path'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.005  # Line width
        line_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
        line_marker.pose.orientation.w = 1.0

        for waypoint in goal_handle.request.waypoints:
            p = Point()
            p.x = waypoint.position.x
            p.y = waypoint.position.y
            p.z = waypoint.position.z
            line_marker.points.append(p)

        marker_array.markers.append(line_marker)

        # Spheres at each waypoint
        for i, waypoint in enumerate(goal_handle.request.waypoints):
            sphere = Marker()
            sphere.header.stamp = stamp
            sphere.header.frame_id = 'world'
            sphere.ns = 'trajectory_points'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = waypoint.position
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.015  # Sphere diameter
            sphere.scale.y = 0.015
            sphere.scale.z = 0.015
            # Color gradient: start=green, end=red
            t = i / max(1, num_waypoints - 1)
            sphere.color = ColorRGBA(r=t, g=1.0 - t, b=0.0, a=1.0)
            marker_array.markers.append(sphere)

        self.waypoints_marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {num_waypoints} waypoints to /main_arm/trajectory_markers')

        self.current_state = ArmStatus.STATE_EXECUTING
        feedback_msg = ExecuteTrajectory.Feedback()

        try:
            # For each waypoint
            for i, waypoint in enumerate(goal_handle.request.waypoints):
                feedback_msg.current_waypoint_index = i
                feedback_msg.progress = (i + 1) / num_waypoints
                feedback_msg.current_pose = waypoint
                feedback_msg.time_elapsed = float(i + 1)
                
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f'Waypoint {i+1}/{num_waypoints} reached')
                
                # Execute waypoint on real robot
                if self.bot is not None:
                    # Convert quaternion to roll, pitch, yaw for Interbotix
                    import tf_transformations
                    quat = [
                        waypoint.orientation.x,
                        waypoint.orientation.y,
                        waypoint.orientation.z,
                        waypoint.orientation.w
                    ]
                    # Check if orientation is specified (non-zero quaternion)
                    has_orientation = quat[3] != 0 or any(q != 0 for q in quat[:3])

                    if has_orientation:
                        # Try with orientation first
                        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quat)
                        success = self.bot.arm.set_ee_pose_components(
                            x=waypoint.position.x,
                            y=waypoint.position.y,
                            z=waypoint.position.z,
                            roll=roll,
                            pitch=pitch,
                            yaw=yaw
                        )
                        # If orientation fails, fallback to position-only
                        if not success:
                            self.get_logger().info(f'Waypoint {i+1}: orientation failed, trying position-only')
                            success = self.bot.arm.set_ee_pose_components(
                                x=waypoint.position.x,
                                y=waypoint.position.y,
                                z=waypoint.position.z
                            )
                    else:
                        # No orientation specified, use position-only
                        success = self.bot.arm.set_ee_pose_components(
                            x=waypoint.position.x,
                            y=waypoint.position.y,
                            z=waypoint.position.z
                        )

                    if not success:
                        self.get_logger().warn(f'Waypoint {i+1} unreachable even with position-only, skipping')
                else:
                    # Test mode - simulate movement
                    time.sleep(1)
            
            # Mark as successful
            goal_handle.succeed()
            
            result = ExecuteTrajectory.Result()
            result.success = True
            result.waypoints_completed = num_waypoints
            result.execution_time = float(num_waypoints)
            result.message = "Trajectory executed successfully"
            
            self.current_state = ArmStatus.STATE_SUCCESS
            self.is_moving = False
            self.get_logger().info('Trajectory execution complete')

        except Exception as e:
            self.get_logger().error(f'Trajectory execution failed: {e}')
            self.current_state = ArmStatus.STATE_ERROR
            self.is_moving = False
            self.error_message = str(e)
            
            goal_handle.abort()
            
            result = ExecuteTrajectory.Result()
            result.success = False
            result.waypoints_completed = i if 'i' in locals() else 0
            result.execution_time = 0.0
            result.message = f"Trajectory execution failed: {e}"
        
        return result

    def pose_command_callback(self, msg):
        """Handle predefined pose commands"""
        command = msg.data.lower().strip()
        self.get_logger().info(f'Received pose command: {command}')

        if self.bot is None:
            if self.test_mode:
                self.get_logger().info(f'TEST: Would move to {command} pose')
                self.current_pose_name = command
                return
            else:
                self.get_logger().error('Robot not initialized!')
                return
        
        try:
            self.is_moving = True
            self.current_state = ArmStatus.STATE_EXECUTING
            
            if command == 'home':
                self.bot.arm.go_to_home_pose()
                self.current_pose_name = 'home'
                
            elif command == 'sleep':
                self.bot.arm.go_to_sleep_pose()
                self.current_pose_name = 'sleep'
                
            elif command == 'upright':
                self.bot.arm.set_joint_positions([0, 0, 0, 0, 0])
                self.current_pose_name = 'upright'
            
            elif command == 'test_fk' or command == 'fk':
                self.test_forward_kinematics()
                self.is_moving = False
                self.current_state = ArmStatus.STATE_SUCCESS
                return
                
            else:
                self.get_logger().warn(f'Unknown pose: {command}')
                self.current_state = ArmStatus.STATE_ERROR
                self.error_message = f'Unknown pose: {command}'
                self.is_moving = False
                return
            
            self.is_moving = False
            self.current_state = ArmStatus.STATE_SUCCESS
            self.error_message = ""
            self.get_logger().info(f'Successfully moved to {command} pose')
            
            # After any movement, print FK
            # if command != 'test_fk':
            #     self.test_forward_kinematics()
            
        except Exception as e:
            self.is_moving = False
            self.current_state = ArmStatus.STATE_ERROR
            self.error_message = str(e)
            self.get_logger().error(f'Failed to move to {command}: {e}')
    
    def gripper_command_callback(self, msg):
        """Gripper control via PWM: open/close snap, inc/dec step"""
        command = msg.data.lower().strip()
        self.get_logger().info(f'Received gripper command: {command}')

        if self.bot is None:
            if self.test_mode:
                self.get_logger().info(f'TEST: gripper {command}')
            return

        try:
            if command == 'open':
                self.bot.gripper.release()
                self.gripper_pwm = 0
                self.get_logger().info('Gripper OPEN (release)')

            elif command == 'close':
                self.bot.gripper.grasp()
                self.gripper_pwm = self.gripper_pwm_max
                self.get_logger().info('Gripper CLOSE (full grasp)')

            elif command == 'inc':
                self.gripper_pwm = max(0, self.gripper_pwm - self.gripper_pwm_step)
                cmd = JointSingleCommand()
                cmd.name = 'gripper'
                cmd.cmd = float(self.gripper_pwm)
                self.gripper_pwm_pub.publish(cmd)
                self.get_logger().info(f'Gripper PWM: {self.gripper_pwm}')

            elif command == 'dec':
                self.gripper_pwm = min(
                    self.gripper_pwm_max,
                    self.gripper_pwm + self.gripper_pwm_step)
                cmd = JointSingleCommand()
                cmd.name = 'gripper'
                cmd.cmd = float(self.gripper_pwm)
                self.gripper_pwm_pub.publish(cmd)
                self.get_logger().info(f'Gripper PWM: {self.gripper_pwm}')

            else:
                self.get_logger().warn(f'Unknown gripper command: {command}')

        except Exception as e:
            self.get_logger().error(f'Gripper command failed: {e}')
    
    def ee_increment_callback(self, msg):
        """
        NON-BLOCKING: Just store the latest command.
        The joy_timer will process it.
        """
        self.latest_ee_increment = msg

        # Switch to fast timing on first joystick input
        if not self.joy_active and self.bot is not None:
            self.joy_active = True
            self.bot.arm.set_trajectory_time(
                moving_time=self.joy_moving_time,
                accel_time=self.joy_accel_time
            )
            self.get_logger().info(
                f'Joystick control active (moving_time={self.joy_moving_time}s)')


    def process_joy_increment(self):
        """
        ALL movement via direct joint control = all smooth.

        Twist mapping:
            linear.x/y/z    = EE translation (Jacobian → joint increments)
            angular.x        = wrist_rotate joint (direct)
            angular.y        = wrist_angle joint (direct)
            angular.z        = waist joint (direct)

        WX200 joint indices: 0=waist, 1=shoulder, 2=elbow, 3=wrist_angle, 4=wrist_rotate
        """
        import numpy as np
        import modern_robotics as mr

        msg = self.latest_ee_increment
        if msg is None:
            if self.joy_active:
                self._joy_idle_count = getattr(self, '_joy_idle_count', 0) + 1
                if self._joy_idle_count > 20:
                    self.joy_active = False
                    self._joy_idle_count = 0
                    if self.bot is not None:
                        moving_time = self.get_parameter('moving_time').value
                        accel_time = self.get_parameter('accel_time').value
                        self.bot.arm.set_trajectory_time(
                            moving_time=moving_time,
                            accel_time=accel_time
                        )
                        self.get_logger().info('Joystick idle - restored normal timing')
            return

        self.latest_ee_increment = None
        self._joy_idle_count = 0

        if self.bot is None:
            if self.test_mode:
                self.get_logger().info(
                    f'TEST: dx={msg.linear.x:.4f} dz={msg.linear.z:.4f}',
                    throttle_duration_sec=0.5)
            return

        try:
            dx = msg.linear.x
            dy = msg.linear.y
            dz = msg.linear.z
            d_wrist_rotate = msg.angular.x   # Right Stick X
            d_wrist_angle = msg.angular.y    # Right Stick Y
            dwaist = msg.angular.z           # Triggers

            # Nothing to do?
            if not any(v != 0.0 for v in [dx, dy, dz, d_wrist_rotate, d_wrist_angle, dwaist]):
                return

            # Get current joint positions
            # WX200: [waist, shoulder, elbow, wrist_angle, wrist_rotate]
            current_joints = list(self.bot.arm.get_joint_commands())

            # ── Direct joint control: waist, wrist_angle, wrist_rotate ──
            if dwaist != 0.0:
                current_joints[0] += dwaist          # waist
            if d_wrist_angle != 0.0:
                current_joints[3] += d_wrist_angle   # wrist_angle
            if d_wrist_rotate != 0.0:
                current_joints[4] += d_wrist_rotate  # wrist_rotate

            # ── Translation: Jacobian → joint increments ──
            if any(v != 0.0 for v in [dx, dy, dz]):
                joint_cmds = self.bot.arm.get_joint_commands()
                jacobian = mr.JacobianSpace(
                    self.bot.arm.robot_des.Slist,
                    joint_cmds
                )

                # modern_robotics twist: [wx, wy, wz, vx, vy, vz]
                # Only translation, no rotation in the twist
                ee_twist = np.array([0.0, 0.0, 0.0, dx, dy, dz])

                # Damped least squares for stability near singularities
                damping = 0.01
                J_JT = jacobian @ jacobian.T
                J_pinv = jacobian.T @ np.linalg.inv(J_JT + damping**2 * np.eye(6))

                joint_deltas = J_pinv @ ee_twist

                for i in range(len(joint_deltas)):
                    # Skip joints we already set directly
                    if i == 0 and dwaist != 0.0:
                        continue
                    if i == 3 and d_wrist_angle != 0.0:
                        continue
                    if i == 4 and d_wrist_rotate != 0.0:
                        continue
                    if i < len(current_joints):
                        current_joints[i] += joint_deltas[i]

            # ── One single call, all joints at once, smooth ──
            self.bot.arm.set_joint_positions(current_joints)

        except Exception as e:
            self.get_logger().warn(
                f'Joy increment failed: {e}',
                throttle_duration_sec=1.0)
    
    # def gripper_effort_callback(self, msg):
    #     """Update gripper effort setting"""
    #     self.gripper_effort = max(0.0, min(1.0, msg.data))
    #     self.get_logger().info(f'Gripper effort set to: {self.gripper_effort:.2f}')

    #     if self.bot is None and not self.use_sim:
    #         self.get_logger().warn('No robot available')
    #         return
        
    #     try:
    #         self.bot.gripper.set_pressure(self.gripper_effort)
    #     except Exception as e:
    #         self.get_logger().error(f'Gripper command failed: {e}')
    
    # Make sure shut down is clean exit
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down Main Arm Interface')
        if self.bot is not None:
            try:
                self.bot.arm.go_to_sleep_pose()
                self.bot.shutdown()
            except:
                pass
    
    def test_position_callback(self, msg):
        """Test moving to an arbitrary Cartesian position with orientation"""
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        
        # Extract roll, pitch, yaw from quaternion
        from scipy.spatial.transform import Rotation as R
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        
        # Check if orientation was specified (non-zero quaternion)
        if sum([abs(q) for q in quat]) > 0.01:
            euler = R.from_quat(quat).as_euler('xyz', degrees=False)
            roll, pitch, yaw = euler
            self.get_logger().info(f'Testing: X={x:.3f}, Y={y:.3f}, Z={z:.3f}, Roll={roll:.3f}, Pitch={pitch:.3f}, Yaw={yaw:.3f}')
        else:
            roll = pitch = yaw = None
            self.get_logger().info(f'Testing position: X={x:.3f}, Y={y:.3f}, Z={z:.3f} (auto orientation)')
        
        if self.bot is None:
            self.get_logger().warn('No robot available')
            return
        
        try:
            self.is_moving = True
            self.current_state = ArmStatus.STATE_EXECUTING
            
            # Move to position with optional orientation
            if roll is not None:
                success = self.bot.arm.set_ee_pose_components(
                    x=x, y=y, z=z,
                    roll=roll, pitch=pitch
                    # Note: For 5-DOF arm, yaw is derived from x,y position
                )
            else:
                # Just position, let IK figure out orientation
                success = self.bot.arm.set_ee_pose_components(x=x, y=y, z=z)
            
            if success:
                self.get_logger().info('Successfully reached target')
                time.sleep(0.5)
                
                # Show actual position achieved
                # self.test_forward_kinematics()
            else:
                self.get_logger().warn(' Target unreachable (IK failed)')
            
            self.is_moving = False
            self.current_state = ArmStatus.STATE_SUCCESS if success else ArmStatus.STATE_ERROR
            
        except Exception as e:
            self.is_moving = False
            self.current_state = ArmStatus.STATE_ERROR
            self.get_logger().error(f'Error: {e}')
    
    def test_forward_kinematics(self):
        """Test function to understand coordinate frames and FK"""
        if self.bot is None:
            self.get_logger().warn('No robot available for FK test')
            return
        
        self.get_logger().info('='*50)
        self.get_logger().info('FORWARD KINEMATICS TEST')
        self.get_logger().info('='*50)
        
        # Get current end-effector pose (4x4 transformation matrix)
        T_sb = self.bot.arm.get_ee_pose()
        
        # Extract position (translation)
        x = T_sb[0, 3]
        y = T_sb[1, 3]
        z = T_sb[2, 3]
        
        self.get_logger().info(f'End-Effector Position (in base frame):')
        self.get_logger().info(f'  X: {x:.4f} meters')
        self.get_logger().info(f'  Y: {y:.4f} meters')
        self.get_logger().info(f'  Z: {z:.4f} meters')
        
        # Extract rotation matrix (orientation)
        R = T_sb[0:3, 0:3]
        
        # Convert rotation matrix to roll-pitch-yaw (Euler angles)
        import numpy as np
        
        # Roll (rotation around X-axis)
        roll = np.arctan2(R[2, 1], R[2, 2])
        
        # Pitch (rotation around Y-axis)
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
        
        # Yaw (rotation around Z-axis)
        yaw = np.arctan2(R[1, 0], R[0, 0])
        
        self.get_logger().info(f'End-Effector Orientation (roll-pitch-yaw):')
        self.get_logger().info(f'  Roll:  {np.degrees(roll):.2f}° ({roll:.4f} rad)')
        self.get_logger().info(f'  Pitch: {np.degrees(pitch):.2f}° ({pitch:.4f} rad)')
        self.get_logger().info(f'  Yaw:   {np.degrees(yaw):.2f}° ({yaw:.4f} rad)')
        
        # Get current joint positions
        joint_positions = self.bot.arm.get_joint_commands()
        joint_names = self.bot.arm.group_info.joint_names
        
        self.get_logger().info(f'Current Joint Positions:')
        for name, pos in zip(joint_names, joint_positions):
            self.get_logger().info(f'  {name}: {np.degrees(pos):.2f}° ({pos:.4f} rad)')
        
        self.get_logger().info('='*50)
        
        # Also publish this as a Pose message
        pose_msg = Pose()
        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.position.z = z
        
        # Convert euler angles to quaternion for ROS message
        from scipy.spatial.transform import Rotation as R_scipy
        quat = R_scipy.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        pose_msg.orientation.x = quat[0]
        pose_msg.orientation.y = quat[1]
        pose_msg.orientation.z = quat[2]
        pose_msg.orientation.w = quat[3]
        
        self.pose_publisher.publish(pose_msg)
        
        return T_sb, x, y, z, roll, pitch, yaw

    def print_tf_tree(self):
        """Print the TF transformation tree"""
        if self.bot is None:
            return
        
        self.get_logger().info('='*50)
        self.get_logger().info('TF FRAME TREE')
        self.get_logger().info('='*50)
        self.get_logger().info('Frame hierarchy:')
        self.get_logger().info('  /wx200/base_link (world frame)')
        self.get_logger().info('    └─> /wx200/shoulder_link')
        self.get_logger().info('        └─> /wx200/upper_arm_link')
        self.get_logger().info('            └─> /wx200/forearm_link')
        self.get_logger().info('                └─> /wx200/wrist_link')
        self.get_logger().info('                    └─> /wx200/gripper_link')
        self.get_logger().info('                        └─> /wx200/ee_gripper_link (end-effector)')
        self.get_logger().info('='*50)

def main(args=None):
    rclpy.init(args=args)
    node = MainArmInterfaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt')
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()