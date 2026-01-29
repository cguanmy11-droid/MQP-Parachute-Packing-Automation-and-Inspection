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
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Float32
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import time

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

        self.test_mode = self.get_parameter('test_mode').value
        self.use_sim = self.get_parameter('use_sim').value
        robot_model = self.get_parameter('robot_model').value
        robot_name = self.get_parameter('robot_name').value
        moving_time = self.get_parameter('moving_time').value
        accel_time = self.get_parameter('accel_time').value
        
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
        self.gripper_effort_sub = self.create_subscription(Float32, '/main_arm/gripper_effort', self.gripper_effort_callback, 10)
        # Add this subscriber for testing positions
        self.test_position_sub = self.create_subscription(Pose, '/main_arm/test_position', self.test_position_callback, 10)
        
        # Publisher for arm and pose status
        self.status_publisher = self.create_publisher(ArmStatus, '/main_arm/status', 10)
        self.simple_status_publisher = self.create_publisher(String, '/main_arm/simple_status', 10)
        self.pose_publisher = self.create_publisher(Pose, '/main_arm/current_pose', 10)
        
        # Timer to publish status
        self.timer = self.create_timer(1.0, self.publish_status)

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
                self.pose_pub.publish(pose_msg)
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
                    success = self.bot.arm.set_ee_pose_components(
                        x=waypoint.position.x,
                        y=waypoint.position.y,
                        z=waypoint.position.z
                    )
                    if not success:
                        self.get_logger().warn(f'Waypoint {i+1} unreachable, skipping')
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
        """Handle gripper open/close commands"""
        command = msg.data.lower().strip()
        self.get_logger().info(f'Received gripper command: {command}')
        
        if self.bot is None and not self.use_sim:
            self.get_logger().warn('No robot available')
            return
        
        if self.use_sim:
            self.get_logger().warn('Gripper commands not yet implemented for simulation')
            return
        
        try:
            if command == 'open':
                self.bot.gripper.open()
                self.get_logger().info('Gripper opened')
                
            elif command == 'close':
                self.bot.gripper.close(effort=self.gripper_effort)
                self.get_logger().info(f'Gripper closed with effort {self.gripper_effort:.2f}')
                
            else:
                self.get_logger().warn(f'Unknown gripper command: {command}')
                
        except Exception as e:
            self.get_logger().error(f'Gripper command failed: {e}')
    
    def gripper_effort_callback(self, msg):
        """Update gripper effort setting"""
        self.gripper_effort = max(0.0, min(1.0, msg.data))
        self.get_logger().info(f'Gripper effort set to: {self.gripper_effort:.2f}')
    
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()