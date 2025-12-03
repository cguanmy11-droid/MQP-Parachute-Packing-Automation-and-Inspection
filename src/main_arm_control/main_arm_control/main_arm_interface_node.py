#!/usr/bin/env python3
"""
Simple Main Arm Interface Node - Test Version
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
        self.test_mode = self.get_parameter('test_mode').value
        self.use_sim = self.get_parameter('use_sim').value
        robot_model = self.get_parameter('robot_model').value
        robot_name = self.get_parameter('robot_name').value

        # Determine if we should initialize robot, not in test or simulation mode
        self.init_robot = not self.test_mode and not self.use_sim
        
        # Initialize robot (only if not in test mode)
        if self.init_robot:
            self.get_logger().info(f'Initializing {robot_model} arm...')
            try:
                self.bot = InterbotixManipulatorXS(
                    robot_model=robot_model,
                    robot_name=robot_name,
                    moving_time=2.0,
                    accel_time=0.5
                )
                self.get_logger().info('Robot arm initialized successfully')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize robot: {e}')
                self.bot = None
        else:
            if self.use_sim:
                self.get_logger().info('Running in SIMULATION MODE')
                self.get_logger().info('Robot controlled via RViz simulation')
            elif self.test_mode:
                self.get_logger().info('Running in TEST MODE - no robot initialization')
            self.bot = None

        # Action server for trajectory execution
        self.action_server = ActionServer(self, ExecuteTrajectory, '/main_arm/execute_trajectory', self.execute_trajectory_callback)
        
        # Subscribers for simple pose and gripper commands
        self.pose_cmd_sub = self.create_subscription(String, '/main_arm/pose_command', self.pose_command_callback, 10)
        self.pose_cmd_sub = self.create_subscription(String, '/main_arm/gripper_command', self.gripper_command_callback, 10)
        self.gripper_effort_sub = self.create_subscription(Float32, '/main_arm/gripper_effort', self.gripper_effort_callback, 10)

        # Publisher for arm and pose status
        self.status_publisher = self.create_publisher(ArmStatus, '/main_arm/status', 10)
        self.simple_status_publisher = self.create_publisher(String, '/main_arm/simple_status', 10)
        self.pose_publisher = self.create_publisher(Pose, '/main_arm/current_pose', 10)
        
        # Timer to publish status
        self.timer = self.create_timer(1.0, self.publish_status)

        # State tracking
        self.current_state = ArmStatus.STATE_IDLE
        self.current_pose_name = 'unknown'
        self.gripper_effort = 0.5
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

        # Also publish simple status fof other nodes
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

        if self.bot is None and not self.test_mode:
            self.get_logger().warn('No robot available (test mode or initialization failed)')
            return
        
        # In simulation mode, we can't use the Python API
        if self.use_sim:
            self.get_logger().warn('Pose commands not yet implemented for simulation mode')
            self.get_logger().info('Tip: Use RViz interactive markers or joint state publisher')
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
                # Custom upright pose
                self.bot.arm.set_joint_positions([0, 0, 0, 0, 0])
                self.current_pose_name = 'upright'
                
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