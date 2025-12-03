#!/usr/bin/env python3
"""
Main Arm Planner Node
Handles advanced motion planning with coordinates, trajectories, and joint angles
Provides smooth motion planning and execution
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import JointState
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np


class MainArmPlannerNode(Node):
    def __init__(self):
        super().__init__('main_arm_planner_node')
        
        # Parameters
        self.declare_parameter('robot_model', 'wx200')
        self.declare_parameter('robot_name', 'wx200')
        robot_model = self.get_parameter('robot_model').value
        robot_name = self.get_parameter('robot_name').value
        
        # Initialize robot
        self.get_logger().info(f'Initializing {robot_model} arm planner...')
        self.bot = InterbotixManipulatorXS(
            robot_model=robot_model,
            robot_name=robot_name,
            moving_time=3.0,
            accel_time=1.0
        )
        
        # Subscribers for different command types
        self.point_sub = self.create_subscription(Point, '/main_arm/target_point', self.target_point_callback, 10)
        
        self.pose_sub = self.create_subscription(Pose, '/main_arm/target_pose', self.target_pose_callback, 10)
        
        self.joint_angles_sub = self.create_subscription(JointState, '/main_arm/target_joint_angles', self.target_joint_angles_callback, 10)
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/main_arm/planner_status', 10)
        
        self.current_joints_pub = self.create_publisher(JointState, '/main_arm/current_joint_angles', 10)
        
        # Workspace limits (meters)
        self.workspace_limits = {
            'x': (0.1, 0.45),
            'y': (-0.3, 0.3),
            'z': (0.05, 0.4)
        }
        
        # Timer for publishing current state
        self.create_timer(0.1, self.publish_current_state)
        
        self.get_logger().info('Main Arm Planner Node ready!')
        self.get_logger().info('Subscribed to:')
        self.get_logger().info('  - /main_arm/target_point (Point: x,y,z)')
        self.get_logger().info('  - /main_arm/target_pose (Pose: position + orientation)')
        self.get_logger().info('  - /main_arm/target_joint_angles (JointState)')
    
    def target_point_callback(self, msg):
        """Move end-effector to target XYZ point"""
        x, y, z = msg.x, msg.y, msg.z
        self.get_logger().info(f'Moving to point: ({x:.3f}, {y:.3f}, {z:.3f})')
        
        # Check workspace limits
        if not self.check_workspace_limits(x, y, z):
            self.get_logger().error('Target point outside workspace limits!')
            self.publish_status_msg('error: out_of_workspace')
            return
        
        try:
            self.publish_status_msg('planning')
            
            # Move to target point (maintain current orientation)
            success = self.bot.arm.set_ee_pose_components(x=x, y=y, z=z)
            
            if success:
                self.get_logger().info('Successfully reached target point')
                self.publish_status_msg('success: point_reached')
            else:
                self.get_logger().warn('Could not reach target point (IK failed)')
                self.publish_status_msg('warning: ik_failed')
                
        except Exception as e:
            self.get_logger().error(f'Failed to move to point: {e}')
            self.publish_status_msg('error: motion_failed')
    
    def target_pose_callback(self, msg):
        """Move end-effector to target pose (position + orientation)"""
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        
        # Extract orientation (quaternion)
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        self.get_logger().info(f'Moving to pose: pos({x:.3f}, {y:.3f}, {z:.3f})')
        
        if not self.check_workspace_limits(x, y, z):
            self.get_logger().error('Target pose outside workspace limits!')
            self.publish_status_msg('error: out_of_workspace')
            return
        
        try:
            self.publish_status_msg('planning')
            
            # Convert quaternion to roll, pitch, yaw
            roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
            
            # Move to target pose
            success = self.bot.arm.set_ee_pose_components(
                x=x, y=y, z=z,
                roll=roll, pitch=pitch
            )
            
            if success:
                self.get_logger().info('Successfully reached target pose')
                self.publish_status_msg('success: pose_reached')
            else:
                self.get_logger().warn('Could not reach target pose (IK failed)')
                self.publish_status_msg('warning: ik_failed')
                
        except Exception as e:
            self.get_logger().error(f'Failed to move to pose: {e}')
            self.publish_status_msg('error: motion_failed')
    
    def target_joint_angles_callback(self, msg):
        """Move to target joint angles directly"""
        joint_positions = list(msg.position)
        
        self.get_logger().info(f'Moving to joint angles: {[f"{j:.3f}" for j in joint_positions]}')
        
        try:
            self.publish_status_msg('executing')
            
            # Move joints to target positions
            self.bot.arm.set_joint_positions(joint_positions)
            
            self.get_logger().info('Successfully reached target joint angles')
            self.publish_status_msg('success: joints_reached')
            
        except Exception as e:
            self.get_logger().error(f'Failed to move joints: {e}')
            self.publish_status_msg('error: joint_motion_failed')
    
    def check_workspace_limits(self, x, y, z):
        """Check if point is within workspace limits"""
        x_ok = self.workspace_limits['x'][0] <= x <= self.workspace_limits['x'][1]
        y_ok = self.workspace_limits['y'][0] <= y <= self.workspace_limits['y'][1]
        z_ok = self.workspace_limits['z'][0] <= z <= self.workspace_limits['z'][1]
        
        return x_ok and y_ok and z_ok
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp) if abs(sinp) <= 1 else np.pi / 2
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def publish_current_state(self):
        """Publish current joint angles"""
        try:
            joint_positions = self.bot.arm.get_joint_commands()
            
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
            joint_state.position = joint_positions
            
            self.current_joints_pub.publish(joint_state)
        except:
            pass
    
    def publish_status_msg(self, status):
        """Publish status message"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down Main Arm Planner')
        try:
            self.bot.arm.go_to_sleep_pose()
            self.bot.shutdown()
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MainArmPlannerNode()
    
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
