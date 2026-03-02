#!/usr/bin/env python3
"""
Main Arm Teleoperation Node
Bridge to Interbotix joystick control
Allows switching between autonomous and manual control
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import subprocess
import signal
import os


class MainArmTeleopNode(Node):
    def __init__(self):
        super().__init__('main_arm_teleop_node')
        
        # Parameters
        self.declare_parameter('robot_model', 'wx200')
        self.declare_parameter('controller_type', 'xboxone')
        self.declare_parameter('auto_start', False)
        
        robot_model = self.get_parameter('robot_model').value
        controller_type = self.get_parameter('controller_type').value
        auto_start = self.get_parameter('auto_start').value
        
        # Subscriber to enable/disable teleop
        self.enable_sub = self.create_subscription(Bool, '/main_arm/enable_teleop', self.enable_teleop_callback, 10)
        
        # Publisher for teleop status
        self.status_pub = self.create_publisher(String, '/main_arm/teleop_status', 10)
        
        # Teleop process
        self.teleop_process = None
        self.is_teleop_active = False
        self.robot_model = robot_model
        self.controller_type = controller_type
        
        self.get_logger().info('Main Arm Teleoperation Node ready!')
        self.get_logger().info(f'Robot: {robot_model}, Controller: {controller_type}')
        self.get_logger().info('Publish Bool to /main_arm/enable_teleop to control')
        
        if auto_start:
            self.get_logger().info('Auto-starting teleoperation...')
            self.start_teleop()
    
    def enable_teleop_callback(self, msg):
        """Enable or disable teleoperation"""
        if msg.data and not self.is_teleop_active:
            self.start_teleop()
        elif not msg.data and self.is_teleop_active:
            self.stop_teleop()
    
    def start_teleop(self):
        """Start the Interbotix joystick control"""
        if self.is_teleop_active:
            self.get_logger().warn('Teleoperation already active')
            return
        
        self.get_logger().info('Starting teleoperation...')
        
        try:
            # Launch interbotix joystick node
            cmd = [
                'ros2', 'launch',
                'interbotix_xsarm_joy', 'xsarm_joy.launch.py',
                f'robot_model:={self.robot_model}',
                f'controller:={self.controller_type}'
            ]
            
            self.teleop_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Create new process group
            )
            
            self.is_teleop_active = True
            self.get_logger().info('Teleoperation started successfully')
            self.publish_status('active')
            
        except Exception as e:
            self.get_logger().error(f'Failed to start teleoperation: {e}')
            self.publish_status('error')
    
    def stop_teleop(self):
        """Stop the Interbotix joystick control"""
        if not self.is_teleop_active:
            self.get_logger().warn('Teleoperation not active')
            return
        
        self.get_logger().info('Stopping teleoperation...')
        
        try:
            if self.teleop_process:
                # Send SIGTERM to entire process group
                os.killpg(os.getpgid(self.teleop_process.pid), signal.SIGTERM)
                self.teleop_process.wait(timeout=5)
                self.teleop_process = None
            
            self.is_teleop_active = False
            self.get_logger().info('Teleoperation stopped')
            self.publish_status('inactive')
            
        except Exception as e:
            self.get_logger().error(f'Failed to stop teleoperation: {e}')
    
    def publish_status(self, status):
        """Publish teleoperation status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down teleoperation node')
        if self.is_teleop_active:
            self.stop_teleop()


def main(args=None):
    rclpy.init(args=args)
    node = MainArmTeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
