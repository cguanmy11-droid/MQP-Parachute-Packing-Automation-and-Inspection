#!/usr/bin/env python3
"""
Xbox Controller for WidowX-200 Arm
Based on interbotix_xsarm_joy patterns
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Pose, Point, Quaternion
from parachute_interfaces.action import ExecuteTrajectory
from rclpy.action import ActionClient
import time

class XboxArmController(Node):
    def __init__(self):
        super().__init__('xbox_arm_controller')
        
        # Subscribe to joy topic
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Publishers for arm commands
        self.pose_cmd_pub = self.create_publisher(
            String,
            '/main_arm/pose_command',
            10
        )
        self.gripper_cmd_pub = self.create_publisher(
            String,
            '/main_arm/gripper_command',
            10
        )
        self.gripper_effort_pub = self.create_publisher(
            Float32,
            '/main_arm/gripper_effort',
            10
        )
        self.gripper_pos_pub = self.create_publisher(
            Float32,
            '/main_arm/gripper_position',
            10
        )
        
        # Action client for trajectories
        self.traj_action_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/main_arm/execute_trajectory'
        )
        
        # State tracking
        self.last_buttons = [0] * 15  # Xbox controller has ~15 buttons
        self.gripper_effort = 0.5
        self.gripper_position = 0.0  # 0.0=open, 1.0=closed
        self.gripper_step_per_sec = 0.6
        self._last_gripper_update_time = time.time()
        
        # Control mode
        self.control_mode = 'pose'  # 'pose' or 'cartesian'
        
        self.get_logger().info('🎮 Xbox Arm Controller initialized')
        self.get_logger().info('Button mapping:')
        self.get_logger().info('  A (0): Home pose')
        self.get_logger().info('  B (1): Sleep pose')
        self.get_logger().info('  X (2): Upright pose')
        self.get_logger().info('  Y (3): Toggle control mode')
        self.get_logger().info('  RB (5): Hold to close gripper (position mode)')
        self.get_logger().info('  Back (6): Hold to open gripper (position mode)')
        self.get_logger().info('  Start (7): Emergency sleep')
        
    def joy_callback(self, msg):
        """Process joystick input"""
        
        # Button indices for Xbox controller (may vary)
        # buttons = [A, B, X, Y, LB, RB, Back, Start, ...]
        buttons = msg.buttons
        # Detect button presses (rising edge)
        for i in range(min(len(buttons), len(self.last_buttons))):
            if buttons[i] == 1 and self.last_buttons[i] == 0:
                self.handle_button_press(i)

        # Long-press control for gripper travel.
        self.update_gripper_position_hold(buttons)
        
        # Handle analog sticks for Cartesian control
        if self.control_mode == 'cartesian':
            # Left stick: X/Y movement
            # Right stick: Z movement / rotation
            # Triggers: Fine control
            pass  # Implement if needed
        
        self.last_buttons = list(buttons)

    def update_gripper_position_hold(self, buttons):
        """Use RB/Back long-press to continuously change gripper position."""
        now = time.time()
        dt = max(0.0, now - self._last_gripper_update_time)
        self._last_gripper_update_time = now

        close_pressed = len(buttons) > 5 and buttons[5] == 1  # RB
        open_pressed = len(buttons) > 6 and buttons[6] == 1   # Back

        if close_pressed and not open_pressed:
            self.gripper_position = min(1.0, self.gripper_position + self.gripper_step_per_sec * dt)
            self.publish_gripper_position()
        elif open_pressed and not close_pressed:
            self.gripper_position = max(0.0, self.gripper_position - self.gripper_step_per_sec * dt)
            self.publish_gripper_position()

    def publish_gripper_position(self):
        msg = Float32()
        msg.data = float(self.gripper_position)
        self.gripper_pos_pub.publish(msg)
    
    def handle_button_press(self, button_index):
        """Handle specific button presses"""
        
        if button_index == 0:  # A button - Home
            self.get_logger().info('Going to HOME pose')
            msg = String()
            msg.data = 'home'
            self.pose_cmd_pub.publish(msg)
            
        elif button_index == 1:  # B button - Sleep
            self.get_logger().info('Going to SLEEP pose')
            msg = String()
            msg.data = 'sleep'
            self.pose_cmd_pub.publish(msg)
            
        elif button_index == 2:  # X button - Upright
            self.get_logger().info('Ging to UPRIGHT pose')
            msg = String()
            msg.data = 'upright'
            self.pose_cmd_pub.publish(msg)
            
        elif button_index == 3:  # Y button - Toggle mode
            self.control_mode = 'cartesian' if self.control_mode == 'pose' else 'pose'
            self.get_logger().info(f'Control mode: {self.control_mode}')
            
        elif button_index == 5:  # RB
            self.get_logger().info('Hold RB to close gripper gradually')

        elif button_index == 6:  # Back
            self.get_logger().info('Hold Back to open gripper gradually')
            
        elif button_index == 7:  # Start - Emergency sleep
            self.get_logger().warn('EMERGENCY SLEEP!')
            msg = String()
            msg.data = 'sleep'
            self.pose_cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = XboxArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()