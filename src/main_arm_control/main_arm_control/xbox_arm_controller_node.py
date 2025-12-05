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
        
        # Action client for trajectories
        self.traj_action_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/main_arm/execute_trajectory'
        )
        
        # State tracking
        self.last_buttons = [0] * 15  # Xbox controller has ~15 buttons
        self.gripper_effort = 0.5
        
        # Control mode
        self.control_mode = 'pose'  # 'pose' or 'cartesian'
        
        self.get_logger().info('🎮 Xbox Arm Controller initialized')
        self.get_logger().info('Button mapping:')
        self.get_logger().info('  A (0): Home pose')
        self.get_logger().info('  B (1): Sleep pose')
        self.get_logger().info('  X (2): Upright pose')
        self.get_logger().info('  Y (3): Toggle control mode')
        self.get_logger().info('  LB (4): Open gripper')
        self.get_logger().info('  RB (5): Close gripper')
        self.get_logger().info('  Start (7): Emergency sleep')
        
    def joy_callback(self, msg):
        """Process joystick input"""
        
        # Button indices for Xbox controller (may vary)
        # buttons = [A, B, X, Y, LB, RB, Back, Start, ...]
        buttons = msg.buttons
        axes = msg.axes
        
        # Detect button presses (rising edge)
        for i in range(min(len(buttons), len(self.last_buttons))):
            if buttons[i] == 1 and self.last_buttons[i] == 0:
                self.handle_button_press(i)
        
        # Handle analog sticks for Cartesian control
        if self.control_mode == 'cartesian':
            # Left stick: X/Y movement
            # Right stick: Z movement / rotation
            # Triggers: Fine control
            pass  # Implement if needed
        
        self.last_buttons = list(buttons)
    
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
            
        elif button_index == 4:  # LB - Open gripper
            self.get_logger().info('Opening gripper')
            msg = String()
            msg.data = 'open'
            self.gripper_cmd_pub.publish(msg)
            
        elif button_index == 5:  # RB - Close gripper
            self.get_logger().info('Closing gripper')
            msg = String()
            msg.data = 'close'
            self.gripper_cmd_pub.publish(msg)
            
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