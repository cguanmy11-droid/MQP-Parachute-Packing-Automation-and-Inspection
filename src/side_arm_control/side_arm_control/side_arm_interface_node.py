#!/usr/bin/env python3
"""
Simple Side Arm Interface Node - Test Version
Provides action server for hook insertion
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from parachute_interfaces.action import InsertHook
from parachute_interfaces.srv import RotateHook
from parachute_interfaces.msg import HookStatus
from geometry_msgs.msg import Point
import time

class SideArmInterfaceNode(Node):
    def __init__(self):
        super().__init__('side_arm_interface_node')

        # Declare and get test_mode parameter
        self.declare_parameter('test_mode', True)
        self.test_mode = self.get_parameter('test_mode').value
        
        # Action server for inserting hook
        self.action_server = ActionServer(self, InsertHook, '/side_arm/insert_hook', self.insert_hook_callback)
        
        # Subscriber for hook rotation
        # Service for rotating hook
        self.rotate_service = self.create_service(RotateHook, '/side_arm/rotate_hook', self.rotate_hook_callback)

        # Publisher for hook status
        self.status_publisher = self.create_publisher(HookStatus, '/side_arm/status', 10)
        
        # Timer to publish status
        self.timer = self.create_timer(1.0, self.publish_status)
        self.current_state = HookStatus.STATE_IDLE
        self.current_angle = 0.0
        
        if self.test_mode:
            self.get_logger().info('TEST MODE: Side Arm Interface Node initialized')
        else:
            self.get_logger().info('Side Arm Interface Node initialized')
        
    def publish_status(self):
        """Publish current hook status"""
        msg = HookStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.current_state
        msg.position = Point(x=0.0, y=0.0, z=0.0)
        msg.is_inserted = (self.current_state == HookStatus.STATE_INSERTED)
        msg.is_retracted = (self.current_state == HookStatus.STATE_IDLE)
        
        self.status_publisher.publish(msg)
    
    def rotate_hook_callback(self, request, response):
        """Service callback - rotate hook by specified angle"""
        angle = request.angle_degrees
        
        if self.test_mode:
            self.get_logger().info(f'TEST: Rotating hook by {angle}°')
        else:
            self.get_logger().info(f'Rotating hook by {angle}°')
        
        # Simulate rotation time (proportional to angle)
        rotation_time = abs(angle) / 90.0  # 1 second per 90 degrees
        time.sleep(rotation_time)
        
        # Update current angle
        self.current_angle += angle
        
        response.success = True
        response.final_angle = self.current_angle
        response.message = f"Rotated to {self.current_angle}°"
        
        if self.test_mode:
            self.get_logger().info(f'TEST: Hook rotated to {self.current_angle}°')
        else:
            self.get_logger().info(f'Hook rotated to {self.current_angle}°')
        
        return response
        
    def insert_hook_callback(self, goal_handle):
        """Action callback - simulate hook insertion"""
        # Get the target loop from the goal request
        target_loop = goal_handle.request.target_loop

        if self.test_mode:
            self.get_logger().info(f'TEST: Inserting hook into loop {target_loop.loop_id}')
        else:
            self.get_logger().info(f'Inserting hook into loop {target_loop.loop_id}')
        
        feedback_msg = InsertHook.Feedback()
        
        # Simulate the insertion process with feedback
        states = [
            (InsertHook.Feedback.STATE_APPROACHING, 0.25, "Approaching loop"),
            (InsertHook.Feedback.STATE_ALIGNING, 0.50, "Aligning with loop"),
            (InsertHook.Feedback.STATE_INSERTING, 0.75, "Inserting hook"),
            (InsertHook.Feedback.STATE_VERIFYING, 1.0, "Verifying insertion")
        ]
        
        for state, progress, description in states:
            feedback_msg.current_state = state
            feedback_msg.progress = progress
            feedback_msg.current_position = Point(x=0.1, y=0.0, z=0.5)
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Progress: {int(progress * 100)}%')
            time.sleep(1)  # Simulate work

            if self.test_mode:
                self.get_logger().info(f'  TEST: {description} - {int(progress * 100)}%')
            
            time.sleep(1.0 if self.test_mode else 0.5)  # Longer delays in test mode for visibility
        
        
        # Mark as successful
        goal_handle.succeed()
        
        result = InsertHook.Result()
        result.success = True
        result.final_hook_position = goal_handle.request.target_loop.pose.pose.position
        result.execution_time = 4.0
        result.message = "Hook inserted successfully"
        
        self.current_state = HookStatus.STATE_INSERTED
        if self.test_mode:
            self.get_logger().info('TEST: Hook insertion complete!')
        else:
            self.get_logger().info('Hook insertion complete')
        
        return result

def main(args=None):
    rclpy.init(args=args)
    node = SideArmInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    