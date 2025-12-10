#!/usr/bin/env python3
"""
Simple Target Selector Node - Test Version
Subscribes to detected loops and selects rightmost one
"""

import rclpy
from rclpy.node import Node
from parachute_interfaces.msg import DetectedLoops, DetectedLoop
from parachute_interfaces.srv import RequestNextTarget

class TargetSelectorNode(Node):
    def __init__(self):
        super().__init__('target_selector_node')

        # Declare and get test_mode parameter
        self.declare_parameter('test_mode', True)
        self.test_mode = self.get_parameter('test_mode').value
        
        # Subscriber for detected loops
        self.subscription = self.create_subscription(DetectedLoops, '/detected_loops', self.loops_callback, 10)
        
        # Service to provide next target
        self.service = self.create_service(RequestNextTarget, '/request_next_target', self.request_target_callback)
        
        self.current_loops = []
        self.get_logger().info('Target Selector Node initialized')
        
    def loops_callback(self, msg):
        """Store the latest detected loops"""
        self.current_loops = msg.loops
        self.get_logger().info(f'Received {len(self.current_loops)} loops')
        
    def request_target_callback(self, request, response):
        """Service callback - return rightmost loop"""
        if not self.current_loops:
            response.target_available = False
            response.target_loop = DetectedLoop()
            response.message = "No loops detected yet"
            self.get_logger().warn('No loops available for target selection')
            return response
        
        # Select rightmost loop (highest x value)
        rightmost_loop = max(self.current_loops, 
                            key=lambda loop: loop.pose.pose.position.x)
        
        response.target_available = True
        response.target_loop = rightmost_loop
        response.message = f"Selected loop {rightmost_loop.loop_id}"
        
        if self.test_mode:
            self.get_logger().info(f'TEST: Selected target: Loop {rightmost_loop.loop_id} at x={rightmost_loop.pose.pose.position.x:.3f}')
        else:
            self.get_logger().info(f'Selected target: Loop {rightmost_loop.loop_id}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TargetSelectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
