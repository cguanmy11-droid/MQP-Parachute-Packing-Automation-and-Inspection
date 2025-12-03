#!/usr/bin/env python3
"""
Simple Packing Coordinator Node - Test Version
Orchestrates the packing sequence
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from parachute_interfaces.srv import RequestNextTarget, RotateHook
from parachute_interfaces.action import InsertHook, ExecuteTrajectory
from geometry_msgs.msg import Pose, Point, Quaternion
import threading

class PackingCoordinatorNode(Node):
    def __init__(self):
        super().__init__('packing_coordinator_node')

        # Declare and get test_mode parameter
        self.declare_parameter('test_mode', False)
        self.declare_parameter('start_delay', 3.0)
        self.test_mode = self.get_parameter('test_mode').value
        start_delay = self.get_parameter('start_delay').value
        
        # Service client for target selection
        self.target_client = self.create_client(RequestNextTarget, '/request_next_target')
        self.rotate_client = self.create_client(RotateHook, '/side_arm/rotate_hook')
        
        # Action clients
        self.hook_action_client = ActionClient(self, InsertHook, '/side_arm/insert_hook')
        self.arm_action_client = ActionClient(self, ExecuteTrajectory, '/main_arm/execute_trajectory')
        
        # Wait for services/actions to be available
        self.get_logger().info('Waiting for services and actions...')
        self.target_client.wait_for_service(timeout_sec=5.0)
        self.rotate_client.wait_for_service(timeout_sec=5.0)
        self.hook_action_client.wait_for_server(timeout_sec=5.0)
        self.arm_action_client.wait_for_server(timeout_sec=5.0)
        
        self.get_logger().info('Packing Coordinator Node initialized')
        
        # Start the sequence after 3 seconds
        self.timer = self.create_timer(3.0, self.request_target)
        self.sequence_running = False
        self.current_target_loop = None
        self.stow_ready = False

    def request_target(self):
        """Step 1: Request target loop"""
        if self.sequence_running:
            return
        self.sequence_running = True
        self.timer.cancel()
        
        self.get_logger().info('=== Starting Packing Sequence ===')
        self.get_logger().info('Step 1: Requesting target loop')
        
        target_request = RequestNextTarget.Request()
        self.get_logger().info('DEBUG: About to call service')
        future = self.target_client.call_async(target_request)
        self.get_logger().info('DEBUG: Service called, adding callback')
        future.add_done_callback(self.target_response_callback)
        self.get_logger().info('DEBUG: Callback added')
    
    def target_response_callback(self, future):
        """Callback when target selection completes"""
        try:
            response = future.result()
            self.get_logger().info(f'DEBUG: Got response, target_available={response.target_available}')
            
            if not response.target_available:
                self.get_logger().error('No target available!')
                return
            
            self.current_target_loop = response.target_loop
            self.get_logger().info(f'Target selected: Loop {self.current_target_loop.loop_id}')
            
            # Move to insert hook step
            self.get_logger().info('DEBUG: About to call step2_insert_hook')
            self.insert_hook()
            
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def insert_hook(self):
        """Step 2: Insert hook through loop"""
        self.get_logger().info('Step 2: Inserting hook through loop')
        
        hook_goal = InsertHook.Goal()
        hook_goal.target_loop = self.current_target_loop
        
        send_goal_future = self.hook_action_client.send_goal_async(
            hook_goal,
            feedback_callback=self.hook_feedback_callback
        )
        send_goal_future.add_done_callback(self.hook_goal_response_callback)
    
    def hook_feedback_callback(self, feedback_msg):
        """Receive feedback from hook insertion"""
        feedback = feedback_msg.feedback
        if self.test_mode:
            self.get_logger().info(f'  Hook insertion progress: {int(feedback.progress * 100)}%')
    
    def hook_goal_response_callback(self, future):
        """Callback when hook action goal is accepted/rejected"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Hook insertion goal rejected!')
            return
        
        self.get_logger().info('Hook insertion goal accepted')
        
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.hook_result_callback)

    def hook_result_callback(self, future):
        """Callback when hook insertion completes"""
        result = future.result().result
        
        if result.success:
            self.get_logger().info('Hook inserted successfully')
            # Move to next step of rotate hook
            self.rotate_hook()
        else:
            self.get_logger().error('Hook insertion failed!')
    
    def rotate_hook(self):
        """Intermediate Step: Rotate hook 90 degrees before stowing"""
        self.get_logger().info('Intermediate Step: Rotating hook 90 degrees before stowing')
        
        rotate_request = RotateHook.Request()
        rotate_request.angle_degrees = 90.0
        
        future = self.rotate_client.call_async(rotate_request)
        future.add_done_callback(self.rotate_hook_callback)
    
    def rotate_hook_callback(self, future):
        """Callback when pre-stow rotation completes"""
        try:
            response = future.result()
            
            if response.success:
                self.get_logger().info(f'Hook rotated to {response.final_angle}°')
                # Move to next step either trajectory or retract
                if self.stow_ready:
                    self.retract_hook()
                else:
                    self.execute_trajectory()
            else:
                self.get_logger().error('Hook rotation failed!')
                
        except Exception as e:
            self.get_logger().error(f'Rotation service call failed: {e}')
    
    def execute_trajectory(self):
        """Step 3: Execute main arm trajectory"""
        self.get_logger().info('Step 3: Executing stowing trajectory')
        
        # Create dummy waypoints for testing
        arm_goal = ExecuteTrajectory.Goal()
        arm_goal.waypoints = []
        for i in range(3):
            waypoint = Pose()
            waypoint.position = Point(x=0.0 + i * 0.1, y=0.0, z=0.5)
            waypoint.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            arm_goal.waypoints.append(waypoint)
        arm_goal.speed_factor = 0.5
        
        send_goal_future = self.arm_action_client.send_goal_async(
            arm_goal,
            feedback_callback=self.arm_feedback_callback
        )
        send_goal_future.add_done_callback(self.arm_goal_response_callback)
    
    def arm_feedback_callback(self, feedback_msg):
        """Receive feedback from trajectory execution"""
        feedback = feedback_msg.feedback
        if self.test_mode:
            self.get_logger().info(f'  Trajectory progress: {int(feedback.progress * 100)}%')
    
    def arm_goal_response_callback(self, future):
        """Callback when arm action goal is accepted/rejected"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Arm trajectory goal rejected!')
            return
        
        self.get_logger().info('Arm trajectory goal accepted')
        
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.arm_result_callback)
    
    def arm_result_callback(self, future):
        """Callback when trajectory execution completes"""
        result = future.result().result
        
        if result.success:
            self.get_logger().info('Trajectory executed successfully')
            # Move to next rotation step
            self.stow_ready = True
            self.rotate_hook()
        else:
            self.get_logger().error('Trajectory execution failed!')
    
    def retract_hook(self):
        """Step 4: Retract hook to stow position"""
        self.get_logger().info('Step 4: Retracting hook')
        
        # You could use another service here for retract, or another rotate call
        # For now, using rotation as placeholder
        rotate_request = RotateHook.Request()
        rotate_request.angle_degrees = 0.0  # Just a placeholder movement
        
        future = self.rotate_client.call_async(rotate_request)
        future.add_done_callback(self.retract_complete_callback)
    
    def retract_complete_callback(self, future):
        """Callback when hook retraction completes"""
        try:
            response = future.result()
            self.get_logger().info('Hook retracted')
            self.get_logger().info('=== Packing Sequence Complete ===')
                
        except Exception as e:
            self.get_logger().error(f'Retract call failed: {e}')

def main(args=None):

    rclpy.init(args=args)
    node = PackingCoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()