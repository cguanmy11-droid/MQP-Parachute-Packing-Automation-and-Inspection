#!/usr/bin/env python3
"""
Packing Coordinator Node

Orchestrates the full packing sequence:
1. Request target loop from perception
2. Insert hook through loop (side arm)
3. Rotate hook to prepare for stowing
4. Execute stowing trajectory (main arm) using motion patterns
5. Rotate hook to release
6. Retract and repeat

Motion patterns are loaded from config/motion_patterns/ directory
or use built-in patterns for common movements.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from parachute_interfaces.srv import RequestNextTarget, RotateHook
from parachute_interfaces.action import InsertHook, ExecuteTrajectory
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
import threading
import os

from .motion_pattern_manager import MotionPatternManager, BUILTIN_PATTERNS


class PackingCoordinatorNode(Node):
    def __init__(self):
        super().__init__('packing_coordinator_node')

        # Declare parameters
        self.declare_parameter('test_mode', False)
        self.declare_parameter('start_delay', 3.0)
        self.declare_parameter('auto_start', False)  # Whether to auto-start sequence
        self.declare_parameter('stow_pattern', 'stow_arc')  # Default stowing pattern
        self.declare_parameter('pattern_dir', '')  # Custom pattern directory (optional)

        self.test_mode = self.get_parameter('test_mode').value
        start_delay = self.get_parameter('start_delay').value
        auto_start = self.get_parameter('auto_start').value
        pattern_dir = self.get_parameter('pattern_dir').value

        # Initialize motion pattern manager
        pattern_dir = pattern_dir if pattern_dir else None
        self.pattern_manager = MotionPatternManager(
            pattern_dir=pattern_dir,
            logger=self.get_logger()
        )

        # Add built-in patterns
        for name, pattern in BUILTIN_PATTERNS.items():
            self.pattern_manager.patterns[name] = pattern
        self.get_logger().info(f"Available patterns: {self.pattern_manager.list_patterns()}")

        # Service client for target selection
        self.target_client = self.create_client(RequestNextTarget, '/request_next_target')
        self.rotate_client = self.create_client(RotateHook, '/side_arm/rotate_hook')

        # Action clients
        self.hook_action_client = ActionClient(self, InsertHook, '/side_arm/insert_hook')
        self.arm_action_client = ActionClient(self, ExecuteTrajectory, '/main_arm/execute_trajectory')

        # Command subscriber for manual triggering
        self.cmd_sub = self.create_subscription(
            String, '/packing_coordinator/command',
            self.command_callback, 10
        )

        # Status publisher
        self.status_pub = self.create_publisher(String, '/packing_coordinator/status', 10)

        # Wait for services/actions to be available (with timeout)
        self.get_logger().info('Waiting for services and actions...')
        self.services_ready = self._wait_for_services(timeout_sec=5.0)

        if not self.services_ready:
            self.get_logger().warn('Some services not available - running in limited mode')

        self.get_logger().info('Packing Coordinator Node initialized')

        # State tracking
        self.sequence_running = False
        self.current_target_loop = None
        self.stow_ready = False
        self.current_pattern = self.get_parameter('stow_pattern').value

        # Auto-start if configured
        if auto_start:
            self.timer = self.create_timer(start_delay, self.request_target)
        else:
            self.get_logger().info('Waiting for command on /packing_coordinator/command')
            self.get_logger().info('  Commands: "start", "stop", "pattern:<name>"')

    def _wait_for_services(self, timeout_sec: float) -> bool:
        """Wait for required services with timeout."""
        all_ready = True

        services = [
            (self.target_client, '/request_next_target'),
            (self.rotate_client, '/side_arm/rotate_hook'),
        ]

        for client, name in services:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().warn(f'Service {name} not available')
                all_ready = False

        actions = [
            (self.hook_action_client, '/side_arm/insert_hook'),
            (self.arm_action_client, '/main_arm/execute_trajectory'),
        ]

        for client, name in actions:
            if not client.wait_for_server(timeout_sec=timeout_sec):
                self.get_logger().warn(f'Action {name} not available')
                all_ready = False

        return all_ready

    def command_callback(self, msg: String):
        """Handle manual commands."""
        cmd = msg.data.strip().lower()

        if cmd == 'start':
            if not self.sequence_running:
                self.get_logger().info('Starting packing sequence via command')
                self.request_target()
            else:
                self.get_logger().warn('Sequence already running')

        elif cmd == 'stop':
            self.sequence_running = False
            self.get_logger().info('Sequence stopped')
            self._publish_status('stopped')

        elif cmd.startswith('pattern:'):
            pattern_name = cmd.split(':', 1)[1]
            if self.pattern_manager.get_pattern(pattern_name):
                self.current_pattern = pattern_name
                self.get_logger().info(f'Switched to pattern: {pattern_name}')
            else:
                self.get_logger().error(f'Pattern not found: {pattern_name}')
                self.get_logger().info(f'Available: {self.pattern_manager.list_patterns()}')

        elif cmd == 'patterns':
            patterns = self.pattern_manager.list_patterns()
            self.get_logger().info(f'Available patterns: {patterns}')

        elif cmd == 'status':
            self._publish_status('query')

        else:
            self.get_logger().warn(f'Unknown command: {cmd}')

    def _publish_status(self, status: str):
        """Publish current status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def request_target(self):
        """Step 1: Request target loop from perception."""
        if self.sequence_running:
            return
        self.sequence_running = True

        if hasattr(self, 'timer') and self.timer:
            self.timer.cancel()

        self.get_logger().info('=== Starting Packing Sequence ===')
        self._publish_status('requesting_target')
        self.get_logger().info('Step 1: Requesting target loop')

        if not self.target_client.service_is_ready():
            self.get_logger().error('Target service not available')
            self.sequence_running = False
            return

        target_request = RequestNextTarget.Request()
        future = self.target_client.call_async(target_request)
        future.add_done_callback(self.target_response_callback)

    def target_response_callback(self, future):
        """Callback when target selection completes."""
        try:
            response = future.result()

            if not response.target_available:
                self.get_logger().error('No target available!')
                self._publish_status('no_target')
                self.sequence_running = False
                return

            self.current_target_loop = response.target_loop
            pos = self.current_target_loop.pose.pose.position
            self.get_logger().info(
                f'Target selected: Loop {self.current_target_loop.loop_id} '
                f'at ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})'
            )

            self._publish_status('inserting_hook')
            self.insert_hook()

        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.sequence_running = False

    def insert_hook(self):
        """Step 2: Insert hook through loop."""
        self.get_logger().info('Step 2: Inserting hook through loop')

        if not self.hook_action_client.server_is_ready():
            self.get_logger().error('Hook action not available')
            self.sequence_running = False
            return

        hook_goal = InsertHook.Goal()
        hook_goal.target_loop = self.current_target_loop

        send_goal_future = self.hook_action_client.send_goal_async(
            hook_goal,
            feedback_callback=self.hook_feedback_callback
        )
        send_goal_future.add_done_callback(self.hook_goal_response_callback)

    def hook_feedback_callback(self, feedback_msg):
        """Receive feedback from hook insertion."""
        feedback = feedback_msg.feedback
        if self.test_mode:
            self.get_logger().info(f'  Hook insertion progress: {int(feedback.progress * 100)}%')

    def hook_goal_response_callback(self, future):
        """Callback when hook action goal is accepted/rejected."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Hook insertion goal rejected!')
            self.sequence_running = False
            return

        self.get_logger().info('Hook insertion goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.hook_result_callback)

    def hook_result_callback(self, future):
        """Callback when hook insertion completes."""
        result = future.result().result

        if result.success:
            self.get_logger().info('Hook inserted successfully')
            self._publish_status('rotating_hook')
            self.rotate_hook()
        else:
            self.get_logger().error('Hook insertion failed!')
            self.sequence_running = False

    def rotate_hook(self):
        """Intermediate Step: Rotate hook 90 degrees before stowing."""
        self.get_logger().info('Intermediate Step: Rotating hook 90 degrees')

        rotate_request = RotateHook.Request()
        rotate_request.angle_degrees = 90.0

        future = self.rotate_client.call_async(rotate_request)
        future.add_done_callback(self.rotate_hook_callback)

    def rotate_hook_callback(self, future):
        """Callback when pre-stow rotation completes."""
        try:
            response = future.result()

            if response.success:
                self.get_logger().info(f'Hook rotated to {response.final_angle}')
                if self.stow_ready:
                    self._publish_status('retracting')
                    self.retract_hook()
                else:
                    self._publish_status('executing_trajectory')
                    self.execute_trajectory()
            else:
                self.get_logger().error('Hook rotation failed!')
                self.sequence_running = False

        except Exception as e:
            self.get_logger().error(f'Rotation service call failed: {e}')
            self.sequence_running = False

    def execute_trajectory(self):
        """Step 3: Execute main arm trajectory using motion pattern."""
        self.get_logger().info(f'Step 3: Executing trajectory with pattern "{self.current_pattern}"')

        # Get target position for the pattern
        # For stowing, we use the loop position as reference
        target_pos = self.current_target_loop.pose.pose.position

        # Generate waypoints from pattern
        waypoints = self.pattern_manager.apply_pattern(self.current_pattern, target_pos)

        if not waypoints:
            self.get_logger().warn(f'No waypoints from pattern, using fallback')
            # Fallback: simple approach to target
            waypoints = [
                self._make_pose(target_pos.x, target_pos.y, target_pos.z + 0.1),
                self._make_pose(target_pos.x, target_pos.y, target_pos.z),
            ]

        self.get_logger().info(f'Generated {len(waypoints)} waypoints')

        # Get pattern speed factor
        pattern = self.pattern_manager.get_pattern(self.current_pattern)
        speed_factor = pattern.speed_factor if pattern else 0.5

        # Create action goal
        arm_goal = ExecuteTrajectory.Goal()
        arm_goal.waypoints = waypoints
        arm_goal.speed_factor = speed_factor

        if not self.arm_action_client.server_is_ready():
            self.get_logger().error('Arm action not available')
            self.sequence_running = False
            return

        send_goal_future = self.arm_action_client.send_goal_async(
            arm_goal,
            feedback_callback=self.arm_feedback_callback
        )
        send_goal_future.add_done_callback(self.arm_goal_response_callback)

    def _make_pose(self, x: float, y: float, z: float) -> Pose:
        """Create a Pose message."""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        return pose

    def arm_feedback_callback(self, feedback_msg):
        """Receive feedback from trajectory execution."""
        feedback = feedback_msg.feedback
        if self.test_mode:
            self.get_logger().info(f'  Trajectory progress: {int(feedback.progress * 100)}%')

    def arm_goal_response_callback(self, future):
        """Callback when arm action goal is accepted/rejected."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Arm trajectory goal rejected!')
            self.sequence_running = False
            return

        self.get_logger().info('Arm trajectory goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.arm_result_callback)

    def arm_result_callback(self, future):
        """Callback when trajectory execution completes."""
        result = future.result().result

        if result.success:
            self.get_logger().info('Trajectory executed successfully')
            self.stow_ready = True
            self._publish_status('post_stow_rotate')
            self.rotate_hook()
        else:
            self.get_logger().error('Trajectory execution failed!')
            self.sequence_running = False

    def retract_hook(self):
        """Step 4: Retract hook to complete stowing."""
        self.get_logger().info('Step 4: Retracting hook')

        # Rotate back to neutral
        rotate_request = RotateHook.Request()
        rotate_request.angle_degrees = 0.0

        future = self.rotate_client.call_async(rotate_request)
        future.add_done_callback(self.retract_complete_callback)

    def retract_complete_callback(self, future):
        """Callback when hook retraction completes."""
        try:
            response = future.result()
            self.get_logger().info('Hook retracted')
            self.get_logger().info('=== Packing Sequence Complete ===')
            self._publish_status('complete')

            # Reset for next cycle
            self.sequence_running = False
            self.stow_ready = False
            self.current_target_loop = None

        except Exception as e:
            self.get_logger().error(f'Retract call failed: {e}')
            self.sequence_running = False


def main(args=None):
    rclpy.init(args=args)
    node = PackingCoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
