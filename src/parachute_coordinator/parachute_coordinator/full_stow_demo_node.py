#!/usr/bin/env python3
"""
Full Stow Demo Node

End-to-end demonstration of the packing sequence using a predefined loop position.
No perception required - uses hardcoded or parameter-specified target.

Sequence:
1. Move side arm to target loop position
2. Insert hook (simulated or real)
3. Rotate hook 90 degrees
4. Execute main arm stowing trajectory (using motion pattern)
5. Rotate hook back
6. Retract side arm

Usage:
    ros2 run parachute_coordinator full_stow_demo_node

    # With custom target position:
    ros2 run parachute_coordinator full_stow_demo_node \
        --ros-args -p target_x:=0.35 -p target_y:=0.15 -p target_z:=0.0
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
from parachute_interfaces.srv import MoveToPosition, RotateHook
from parachute_interfaces.action import MoveToCoordinate, ExecuteTrajectory
from parachute_interfaces.msg import DetectedLoop
import time

from .motion_pattern_manager import MotionPatternManager, BUILTIN_PATTERNS


class FullStowDemoNode(Node):
    """
    Demonstrates the full stowing sequence with predefined positions.
    """

    def __init__(self):
        super().__init__('full_stow_demo_node')

        # ==================== PARAMETERS ====================
        # Target loop position (in world frame)
        self.declare_parameter('target_x', 0.35)  # meters
        self.declare_parameter('target_y', 0.15)  # meters
        self.declare_parameter('target_z', 0.0)   # meters

        # Side arm approach position (in side arm frame, mm)
        self.declare_parameter('side_arm_x_mm', 150.0)
        self.declare_parameter('side_arm_y_mm', 100.0)
        self.declare_parameter('side_arm_z_mm', 50.0)

        # Motion pattern for main arm
        self.declare_parameter('stow_pattern', 'stow_arc')

        # Timing
        self.declare_parameter('step_delay', 2.0)  # Delay between steps (seconds)
        self.declare_parameter('auto_start', True)  # Start automatically
        self.declare_parameter('start_delay', 3.0)  # Delay before starting

        # Get parameters
        self.target = Point(
            x=self.get_parameter('target_x').value,
            y=self.get_parameter('target_y').value,
            z=self.get_parameter('target_z').value
        )
        self.side_arm_pos = {
            'x': self.get_parameter('side_arm_x_mm').value,
            'y': self.get_parameter('side_arm_y_mm').value,
            'z': self.get_parameter('side_arm_z_mm').value,
        }
        self.stow_pattern = self.get_parameter('stow_pattern').value
        self.step_delay = self.get_parameter('step_delay').value

        # ==================== MOTION PATTERNS ====================
        self.pattern_manager = MotionPatternManager(logger=self.get_logger())
        for name, pattern in BUILTIN_PATTERNS.items():
            self.pattern_manager.patterns[name] = pattern

        # ==================== SERVICE CLIENTS ====================
        self.side_arm_move_client = self.create_client(
            MoveToPosition, '/side_arm/move_to_position'
        )
        self.side_arm_rotate_client = self.create_client(
            RotateHook, '/side_arm/rotate_hook'
        )

        # ==================== ACTION CLIENTS ====================
        self.side_arm_action_client = ActionClient(
            self, MoveToCoordinate, '/side_arm/move_to_coordinate'
        )
        self.main_arm_action_client = ActionClient(
            self, ExecuteTrajectory, '/main_arm/execute_trajectory'
        )

        # ==================== PUBLISHERS ====================
        self.main_arm_pose_pub = self.create_publisher(
            String, '/main_arm/pose_command', 10
        )
        self.main_arm_gripper_pub = self.create_publisher(
            String, '/main_arm/gripper_command', 10
        )
        self.status_pub = self.create_publisher(
            String, '/demo/status', 10
        )

        # ==================== STATE ====================
        self.demo_running = False
        self.current_step = 0
        self.steps = [
            ('init', self.step_init),
            ('home_main_arm', self.step_home_main_arm),
            ('move_side_arm', self.step_move_side_arm),
            ('rotate_hook_pre', self.step_rotate_hook_pre),
            ('execute_trajectory', self.step_execute_trajectory),
            ('rotate_hook_post', self.step_rotate_hook_post),
            ('retract_side_arm', self.step_retract_side_arm),
            ('complete', self.step_complete),
        ]

        # Wait for services
        self.get_logger().info('='*60)
        self.get_logger().info('FULL STOW DEMO')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Target position: ({self.target.x:.3f}, {self.target.y:.3f}, {self.target.z:.3f})')
        self.get_logger().info(f'Side arm position: X={self.side_arm_pos["x"]:.0f}, Y={self.side_arm_pos["y"]:.0f}, Z={self.side_arm_pos["z"]:.0f} mm')
        self.get_logger().info(f'Motion pattern: {self.stow_pattern}')
        self.get_logger().info('='*60)

        self._check_services()

        # Auto-start if configured
        if self.get_parameter('auto_start').value:
            start_delay = self.get_parameter('start_delay').value
            self.get_logger().info(f'Starting demo in {start_delay} seconds...')
            self.start_timer = self.create_timer(start_delay, self._start_demo)
        else:
            self.get_logger().info('Waiting for start command on /demo/command')
            self.cmd_sub = self.create_subscription(
                String, '/demo/command', self._command_callback, 10
            )

    def _check_services(self):
        """Check which services/actions are available."""
        self.get_logger().info('Checking services...')

        self.side_arm_move_available = self.side_arm_move_client.wait_for_service(timeout_sec=2.0)
        self.side_arm_rotate_available = self.side_arm_rotate_client.wait_for_service(timeout_sec=2.0)
        self.side_arm_action_available = self.side_arm_action_client.wait_for_server(timeout_sec=2.0)
        self.main_arm_action_available = self.main_arm_action_client.wait_for_server(timeout_sec=2.0)

        self.get_logger().info(f'  /side_arm/move_to_position: {"✓" if self.side_arm_move_available else "✗"}')
        self.get_logger().info(f'  /side_arm/rotate_hook: {"✓" if self.side_arm_rotate_available else "✗"}')
        self.get_logger().info(f'  /side_arm/move_to_coordinate: {"✓" if self.side_arm_action_available else "✗"}')
        self.get_logger().info(f'  /main_arm/execute_trajectory: {"✓" if self.main_arm_action_available else "✗"}')

    def _command_callback(self, msg):
        """Handle manual commands."""
        cmd = msg.data.strip().lower()
        if cmd == 'start' and not self.demo_running:
            self._start_demo()
        elif cmd == 'stop':
            self.demo_running = False
            self.get_logger().info('Demo stopped')

    def _start_demo(self):
        """Start the demo sequence."""
        if hasattr(self, 'start_timer'):
            self.start_timer.cancel()

        self.demo_running = True
        self.current_step = 0
        self._run_next_step()

    def _publish_status(self, status: str):
        """Publish current status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _run_next_step(self):
        """Execute the next step in the sequence."""
        if not self.demo_running:
            return

        if self.current_step >= len(self.steps):
            self.demo_running = False
            return

        step_name, step_func = self.steps[self.current_step]
        self.get_logger().info('')
        self.get_logger().info(f'[Step {self.current_step + 1}/{len(self.steps)}] {step_name.upper()}')
        self._publish_status(step_name)

        try:
            step_func()
        except Exception as e:
            self.get_logger().error(f'Step failed: {e}')
            self.demo_running = False

    def _step_complete(self, success: bool = True):
        """Called when a step completes."""
        if success:
            self.current_step += 1
            # Delay before next step
            self.create_timer(self.step_delay, self._run_next_step_once)
        else:
            self.get_logger().error('Step failed, stopping demo')
            self.demo_running = False

    def _run_next_step_once(self):
        """Timer callback to run next step (one-shot)."""
        # This is a hack to make a one-shot timer
        self._run_next_step()

    # ==================== DEMO STEPS ====================

    def step_init(self):
        """Initialize the demo."""
        self.get_logger().info('Initializing demo sequence...')
        self._step_complete(True)

    def step_home_main_arm(self):
        """Send main arm to home position."""
        self.get_logger().info('Sending main arm to home position...')

        msg = String()
        msg.data = 'home'
        self.main_arm_pose_pub.publish(msg)

        # Give it time to move
        self.create_timer(3.0, lambda: self._step_complete(True))

    def step_move_side_arm(self):
        """Move side arm to target position."""
        self.get_logger().info(f'Moving side arm to X={self.side_arm_pos["x"]:.0f}, Y={self.side_arm_pos["y"]:.0f}, Z={self.side_arm_pos["z"]:.0f} mm')

        if self.side_arm_move_available:
            request = MoveToPosition.Request()
            request.x_mm = self.side_arm_pos['x']
            request.y_mm = self.side_arm_pos['y']
            request.z_mm = self.side_arm_pos['z']
            request.speed_scale = 0.5

            future = self.side_arm_move_client.call_async(request)
            future.add_done_callback(self._side_arm_move_callback)
        else:
            self.get_logger().warn('Side arm move service not available, simulating...')
            self.create_timer(2.0, lambda: self._step_complete(True))

    def _side_arm_move_callback(self, future):
        """Callback when side arm move completes."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Side arm move complete')
                self._step_complete(True)
            else:
                self.get_logger().error(f'Side arm move failed: {response.message}')
                self._step_complete(False)
        except Exception as e:
            self.get_logger().error(f'Side arm move error: {e}')
            self._step_complete(False)

    def step_rotate_hook_pre(self):
        """Rotate hook before stowing."""
        self.get_logger().info('Rotating hook 90 degrees...')

        if self.side_arm_rotate_available:
            request = RotateHook.Request()
            request.angle_degrees = 90.0

            future = self.side_arm_rotate_client.call_async(request)
            future.add_done_callback(self._rotate_callback)
        else:
            self.get_logger().warn('Rotate hook service not available, simulating...')
            self.create_timer(1.0, lambda: self._step_complete(True))

    def _rotate_callback(self, future):
        """Callback when hook rotation completes."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Hook rotated to {response.final_angle} degrees')
                self._step_complete(True)
            else:
                self.get_logger().error('Hook rotation failed')
                self._step_complete(False)
        except Exception as e:
            self.get_logger().error(f'Rotation error: {e}')
            self._step_complete(False)

    def step_execute_trajectory(self):
        """Execute main arm stowing trajectory."""
        self.get_logger().info(f'Executing trajectory with pattern "{self.stow_pattern}"')

        # Generate waypoints from pattern
        waypoints = self.pattern_manager.apply_pattern(self.stow_pattern, self.target)

        if not waypoints:
            self.get_logger().warn('No waypoints from pattern, using fallback')
            waypoints = [
                self._make_pose(self.target.x, self.target.y, self.target.z + 0.1),
                self._make_pose(self.target.x, self.target.y, self.target.z),
            ]

        self.get_logger().info(f'Generated {len(waypoints)} waypoints:')
        for i, wp in enumerate(waypoints):
            self.get_logger().info(f'  {i+1}: ({wp.position.x:.3f}, {wp.position.y:.3f}, {wp.position.z:.3f})')

        if self.main_arm_action_available:
            goal = ExecuteTrajectory.Goal()
            goal.waypoints = waypoints
            goal.speed_factor = 0.5

            send_future = self.main_arm_action_client.send_goal_async(
                goal, feedback_callback=self._trajectory_feedback
            )
            send_future.add_done_callback(self._trajectory_goal_callback)
        else:
            self.get_logger().warn('Main arm action not available, simulating...')
            self.create_timer(3.0, lambda: self._step_complete(True))

    def _make_pose(self, x: float, y: float, z: float) -> Pose:
        """Create a Pose message."""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        return pose

    def _trajectory_feedback(self, feedback_msg):
        """Handle trajectory feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'  Trajectory progress: {int(feedback.progress * 100)}%')

    def _trajectory_goal_callback(self, future):
        """Callback when trajectory goal is accepted."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected')
            self._step_complete(False)
            return

        self.get_logger().info('Trajectory goal accepted, executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._trajectory_result_callback)

    def _trajectory_result_callback(self, future):
        """Callback when trajectory completes."""
        result = future.result().result
        if result.success:
            self.get_logger().info('Trajectory complete')
            self._step_complete(True)
        else:
            self.get_logger().error(f'Trajectory failed: {result.message}')
            self._step_complete(False)

    def step_rotate_hook_post(self):
        """Rotate hook back after stowing."""
        self.get_logger().info('Rotating hook back to 0 degrees...')

        if self.side_arm_rotate_available:
            request = RotateHook.Request()
            request.angle_degrees = 0.0

            future = self.side_arm_rotate_client.call_async(request)
            future.add_done_callback(self._rotate_callback)
        else:
            self.get_logger().warn('Rotate hook service not available, simulating...')
            self.create_timer(1.0, lambda: self._step_complete(True))

    def step_retract_side_arm(self):
        """Retract side arm to home position."""
        self.get_logger().info('Retracting side arm to home...')

        if self.side_arm_move_available:
            request = MoveToPosition.Request()
            request.x_mm = 0.0
            request.y_mm = 0.0
            request.z_mm = 0.0
            request.speed_scale = 0.5

            future = self.side_arm_move_client.call_async(request)
            future.add_done_callback(self._side_arm_move_callback)
        else:
            self.get_logger().warn('Side arm move service not available, simulating...')
            self.create_timer(2.0, lambda: self._step_complete(True))

    def step_complete(self):
        """Demo complete."""
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('DEMO COMPLETE!')
        self.get_logger().info('='*60)
        self._publish_status('complete')
        self.demo_running = False


def main(args=None):
    rclpy.init(args=args)
    node = FullStowDemoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Demo interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
