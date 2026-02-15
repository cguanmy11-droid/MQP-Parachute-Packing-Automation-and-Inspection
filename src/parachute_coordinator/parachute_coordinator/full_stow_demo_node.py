#!/usr/bin/env python3
"""
Full Stow Demo Node

End-to-end demonstration of the packing sequence using a predefined loop position.
No perception required - uses hardcoded or parameter-specified target.

Sequence:
1. Initialize
2. Home main arm
3. Home side arm (wait for completion)
4. Move side arm to target position (wait for completion)
5. Rotate hook 90 degrees
6. Execute main arm stowing trajectory
7. Rotate hook back to 0
8. Retract side arm (wait for completion)
9. Complete

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
        self.declare_parameter('side_arm_x_mm', 82.0)
        self.declare_parameter('side_arm_y_mm', 27.0)
        self.declare_parameter('side_arm_z_mm', 140.0)

        # Motion pattern for main arm
        self.declare_parameter('stow_pattern', 'recorded_stow')

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
            # ('home_main_arm', self.step_home_main_arm),
            # # ('home_side_arm', self.step_home_side_arm),
            # ('move_side_arm', self.step_move_side_arm),
            # ('insert_side_arm', self.step_insert_side_arm),
            # ('rotate_hook_pre', self.step_rotate_hook_pre),
            ('execute_trajectory', self.step_execute_trajectory),
            # ('rotate_hook_post', self.step_rotate_hook_post),
            # ('retract_side_arm', self.step_retract_side_arm),
            # ('complete', self.step_complete),
        ]

        # Wait for services
        self.get_logger().info('=' * 60)
        self.get_logger().info('FULL STOW DEMO')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Target position: ({self.target.x:.3f}, {self.target.y:.3f}, {self.target.z:.3f})')
        self.get_logger().info(f'Side arm position: X={self.side_arm_pos["x"]:.0f}, Y={self.side_arm_pos["y"]:.0f}, Z={self.side_arm_pos["z"]:.0f} mm')
        self.get_logger().info(f'Motion pattern: {self.stow_pattern}')
        self.get_logger().info('=' * 60)

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

    # ==================== SETUP ====================

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

    # ==================== STEP SEQUENCING ====================

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
        """Called when a step completes. Advances to next step after delay."""
        if not self.demo_running:
            return

        if success:
            self.current_step += 1
            timer = self.create_timer(self.step_delay, lambda: self._delayed_next_step(timer))
        else:
            self.get_logger().error('Step failed, stopping demo')
            self.demo_running = False

    def _delayed_next_step(self, timer):
        """One-shot timer callback for step delay."""
        timer.cancel()
        self._run_next_step()

    def _sim_complete(self, timer):
        """One-shot timer callback for simulated steps."""
        timer.cancel()
        self._step_complete(True)

    # ==================== SIDE ARM MOVEMENT (SHARED) ====================

    def _move_side_arm_to(self, x_mm: float, y_mm: float, z_mm: float, speed: float = 1.5):
        """Move side arm to a position using action client, with service fallback."""
        self.get_logger().info(f'Moving side arm to ({x_mm:.0f}, {y_mm:.0f}, {z_mm:.0f}) mm')

        if self.side_arm_action_available:
            goal = MoveToCoordinate.Goal()
            goal.x_mm = x_mm
            goal.y_mm = y_mm
            goal.z_mm = z_mm
            goal.speed_scale = speed

            send_future = self.side_arm_action_client.send_goal_async(
                goal, feedback_callback=self._side_arm_feedback
            )
            send_future.add_done_callback(self._side_arm_goal_callback)
        elif self.side_arm_move_available:
            request = MoveToPosition.Request()
            request.x_mm = x_mm
            request.y_mm = y_mm
            request.z_mm = z_mm
            request.speed_scale = speed
            future = self.side_arm_move_client.call_async(request)
            future.add_done_callback(self._side_arm_service_callback)
        else:
            self.get_logger().warn('No side arm interface available, simulating...')
            timer = self.create_timer(2.0, lambda: self._sim_complete(timer))

    def _side_arm_feedback(self, feedback_msg):
        """Handle side arm action feedback."""
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'  Side arm progress: {int(feedback.progress * 100)}%')

    def _side_arm_goal_callback(self, future):
        """Callback when side arm action goal is accepted/rejected."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Side arm goal rejected')
            self._step_complete(False)
            return
        self.get_logger().info('Side arm goal accepted, moving...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._side_arm_result_callback)

    def _side_arm_result_callback(self, future):
        """Callback when side arm action completes."""
        result = future.result().result
        if result.success:
            self.get_logger().info('Side arm move complete')
            self._step_complete(True)
        else:
            self.get_logger().error(f'Side arm move failed: {result.message}')
            self._step_complete(False)

    def _side_arm_service_callback(self, future):
        """Callback when side arm service call completes (fallback)."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Side arm move complete')
                self._step_complete(True)
            else:
                self.get_logger().error(f'Side arm move failed: {response.message}')
                self._step_complete(False)
        except Exception as e:
            self.get_logger().error(f'Side arm service error: {e}')
            self._step_complete(False)

    # ==================== HOOK ROTATION (SHARED) ====================

    def _rotate_hook(self, angle_degrees: float):
        """Rotate hook to a given angle."""
        self.get_logger().info(f'Rotating hook to {angle_degrees:.0f} degrees...')

        if self.side_arm_rotate_available:
            request = RotateHook.Request()
            request.angle_degrees = angle_degrees

            future = self.side_arm_rotate_client.call_async(request)
            future.add_done_callback(self._rotate_callback)
        else:
            self.get_logger().warn('Rotate hook service not available, simulating...')
            timer = self.create_timer(1.0, lambda: self._sim_complete(timer))

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
        # Main arm home is fire-and-forget via topic, wait a fixed time
        timer = self.create_timer(3.0, lambda: self._sim_complete(timer))

    def step_home_side_arm(self):
        """Send side arm to home position."""
        self._move_side_arm_to(0.0, 0.0, 0.0)

    def step_move_side_arm(self):
        """Move side arm to ready position."""
        self._move_side_arm_to(
            self.side_arm_pos['x'],
            self.side_arm_pos['y'],
            0.0
        )
    
    def step_insert_side_arm(self):
        """Move side arm to loop position."""
        self._move_side_arm_to(
            self.side_arm_pos['x'],
            self.side_arm_pos['y'],
            self.side_arm_pos['z']
        )

    def step_rotate_hook_pre(self):
        """Rotate hook before stowing."""
        self._rotate_hook(90.0)

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
            goal.speed_factor = 1.0

            send_future = self.main_arm_action_client.send_goal_async(
                goal, feedback_callback=self._trajectory_feedback
            )
            send_future.add_done_callback(self._trajectory_goal_callback)
        else:
            self.get_logger().warn('Main arm action not available, simulating...')
            timer = self.create_timer(3.0, lambda: self._sim_complete(timer))

    def step_rotate_hook_post(self):
        """Rotate hook back after stowing."""
        self._rotate_hook(0.0)

    def step_retract_side_arm(self):
        """Retract side arm to outside of loop position."""
        self._move_side_arm_to(
            self.side_arm_pos['x'],
            self.side_arm_pos['y'],
            0.0
        )

    def step_complete(self):
        """Demo complete."""
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('DEMO COMPLETE!')
        self.get_logger().info('=' * 60)
        self._publish_status('complete')
        self.demo_running = False

    # ==================== TRAJECTORY HELPERS ====================

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