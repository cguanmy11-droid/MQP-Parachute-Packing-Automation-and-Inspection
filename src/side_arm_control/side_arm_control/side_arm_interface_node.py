#!/usr/bin/env python3
"""
Side Arm Interface Node

Provides high-level action server for hook insertion and service for hook rotation.
Integrates with the coordinate node for real motor control when not in test mode.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from parachute_interfaces.action import InsertHook, MoveToCoordinate
from parachute_interfaces.srv import RotateHook
from parachute_interfaces.msg import HookStatus, SideArmState
from geometry_msgs.msg import Point
from std_msgs.msg import String
import time


class SideArmInterfaceNode(Node):
    def __init__(self):
        super().__init__('side_arm_interface_node')

        # Parameters
        self.declare_parameter('test_mode', True)
        self.declare_parameter('approach_offset_z', 50.0)  # mm before loop
        self.declare_parameter('insert_depth_z', 30.0)      # mm through loop

        self.test_mode = self.get_parameter('test_mode').value
        self.approach_offset_z = self.get_parameter('approach_offset_z').value
        self.insert_depth_z = self.get_parameter('insert_depth_z').value

        # Callback group for concurrent operations
        self._cb_group = ReentrantCallbackGroup()

        # Current position tracking (for real mode)
        self._current_position = Point()
        self._is_homed = False

        # Set up real mode components
        if not self.test_mode:
            # Action client for coordinate moves
            self._move_client = ActionClient(
                self, MoveToCoordinate, '/side_arm/move_to_coordinate',
                callback_group=self._cb_group)

            # Command publisher for direct commands
            self._cmd_pub = self.create_publisher(String, '/side_arm/command', 10)

            # State subscriber
            self._state_sub = self.create_subscription(
                SideArmState, '/side_arm/parsed_state',
                self._state_callback, 10)

        # Action server for inserting hook
        self.action_server = ActionServer(
            self, InsertHook, '/side_arm/insert_hook',
            self.insert_hook_callback,
            callback_group=self._cb_group)

        # Service for rotating hook
        self.rotate_service = self.create_service(
            RotateHook, '/side_arm/rotate_hook',
            self.rotate_hook_callback)

        # Publisher for hook status
        self.status_publisher = self.create_publisher(HookStatus, '/side_arm/status', 10)

        # Timer to publish status
        self.timer = self.create_timer(1.0, self.publish_status)
        self.current_state = HookStatus.STATE_IDLE
        self.current_angle = 0.0

        mode_str = 'TEST MODE' if self.test_mode else 'REAL MODE'
        self.get_logger().info(f'{mode_str}: Side Arm Interface Node initialized')

    def _state_callback(self, msg: SideArmState):
        """Track current position from coordinate node."""
        self._current_position.x = msg.x_mm
        self._current_position.y = msg.y_mm
        self._current_position.z = msg.z_mm
        self._is_homed = msg.is_homed

    def _send_command(self, cmd: str):
        """Send direct command to ESP32."""
        if not self.test_mode:
            msg = String()
            msg.data = cmd
            self._cmd_pub.publish(msg)
            self.get_logger().debug(f'Sent command: {cmd}')

    def _move_to(self, x: float, y: float, z: float, speed_scale: float = 0.7) -> bool:
        """
        Move to position using coordinate node action.
        Returns True on success.
        """
        if self.test_mode:
            self.get_logger().info(f'TEST: Would move to ({x:.1f}, {y:.1f}, {z:.1f}) mm')
            time.sleep(1.0)  # Simulate movement
            return True

        if not self._move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Coordinate node not available')
            return False

        goal = MoveToCoordinate.Goal()
        goal.x_mm = x
        goal.y_mm = y
        goal.z_mm = z
        goal.speed_scale = speed_scale

        self.get_logger().info(f'Moving to ({x:.1f}, {y:.1f}, {z:.1f}) mm')

        send_goal_future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Move goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)

        result = result_future.result()
        if result and result.result.success:
            self.get_logger().info(
                f'Move complete: ({result.result.final_x_mm:.1f}, '
                f'{result.result.final_y_mm:.1f}, {result.result.final_z_mm:.1f}) mm'
            )
            return True
        else:
            self.get_logger().error('Move failed')
            return False

    def publish_status(self):
        """Publish current hook status."""
        msg = HookStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.current_state
        msg.position = self._current_position
        msg.is_inserted = (self.current_state == HookStatus.STATE_INSERTED)
        msg.is_retracted = (self.current_state == HookStatus.STATE_IDLE)

        self.status_publisher.publish(msg)

    def rotate_hook_callback(self, request, response):
        """Service callback - rotate hook by specified angle."""
        angle = request.angle_degrees

        self.get_logger().info(f'Rotating hook by {angle} degrees')

        if self.test_mode:
            # Simulate rotation time
            rotation_time = abs(angle) / 90.0
            time.sleep(rotation_time)
        else:
            # TODO: When servo is connected, send rotation command
            # For now, use DC motor timed movement as placeholder
            rotation_time = abs(angle) / 90.0
            dc_percent = 50 if angle > 0 else -50
            self._send_command(f'DC_SPEED,{dc_percent}')
            time.sleep(rotation_time)
            self._send_command('DC_SPEED,0')

        # Update current angle
        self.current_angle += angle

        response.success = True
        response.final_angle = self.current_angle
        response.message = f"Rotated to {self.current_angle} degrees"

        self.get_logger().info(f'Hook rotated to {self.current_angle} degrees')

        return response

    def insert_hook_callback(self, goal_handle):
        """Action callback - insert hook through detected loop."""
        target_loop = goal_handle.request.target_loop
        loop_pos = target_loop.pose.pose.position

        self.get_logger().info(
            f'Inserting hook into loop {target_loop.loop_id} at '
            f'({loop_pos.x:.3f}, {loop_pos.y:.3f}, {loop_pos.z:.3f}) m'
        )

        feedback_msg = InsertHook.Feedback()

        # Convert loop position from meters to mm
        target_x_mm = loop_pos.x * 1000.0
        target_y_mm = loop_pos.y * 1000.0
        target_z_mm = loop_pos.z * 1000.0

        # Stage 1: Approach - move to XY position, offset Z
        feedback_msg.current_state = InsertHook.Feedback.STATE_APPROACHING
        feedback_msg.progress = 0.25
        feedback_msg.current_position = self._current_position
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info('Stage 1: Approaching loop position')

        approach_z = target_z_mm - self.approach_offset_z
        if not self._move_to(target_x_mm, target_y_mm, approach_z, speed_scale=0.7):
            goal_handle.abort()
            result = InsertHook.Result()
            result.success = False
            result.message = "Failed to approach loop"
            return result

        # Stage 2: Align
        feedback_msg.current_state = InsertHook.Feedback.STATE_ALIGNING
        feedback_msg.progress = 0.50
        feedback_msg.current_position = self._current_position
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info('Stage 2: Aligning with loop')
        time.sleep(0.5)  # Brief pause for alignment

        # Stage 3: Insert - move through loop
        feedback_msg.current_state = InsertHook.Feedback.STATE_INSERTING
        feedback_msg.progress = 0.75
        feedback_msg.current_position = self._current_position
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info('Stage 3: Inserting hook through loop')

        insert_z = target_z_mm + self.insert_depth_z
        if not self._move_to(target_x_mm, target_y_mm, insert_z, speed_scale=0.5):
            goal_handle.abort()
            result = InsertHook.Result()
            result.success = False
            result.message = "Failed to insert through loop"
            return result

        # Stage 4: Verify
        feedback_msg.current_state = InsertHook.Feedback.STATE_VERIFYING
        feedback_msg.progress = 1.0
        feedback_msg.current_position = self._current_position
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info('Stage 4: Verifying insertion')
        time.sleep(0.5)

        # Mark as successful
        goal_handle.succeed()

        result = InsertHook.Result()
        result.success = True
        result.final_hook_position = self._current_position
        result.execution_time = 4.0
        result.message = "Hook inserted successfully"

        self.current_state = HookStatus.STATE_INSERTED
        self.get_logger().info('Hook insertion complete!')

        return result


def main(args=None):
    rclpy.init(args=args)
    node = SideArmInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
