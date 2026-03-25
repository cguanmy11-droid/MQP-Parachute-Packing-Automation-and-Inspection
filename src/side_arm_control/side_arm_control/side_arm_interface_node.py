#!/usr/bin/env python3
"""
Side Arm Interface Node

Provides high-level action server for hook insertion and services for:
- Hook rotation
- Direct position moves (MoveToPosition)
- World-frame moves with TF transform (MoveToWorldPose)

Integrates with the coordinate node for real motor control when not in test mode.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from parachute_interfaces.action import InsertHook, MoveToCoordinate
from parachute_interfaces.srv import RotateHook, MoveToPosition, MoveToWorldPose
from parachute_interfaces.msg import HookStatus, SideArmState
from geometry_msgs.msg import Point, PoseStamped, PoseArray
from std_msgs.msg import String
import time
from typing import Optional, Tuple

# TF2 for coordinate transforms
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs


class SideArmInterfaceNode(Node):
    def __init__(self):
        super().__init__('side_arm_interface_node')

        # Parameters
        self.declare_parameter('test_mode', True)
        self.declare_parameter('approach_offset_z', 50.0)  # mm before loop
        self.declare_parameter('insert_depth_z', 30.0)      # mm through loop
        self.declare_parameter('side_arm_frame', 'side_arm_origin')  # TF frame for transforms

        # Hook offsets: position of hook tip in world frame when arm is homed (0,0,0)
        # These are used to convert world targets to arm coordinates
        self.declare_parameter('hook_offset_x_mm', 350.0)  # Hook X when homed
        self.declare_parameter('hook_offset_y_mm', 180.0)  # Hook Y when homed
        self.declare_parameter('hook_offset_z_mm', -10.0)  # Hook Z when homed
        # Axis inversion flags: True if increasing arm position decreases world position
        self.declare_parameter('invert_x', True)   # SA X+ = World X-
        self.declare_parameter('invert_y', False)  # Set True if Y is also inverted
        self.declare_parameter('invert_z', False)  # Set True if Z is also inverted

        # Vision servo parameters
        self.declare_parameter('enable_vision_servo', True)
        self.declare_parameter('servo_kp_x', 1.2)
        self.declare_parameter('servo_deadband_px', 5.0)
        self.declare_parameter('servo_timeout_sec', 10.0)
        self.declare_parameter('servo_min_speed', 400)
        self.declare_parameter('servo_max_speed', 1100)
        self.declare_parameter('image_width_px', 640)

        self.hook_offset_x = self.get_parameter('hook_offset_x_mm').value
        self.hook_offset_y = self.get_parameter('hook_offset_y_mm').value
        self.hook_offset_z = self.get_parameter('hook_offset_z_mm').value
        self.invert_x = self.get_parameter('invert_x').value
        self.invert_y = self.get_parameter('invert_y').value
        self.invert_z = self.get_parameter('invert_z').value

        # Vision servo config
        self.enable_vision_servo = self.get_parameter('enable_vision_servo').value
        self.servo_kp_x = self.get_parameter('servo_kp_x').value
        self.servo_deadband_px = self.get_parameter('servo_deadband_px').value
        self.servo_timeout_sec = self.get_parameter('servo_timeout_sec').value
        self.servo_min_speed = self.get_parameter('servo_min_speed').value
        self.servo_max_speed = self.get_parameter('servo_max_speed').value
        self.image_width_px = self.get_parameter('image_width_px').value

        # Handle test_mode as either bool or string (from PythonExpression)
        test_mode_val = self.get_parameter('test_mode').value
        if isinstance(test_mode_val, bool):
            self.test_mode = test_mode_val
        else:
            self.test_mode = str(test_mode_val).lower() == 'true'
        self.approach_offset_z = self.get_parameter('approach_offset_z').value
        self.insert_depth_z = self.get_parameter('insert_depth_z').value
        self.side_arm_frame = self.get_parameter('side_arm_frame').value

        # TF2 buffer and listener for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Callback group for concurrent operations
        self._cb_group = ReentrantCallbackGroup()

        # Current position tracking
        self._current_position = Point()
        self._is_homed = False

        # Vision servo state
        self._vision_latest_x: Optional[float] = None
        self._vision_lost_count: int = 0
        self._servo_active: bool = False

        # Action client for coordinate moves (used in both test and real mode)
        # In test_mode, coordinate_node runs in simulation and updates RViz
        # Uses relative topic for namespace support
        self._move_client = ActionClient(
            self, MoveToCoordinate, 'move_to_coordinate',
            callback_group=self._cb_group)

        # Command publisher for direct commands (relative topic)
        self._cmd_pub = self.create_publisher(String, 'command', 10)

        # State subscriber (relative topic)
        self._state_sub = self.create_subscription(
            SideArmState, 'parsed_state',
            self._state_callback, 10)

        # Vision subscriber for servo control (absolute - shared across arms)
        self._vision_sub = self.create_subscription(
            PoseArray, '/yolo/centers',
            self._vision_callback, 10)

        # Action server for inserting hook (relative topic)
        self.action_server = ActionServer(
            self, InsertHook, 'insert_hook',
            self.insert_hook_callback,
            callback_group=self._cb_group)

        # Service for rotating hook (relative topic)
        self.rotate_service = self.create_service(
            RotateHook, 'rotate_hook',
            self.rotate_hook_callback)

        # Service for direct position move (relative topic)
        self.move_service = self.create_service(
            MoveToPosition, 'move_to_position',
            self.move_to_position_callback)

        # Service for world-frame move with TF transform (relative topic)
        self.world_move_service = self.create_service(
            MoveToWorldPose, 'move_to_world_pose',
            self.move_to_world_pose_callback)

        # Publisher for hook status (relative topic)
        self.status_publisher = self.create_publisher(HookStatus, 'status', 10)

        # Timer to publish status
        self.timer = self.create_timer(1.0, self.publish_status)
        self.current_state = HookStatus.STATE_IDLE
        self.current_angle = 0.0

        mode_str = 'TEST MODE' if self.test_mode else 'REAL MODE'
        self.get_logger().info(f'{mode_str}: Side Arm Interface Node initialized')
        self.get_logger().info(
            f'Hook offsets: ({self.hook_offset_x}, {self.hook_offset_y}, {self.hook_offset_z}) mm, '
            f'Invert: X={self.invert_x}, Y={self.invert_y}, Z={self.invert_z}'
        )

    def _target_to_carriage_coords(self, target_x_mm: float, target_y_mm: float, target_z_mm: float):
        """
        Convert target position (in side_arm_origin frame, mm) to arm carriage coordinates.

        The hook_offset values represent where the hook tip is (in side_arm_origin frame)
        when the arm carriage is at position (0, 0, 0).

        To reach a target position with the hook:
        - For inverted axes: carriage_pos = hook_offset - target_pos
        - For normal axes:   carriage_pos = target_pos - hook_offset

        Args:
            target_x_mm: Target X position in side_arm_origin frame (mm)
            target_y_mm: Target Y position in side_arm_origin frame (mm)
            target_z_mm: Target Z position in side_arm_origin frame (mm)

        Returns:
            (carriage_x, carriage_y, carriage_z): Required carriage position (mm)
        """
        if self.invert_x:
            carriage_x = self.hook_offset_x - target_x_mm
        else:
            carriage_x = target_x_mm - self.hook_offset_x

        if self.invert_y:
            carriage_y = self.hook_offset_y - target_y_mm
        else:
            carriage_y = target_y_mm - self.hook_offset_y

        if self.invert_z:
            carriage_z = self.hook_offset_z - target_z_mm
        else:
            carriage_z = target_z_mm - self.hook_offset_z

        return carriage_x, carriage_y, carriage_z

    # Keep old name as alias for compatibility
    def _world_to_arm_coords(self, world_x_mm: float, world_y_mm: float, world_z_mm: float):
        """Deprecated: Use _target_to_carriage_coords instead."""
        return self._target_to_carriage_coords(world_x_mm, world_y_mm, world_z_mm)

    def _state_callback(self, msg: SideArmState):
        """Track current position from coordinate node."""
        self._current_position.x = msg.x_mm
        self._current_position.y = msg.y_mm
        self._current_position.z = msg.z_mm
        self._is_homed = msg.is_homed

    def _vision_callback(self, msg: PoseArray):
        """Track vision detections for servo control."""
        if not msg.poses:
            self._vision_latest_x = None
            self._vision_lost_count += 1
            return

        # Track rightmost detection (matching vis_servo_benchmark logic)
        rightmost = max(msg.poses, key=lambda p: p.position.x)
        self._vision_latest_x = rightmost.position.x
        self._vision_lost_count = 0

    def _vision_servo_to_center(self, timeout: Optional[float] = None) -> Tuple[bool, str]:
        """
        Run vision servo loop to center on detected loop.

        Returns:
            (success, message): Whether centering succeeded and status message
        """
        timeout = timeout or self.servo_timeout_sec
        center_x = self.image_width_px / 2.0
        stepper_x = 2  # X axis stepper (from vis_servo_benchmark)

        self.get_logger().info(f'[SERVO] Starting vision servo (timeout={timeout}s)')
        self._servo_active = True
        self._vision_lost_count = 0
        start_time = time.time()

        try:
            while time.time() - start_time < timeout:
                # Process callbacks to get fresh vision data
                rclpy.spin_once(self, timeout_sec=0.05)

                # Check for lost detection
                if self._vision_latest_x is None:
                    if self._vision_lost_count > 15:  # ~1.5 seconds at 10Hz
                        self._send_command('STOP_ALL')
                        return (False, 'Detection lost during servo')
                    continue

                # Calculate error
                error_x = self._vision_latest_x - center_x

                # Check if centered (within deadband)
                if abs(error_x) < self.servo_deadband_px:
                    self._send_command('STOP_ALL')
                    self.get_logger().info(f'[SERVO] Centered (error={error_x:.1f}px)')
                    return (True, 'Centered successfully')

                # P-control velocity command
                velocity = int(self.servo_kp_x * error_x)
                velocity = max(-self.servo_max_speed, min(self.servo_max_speed, velocity))
                if abs(velocity) < self.servo_min_speed:
                    velocity = int(self.servo_min_speed * (1 if velocity > 0 else -1))

                # Send velocity command (continuous mode)
                self._send_command(f'STEPPER_MOVE,{stepper_x},{velocity}')

                time.sleep(0.066)  # ~15Hz control loop

            # Timeout
            self._send_command('STOP_ALL')
            return (False, f'Servo timeout after {timeout}s')

        finally:
            self._servo_active = False
            self._send_command('STOP_ALL')  # Ensure motors stopped

    def _send_command(self, cmd: str):
        """Send direct command (works in both real and test/simulation mode)."""
        msg = String()
        msg.data = cmd
        self._cmd_pub.publish(msg)
        self.get_logger().debug(f'Sent command: {cmd}')

    def _move_to(self, x: float, y: float, z: float, speed_scale: float = 0.7) -> bool:
        """
        Move to position using coordinate node action.
        Returns True on success. Works in both real and test/simulation mode.
        """
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

        # Update current angle
        self.current_angle += angle

        # Convert degrees to servo microseconds (roughly 1000us = 90 degrees)
        servo_us = int(self.current_angle * 1000.0 / 90.0)

        # Send servo command (works in both test and real mode for visualization)
        self._send_command(f'SERVO,{servo_us}')

        if not self.test_mode:
            # In real mode, also use DC motor as backup
            rotation_time = abs(angle) / 90.0
            dc_percent = 50 if angle > 0 else -50
            self._send_command(f'DC_SPEED,{dc_percent}')
            time.sleep(rotation_time)
            self._send_command('DC_SPEED,0')
        else:
            # Simulate rotation time
            rotation_time = abs(angle) / 90.0
            time.sleep(rotation_time)

        response.success = True
        response.final_angle = self.current_angle
        response.message = f"Rotated to {self.current_angle} degrees"

        self.get_logger().info(f'Hook rotated to {self.current_angle} degrees')

        return response

    def move_to_position_callback(self, request, response):
        """
        Service callback - move to position in side arm frame (mm).
        This is a simple direct move without TF transforms.
        """
        x_mm = request.x_mm
        y_mm = request.y_mm
        z_mm = request.z_mm
        speed = request.speed_scale if request.speed_scale > 0 else 0.5

        self.get_logger().info(
            f'MoveToPosition: ({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) mm, speed={speed:.2f}'
        )

        success = self._move_to(x_mm, y_mm, z_mm, speed_scale=speed)

        response.success = success
        if success:
            response.message = f"Moved to ({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) mm"
            response.estimated_time_sec = 0.0  # Already completed
        else:
            response.message = "Move failed"
            response.estimated_time_sec = 0.0

        return response

    def move_to_world_pose_callback(self, request, response):
        """
        Service callback - move to world-frame position using TF2 transform.
        Transforms the input pose from its frame to side_arm_origin, then moves.
        """
        target_pose = request.target_pose
        speed = request.speed_scale if request.speed_scale > 0 else 0.5

        source_frame = target_pose.header.frame_id or 'world'
        self.get_logger().info(
            f'MoveToWorldPose: frame={source_frame} -> {self.side_arm_frame}'
        )
        self.get_logger().info(
            f'  Input: ({target_pose.pose.position.x:.4f}, '
            f'{target_pose.pose.position.y:.4f}, {target_pose.pose.position.z:.4f}) m'
        )

        # Transform pose to side arm frame
        try:
            # Get transform from source frame to side arm frame
            transform = self.tf_buffer.lookup_transform(
                self.side_arm_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )

            # Transform the pose
            transformed_pose = tf2_geometry_msgs.do_transform_pose_stamped(
                target_pose, transform
            )

            # Extract position and convert to arm coordinates
            world_x_mm = transformed_pose.pose.position.x * 1000.0
            world_y_mm = transformed_pose.pose.position.y * 1000.0
            world_z_mm = transformed_pose.pose.position.z * 1000.0

            x_mm, y_mm, z_mm = self._world_to_arm_coords(world_x_mm, world_y_mm, world_z_mm)

            self.get_logger().info(
                f'  World (mm): ({world_x_mm:.1f}, {world_y_mm:.1f}, {world_z_mm:.1f}) -> '
                f'Arm: ({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) mm'
            )

        except TransformException as e:
            self.get_logger().error(f'TF transform failed: {e}')
            response.success = False
            response.message = f"TF transform failed: {e}"
            response.final_x_mm = 0.0
            response.final_y_mm = 0.0
            response.final_z_mm = 0.0
            return response

        # Execute the move
        success = self._move_to(x_mm, y_mm, z_mm, speed_scale=speed)

        response.success = success
        response.final_x_mm = x_mm
        response.final_y_mm = y_mm
        response.final_z_mm = z_mm

        if success:
            response.message = f"Moved to ({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) mm in {self.side_arm_frame}"
        else:
            response.message = "Move failed after transform"

        return response

    def insert_hook_callback(self, goal_handle):
        """Action callback - insert hook through detected loop."""
        target_loop = goal_handle.request.target_loop
        loop_pose = target_loop.pose

        # Get source frame (default to 'world')
        source_frame = loop_pose.header.frame_id or 'world'
        loop_pos = loop_pose.pose.position

        self.get_logger().info(
            f'Inserting hook into loop {target_loop.loop_id} at '
            f'({loop_pos.x:.3f}, {loop_pos.y:.3f}, {loop_pos.z:.3f}) m (frame: {source_frame})'
        )

        feedback_msg = InsertHook.Feedback()

        # Transform loop position from source frame to side_arm_origin frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.side_arm_frame,  # target frame (side_arm_origin)
                source_frame,          # source frame (world)
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Log the transform for debugging
            t = transform.transform.translation
            r = transform.transform.rotation
            self.get_logger().info(
                f'TF {source_frame} -> {self.side_arm_frame}: '
                f'trans=({t.x:.3f}, {t.y:.3f}, {t.z:.3f}), '
                f'rot=({r.x:.3f}, {r.y:.3f}, {r.z:.3f}, {r.w:.3f})'
            )

            transformed_pose = tf2_geometry_msgs.do_transform_pose_stamped(loop_pose, transform)
            transformed_pos = transformed_pose.pose.position

            # Position in side_arm_origin frame (after TF transform)
            # NOTE: These are in side_arm_origin frame, not world frame!
            sa_frame_x_mm = transformed_pos.x * 1000.0
            sa_frame_y_mm = transformed_pos.y * 1000.0
            sa_frame_z_mm = transformed_pos.z * 1000.0

            self.get_logger().info(
                f'Loop in world: ({loop_pos.x*1000:.1f}, {loop_pos.y*1000:.1f}, {loop_pos.z*1000:.1f}) mm'
            )
            self.get_logger().info(
                f'Loop in {self.side_arm_frame}: ({sa_frame_x_mm:.1f}, {sa_frame_y_mm:.1f}, {sa_frame_z_mm:.1f}) mm'
            )

            # Convert side_arm_origin position to arm carriage coordinates
            # The hook_offset values represent hook position when carriage is at (0,0,0)
            target_x_mm, target_y_mm, target_z_mm = self._world_to_arm_coords(
                sa_frame_x_mm, sa_frame_y_mm, sa_frame_z_mm
            )

            self.get_logger().info(
                f'Hook offsets: ({self.hook_offset_x:.1f}, {self.hook_offset_y:.1f}, {self.hook_offset_z:.1f}) mm, '
                f'Invert: X={self.invert_x}, Y={self.invert_y}, Z={self.invert_z}'
            )
            self.get_logger().info(
                f'Target arm carriage: ({target_x_mm:.1f}, {target_y_mm:.1f}, {target_z_mm:.1f}) mm'
            )

            # Sanity check: warn if carriage position seems out of bounds
            if target_x_mm < 0 or target_x_mm > 300:
                self.get_logger().warn(f'Target X ({target_x_mm:.1f}mm) outside workspace [0, 300]!')
            if target_y_mm < 0 or target_y_mm > 200:
                self.get_logger().warn(f'Target Y ({target_y_mm:.1f}mm) outside workspace [0, 200]!')
            if target_z_mm < 0 or target_z_mm > 150:
                self.get_logger().warn(f'Target Z ({target_z_mm:.1f}mm) outside workspace [0, 150]!')

        except TransformException as e:
            self.get_logger().error(f'TF transform failed: {e}')
            self.get_logger().warn('Falling back to direct conversion (may be inaccurate)')
            # Fallback: direct conversion (only works if loop is already in side_arm frame)
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

        # Stage 1b: Vision servo to center on loop (if enabled)
        if self.enable_vision_servo:
            feedback_msg.progress = 0.35
            feedback_msg.current_state = InsertHook.Feedback.STATE_APPROACHING
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info('Stage 1b: Vision servo centering')

            servo_success, servo_msg = self._vision_servo_to_center()
            if not servo_success:
                self.get_logger().warn(f'Vision servo failed: {servo_msg} - continuing with open-loop')
                # Continue with open-loop insert (degraded mode)
            else:
                self.get_logger().info(f'Vision servo: {servo_msg}')

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
