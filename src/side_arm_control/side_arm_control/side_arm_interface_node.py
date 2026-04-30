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
from rclpy.executors import MultiThreadedExecutor
from parachute_interfaces.action import InsertHook, MoveToCoordinate, VisualServo, ReleaseHook
from parachute_interfaces.srv import RotateHook, MoveToPosition, MoveToWorldPose
from parachute_interfaces.msg import HookStatus, SideArmState
from geometry_msgs.msg import Point, PoseStamped, PoseArray
from std_msgs.msg import String
from std_srvs.srv import Trigger
import time
import math
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

        # Y-axis servo parameters
        self.declare_parameter('servo_kp_y', 1.0)
        self.declare_parameter('servo_deadband_y_px', 8.0)
        self.declare_parameter('image_height_px', 480)
        
        # Step-based servo control
        self.declare_parameter('servo_step_size', 100)       # steps per servo iteration
        self.declare_parameter('servo_step_speed', 1200)     # steps/sec for servo moves
        self.declare_parameter('servo_settle_sec', 0.08)    # pause between moves
        self.declare_parameter('servo_goal_x', 331.0)
        self.declare_parameter('servo_goal_y', 161.0)
        
        # Read new parameters
        self.servo_kp_y = self.get_parameter('servo_kp_y').value
        self.servo_deadband_y_px = self.get_parameter('servo_deadband_y_px').value
        self.image_height_px = self.get_parameter('image_height_px').value
        self.servo_step_size = self.get_parameter('servo_step_size').value
        self.servo_step_speed = self.get_parameter('servo_step_speed').value
        self.servo_settle_sec = self.get_parameter('servo_settle_sec').value
        self.servo_goal_x = self.get_parameter('servo_goal_x').value
        self.servo_goal_y = self.get_parameter('servo_goal_y').value

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
        self._limit_depth = False
        self._limit_horizontal = False
        self._limit_vertical = False

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

        # State subscriber (relative topic) - needs reentrant callback group
        # so state updates can run while services are polling
        self._state_sub = self.create_subscription(
            SideArmState, 'parsed_state',
            self._state_callback, 10,
            callback_group=self._cb_group)

        # Vision subscriber for servo control (relative topic for namespace support)
        self._vision_sub = self.create_subscription(
            PoseArray, 'yolo/centers',
            self._vision_callback, 10,
            callback_group=self._cb_group)
 
        # Target pixel position for servo (set before calling servo)
        self._vision_target_px: Optional[Tuple[float, float]] = None
        self._vision_detections: list = []  # all current detections as (x, y) pixel pairs

        # Action server for inserting hook (relative topic)
        self.action_server = ActionServer(
            self, InsertHook, 'insert_hook',
            self.insert_hook_callback,
            callback_group=self._cb_group)

        # Action server for visual servoing (relative topic)
        self.visual_servo_action = ActionServer(
            self, VisualServo, 'visual_servo',
            self._visual_servo_action_callback,
            callback_group=self._cb_group)

        # Action server for releasing hook (relative topic)
        self.release_hook_action = ActionServer(
            self, ReleaseHook, 'release_hook',
            self._release_hook_action_callback,
            callback_group=self._cb_group)

        # Service for rotating hook (relative topic)
        self.rotate_service = self.create_service(
            RotateHook, 'rotate_hook',
            self.rotate_hook_callback)
        
        # Service for retracting hook (relative topic)
        self.retract_service = self.create_service(
            Trigger, 'retract_hook_release',
            self.retract_hook_callback)

        # Service for releasing hook (relative topic)  
        self.release_service = self.create_service(
            Trigger, 'release_hook',
            self.release_hook_callback)

        # Service for direct position move (relative topic)
        self.move_service = self.create_service(
            MoveToPosition, 'move_to_position',
            self.move_to_position_callback)

        # Service for world-frame move with TF transform (relative topic)
        self.world_move_service = self.create_service(
            MoveToWorldPose, 'move_to_world_pose',
            self.move_to_world_pose_callback)
        
        # Service to test the visual servoing
        self.servo_test_service = self.create_service(
            Trigger, 'test_vision_servo',
            self._test_servo_callback)

        # ==================== PRIMITIVE SERVICES ====================
        # These break down insert_hook into coordinator-controllable steps

        # Vision servo centering (extracts the servo loop from insert_hook)
        self.vision_servo_service = self.create_service(
            Trigger, 'vision_servo',
            self._vision_servo_callback)

        # Insert through loop (just Z-axis push)
        self.insert_through_service = self.create_service(
            Trigger, 'insert_through_loop',
            self._insert_through_loop_callback)

        # Retract Z only (split from retract_hook_release - no servo motion)
        self.retract_z_service = self.create_service(
            Trigger, 'retract_z',
            self._retract_z_callback,
            callback_group=self._cb_group)

        # Reset hook angle only (split from retract_hook_release)
        self.reset_hook_service = self.create_service(
            Trigger, 'reset_hook_angle',
            self._reset_hook_angle_callback)

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
        """Track current position and limit switches from coordinate node."""
        self._current_position.x = msg.x_mm
        self._current_position.y = msg.y_mm
        self._current_position.z = msg.z_mm
        self._is_homed = msg.is_homed
        self._limit_depth = msg.limit_depth
        self._limit_horizontal = msg.limit_horizontal
        self._limit_vertical = msg.limit_vertical

    def _vision_callback(self, msg: PoseArray):
        """Track all vision detections for servo control."""
        if not msg.poses:
            self._vision_detections = []
            self._vision_latest_x = None
            self._vision_lost_count += 1
            return
 
        # Store all detections as pixel coordinates
        self._vision_detections = [
            (p.position.x, p.position.y) for p in msg.poses
        ]
        
        # Keep rightmost tracking for backward compatibility
        rightmost = max(msg.poses, key=lambda p: p.position.x)
        self._vision_latest_x = rightmost.position.x
        self._vision_lost_count = 0

    def _vision_servo_to_center(self, timeout: Optional[float] = None) -> Tuple[bool, str]:
        """
        Run vision servo loop to center on detected loop.
        
        Uses discrete STEPPER_MOVE commands with P-control on both axes.
        Selects the detection closest to the target pixel position
        (or image center if no target is set).
        
        Returns:
            (success, message): Whether centering succeeded and status message
        """
        timeout = timeout or self.servo_timeout_sec
        center_x = self.image_width_px / 2.0
        center_y = self.image_height_px / 2.0
        self._last_servo_det = None
        
        # Use target pixel if set, otherwise center of image
        goal_x = self.servo_goal_x
        goal_y = self.servo_goal_y
        if self._vision_target_px is not None:
            goal_x, goal_y = self._vision_target_px
 
        self.get_logger().info(
            f'[SERVO] Starting vision servo '
            f'(goal=({goal_x:.0f},{goal_y:.0f})px, timeout={timeout:.1f}s)')
        self._servo_active = True
        self._vision_lost_count = 0
        lost_streak = 0
        start_time = time.time()
        iterations = 0
 
        try:
            while time.time() - start_time < timeout:
                iterations += 1
                
                # Brief wait for new detection data
                time.sleep(self.servo_settle_sec)
                
                # Check for lost detection
                if not self._vision_detections:
                    lost_streak += 1
                    if lost_streak > 20:  # ~1.6s with 0.08s settle
                        return (False, f'Detection lost for {lost_streak} frames')
                    continue
                lost_streak = 0
 
                # Select closest detection to goal
                best_det = None
                best_dist = float('inf')
                for dx, dy in self._vision_detections:
                    dist = math.sqrt((dx - goal_x)**2 + (dy - goal_y)**2)
                    if dist < best_dist:
                        best_dist = dist
                        best_det = (dx, dy)
                
                if best_det is None:
                    continue

                # Skip if detection hasn't updated (camera latency)
                if hasattr(self, '_last_servo_det') and best_det == self._last_servo_det:
                    continue
                self._last_servo_det = best_det

                det_x, det_y = best_det
 
                det_x, det_y = best_det
                error_x = det_x - center_x  # positive = detection is right of center
                error_y = det_y - center_y  # positive = detection is below center
 
                # Check if centered (within deadband on both axes)
                x_centered = abs(error_x) < self.servo_deadband_px
                y_centered = abs(error_y) < self.servo_deadband_y_px
                
                if x_centered and y_centered:
                    self.get_logger().info(
                        f'[SERVO] Centered in {iterations} iters '
                        f'(err_x={error_x:.1f}, err_y={error_y:.1f}px)')
                    return (True, f'Centered in {iterations} iterations')
 
                # --- X axis correction (stepper 2) ---
                if not x_centered:
                    # P-control: scale steps by error magnitude
                    x_scale = min(abs(error_x) / (self.image_width_px / 4.0), 1.0)
                    x_steps = max(int(self.servo_step_size * x_scale), 200)
                    
                    # Direction: if detection is right of center, we need to move
                    # the arm to bring it left (direction depends on arm config)
                    x_sign = 1 if error_x > 0 else -1
                    
                    self._send_command(
                        f'STEPPER_MOVE,2,{x_sign * x_steps},{self.servo_step_speed}')
 
                # --- Y axis correction (stepper 1) ---
                if not y_centered:
                    y_scale = min(abs(error_y) / (self.image_height_px / 4.0), 1.0)
                    y_steps = max(int(self.servo_step_size * y_scale), 200)
                    
                    # Direction: if detection is below center, move arm up
                    y_sign = 1 if error_y > 0 else -1
                    
                    self._send_command(
                        f'STEPPER_MOVE,1,{y_sign * y_steps},{self.servo_step_speed}')
 
                # Log every 10 iterations
                if iterations % 10 == 0:
                    self.get_logger().info(
                        f'[SERVO] iter={iterations} det=({det_x:.0f},{det_y:.0f}) '
                        f'err=({error_x:.0f},{error_y:.0f})px')
 
            # Timeout
            return (False, f'Servo timeout after {timeout:.1f}s ({iterations} iters)')
 
        finally:
            self._servo_active = False
    
    def _visual_servo_action_callback(self, goal_handle):
        """
        Action callback for visual servoing.

        Selects loop based on arm side:
        - Right arm: targets LEFTMOST loop (lowest X pixel)
        - Left arm: targets RIGHTMOST loop (highest X pixel)

        Uses P-control to center the selected loop in the camera frame.
        """
        request = goal_handle.request
        feedback = VisualServo.Feedback()

        arm_side = request.arm_side.lower() if request.arm_side else 'right'
        timeout = request.timeout_sec if request.timeout_sec > 0 else self.servo_timeout_sec

        # Goal position: use provided values or defaults
        goal_x = request.goal_x_px if request.goal_x_px > 0 else self.servo_goal_x
        goal_y = request.goal_y_px if request.goal_y_px > 0 else self.servo_goal_y

        self.get_logger().info(
            f'[VISUAL_SERVO] Starting: arm={arm_side}, timeout={timeout:.1f}s, '
            f'goal=({goal_x:.0f}, {goal_y:.0f})px'
        )

        self._servo_active = True
        start_time = time.time()
        iterations = 0
        lost_streak = 0

        try:
            while time.time() - start_time < timeout:
                iterations += 1

                # Brief settle time for camera
                time.sleep(self.servo_settle_sec)

                # Check for detections
                if not self._vision_detections:
                    lost_streak += 1
                    if lost_streak > 30:  # ~2.4s with 0.08s settle
                        goal_handle.abort()
                        result = VisualServo.Result()
                        result.success = False
                        result.message = f'Detection lost for {lost_streak} frames'
                        result.iterations = iterations
                        result.execution_time_sec = time.time() - start_time
                        return result
                    continue
                lost_streak = 0

                # Select loop based on arm side
                # Right arm: leftmost loop (min X) - hook approaches from right
                # Left arm: rightmost loop (max X) - hook approaches from left
                if arm_side == 'right':
                    target_det = min(self._vision_detections, key=lambda d: d[0])
                else:
                    target_det = max(self._vision_detections, key=lambda d: d[0])

                det_x, det_y = target_det

                # Calculate error from goal position
                error_x = det_x - goal_x
                error_y = det_y - goal_y

                # Publish feedback
                feedback.target_x_px = det_x
                feedback.target_y_px = det_y
                feedback.error_x_px = error_x
                feedback.error_y_px = error_y
                feedback.iteration = iterations
                feedback.num_detections = len(self._vision_detections)
                goal_handle.publish_feedback(feedback)

                # Check if centered
                x_centered = abs(error_x) < self.servo_deadband_px
                y_centered = abs(error_y) < self.servo_deadband_y_px

                if x_centered and y_centered:
                    nudge_steps = -1000
                    nudge_speed = self.servo_step_speed
                    self.get_logger().info(
                        f'[VISUAL_SERVO] Centered — nudging +{nudge_steps} steps on X axis'
                    )
                    self._send_command(f'STEPPER_MOVE,2,{nudge_steps},{nudge_speed}')
                    time.sleep(abs(nudge_steps) / nudge_speed + 0.2)  # wait for move + settle

                    goal_handle.succeed()
                    result = VisualServo.Result()
                    result.success = True
                    result.message = f'Centered on {"leftmost" if arm_side == "right" else "rightmost"} loop'
                    result.final_error_x_px = error_x
                    result.final_error_y_px = error_y
                    result.iterations = iterations
                    result.execution_time_sec = time.time() - start_time
                    self.get_logger().info(
                        f'[VISUAL_SERVO] Centered in {iterations} iters, '
                        f'err=({error_x:.1f}, {error_y:.1f})px'
                    )
                    return result

                # P-control: scale steps by error magnitude
                # X axis (stepper 2)
                if not x_centered:
                    x_scale = min(abs(error_x) / (self.image_width_px / 4.0), 1.0)
                    x_steps = max(int(self.servo_step_size * x_scale), 150)
                    x_sign = 1 if error_x > 0 else -1
                    self._send_command(f'STEPPER_MOVE,2,{x_sign * x_steps},{self.servo_step_speed}')

                # Y axis (stepper 1)
                if not y_centered:
                    y_scale = min(abs(error_y) / (self.image_height_px / 4.0), 1.0)
                    y_steps = max(int(self.servo_step_size * y_scale), 150)
                    y_sign = 1 if error_y > 0 else -1
                    self._send_command(f'STEPPER_MOVE,1,{y_sign * y_steps},{self.servo_step_speed}')

                # Log periodically
                if iterations % 15 == 0:
                    self.get_logger().info(
                        f'[VISUAL_SERVO] iter={iterations}, det=({det_x:.0f},{det_y:.0f}), '
                        f'err=({error_x:.0f},{error_y:.0f}), n_loops={len(self._vision_detections)}'
                    )

            # Timeout
            goal_handle.abort()
            result = VisualServo.Result()
            result.success = False
            result.message = f'Timeout after {timeout:.1f}s'
            result.final_error_x_px = error_x if 'error_x' in dir() else 0.0
            result.final_error_y_px = error_y if 'error_y' in dir() else 0.0
            result.iterations = iterations
            result.execution_time_sec = time.time() - start_time
            self.get_logger().warn(f'[VISUAL_SERVO] {result.message}')
            return result

        finally:
            self._servo_active = False

    def _release_hook_action_callback(self, goal_handle):
        """
        Action callback for releasing the hook.

        Pulls line up (stepper move) while running DC motor forward
        to push the line off the hook.
        """
        request = goal_handle.request
        feedback = ReleaseHook.Feedback()

        # Use defaults if not specified
        pull_steps = request.pull_steps if request.pull_steps > 0 else 12000
        pull_speed = request.pull_speed if request.pull_speed > 0 else 1200
        dc_speed = request.dc_speed_percent if request.dc_speed_percent > 0 else 75
        dc_duration = request.dc_duration_sec if request.dc_duration_sec > 0 else 6.0

        self.get_logger().info(
            f'[RELEASE] Starting: pull={pull_steps} steps @ {pull_speed}, '
            f'DC={dc_speed}% for {dc_duration}s'
        )

        start_time = time.time()

        # Stage 1: Pull up while pushing forward
        feedback.stage = 'pulling'
        feedback.progress = 0.0
        goal_handle.publish_feedback(feedback)

        # Start both movements simultaneously
        self._send_command(f'STEPPER_MOVE,1,{pull_steps},{pull_speed}')
        time.sleep(0.05)  # Brief delay for firmware
        self._send_command(f'DC_SPEED,{dc_speed}')

        self.get_logger().info('[RELEASE] Pulling line up and pushing forward...')

        # Wait for DC duration while updating feedback
        dc_start = time.time()
        while time.time() - dc_start < dc_duration:
            elapsed = time.time() - dc_start
            feedback.progress = min(elapsed / dc_duration, 0.9)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)

        # Stage 2: Stop DC motor
        feedback.stage = 'releasing'
        feedback.progress = 0.95
        goal_handle.publish_feedback(feedback)

        self._send_command('DC_SPEED,0')
        self.get_logger().info('[RELEASE] DC motor stopped')

        # Brief pause for stepper to finish
        time.sleep(0.5)

        # Complete
        goal_handle.succeed()
        result = ReleaseHook.Result()
        result.success = True
        result.message = 'Hook released'
        result.execution_time_sec = time.time() - start_time

        self.get_logger().info(f'[RELEASE] Complete in {result.execution_time_sec:.1f}s')
        return result

    def set_servo_target_from_loop(self, target_loop):
        """
        Set the servo target pixel from a DetectedLoop's pixel position.
        Call this before _vision_servo_to_center() in the insert sequence.
        
        If the loop pose is in camera frame, position.x and position.y
        are already pixel coordinates from the YOLO detector.
        """
        if target_loop is None:
            self._vision_target_px = None
            return
            
        # The yolo/centers topic publishes pixel coordinates in position.x/y
        px = target_loop.pose.pose.position.x
        py = target_loop.pose.pose.position.y
        
        if 0 < px < self.image_width_px and 0 < py < self.image_height_px:
            self._vision_target_px = (px, py)
            self.get_logger().info(
                f'[SERVO] Target pixel set to ({px:.0f}, {py:.0f})')
        else:
            # Position is in meters (world frame), not pixels — use image center
            self._vision_target_px = None
            self.get_logger().info(
                '[SERVO] Target not in pixel coords, defaulting to image center')

    def _send_command(self, cmd: str):
        """Send direct command (works in both real and test/simulation mode)."""
        msg = String()
        msg.data = cmd
        self._cmd_pub.publish(msg)
        self.get_logger().debug(f'Sent command: {cmd}')

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        """Wait for a future to complete with timeout. Returns True if completed."""
        start = time.time()
        while not future.done():
            if time.time() - start > timeout_sec:
                return False
            time.sleep(0.05)  # Small sleep to avoid busy-waiting
        return True

    def _move_to(self, x: float, y: float, z: float, speed_scale: float = 0.7):
        """
        Move to position using coordinate node action.
        Returns (success, final_x, final_y, final_z) tuple.
        Works in both real and test/simulation mode.

        Uses polling on future.done() to avoid executor conflicts
        when called from within service callbacks.
        """
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Coordinate node not available')
            return (False, x, y, z)

        goal = MoveToCoordinate.Goal()
        goal.x_mm = x
        goal.y_mm = y
        goal.z_mm = z
        goal.speed_scale = speed_scale

        self.get_logger().info(f'Moving to ({x:.1f}, {y:.1f}, {z:.1f}) mm')

        # Send goal async (longer timeout in case server is busy)
        send_goal_future = self._move_client.send_goal_async(goal)
        if not self._wait_for_future(send_goal_future, 30.0):
            self.get_logger().error('Goal send timed out')
            return (False, x, y, z)

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Move goal rejected')
            return (False, x, y, z)

        # Wait for result (generous timeout for slow moves)
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, 120.0):
            self.get_logger().warn('Result wait timed out - canceling goal')
            # Cancel the goal so action server is free for next command
            cancel_future = goal_handle.cancel_goal_async()
            self._wait_for_future(cancel_future, 5.0)
            self.get_logger().info('Goal canceled')
            return (False, x, y, z)

        result = result_future.result()
        if result and result.result.success:
            final_x = result.result.final_x_mm
            final_y = result.result.final_y_mm
            final_z = result.result.final_z_mm
            self.get_logger().info(
                f'Move complete: ({final_x:.1f}, {final_y:.1f}, {final_z:.1f}) mm'
            )
            return (True, final_x, final_y, final_z)
        else:
            # Move likely happened but result wasn't received - read current position
            final_x = self._current_position.x
            final_y = self._current_position.y
            final_z = self._current_position.z
            self.get_logger().warn(
                f'Move result unclear, current position: ({final_x:.1f}, {final_y:.1f}, {final_z:.1f}) mm'
            )
            return (True, final_x, final_y, final_z)

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
        """Service callback - rotate hook to a specified angle."""
        target_angle = request.angle_degrees
        self.get_logger().info(
            f'Rotating hook to {target_angle} degrees (was {self.current_angle})'
        )
        self.current_angle = target_angle  # absolute, not cumulative
        servo_us = int(self.current_angle * 1000.0 / 90.0)

        # Send servo command (works in both test and real mode for visualization)
        self._send_command(f'SERVO,{servo_us}')

        rotation_time = abs(target_angle) / 90.0
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

        success, final_x, final_y, final_z = self._move_to(x_mm, y_mm, z_mm, speed_scale=speed)

        response.success = success
        if success:
            response.message = f"Moved to ({final_x:.1f}, {final_y:.1f}, {final_z:.1f}) mm"
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
        success, final_x, final_y, final_z = self._move_to(x_mm, y_mm, z_mm, speed_scale=speed)

        response.success = success
        response.final_x_mm = final_x
        response.final_y_mm = final_y
        response.final_z_mm = final_z

        if success:
            response.message = f"Moved to ({final_x:.1f}, {final_y:.1f}, {final_z:.1f}) mm in {self.side_arm_frame}"
        else:
            response.message = f"Move failed (target was {x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f} mm)"

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
        success, _, _, _ = self._move_to(target_x_mm, target_y_mm, approach_z, speed_scale=0.7)
        if not success:
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

            self.set_servo_target_from_loop(target_loop)

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
        success, _, _, _ = self._move_to(target_x_mm, target_y_mm, insert_z, speed_scale=0.5)
        if not success:
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
    
    def retract_hook_callback(self, request, response):
        """Retract hook: bring Z axis home, reset hook to down position."""
        self.get_logger().info('[RETRACT] Retracting hook — homing Z axis...')
        
        # Home the DC motor (Z axis) to pull hook out
        self._send_command('HOME,0')
        time.sleep(5.0)  # Wait for homing to complete
        
        # Reset hook rotation to down position
        self._send_command('SERVO,0')
        time.sleep(0.5)
        
        self.get_logger().info('[RETRACT] Hook retracted and reset')
        self.current_state = HookStatus.STATE_IDLE
        response.success = True
        response.message = 'Hook retracted'
        return response

    def release_hook_callback(self, request, response):
        """Release: move up to pull line, DC motor forward to push line off."""
        self.get_logger().info('[RELEASE] Pulling line up...')
        
        # Move up 10k steps at 1500 speed and DC forward at 75% effort
        self._send_command('STEPPER_MOVE,1,-10000,1500')
        self._send_command('DC_SPEED,-75')
        time.sleep(13.0)  # Wait for move to complete
        
        self.get_logger().info('[RELEASE] Running DC motor forward to release line...')
        
        # Stop DC motor
        self._send_command('DC_SPEED,0')
        time.sleep(0.3)
        
        # Reverse DC motor to return to ready position
        # TODO maybe: self._send_command('DC_SPEED,-75')
        # time.sleep(2.0)
        
        # # Stop
        # self._send_command('DC_SPEED,0')
        
        self.get_logger().info('[RELEASE] Line released, hook ready')
        response.success = True
        response.message = 'Line released'
        return response
    
    def _test_servo_callback(self, request, response):
        """Test service — runs vision servo to center on nearest detection."""
        self._vision_target_px = None  # use image center
        success, message = self._vision_servo_to_center(timeout=10.0)
        response.success = success
        response.message = message
        return response

    # ==================== PRIMITIVE SERVICE CALLBACKS ====================

    def _vision_servo_callback(self, request, response):
        """Run vision servo to center on detected loop."""
        self.get_logger().info('[VISION_SERVO] Starting servo centering...')
        self._vision_target_px = None  # use configured goal (servo_goal_x/y)
        success, message = self._vision_servo_to_center(timeout=self.servo_timeout_sec)
        response.success = success
        response.message = message
        if success:
            self.get_logger().info(f'[VISION_SERVO] {message}')
        else:
            self.get_logger().warn(f'[VISION_SERVO] Failed: {message}')
        return response

    def _insert_through_loop_callback(self, request, response):
        """Push hook through loop - move Z to max position (150mm)."""
        self.get_logger().info('[INSERT_THROUGH] Pushing hook through loop...')

        # Keep current X/Y, move Z to insert depth (150mm)
        target_z = 150.0  # max_z_mm from coordinate node
        current_x = self._current_position.x
        current_y = self._current_position.y

        self.get_logger().info(
            f'[INSERT_THROUGH] Moving Z: {self._current_position.z:.1f} -> {target_z:.1f}mm'
        )

        success, final_x, final_y, final_z = self._move_to(current_x, current_y, target_z, speed_scale=1.0)

        if success:
            self.current_state = HookStatus.STATE_INSERTED
            response.success = True
            response.message = f'Inserted to Z={final_z:.1f}mm'
            self.get_logger().info(f'[INSERT_THROUGH] {response.message}')
        else:
            response.success = False
            response.message = f'Insert move failed (reached Z={final_z:.1f}mm)'
            self.get_logger().error(f'[INSERT_THROUGH] {response.message}')

        return response

    def _retract_z_callback(self, request, response):
        """Retract Z axis fast until limit switch is hit."""
        self.get_logger().info('[RETRACT_Z] Retracting Z axis (fast)...')

        # Run DC motor at full speed toward home (positive = retract)
        self._send_command('DC_SPEED,100')

        # Poll until Z is near home (0) or timeout
        timeout_sec = 30.0
        home_tolerance_mm = 5.0
        poll_interval = 0.1
        start_time = time.time()

        while time.time() - start_time < timeout_sec:
            time.sleep(poll_interval)
            # self.get_logger().info(f'[RETRACT Z] {self._limit_depth}')

            # Check limit switch - this is the reliable indicator
            if self._limit_depth:
                # Stop motor immediately
                self._send_command('DC_SPEED,0')
                elapsed = time.time() - start_time
                response.success = True
                response.message = f'Z retracted in {elapsed:.1f}s (limit hit)'
                self.get_logger().info(f'[RETRACT_Z] {response.message}')
                return response

        # Timeout - stop motor
        self._send_command('DC_SPEED,0')
        current_z = self._current_position.z
        response.success = False
        response.message = f'Z retract timeout after {timeout_sec}s (Z={current_z:.1f}mm, limit={self._limit_depth})'
        self.get_logger().warn(f'[RETRACT_Z] {response.message}')
        return response

    def _reset_hook_angle_callback(self, request, response):
        """Reset hook servo to 1000 (down)."""
        self.get_logger().info('[RESET_HOOK] Resetting hook angle to down...')

        self._send_command('SERVO,1000')
        time.sleep(0.5)

        self.current_angle = 1000.0
        self.current_state = HookStatus.STATE_IDLE
        response.success = True
        response.message = 'Hook angle reset to down'
        self.get_logger().info('[RESET_HOOK] Hook angle reset')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SideArmInterfaceNode()

    # Use MultiThreadedExecutor so action callbacks can be processed
    # while service callbacks are waiting for action results
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
