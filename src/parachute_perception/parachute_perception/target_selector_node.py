#!/usr/bin/env python3
"""
Target Selector Node

Subscribes to detected loops and provides a service to select the next
loop to stow. Tracks which loops have been stowed to avoid repeats.

Selection strategy: rightmost loop (highest X in world frame).
This matches the physical stowing order for parachute lines.

Integrates with:
    - /detected_loops (from detection_simulator or real YOLO detector)
    - /request_next_target (called by packing_coordinator)
    - /stow/command (listens for 'reset' to clear stowed list)

Usage:
    ros2 run parachute_perception target_selector_node

    # With test positions (no detection pipeline needed):
    ros2 run parachute_perception target_selector_node --ros-args -p use_test_loops:=true
"""

import rclpy
from rclpy.node import Node
from parachute_interfaces.msg import DetectedLoops, DetectedLoop, LoopGroundTruth, LoopStateArray
from parachute_interfaces.srv import RequestNextTarget, CaptureLoops
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import String
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped


class TargetSelectorNode(Node):
    def __init__(self):
        super().__init__('target_selector_node')

        # ==================== PARAMETERS ====================
        self.declare_parameter('use_test_loops', False)
        self.declare_parameter('selection_strategy', 'rightmost')  # rightmost, leftmost, nearest
        self.declare_parameter('stow_proximity_threshold', 0.01)  # meters — how close to consider "same loop"
        self.declare_parameter('output_frame', 'world')  # Frame to transform detections into
        self.declare_parameter('loop_source', 'top_camera')  # 'side_camera' or 'top_camera'

        self.use_test_loops = self.get_parameter('use_test_loops').value
        self.selection_strategy = self.get_parameter('selection_strategy').value
        self.proximity_threshold = self.get_parameter('stow_proximity_threshold').value
        self.loop_source = self.get_parameter('loop_source').value
        self.output_frame = self.get_parameter('output_frame').value
        self._next_side = 'left'

        # ==================== TF2 ====================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ==================== STATE ====================
        self.current_loops: list[DetectedLoop] = []
        self.stowed_positions: list[Point] = []  # Track stowed loop positions

        # ==================== CAPTURE/LOCK STATE ====================
        self._is_locked: bool = False  # True after successful capture
        self._locked_loops: list[DetectedLoop] = []  # Captured loop positions
        self._capture_timestamp = None  # When capture occurred

        # ==================== SUBSCRIBERS ====================
        # Subscribe based on loop_source parameter
        if self.loop_source == 'top_camera':
            self.loop_sub = self.create_subscription(
                LoopStateArray, '/top_cam/loop_states',
                self._top_cam_callback, 10
            )
            self.get_logger().info('Using TOP CAMERA as loop source')
        else:
            self.loop_sub = self.create_subscription(
                DetectedLoops, '/side_arm_right/detected_loops',
                self._loops_callback, 10
            )
            self.get_logger().info('Using SIDE CAMERA as loop source')

        self.cmd_sub = self.create_subscription(
            String, '/stow/command',
            self._command_callback, 10
        )

        # Always subscribe to ground truth (used in test mode)
        self.ground_truth_sub = self.create_subscription(
            LoopGroundTruth, '/loop_ground_truth',
            self._ground_truth_callback, 10
        )

        # ==================== SERVICES ====================
        self.target_service = self.create_service(
            RequestNextTarget, '/request_next_target',
            self._request_target_callback
        )
        self.capture_service = self.create_service(
            CaptureLoops, '/capture_loops',
            self._capture_loops_callback
        )

        # ==================== PUBLISHER ====================
        self.target_pub = self.create_publisher(
            DetectedLoop, '/target_loop', 10 # change back or to var name
        )

        # ==================== TEST LOOPS ====================
        if self.use_test_loops:
            self.get_logger().info('Test mode: using /loop_ground_truth as source')

        mode = 'TEST LOOPS' if self.use_test_loops else self.loop_source.upper()
        self.get_logger().info(
            f'Target Selector initialized (source={mode}, strategy={self.selection_strategy})'
        )

    # def _generate_test_loops(self):
    #     """Generate hardcoded test loops for development without perception."""
    #     self.current_loops = []
    #     test_positions = [
    #         # Left side (positive Y)
    #         (0.25,  0.15, -0.02),
    #         (0.29,  0.15,  0.00),
    #         (0.33,  0.15, -0.01),
    #         (0.37,  0.15,  0.01),
    #         (0.41,  0.15, -0.02),
    #         # Right side (negative Y)
    #         (0.25, -0.15, -0.02),
    #         (0.29, -0.15,  0.00),
    #         (0.33, -0.15, -0.01),
    #         (0.37, -0.15,  0.01),
    #         (0.41, -0.15, -0.02),
    #     ]
    #     for i, (x, y, z) in enumerate(test_positions):
    #         loop = DetectedLoop()
    #         loop.loop_id = i
    #         loop.confidence = 1.0
    #         loop.pose = PoseStamped()
    #         loop.pose.header.frame_id = 'world'
    #         loop.pose.pose.position = Point(x=x, y=y, z=z)
    #         loop.pose.pose.orientation = Quaternion(w=1.0)
    #         self.current_loops.append(loop)

    #     self.get_logger().info(f'Generated {len(self.current_loops)} test loops (5L + 5R)')

    def _ground_truth_callback(self, msg):
        """Use ground truth positions as test loops (single source of truth)."""
        if not self.use_test_loops:
            return
        if self._is_locked:
            return
        
        self.current_loops = []
        for i, pos in enumerate(msg.positions):
            loop = DetectedLoop()
            loop.loop_id = msg.loop_ids[i] if i < len(msg.loop_ids) else i
            loop.confidence = 1.0
            loop.pose = PoseStamped()
            loop.pose.header.frame_id = msg.header.frame_id or 'world'
            loop.pose.pose.position = Point(x=pos.x, y=pos.y, z=pos.z)
            loop.pose.pose.orientation = Quaternion(w=1.0)
            self.current_loops.append(loop)

    def _transform_to_output_frame(self, pose_stamped: PoseStamped) -> PoseStamped:
        """Transform a pose to the output frame (world)."""
        source_frame = pose_stamped.header.frame_id
        if not source_frame:
            source_frame = 'camera_frame'  # Default assumption
            pose_stamped.header.frame_id = source_frame

        if source_frame == self.output_frame:
            return pose_stamped  # Already in correct frame

        try:
            transform = self.tf_buffer.lookup_transform(
                self.output_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            transformed = do_transform_pose_stamped(pose_stamped, transform)
            return transformed
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF transform failed ({source_frame} -> {self.output_frame}): {e}')
            return pose_stamped  # Return original if transform fails

    def _loops_callback(self, msg: DetectedLoops):
        """Store detected loops from perception pipeline, transformed to world frame."""
        if self.use_test_loops:
            return  # Ignore detections in test mode

        # If locked, do NOT update current_loops - we use locked_loops instead
        if self._is_locked:
            return

        self.current_loops = []

        for i, loop in enumerate(msg.loops):
            # Transform pose to world frame
            transformed_pose = self._transform_to_output_frame(loop.pose)

            # Create new loop with transformed pose
            transformed_loop = DetectedLoop()
            transformed_loop.loop_id = loop.loop_id if loop.loop_id >= 0 else i
            transformed_loop.confidence = loop.confidence
            transformed_loop.pose = transformed_pose

            self.current_loops.append(transformed_loop)

    def _top_cam_callback(self, msg: LoopStateArray):
        """Convert top camera LoopStateArray to DetectedLoop format."""
        if self.use_test_loops:
            return  # Ignore detections in test mode

        if self._is_locked:
            return

        self.current_loops = []

        for i, ls in enumerate(msg.loops):
            # Skip loops without valid world position
            if ls.world_position.x == 0.0 and ls.world_position.y == 0.0:
                continue

            # Convert LoopState to DetectedLoop
            loop = DetectedLoop()
            # Convert string ID like "L1" to int (extract number)
            try:
                loop.loop_id = int(ls.loop_id[1:]) if ls.loop_id[1:].isdigit() else i
            except (ValueError, IndexError):
                loop.loop_id = i
            loop.confidence = ls.confidence

            # Create PoseStamped from world_position
            loop.pose = PoseStamped()
            loop.pose.header.frame_id = 'world'
            loop.pose.header.stamp = msg.header.stamp
            loop.pose.pose.position = ls.world_position
            loop.pose.pose.orientation = Quaternion(w=1.0)

            self.current_loops.append(loop)

        if self.current_loops:
            self.get_logger().debug(f'Top cam: {len(self.current_loops)} loops received')

    def _command_callback(self, msg: String):
        """Handle commands (reset stowed list, unlock, etc.)."""
        cmd = msg.data.strip().lower()
        if cmd == 'reset_targets':
            self.stowed_positions.clear()
            self._is_locked = False
            self._locked_loops.clear()
            self.get_logger().info('Stowed positions cleared, loops unlocked')
        elif cmd == 'unlock_loops':
            self._is_locked = False
            self._locked_loops.clear()
            self.get_logger().info('Loops unlocked (positions retained)')
        elif cmd == 'refresh_test_loops' and self.use_test_loops:
            self.stowed_positions.clear()
            self._generate_test_loops()
            self.get_logger().info('Test loops regenerated')
        elif cmd == 'reset_targets':
            self.stowed_positions.clear()
            self._is_locked = False
            self._locked_loops.clear()
            self._next_side = 'left'  # add this line
            self.get_logger().info('Stowed positions cleared, loops unlocked')

    def _capture_loops_callback(self, request, response):
        """
        Service callback - capture and lock current loop positions.

        This freezes the loop list for the duration of the stowing sequence.
        Call reset_targets to unlock and allow fresh detection.
        """
        import time

        expected_count = request.expected_count
        timeout = request.timeout_sec if request.timeout_sec > 0 else 2.0

        # Already locked?
        if self._is_locked:
            response.success = True  # Not a failure - already captured
            response.captured_count = len(self._locked_loops)
            response.message = f'Already locked with {response.captured_count} loops'
            response.loops = list(self._locked_loops)
            self.get_logger().info(response.message)
            return response

        # Wait for stable detection (3 consecutive identical counts)
        start = time.time()
        stable_count = 0
        last_count = -1

        while time.time() - start < timeout:
            current_count = len(self.current_loops)
            if current_count > 0 and current_count == last_count:
                stable_count += 1
                if stable_count >= 3:  # 3 consecutive stable readings
                    break
            else:
                stable_count = 0
            last_count = current_count
            time.sleep(0.1)

        # Verify count if expected_count > 0
        actual_count = len(self.current_loops)
        if expected_count > 0 and actual_count != expected_count:
            response.success = False
            response.captured_count = actual_count
            response.message = f'Count mismatch: expected {expected_count}, got {actual_count}'
            response.loops = []
            self.get_logger().warn(response.message)
            return response

        # No loops detected?
        if actual_count == 0:
            response.success = False
            response.captured_count = 0
            response.message = 'No loops detected to capture'
            response.loops = []
            self.get_logger().warn(response.message)
            return response

        # Capture and lock - deep copy current loops
        self._locked_loops = list(self.current_loops)
        self._is_locked = True
        self._capture_timestamp = self.get_clock().now()

        response.success = True
        response.captured_count = actual_count
        response.message = f'Captured {actual_count} loops'
        response.loops = list(self._locked_loops)

        self.get_logger().info(
            f'Loop capture: {actual_count} loops locked at positions:'
        )
        for loop in self._locked_loops:
            pos = loop.pose.pose.position
            self.get_logger().info(
                f'  Loop {loop.loop_id}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})'
            )

        return response

    def _is_already_stowed(self, loop: DetectedLoop) -> bool:
        """Check if a loop position has already been stowed."""
        pos = loop.pose.pose.position
        for stowed in self.stowed_positions:
            dist = (
                (pos.x - stowed.x) ** 2 +
                (pos.y - stowed.y) ** 2 +
                (pos.z - stowed.z) ** 2
            ) ** 0.5
            if dist < self.proximity_threshold:
                return True
        return False

    def _select_target(self, loops: list[DetectedLoop], preferred_side: str = None) -> DetectedLoop | None:
        """Select next target. preferred_side: 'left' (Y>=0), 'right' (Y<0), or None for any."""
        source_loops = self._locked_loops if self._is_locked else loops
        available = [l for l in source_loops if not self._is_already_stowed(l)]

        if not available:
            return None

        # Filter by side if requested
        if preferred_side == 'left':
            side_loops = [l for l in available if l.pose.pose.position.y >= 0]
            available = side_loops if side_loops else available  # fall back to any
        elif preferred_side == 'right':
            side_loops = [l for l in available if l.pose.pose.position.y < 0]
            available = side_loops if side_loops else available

        if self.selection_strategy == 'rightmost':
            return max(available, key=lambda l: l.pose.pose.position.x)
        elif self.selection_strategy in ('leftmost', 'nearest'):
            return min(available, key=lambda l: l.pose.pose.position.x)
        else:
            self.get_logger().warn(f'Unknown strategy: {self.selection_strategy}, using rightmost')
            return max(available, key=lambda l: l.pose.pose.position.x)

    def _request_target_callback(self, request, response):
        """Service callback — select and return next loop to stow."""
        # Try preferred side first, fall back to other side
        target = self._select_target(self.current_loops, preferred_side=self._next_side)

        if target is None:
            # Try other side before giving up
            other_side = 'right' if self._next_side == 'left' else 'left'
            target = self._select_target(self.current_loops, preferred_side=other_side)

        if target is None:
            response.target_available = False
            response.target_loop = DetectedLoop()
            response.message = f'All {len(self.current_loops)} loops already stowed'
            self.get_logger().info(response.message)
            return response

        # Alternate side for next call
        self._next_side = 'right' if self._next_side == 'left' else 'left'

        self.stowed_positions.append(Point(
            x=target.pose.pose.position.x,
            y=target.pose.pose.position.y,
            z=target.pose.pose.position.z,
        ))

        response.target_available = True
        response.target_loop = target
        response.message = f'Selected loop {target.loop_id}'
        self.target_pub.publish(target)

        remaining = len(self.current_loops) - len(self.stowed_positions)
        pos = target.pose.pose.position
        side = 'LEFT' if pos.y >= 0 else 'RIGHT'
        self.get_logger().info(
            f'Target: loop {target.loop_id} [{side}] at '
            f'({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) '
            f'[{remaining} remaining]'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TargetSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()