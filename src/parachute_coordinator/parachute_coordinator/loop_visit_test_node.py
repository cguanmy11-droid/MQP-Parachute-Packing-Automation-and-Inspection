#!/usr/bin/env python3
"""
Loop Visit Test - moves side arm to each detected loop position (z=0) to verify transforms.

Usage:
    ros2 run parachute_coordinator loop_visit_test_node

    # With test positions (no camera needed):
    ros2 run parachute_coordinator loop_visit_test_node --ros-args -p use_test_loops:=true
"""

import time
import rclpy
from rclpy.node import Node
from parachute_interfaces.msg import DetectedLoops
from parachute_interfaces.srv import MoveToPosition
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs


class LoopVisitTestNode(Node):
    def __init__(self):
        super().__init__('loop_visit_test_node')

        self.declare_parameter('use_test_loops', False)
        self.declare_parameter('pause_sec', 2.0)
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('side_arm_frame', 'side_arm_origin')

        self.use_test_loops = self.get_parameter('use_test_loops').value
        self.pause_sec = self.get_parameter('pause_sec').value
        self.speed = self.get_parameter('speed').value
        self.side_arm_frame = self.get_parameter('side_arm_frame').value

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Service client
        self.move_client = self.create_client(MoveToPosition, '/side_arm/move_to_position')

        # Detected loops
        self.loops = []
        self.loops_received = False

        if self.use_test_loops:
            self.loops = [
                (0.25, 0.15, 0.0),
                (0.29, 0.15, 0.0),
                (0.33, 0.15, 0.0),
                (0.37, 0.15, 0.0),
            ]
            self.loops_received = True
            self.get_logger().info(f'Using {len(self.loops)} test loops')
        else:
            self.loop_sub = self.create_subscription(
                DetectedLoops, '/detected_loops', self._loops_cb, 10
            )
            self.get_logger().info('Waiting for /detected_loops...')

        # Wait then run
        self.create_timer(1.0, self._tick)
        self.started = False

    def _loops_cb(self, msg):
        if self.loops_received:
            return
        if len(msg.loops) == 0:
            return

        self.loops = []
        for loop in msg.loops:
            p = loop.pose.pose.position
            self.loops.append((p.x, p.y, p.z))

        self.loops_received = True
        self.get_logger().info(f'Got {len(self.loops)} loops')

    def _transform_to_side_arm(self, x, y, z, source_frame='world'):
        """Transform world point to side arm mm coordinates."""
        pose = PoseStamped()
        pose.header.frame_id = source_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0

        try:
            transform = self.tf_buffer.lookup_transform(
                self.side_arm_frame, source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            transformed = tf2_geometry_msgs.do_transform_pose_stamped(pose, transform)
            tp = transformed.pose.position
            return tp.x * 1000.0, tp.y * 1000.0, tp.z * 1000.0
        except TransformException as e:
            self.get_logger().error(f'TF failed: {e}')
            return None

    def _move_to(self, x_mm, y_mm, z_mm):
        """Blocking move via service."""
        if not self.move_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Move service not available')
            return False

        req = MoveToPosition.Request()
        req.x_mm = float(x_mm)
        req.y_mm = float(y_mm)
        req.z_mm = float(z_mm)
        req.speed_scale = float(self.speed)

        future = self.move_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        result = future.result()
        return result and result.success

    def _tick(self):
        if self.started or not self.loops_received:
            return
        self.started = True

        self.get_logger().info('=' * 40)
        self.get_logger().info('LOOP VISIT TEST')
        self.get_logger().info('=' * 40)

        # Home first
        self.get_logger().info('Homing side arm...')
        self._move_to(0.0, 0.0, 0.0)
        time.sleep(1.0)

        # Sort by X so we visit left to right
        sorted_loops = sorted(self.loops, key=lambda p: p[0])

        for i, (wx, wy, wz) in enumerate(sorted_loops):
            self.get_logger().info(f'--- Loop {i+1}/{len(sorted_loops)} ---')
            self.get_logger().info(f'  World: ({wx:.3f}, {wy:.3f}, {wz:.3f}) m')

            result = self._transform_to_side_arm(wx, wy, wz)
            if result is None:
                self.get_logger().error('  Transform failed, skipping')
                continue

            sa_x, sa_y, sa_z = result
            self.get_logger().info(f'  Side arm: ({sa_x:.1f}, {sa_y:.1f}, {sa_z:.1f}) mm')
            self.get_logger().info(f'  Moving to ({sa_x:.1f}, {sa_y:.1f}, 0.0) mm (z=0)')

            if self._move_to(sa_x, sa_y, 0.0):
                self.get_logger().info(f'  ✓ Arrived. Pausing {self.pause_sec}s...')
            else:
                self.get_logger().error(f'  ✗ Move failed')

            time.sleep(self.pause_sec)

        # Home at end
        self.get_logger().info('Done! Homing...')
        self._move_to(0.0, 0.0, 0.0)
        self.get_logger().info('COMPLETE')


def main(args=None):
    rclpy.init(args=args)
    node = LoopVisitTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
