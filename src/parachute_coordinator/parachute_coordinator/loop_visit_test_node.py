#!/usr/bin/env python3
"""
Loop Visit Test - moves side arm to each detected loop position (X/Y only, z=0)
to verify vision and coordinate transforms.

This is a demo mode that:
1. Captures detected loop positions from the camera
2. Homes the side arm
3. Visits each loop in left-to-right order (X/Y positioning only)
4. Pauses at each loop for visual verification
5. Returns home when complete

Usage:
    # With real camera (requires dual_arm_test with use_real_camera:=true running):
    ros2 run parachute_coordinator loop_visit_test_node

    # With test positions (no camera needed):
    ros2 run parachute_coordinator loop_visit_test_node --ros-args -p use_test_loops:=true

    # Adjust pause time:
    ros2 run parachute_coordinator loop_visit_test_node --ros-args -p pause_sec:=3.0
"""

import time
import rclpy
from rclpy.node import Node
from parachute_interfaces.msg import DetectedLoops
from parachute_interfaces.srv import MoveToPosition
from std_msgs.msg import String


class LoopVisitTestNode(Node):
    def __init__(self):
        super().__init__('loop_visit_test_node')

        self.declare_parameter('use_test_loops', False)
        self.declare_parameter('pause_sec', 2.0)
        self.declare_parameter('speed', 0.5)

        # Hook offset calibration - where is the hook in world frame when arm is homed
        # These should match side_arm_interface_node parameters
        self.declare_parameter('hook_offset_x_mm', 350.0)
        self.declare_parameter('hook_offset_y_mm', 180.0)
        self.declare_parameter('hook_offset_z_mm', -10.0)
        # Axis inversion flags
        self.declare_parameter('invert_x', True)
        self.declare_parameter('invert_y', False)
        self.declare_parameter('invert_z', False)

        self.use_test_loops = self.get_parameter('use_test_loops').value
        self.pause_sec = self.get_parameter('pause_sec').value
        self.speed = self.get_parameter('speed').value

        # Load hook offset parameters
        self.hook_offset_x = self.get_parameter('hook_offset_x_mm').value
        self.hook_offset_y = self.get_parameter('hook_offset_y_mm').value
        self.hook_offset_z = self.get_parameter('hook_offset_z_mm').value
        self.invert_x = self.get_parameter('invert_x').value
        self.invert_y = self.get_parameter('invert_y').value
        self.invert_z = self.get_parameter('invert_z').value

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

        # Log calibration
        self.get_logger().info(
            f'Hook offsets: ({self.hook_offset_x}, {self.hook_offset_y}, {self.hook_offset_z}) mm'
        )
        self.get_logger().info(
            f'Axis inversion: X={self.invert_x}, Y={self.invert_y}, Z={self.invert_z}'
        )

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

    def _world_to_arm_coords(self, world_x_m, world_y_m, world_z_m):
        """
        Convert world-frame position (in meters) to side arm carriage coordinates (in mm).

        Uses hook offset calibration: when arm is at (0,0,0), hook is at hook_offset in world.
        For inverted axes: arm_pos = hook_offset - world_pos
        For normal axes:   arm_pos = world_pos - hook_offset
        """
        # Convert meters to mm
        world_x_mm = world_x_m * 1000.0
        world_y_mm = world_y_m * 1000.0
        world_z_mm = world_z_m * 1000.0

        if self.invert_x:
            arm_x = self.hook_offset_x - world_x_mm
        else:
            arm_x = world_x_mm - self.hook_offset_x

        if self.invert_y:
            arm_y = self.hook_offset_y - world_y_mm
        else:
            arm_y = world_y_mm - self.hook_offset_y

        if self.invert_z:
            arm_z = self.hook_offset_z - world_z_mm
        else:
            arm_z = world_z_mm - self.hook_offset_z

        return arm_x, arm_y, arm_z

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

            # Convert world position to arm coordinates
            sa_x, sa_y, sa_z = self._world_to_arm_coords(wx, wy, wz)
            self.get_logger().info(f'  Arm coords: ({sa_x:.1f}, {sa_y:.1f}, {sa_z:.1f}) mm')
            self.get_logger().info(f'  Moving to ({sa_x:.1f}, {sa_y:.1f}, 0.0) mm (Z=0 for safety)')

            if self._move_to(sa_x, sa_y, 0.0):
                self.get_logger().info(f'  Arrived at loop {i+1}. Pausing {self.pause_sec}s...')
            else:
                self.get_logger().error(f'  Move failed for loop {i+1}')

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
