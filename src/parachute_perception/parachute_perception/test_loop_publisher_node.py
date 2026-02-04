#!/usr/bin/env python3
"""
Test Loop Publisher Node
Publishes simulated DetectedLoops messages for testing the visualization
and processing pipeline without requiring a connected camera.

Supports multiple patterns:
- random: Scattered loops within bounds (default)
- grid: Regular grid pattern for calibration
- line: Horizontal line of loops
- static: Fixed positions (no movement)
"""
import rclpy
from rclpy.node import Node
from parachute_interfaces.msg import DetectedLoops, DetectedLoop
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
import random
import math


class TestLoopPublisherNode(Node):
    def __init__(self):
        super().__init__('test_loop_publisher_node')

        # Declare parameters
        self.declare_parameter('publish_rate', 2.0)  # Hz
        self.declare_parameter('num_loops', 5)  # Number of test loops
        self.declare_parameter('pattern', 'random')  # random, grid, line, static
        self.declare_parameter('x_min', 0.05)  # X coordinate min (meters)
        self.declare_parameter('x_max', 0.35)  # X coordinate max (meters)
        self.declare_parameter('y_fixed', -0.11)  # Fixed Y depth (meters)
        self.declare_parameter('z_min', 0.02)  # Z coordinate min (meters)
        self.declare_parameter('z_max', 0.15)  # Z coordinate max (meters)
        self.declare_parameter('movement', False)  # Animate loop positions
        self.declare_parameter('movement_speed', 0.02)  # Movement speed (meters/sec)
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('base_confidence', 0.85)  # Base confidence value
        self.declare_parameter('confidence_variation', 0.1)  # Random variation

        # Publisher for detected loops
        self.publisher = self.create_publisher(DetectedLoops, '/detected_loops', 10)

        # Get parameters
        publish_rate = self.get_parameter('publish_rate').value
        self.pattern = self.get_parameter('pattern').value
        self.num_loops = self.get_parameter('num_loops').value

        # Initialize loop positions based on pattern
        self.loop_positions = []
        self.loop_velocities = []
        self._initialize_positions()

        # Create timer for publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_loops)

        self.get_logger().info(f'Test Loop Publisher Node initialized')
        self.get_logger().info(f'  Pattern: {self.pattern}')
        self.get_logger().info(f'  Num loops: {self.num_loops}')
        self.get_logger().info(f'  Publishing to: /detected_loops at {publish_rate} Hz')

    def _initialize_positions(self):
        """Initialize loop positions based on the selected pattern."""
        x_min = self.get_parameter('x_min').value
        x_max = self.get_parameter('x_max').value
        y_fixed = self.get_parameter('y_fixed').value
        z_min = self.get_parameter('z_min').value
        z_max = self.get_parameter('z_max').value
        movement = self.get_parameter('movement').value

        self.loop_positions = []
        self.loop_velocities = []

        if self.pattern == 'random':
            # Random positions within bounds
            for _ in range(self.num_loops):
                x = random.uniform(x_min, x_max)
                z = random.uniform(z_min, z_max)
                self.loop_positions.append([x, y_fixed, z])

                if movement:
                    # Random velocity direction
                    vx = random.uniform(-1, 1) * 0.01
                    vz = random.uniform(-1, 1) * 0.01
                    self.loop_velocities.append([vx, 0, vz])
                else:
                    self.loop_velocities.append([0, 0, 0])

        elif self.pattern == 'grid':
            # Regular grid pattern
            cols = int(math.ceil(math.sqrt(self.num_loops)))
            rows = int(math.ceil(self.num_loops / cols))
            x_step = (x_max - x_min) / max(cols - 1, 1)
            z_step = (z_max - z_min) / max(rows - 1, 1)

            count = 0
            for row in range(rows):
                for col in range(cols):
                    if count >= self.num_loops:
                        break
                    x = x_min + col * x_step
                    z = z_min + row * z_step
                    self.loop_positions.append([x, y_fixed, z])
                    self.loop_velocities.append([0, 0, 0])
                    count += 1

        elif self.pattern == 'line':
            # Horizontal line of loops
            x_step = (x_max - x_min) / max(self.num_loops - 1, 1)
            z_center = (z_min + z_max) / 2

            for i in range(self.num_loops):
                x = x_min + i * x_step
                self.loop_positions.append([x, y_fixed, z_center])
                self.loop_velocities.append([0, 0, 0])

        elif self.pattern == 'static':
            # Fixed positions from a predefined set
            static_positions = [
                [0.10, y_fixed, 0.05],
                [0.20, y_fixed, 0.08],
                [0.30, y_fixed, 0.05],
                [0.15, y_fixed, 0.12],
                [0.25, y_fixed, 0.10],
            ]
            for i in range(min(self.num_loops, len(static_positions))):
                self.loop_positions.append(static_positions[i])
                self.loop_velocities.append([0, 0, 0])

        else:
            self.get_logger().warn(f'Unknown pattern: {self.pattern}, using random')
            self.pattern = 'random'
            self._initialize_positions()

    def _update_positions(self):
        """Update loop positions if movement is enabled."""
        movement = self.get_parameter('movement').value
        if not movement:
            return

        x_min = self.get_parameter('x_min').value
        x_max = self.get_parameter('x_max').value
        z_min = self.get_parameter('z_min').value
        z_max = self.get_parameter('z_max').value
        speed = self.get_parameter('movement_speed').value

        for i, (pos, vel) in enumerate(zip(self.loop_positions, self.loop_velocities)):
            # Update position
            pos[0] += vel[0] * speed
            pos[2] += vel[2] * speed

            # Bounce off boundaries
            if pos[0] < x_min or pos[0] > x_max:
                vel[0] = -vel[0]
                pos[0] = max(x_min, min(x_max, pos[0]))
            if pos[2] < z_min or pos[2] > z_max:
                vel[2] = -vel[2]
                pos[2] = max(z_min, min(z_max, pos[2]))

    def publish_loops(self):
        """Publish detected loops message."""
        # Update positions if movement enabled
        self._update_positions()

        frame_id = self.get_parameter('frame_id').value
        base_confidence = self.get_parameter('base_confidence').value
        confidence_var = self.get_parameter('confidence_variation').value

        msg = DetectedLoops()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        for i, pos in enumerate(self.loop_positions):
            loop = DetectedLoop()
            loop.header = msg.header
            loop.loop_id = i
            loop.confidence = min(1.0, max(0.0,
                base_confidence + random.uniform(-confidence_var, confidence_var)))

            loop.pose = PoseStamped()
            loop.pose.header = msg.header
            loop.pose.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
            loop.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            msg.loops.append(loop)

        msg.total_count = len(msg.loops)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestLoopPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
