#!/usr/bin/env python3
"""
Loop Ground Truth Node
Maintains and publishes the actual/ground truth loop positions in the world frame.
Used for simulation and visualization of where loops physically exist.

Publishes:
- /loop_ground_truth (LoopGroundTruth): Loop positions for detection simulator
- /loop_ground_truth_markers (MarkerArray): Visualization markers for RViz
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from parachute_interfaces.msg import LoopGroundTruth
import random
import math


class LoopGroundTruthNode(Node):
    def __init__(self):
        super().__init__('loop_ground_truth_node')

        # Declare parameters
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('publish_rate', 10.0)  # Hz

        # Loop generation parameters
        self.declare_parameter('pattern', 'static')  # static, random, grid, line
        self.declare_parameter('num_loops', 5)
        self.declare_parameter('loop_radius', 0.015)  # meters (radius of physical loop)

        # Position bounds (in world frame)
        self.declare_parameter('x_min', 0.30)
        self.declare_parameter('x_max', 0.50)
        self.declare_parameter('y_min', -0.15)
        self.declare_parameter('y_max', 0.05)
        self.declare_parameter('z_min', -0.10)
        self.declare_parameter('z_max', 0.05)

        # Static positions (used when pattern='static')
        # Default: 5 loops in a rough line, simulating loops on parachute
        self.declare_parameter('static_positions', [
            0.35, -0.05, -0.02,   # Loop 0: x, y, z
            0.38, -0.05, 0.00,    # Loop 1
            0.41, -0.05, -0.01,   # Loop 2
            0.44, -0.05, 0.01,    # Loop 3
            0.47, -0.05, -0.02,   # Loop 4
        ])

        # Marker visualization parameters
        self.declare_parameter('marker_color_r', 0.2)
        self.declare_parameter('marker_color_g', 0.4)
        self.declare_parameter('marker_color_b', 0.9)
        self.declare_parameter('marker_color_a', 0.7)
        self.declare_parameter('marker_type', 'sphere')  # sphere, ring

        # Publishers
        self.ground_truth_pub = self.create_publisher(
            LoopGroundTruth, '/loop_ground_truth', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/loop_ground_truth_markers', 10)

        # Initialize loop positions
        self.loop_positions = []
        self.loop_radii = []
        self.loop_ids = []
        self._initialize_loops()

        # Create timer for publishing
        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_ground_truth)

        self.get_logger().info(f'Loop Ground Truth Node initialized')
        self.get_logger().info(f'  Pattern: {self.get_parameter("pattern").value}')
        self.get_logger().info(f'  Num loops: {len(self.loop_positions)}')
        self.get_logger().info(f'  Frame: {self.get_parameter("frame_id").value}')

    def _initialize_loops(self):
        """Initialize loop positions based on pattern parameter."""
        pattern = self.get_parameter('pattern').value
        num_loops = self.get_parameter('num_loops').value
        loop_radius = self.get_parameter('loop_radius').value

        x_min = self.get_parameter('x_min').value
        x_max = self.get_parameter('x_max').value
        y_min = self.get_parameter('y_min').value
        y_max = self.get_parameter('y_max').value
        z_min = self.get_parameter('z_min').value
        z_max = self.get_parameter('z_max').value

        self.loop_positions = []
        self.loop_radii = []
        self.loop_ids = []

        if pattern == 'static':
            # Use predefined static positions
            static_pos = self.get_parameter('static_positions').value
            # Parse flat list into [x,y,z] triplets
            for i in range(0, len(static_pos), 3):
                if i + 2 < len(static_pos):
                    self.loop_positions.append([
                        static_pos[i],
                        static_pos[i + 1],
                        static_pos[i + 2]
                    ])
                    self.loop_radii.append(loop_radius)
                    self.loop_ids.append(len(self.loop_ids))

        elif pattern == 'random':
            for i in range(num_loops):
                x = random.uniform(x_min, x_max)
                y = random.uniform(y_min, y_max)
                z = random.uniform(z_min, z_max)
                self.loop_positions.append([x, y, z])
                self.loop_radii.append(loop_radius)
                self.loop_ids.append(i)

        elif pattern == 'grid':
            # Create a grid of loops
            cols = int(math.ceil(math.sqrt(num_loops)))
            rows = int(math.ceil(num_loops / cols))
            x_step = (x_max - x_min) / max(cols - 1, 1)
            z_step = (z_max - z_min) / max(rows - 1, 1)
            y_center = (y_min + y_max) / 2

            count = 0
            for row in range(rows):
                for col in range(cols):
                    if count >= num_loops:
                        break
                    x = x_min + col * x_step
                    z = z_min + row * z_step
                    self.loop_positions.append([x, y_center, z])
                    self.loop_radii.append(loop_radius)
                    self.loop_ids.append(count)
                    count += 1

        elif pattern == 'line':
            # Create a horizontal line of loops
            x_step = (x_max - x_min) / max(num_loops - 1, 1)
            y_center = (y_min + y_max) / 2
            z_center = (z_min + z_max) / 2

            for i in range(num_loops):
                x = x_min + i * x_step
                self.loop_positions.append([x, y_center, z_center])
                self.loop_radii.append(loop_radius)
                self.loop_ids.append(i)

        else:
            self.get_logger().warn(f'Unknown pattern: {pattern}, using static')
            self._initialize_loops_static()

        self.get_logger().info(f'Initialized {len(self.loop_positions)} ground truth loops')

    def publish_ground_truth(self):
        """Publish ground truth message and visualization markers."""
        frame_id = self.get_parameter('frame_id').value
        stamp = self.get_clock().now().to_msg()

        # Publish LoopGroundTruth message
        gt_msg = LoopGroundTruth()
        gt_msg.header = Header()
        gt_msg.header.stamp = stamp
        gt_msg.header.frame_id = frame_id
        gt_msg.count = len(self.loop_positions)

        for pos in self.loop_positions:
            point = Point()
            point.x = pos[0]
            point.y = pos[1]
            point.z = pos[2]
            gt_msg.positions.append(point)

        gt_msg.radii = [float(r) for r in self.loop_radii]
        gt_msg.loop_ids = self.loop_ids

        self.ground_truth_pub.publish(gt_msg)

        # Publish visualization markers
        self.publish_markers(frame_id, stamp)

    def publish_markers(self, frame_id: str, stamp):
        """Publish visualization markers for ground truth loops."""
        marker_array = MarkerArray()

        # Get marker parameters
        color_r = self.get_parameter('marker_color_r').value
        color_g = self.get_parameter('marker_color_g').value
        color_b = self.get_parameter('marker_color_b').value
        color_a = self.get_parameter('marker_color_a').value
        marker_type = self.get_parameter('marker_type').value
        loop_radius = self.get_parameter('loop_radius').value

        for i, (pos, radius) in enumerate(zip(self.loop_positions, self.loop_radii)):
            # Main loop marker (sphere or torus-like visualization)
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = 'ground_truth_loops'
            marker.id = i
            marker.action = Marker.ADD

            if marker_type == 'ring':
                # Use a cylinder to represent the loop ring
                marker.type = Marker.CYLINDER
                marker.scale.x = radius * 2  # diameter
                marker.scale.y = radius * 2
                marker.scale.z = 0.003  # thin ring
            else:
                # Use sphere (default)
                marker.type = Marker.SPHERE
                marker.scale.x = radius * 2
                marker.scale.y = radius * 2
                marker.scale.z = radius * 2

            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]
            marker.pose.orientation.w = 1.0

            # Blue color for ground truth (distinguishes from green detections)
            marker.color.r = color_r
            marker.color.g = color_g
            marker.color.b = color_b
            marker.color.a = color_a

            marker_array.markers.append(marker)

            # Add text label
            text_marker = Marker()
            text_marker.header.frame_id = frame_id
            text_marker.header.stamp = stamp
            text_marker.ns = 'ground_truth_labels'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = pos[0]
            text_marker.pose.position.y = pos[1]
            text_marker.pose.position.z = pos[2] + radius * 3
            text_marker.pose.orientation.w = 1.0

            text_marker.text = f'GT{i}'
            text_marker.scale.z = 0.02  # text height

            text_marker.color.r = 0.8
            text_marker.color.g = 0.8
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = LoopGroundTruthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
