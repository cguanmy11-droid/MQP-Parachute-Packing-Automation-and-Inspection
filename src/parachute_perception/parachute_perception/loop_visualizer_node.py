#!/usr/bin/env python3
"""
Loop Visualizer Node
Visualizes detected parachute loops in RViz as sphere markers with a reference grid.
Subscribes to DetectedLoops messages and publishes MarkerArray for RViz display.
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from parachute_interfaces.msg import DetectedLoops


class LoopVisualizerNode(Node):
    def __init__(self):
        super().__init__('loop_visualizer_node')

        # Declare parameters
        self.declare_parameter('marker_scale', 0.015)  # Loop marker radius (meters)
        self.declare_parameter('marker_color_r', 0.2)
        self.declare_parameter('marker_color_g', 0.8)
        self.declare_parameter('marker_color_b', 0.2)
        self.declare_parameter('marker_color_a', 1.0)
        self.declare_parameter('selected_color_r', 1.0)  # Color for selected/target loop
        self.declare_parameter('selected_color_g', 0.2)
        self.declare_parameter('selected_color_b', 0.2)
        self.declare_parameter('grid_enabled', True)
        self.declare_parameter('grid_size_x', 0.4)  # Grid width (meters)
        self.declare_parameter('grid_size_y', 0.3)  # Grid height (meters)
        self.declare_parameter('grid_cells_x', 16)  # Number of grid cells in X
        self.declare_parameter('grid_cells_y', 12)  # Number of grid cells in Y
        self.declare_parameter('grid_offset_z', 0.0)  # Z offset for grid plane
        self.declare_parameter('pixel_scale', 0.001)  # Meters per pixel (for coordinate conversion)
        self.declare_parameter('frame_id', 'camera_frame')  # TF frame for visualization

        # Publisher for RViz markers
        self.marker_pub = self.create_publisher(MarkerArray, '/loop_markers', 10)

        # Subscriber for detected loops
        self.loops_sub = self.create_subscription(
            DetectedLoops,
            '/detected_loops',
            self.loops_callback,
            10
        )

        # Store last received loops for visualization
        self.current_loops = []
        self.last_msg_time = self.get_clock().now()

        # Publish markers at steady rate
        self.timer = self.create_timer(0.1, self.publish_markers)  # 10 Hz

        self.get_logger().info('Loop Visualizer Node initialized')
        self.get_logger().info(f'  Subscribing to: /detected_loops')
        self.get_logger().info(f'  Publishing to: /loop_markers')

    def loops_callback(self, msg: DetectedLoops):
        """Handle incoming detected loops."""
        self.current_loops = msg.loops
        self.last_msg_time = self.get_clock().now()

    def publish_markers(self):
        """Publish visualization markers for all detected loops and the reference grid."""
        marker_array = MarkerArray()

        frame_id = self.get_parameter('frame_id').value
        stamp = self.get_clock().now().to_msg()

        # Add grid marker if enabled
        if self.get_parameter('grid_enabled').value:
            grid_marker = self.create_grid_marker(frame_id, stamp)
            marker_array.markers.append(grid_marker)

        # Get marker parameters
        scale = self.get_parameter('marker_scale').value
        color = ColorRGBA(
            r=self.get_parameter('marker_color_r').value,
            g=self.get_parameter('marker_color_g').value,
            b=self.get_parameter('marker_color_b').value,
            a=self.get_parameter('marker_color_a').value
        )
        selected_color = ColorRGBA(
            r=self.get_parameter('selected_color_r').value,
            g=self.get_parameter('selected_color_g').value,
            b=self.get_parameter('selected_color_b').value,
            a=self.get_parameter('marker_color_a').value
        )

        # Clear old markers if no loops detected
        time_since_msg = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if time_since_msg > 2.0 or len(self.current_loops) == 0:
            # Publish delete markers for cleanup
            delete_marker = Marker()
            delete_marker.header.frame_id = frame_id
            delete_marker.header.stamp = stamp
            delete_marker.ns = 'detected_loops'
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)
            self.marker_pub.publish(marker_array)
            return

        # Find the rightmost loop (highest X) to highlight as target
        rightmost_idx = -1
        max_x = float('-inf')
        for i, loop in enumerate(self.current_loops):
            if loop.pose.pose.position.x > max_x:
                max_x = loop.pose.pose.position.x
                rightmost_idx = i

        # Create sphere markers for each detected loop
        for i, loop in enumerate(self.current_loops):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = 'detected_loops'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Position from detected loop
            marker.pose.position.x = loop.pose.pose.position.x
            marker.pose.position.y = loop.pose.pose.position.y
            marker.pose.position.z = loop.pose.pose.position.z
            marker.pose.orientation.w = 1.0

            # Scale (sphere diameter)
            marker.scale.x = scale * 2
            marker.scale.y = scale * 2
            marker.scale.z = scale * 2

            # Color - highlight rightmost loop as target
            if i == rightmost_idx:
                marker.color = selected_color
            else:
                # Adjust color based on confidence if available
                if loop.confidence > 0:
                    marker.color.r = color.r
                    marker.color.g = color.g * loop.confidence
                    marker.color.b = color.b
                    marker.color.a = color.a
                else:
                    marker.color = color

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 500000000  # 0.5 seconds

            marker_array.markers.append(marker)

        # Add text labels for loop IDs
        for i, loop in enumerate(self.current_loops):
            text_marker = Marker()
            text_marker.header.frame_id = frame_id
            text_marker.header.stamp = stamp
            text_marker.ns = 'loop_labels'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            # Position slightly above the loop
            text_marker.pose.position.x = loop.pose.pose.position.x
            text_marker.pose.position.y = loop.pose.pose.position.y
            text_marker.pose.position.z = loop.pose.pose.position.z + scale * 3
            text_marker.pose.orientation.w = 1.0

            # Text content
            if loop.confidence > 0:
                text_marker.text = f'{loop.loop_id}: {loop.confidence:.0%}'
            else:
                text_marker.text = f'{loop.loop_id}'

            text_marker.scale.z = scale * 1.5  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.lifetime.sec = 0
            text_marker.lifetime.nanosec = 500000000

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

    def create_grid_marker(self, frame_id: str, stamp) -> Marker:
        """Create a reference grid marker for the camera frame."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'camera_grid'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        # Grid parameters
        size_x = self.get_parameter('grid_size_x').value
        size_y = self.get_parameter('grid_size_y').value
        cells_x = self.get_parameter('grid_cells_x').value
        cells_y = self.get_parameter('grid_cells_y').value
        offset_z = self.get_parameter('grid_offset_z').value

        # Grid spans from -size/2 to +size/2
        x_min, x_max = -size_x / 2, size_x / 2
        y_min, y_max = -size_y / 2, size_y / 2

        cell_width = size_x / cells_x
        cell_height = size_y / cells_y

        # Vertical lines
        for i in range(cells_x + 1):
            x = x_min + i * cell_width
            marker.points.append(Point(x=x, y=y_min, z=offset_z))
            marker.points.append(Point(x=x, y=y_max, z=offset_z))

        # Horizontal lines
        for i in range(cells_y + 1):
            y = y_min + i * cell_height
            marker.points.append(Point(x=x_min, y=y, z=offset_z))
            marker.points.append(Point(x=x_max, y=y, z=offset_z))

        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.001  # Line width

        # Grid color (light blue)
        marker.color.r = 0.3
        marker.color.g = 0.5
        marker.color.b = 0.8
        marker.color.a = 0.5

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = LoopVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
