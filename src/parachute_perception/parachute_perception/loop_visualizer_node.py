#!/usr/bin/env python3
"""
Loop Visualizer Node
Visualizes detected parachute loops in RViz as sphere markers.
Transforms detections from camera_frame to world frame for consistent display.

Subscribes to DetectedLoops messages (in camera_frame) and publishes
MarkerArray (in world frame) for RViz display.
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import ColorRGBA
from parachute_interfaces.msg import DetectedLoops
import tf2_ros
from tf2_geometry_msgs import do_transform_point


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

        # Frame configuration
        self.declare_parameter('input_frame_id', 'camera_frame')  # Frame detections arrive in
        self.declare_parameter('output_frame_id', 'world')  # Frame to publish markers in

        # Grid parameters (grid stays in camera_frame to show camera view)
        self.declare_parameter('grid_enabled', True)
        self.declare_parameter('grid_size_x', 0.4)  # Grid width (meters)
        self.declare_parameter('grid_size_y', 0.3)  # Grid height (meters)
        self.declare_parameter('grid_cells_x', 16)
        self.declare_parameter('grid_cells_y', 12)
        self.declare_parameter('grid_offset_z', 0.1)  # Z offset in camera_frame (forward)

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher for RViz markers
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_loop_markers', 10)

        # Subscriber for detected loops
        self.loops_sub = self.create_subscription(
            DetectedLoops,
            '/detected_loops',
            self.loops_callback,
            10
        )

        # Store last received loops for visualization
        self.current_loops = []
        self.current_loops_frame = ''
        self.last_msg_time = self.get_clock().now()

        # Publish markers at steady rate
        self.timer = self.create_timer(0.1, self.publish_markers)  # 10 Hz

        self.get_logger().info('Loop Visualizer Node initialized')
        self.get_logger().info(f'  Input frame: {self.get_parameter("input_frame_id").value}')
        self.get_logger().info(f'  Output frame: {self.get_parameter("output_frame_id").value}')
        self.get_logger().info(f'  Publishing to: /detected_loop_markers')

    def loops_callback(self, msg: DetectedLoops):
        """Handle incoming detected loops."""
        self.current_loops = msg.loops
        self.current_loops_frame = msg.header.frame_id
        self.last_msg_time = self.get_clock().now()

    def transform_point_to_output_frame(self, point, source_frame):
        """Transform a point from source_frame to output_frame."""
        output_frame = self.get_parameter('output_frame_id').value

        if source_frame == output_frame:
            # No transform needed
            return point, True

        try:
            transform = self.tf_buffer.lookup_transform(
                output_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            point_stamped = PointStamped()
            point_stamped.header.frame_id = source_frame
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point = point

            transformed = do_transform_point(point_stamped, transform)
            return transformed.point, True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
            return point, False

    def publish_markers(self):
        """Publish visualization markers for all detected loops."""
        marker_array = MarkerArray()

        output_frame = self.get_parameter('output_frame_id').value
        input_frame = self.get_parameter('input_frame_id').value
        stamp = self.get_clock().now().to_msg()

        # Add grid marker in world frame (X-Z plane)
        # Grid is now in world frame so no TF lookup needed
        if self.get_parameter('grid_enabled').value:
            grid_marker = self.create_grid_marker(input_frame, stamp)
            if grid_marker:
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
            delete_marker.header.frame_id = output_frame
            delete_marker.header.stamp = stamp
            delete_marker.ns = 'detected_loops'
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)

            delete_labels = Marker()
            delete_labels.header.frame_id = output_frame
            delete_labels.header.stamp = stamp
            delete_labels.ns = 'loop_labels'
            delete_labels.action = Marker.DELETEALL
            marker_array.markers.append(delete_labels)

            self.marker_pub.publish(marker_array)
            return

        # Determine source frame for detections
        source_frame = self.current_loops_frame if self.current_loops_frame else input_frame

        # Transform all loop positions to output frame first
        transformed_positions = []
        for loop in self.current_loops:
            pos = loop.pose.pose.position
            transformed_pos, success = self.transform_point_to_output_frame(pos, source_frame)
            if success:
                transformed_positions.append((loop, transformed_pos))

        if not transformed_positions:
            return

        # Find the rightmost loop (highest X in output frame) to highlight as target
        rightmost_idx = -1
        max_x = float('-inf')
        for i, (loop, pos) in enumerate(transformed_positions):
            if pos.x > max_x:
                max_x = pos.x
                rightmost_idx = i

        # Create sphere markers for each detected loop
        for i, (loop, pos) in enumerate(transformed_positions):
            marker = Marker()
            marker.header.frame_id = output_frame
            marker.header.stamp = stamp
            marker.ns = 'detected_loops'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Position (transformed to output frame)
            marker.pose.position = pos
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
        for i, (loop, pos) in enumerate(transformed_positions):
            text_marker = Marker()
            text_marker.header.frame_id = output_frame
            text_marker.header.stamp = stamp
            text_marker.ns = 'loop_labels'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            # Position slightly above the loop
            text_marker.pose.position.x = pos.x
            text_marker.pose.position.y = pos.y
            text_marker.pose.position.z = pos.z + scale * 3
            text_marker.pose.orientation.w = 1.0

            # Text content
            if loop.confidence > 0:
                text_marker.text = f'D{loop.loop_id}: {loop.confidence:.0%}'
            else:
                text_marker.text = f'D{loop.loop_id}'

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
        """Create a reference grid marker in world frame aligned with X-Z plane."""
        marker = Marker()
        marker.header.frame_id = 'world'  # Always in world frame for stability
        marker.header.stamp = stamp
        marker.ns = 'camera_grid'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        # Grid parameters
        size_x = self.get_parameter('grid_size_x').value
        size_z = self.get_parameter('grid_size_y').value  # Use Y param for Z size
        cells_x = self.get_parameter('grid_cells_x').value
        cells_z = self.get_parameter('grid_cells_y').value
        offset_y = self.get_parameter('grid_offset_z').value  # Y offset in world

        # Grid spans X-Z plane at a fixed Y position
        x_min, x_max = 0.2, 0.2 + size_x  # Position grid in front of robot
        z_min, z_max = -size_z / 2, size_z / 2

        cell_width = size_x / cells_x
        cell_height = size_z / cells_z

        # Vertical lines (along Z)
        for i in range(cells_x + 1):
            x = x_min + i * cell_width
            marker.points.append(Point(x=x, y=offset_y, z=z_min))
            marker.points.append(Point(x=x, y=offset_y, z=z_max))

        # Horizontal lines (along X)
        for i in range(cells_z + 1):
            z = z_min + i * cell_height
            marker.points.append(Point(x=x_min, y=offset_y, z=z))
            marker.points.append(Point(x=x_max, y=offset_y, z=z))

        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.002  # Line width

        # Grid color (light blue)
        marker.color.r = 0.3
        marker.color.g = 0.5
        marker.color.b = 0.8
        marker.color.a = 0.6

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = LoopVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
