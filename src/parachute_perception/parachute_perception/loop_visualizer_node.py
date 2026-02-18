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
from builtin_interfaces.msg import Time


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
        self.grid_pub = self.create_publisher(MarkerArray, '/camera_fov_grid', 10)

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
        self._had_detections = False

        # Publish markers at steady rate
        self.timer = self.create_timer(0.1, self.publish_markers)  # 10 Hz
        self.grid_timer = self.create_timer(0.5, self.publish_grid)  # slower rate is fine

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

            # Longer lifetime to smooth over intermittent detections
            marker.lifetime.sec = 1
            marker.lifetime.nanosec = 0

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

            # Longer lifetime to smooth over intermittent detections
            text_marker.lifetime.sec = 1
            text_marker.lifetime.nanosec = 0

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

    def publish_grid(self):
        if not self.get_parameter('grid_enabled').value:
            return

        # Use actual current time (not Time() which is zero and causes jitter)
        stamp = self.get_clock().now().to_msg()

        marker_array = MarkerArray()
        grid_marker = self.create_grid_marker('camera_frame', stamp)
        if grid_marker:
            # Use lifetime of 0 (persistent) - marker stays until explicitly deleted
            # This prevents blinking from timer jitter
            grid_marker.lifetime.sec = 0
            grid_marker.lifetime.nanosec = 0
            marker_array.markers.append(grid_marker)

        grid_z = self.get_parameter('grid_offset_z').value

        if len(self.current_loops) > 0:
            self._had_detections = True
            for i, loop in enumerate(self.current_loops):
                pos = loop.pose.pose.position

                if pos.z <= 0.001:
                    continue

                dot = Marker()
                dot.header.frame_id = 'camera_frame'
                dot.header.stamp = stamp
                dot.ns = 'grid_detections'
                dot.id = i
                dot.type = Marker.SPHERE
                dot.action = Marker.ADD
                dot.pose.position = Point(x=pos.x, y=pos.y, z=grid_z)
                dot.pose.orientation.w = 1.0
                dot.scale.x = 0.008
                dot.scale.y = 0.008
                dot.scale.z = 0.002
                dot.color = ColorRGBA(r=1.0, g=0.3, b=0.3, a=0.9)
                # Longer lifetime (2 sec) to smooth over intermittent detections
                dot.lifetime.sec = 2
                dot.lifetime.nanosec = 0
                marker_array.markers.append(dot)
        elif getattr(self, '_had_detections', False):
            # Only clear once when detections disappear
            clear = Marker()
            clear.header.frame_id = 'camera_frame'
            clear.header.stamp = stamp
            clear.ns = 'grid_detections'
            clear.action = Marker.DELETEALL
            marker_array.markers.append(clear)
            self._had_detections = False

        self.grid_pub.publish(marker_array)

    def create_grid_marker(self, frame_id: str, stamp) -> Marker:
        """Create a reference grid marker in camera_frame to show camera field of view.

        The grid is placed in camera_frame so it moves with the camera/hook,
        representing what the camera can see. Loops appearing within this grid
        are visible to the camera.
        """
        marker = Marker()
        marker.header.frame_id = 'camera_frame'  # Moves with the camera
        marker.header.stamp = stamp
        marker.ns = 'camera_fov_grid'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        # Grid parameters - represents camera FOV area
        size_x = self.get_parameter('grid_size_x').value  # Width of FOV
        size_y = self.get_parameter('grid_size_y').value  # Height of FOV
        cells_x = self.get_parameter('grid_cells_x').value
        cells_y = self.get_parameter('grid_cells_y').value
        offset_z = self.get_parameter('grid_offset_z').value  # Distance from camera

        # Grid in camera_frame: X-Y plane at Z offset (looking forward)
        # Camera typically looks along +Z or -Z axis
        x_min, x_max = -size_x / 2, size_x / 2
        y_min, y_max = -size_y / 2, size_y / 2

        cell_width = size_x / cells_x
        cell_height = size_y / cells_y

        # Vertical lines (along Y in camera frame)
        for i in range(cells_x + 1):
            x = x_min + i * cell_width
            marker.points.append(Point(x=x, y=y_min, z=offset_z))
            marker.points.append(Point(x=x, y=y_max, z=offset_z))

        # Horizontal lines (along X in camera frame)
        for i in range(cells_y + 1):
            y = y_min + i * cell_height
            marker.points.append(Point(x=x_min, y=y, z=offset_z))
            marker.points.append(Point(x=x_max, y=y, z=offset_z))

        # Add corner lines to show frustum (optional depth visualization)
        # Lines from camera origin to grid corners
        corners = [
            (x_min, y_min), (x_max, y_min),
            (x_max, y_max), (x_min, y_max)
        ]
        for cx, cy in corners:
            marker.points.append(Point(x=0.0, y=0.0, z=0.0))  # Camera origin
            marker.points.append(Point(x=cx, y=cy, z=offset_z))

        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.002  # Line width

        # Grid color (cyan for camera FOV)
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.7

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = LoopVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
