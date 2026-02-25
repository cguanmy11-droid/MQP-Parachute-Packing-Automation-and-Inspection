#!/usr/bin/env python3
"""
Detection Simulator Node
Simulates what a camera would detect based on ground truth loop positions
and the current camera position/orientation.

This node:
1. Subscribes to /loop_ground_truth (actual loop positions in world frame)
2. Looks up TF: world -> camera_frame to get camera pose
3. Transforms loop positions to camera frame
4. Checks if loops are within camera FOV and detection range
5. Publishes DetectedLoops in camera_frame (same as real YOLO would)

Subscriptions:
- /loop_ground_truth (LoopGroundTruth): Actual loop positions in world frame

Publications:
- /detected_loops (DetectedLoops): Simulated detections in camera_frame
"""
import rclpy
from rclpy.node import Node
from parachute_interfaces.msg import LoopGroundTruth, DetectedLoops, DetectedLoop
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
import math
import random


class DetectionSimulatorNode(Node):
    def __init__(self):
        super().__init__('detection_simulator_node')

        # Declare parameters
        # Camera frame configuration
        self.declare_parameter('camera_frame_id', 'camera_frame')
        self.declare_parameter('world_frame_id', 'world')

        # Camera FOV parameters (camera looks along +Z axis in camera_frame)
        self.declare_parameter('camera_fov_horizontal', 60.0)  # degrees
        self.declare_parameter('camera_fov_vertical', 45.0)    # degrees
        self.declare_parameter('max_detection_range', 0.5)     # meters
        self.declare_parameter('min_detection_range', 0.02)    # meters

        # Detection simulation parameters
        self.declare_parameter('detection_noise_stddev', 0.00005)  # meters
        self.declare_parameter('confidence_base', 0.90)
        self.declare_parameter('confidence_noise', 0.05)
        self.declare_parameter('false_negative_rate', 0.0)  # Probability of missing a visible loop

        # Debug parameters
        self.declare_parameter('debug_bypass_fov', False)  # Bypass FOV/range checks for debugging
        self.declare_parameter('debug_verbose', False)  # Print verbose debug info

        # Publishing rate
        self.declare_parameter('publish_rate', 5.0)  # Hz

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Store latest ground truth
        self.latest_ground_truth = None

        # Subscriber for ground truth
        self.ground_truth_sub = self.create_subscription(
            LoopGroundTruth,
            '/loop_ground_truth',
            self.ground_truth_callback,
            10
        )

        # Publisher for simulated detections
        self.detection_pub = self.create_publisher(
            DetectedLoops,
            '/detected_loops',
            10
        )

        # Timer for publishing detections
        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / publish_rate, self.simulate_detection)

        # self.get_logger().info('Detection Simulator Node initialized')
        # self.get_logger().info(f'  Camera frame: {self.get_parameter("camera_frame_id").value}')
        # self.get_logger().info(f'  FOV: {self.get_parameter("camera_fov_horizontal").value}° x {self.get_parameter("camera_fov_vertical").value}°')
        # self.get_logger().info(f'  Range: {self.get_parameter("min_detection_range").value}m - {self.get_parameter("max_detection_range").value}m')

    def ground_truth_callback(self, msg: LoopGroundTruth):
        """Store latest ground truth positions."""
        self.latest_ground_truth = msg

    def simulate_detection(self):
        """Simulate camera detection based on ground truth and camera pose."""
        if self.latest_ground_truth is None:
            return

        camera_frame = self.get_parameter('camera_frame_id').value
        world_frame = self.get_parameter('world_frame_id').value

        # Try to get transform from world to camera_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                camera_frame,
                world_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
            return

        # Get parameters
        fov_h = math.radians(self.get_parameter('camera_fov_horizontal').value)
        fov_v = math.radians(self.get_parameter('camera_fov_vertical').value)
        min_range = self.get_parameter('min_detection_range').value
        max_range = self.get_parameter('max_detection_range').value
        noise_stddev = self.get_parameter('detection_noise_stddev').value
        conf_base = self.get_parameter('confidence_base').value
        conf_noise = self.get_parameter('confidence_noise').value
        false_neg_rate = self.get_parameter('false_negative_rate').value
        debug_bypass = self.get_parameter('debug_bypass_fov').value
        debug_verbose = self.get_parameter('debug_verbose').value

        # Create DetectedLoops message
        detected_msg = DetectedLoops()
        detected_msg.header = Header()
        detected_msg.header.stamp = self.get_clock().now().to_msg()
        detected_msg.header.frame_id = camera_frame

        # Process each ground truth loop
        gt = self.latest_ground_truth
        for i, (pos, loop_id) in enumerate(zip(gt.positions, gt.loop_ids)):
            # Transform point from world to camera frame
            point_world = PointStamped()
            point_world.header.frame_id = world_frame
            point_world.header.stamp = self.get_clock().now().to_msg()
            point_world.point = pos

            try:
                point_camera = do_transform_point(point_world, transform)
            except Exception as e:
                self.get_logger().debug(f'Transform failed for loop {loop_id}: {e}')
                continue

            # Check if loop is visible (in FOV and range)
            # Camera convention: +Z is forward (into scene), +X is right, +Y is down
            x = point_camera.point.x
            y = point_camera.point.y
            z = point_camera.point.z

            # Debug: always log transformed position when verbose
            if debug_verbose:
                self.get_logger().info(
                    f'Loop {loop_id} in camera_frame: x={x:.3f}, y={y:.3f}, z={z:.3f}')

            # Check range (z should be positive - in front of camera)
            # Use abs(z) for range check if bypassing, to detect loops behind camera
            z_for_range = abs(z) if debug_bypass else z
            if z_for_range < min_range or z_for_range > max_range:
                if debug_verbose:
                    self.get_logger().warn(
                        f'Loop {loop_id} REJECTED: z={z:.3f} outside range [{min_range}, {max_range}]')
                if not debug_bypass:
                    continue

            # Check FOV (angle from optical axis)
            # Use abs(z) for angle calculation if z is negative (camera pointing wrong way)
            z_for_angle = abs(z) if z < 0 else z
            angle_h = math.atan2(x, z_for_angle)  # Horizontal angle
            angle_v = math.atan2(y, z_for_angle)  # Vertical angle

            if abs(angle_h) > fov_h / 2 or abs(angle_v) > fov_v / 2:
                if debug_verbose:
                    self.get_logger().warn(
                        f'Loop {loop_id} REJECTED: angles h={math.degrees(angle_h):.1f}° v={math.degrees(angle_v):.1f}° outside FOV')
                if not debug_bypass:
                    continue

            # Warn if loop is behind camera but we're bypassing checks
            if debug_bypass and z < 0:
                self.get_logger().warn(
                    f'Loop {loop_id} DETECTED despite being behind camera (z={z:.3f}). Fix camera orientation!')

            # Simulate false negative (randomly miss some detections)
            if random.random() < false_neg_rate:
                continue

            # Loop is visible - create detection
            detected_loop = DetectedLoop()
            detected_loop.header = detected_msg.header
            detected_loop.loop_id = loop_id

            # Add detection noise
            noisy_x = x + random.gauss(0, noise_stddev)
            noisy_y = y + random.gauss(0, noise_stddev)
            noisy_z = z + random.gauss(0, noise_stddev)

            detected_loop.pose = PoseStamped()
            detected_loop.pose.header = detected_msg.header
            detected_loop.pose.pose.position = Point(x=noisy_x, y=noisy_y, z=noisy_z)
            detected_loop.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            # Calculate confidence based on distance and angle
            # Closer and more centered = higher confidence
            distance_factor = 1.0 - (z - min_range) / (max_range - min_range)
            angle_factor = 1.0 - (abs(angle_h) + abs(angle_v)) / (fov_h / 2 + fov_v / 2)
            base_confidence = conf_base * (0.7 + 0.3 * distance_factor * angle_factor)
            detected_loop.confidence = min(1.0, max(0.0,
                base_confidence + random.uniform(-conf_noise, conf_noise)))

            detected_msg.loops.append(detected_loop)

        detected_msg.total_count = len(detected_msg.loops)

        # Publish detections
        self.detection_pub.publish(detected_msg)

        # Always log detection status for debugging
        if detected_msg.total_count > 0:
            # self.get_logger().info(f'Published {detected_msg.total_count} simulated detections')
            pass
        else:
            self.get_logger().debug(f'No loops in camera FOV (checked {len(gt.positions)} ground truth loops)')


def main(args=None):
    rclpy.init(args=args)
    node = DetectionSimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
