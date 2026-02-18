#!/usr/bin/env python3
"""
Loop Calibration Node
Calibrates loop positions from real camera detections.

This node:
1. Homes the side arm to a known position
2. Collects detections over a configurable duration
3. Clusters nearby detections using spatial tolerance
4. Filters by detection count threshold
5. Publishes calibrated positions as ground truth

Usage:
    # Start calibration via service call
    ros2 service call /calibrate_loops std_srvs/srv/Trigger

    # Or with custom parameters
    ros2 run parachute_perception loop_calibration_node --ros-args \
        -p collection_duration:=5.0 \
        -p spatial_tolerance:=0.01 \
        -p min_detection_count:=10
"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from parachute_interfaces.msg import DetectedLoops, LoopGroundTruth
from parachute_interfaces.srv import MoveToPosition
import numpy as np
import json
import os
from datetime import datetime


class LoopCalibrationNode(Node):
    def __init__(self):
        super().__init__('loop_calibration_node')

        # Callback group for async service calls
        self.callback_group = ReentrantCallbackGroup()

        # ==================== PARAMETERS ====================
        # Calibration settings
        self.declare_parameter('collection_duration', 5.0)  # seconds
        self.declare_parameter('spatial_tolerance', 0.008)  # meters (8mm)
        self.declare_parameter('min_detection_count', 15)   # minimum detections to be valid
        self.declare_parameter('home_before_calibration', True)

        # Frame settings
        self.declare_parameter('camera_frame_id', 'camera_frame')
        self.declare_parameter('world_frame_id', 'world')

        # Output settings
        self.declare_parameter('save_to_file', True)
        self.declare_parameter('save_directory', '/tmp/loop_calibration')
        self.declare_parameter('loop_radius', 0.015)  # Default loop radius (meters)

        # Publishing
        self.declare_parameter('publish_rate', 10.0)  # Hz

        # ==================== STATE ====================
        self.is_calibrating = False
        self.calibration_start_time = None
        self.raw_detections = []  # List of (x, y, z, confidence) tuples
        self.calibrated_loops = []  # List of {'position': [x,y,z], 'count': n, 'confidence': avg}
        self.has_calibrated_data = False

        # ==================== SUBSCRIBERS ====================
        self.detection_sub = self.create_subscription(
            DetectedLoops,
            '/detected_loops',
            self.detection_callback,
            10
        )

        # ==================== PUBLISHERS ====================
        self.ground_truth_pub = self.create_publisher(
            LoopGroundTruth,
            '/loop_ground_truth',
            10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/loop_ground_truth_markers',
            10
        )
        self.status_pub = self.create_publisher(
            String,
            '/calibration_status',
            10
        )

        # ==================== SERVICES ====================
        self.calibrate_srv = self.create_service(
            Trigger,
            '/calibrate_loops',
            self.calibrate_callback,
            callback_group=self.callback_group
        )

        self.save_srv = self.create_service(
            Trigger,
            '/save_calibration',
            self.save_callback,
            callback_group=self.callback_group
        )

        self.load_srv = self.create_service(
            Trigger,
            '/load_calibration',
            self.load_callback,
            callback_group=self.callback_group
        )

        # ==================== SERVICE CLIENTS ====================
        self.move_client = self.create_client(
            MoveToPosition,
            '/side_arm/move_to_position',
            callback_group=self.callback_group
        )

        # ==================== TIMERS ====================
        publish_rate = self.get_parameter('publish_rate').value
        self.publish_timer = self.create_timer(1.0 / publish_rate, self.publish_ground_truth)

        self.get_logger().info('Loop Calibration Node initialized')
        self.get_logger().info(f'  Collection duration: {self.get_parameter("collection_duration").value}s')
        self.get_logger().info(f'  Spatial tolerance: {self.get_parameter("spatial_tolerance").value*1000:.1f}mm')
        self.get_logger().info(f'  Min detection count: {self.get_parameter("min_detection_count").value}')
        self.get_logger().info('  Call /calibrate_loops service to start calibration')

    def detection_callback(self, msg: DetectedLoops):
        """Collect detections during calibration."""
        if not self.is_calibrating:
            return

        # Check if calibration time has elapsed
        elapsed = (self.get_clock().now() - self.calibration_start_time).nanoseconds / 1e9
        duration = self.get_parameter('collection_duration').value

        if elapsed >= duration:
            self.finish_calibration()
            return

        # Store detections (in camera frame)
        for loop in msg.loops:
            pos = loop.pose.pose.position
            self.raw_detections.append((pos.x, pos.y, pos.z, loop.confidence))

        # Publish status
        count = len(self.raw_detections)
        remaining = duration - elapsed
        status_msg = String()
        status_msg.data = f'Calibrating: {count} detections, {remaining:.1f}s remaining'
        self.status_pub.publish(status_msg)

    def calibrate_callback(self, request, response):
        """Service callback to start calibration."""
        if self.is_calibrating:
            response.success = False
            response.message = 'Calibration already in progress'
            return response

        self.get_logger().info('Starting loop calibration...')

        # Home side arm if configured
        if self.get_parameter('home_before_calibration').value:
            if not self.home_side_arm():
                response.success = False
                response.message = 'Failed to home side arm'
                return response

        # Start collecting detections
        self.is_calibrating = True
        self.calibration_start_time = self.get_clock().now()
        self.raw_detections = []

        # Wait for collection to complete
        duration = self.get_parameter('collection_duration').value
        self.get_logger().info(f'Collecting detections for {duration}s...')

        # Use a simple polling loop (service is async)
        import time
        while self.is_calibrating:
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)

        response.success = True
        response.message = f'Calibration complete: {len(self.calibrated_loops)} loops found'
        return response

    def home_side_arm(self) -> bool:
        """Move side arm to home position (0, 0, 0)."""
        if not self.move_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Side arm move service not available')
            return True  # Continue anyway

        self.get_logger().info('Homing side arm to (0, 0, 0)...')

        request = MoveToPosition.Request()
        request.x_mm = 0.0
        request.y_mm = 0.0
        request.z_mm = 0.0
        request.speed_scale = 0.5

        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if future.result() is not None:
            result = future.result()
            if result.success:
                self.get_logger().info('Side arm homed successfully')
                # Wait a moment for arm to settle
                import time
                time.sleep(1.0)
                return True
            else:
                self.get_logger().error(f'Homing failed: {result.message}')
                return False
        else:
            self.get_logger().error('Homing service call failed')
            return False

    def finish_calibration(self):
        """Process collected detections and create calibrated loop positions."""
        self.is_calibrating = False

        self.get_logger().info(f'Processing {len(self.raw_detections)} raw detections...')

        if len(self.raw_detections) == 0:
            self.get_logger().warn('No detections collected during calibration')
            status_msg = String()
            status_msg.data = 'Calibration failed: no detections'
            self.status_pub.publish(status_msg)
            return

        # Cluster detections
        tolerance = self.get_parameter('spatial_tolerance').value
        min_count = self.get_parameter('min_detection_count').value

        clusters = self.cluster_detections(self.raw_detections, tolerance)

        # Filter by count threshold
        self.calibrated_loops = []
        for cluster in clusters:
            if cluster['count'] >= min_count:
                self.calibrated_loops.append(cluster)
                self.get_logger().info(
                    f'  Loop {len(self.calibrated_loops)-1}: '
                    f'pos=({cluster["position"][0]:.4f}, {cluster["position"][1]:.4f}, {cluster["position"][2]:.4f}), '
                    f'count={cluster["count"]}, conf={cluster["confidence"]:.2f}'
                )

        # Sort by X position (left to right in camera frame)
        self.calibrated_loops.sort(key=lambda c: c['position'][0])

        self.has_calibrated_data = len(self.calibrated_loops) > 0

        # Log results
        self.get_logger().info(f'Calibration complete: {len(self.calibrated_loops)} loops found')
        self.get_logger().info(f'  (Rejected {len(clusters) - len(self.calibrated_loops)} clusters with < {min_count} detections)')

        # Publish status
        status_msg = String()
        status_msg.data = f'Calibration complete: {len(self.calibrated_loops)} loops'
        self.status_pub.publish(status_msg)

        # Auto-save if configured
        if self.get_parameter('save_to_file').value and self.has_calibrated_data:
            self.save_calibration()

    def cluster_detections(self, detections: list, tolerance: float) -> list:
        """
        Cluster detections using simple distance-based grouping.

        Args:
            detections: List of (x, y, z, confidence) tuples
            tolerance: Maximum distance to consider same cluster (meters)

        Returns:
            List of cluster dicts with 'position', 'count', 'confidence'
        """
        if not detections:
            return []

        # Convert to numpy for easier math
        points = np.array([[d[0], d[1], d[2]] for d in detections])
        confidences = np.array([d[3] for d in detections])

        clusters = []
        used = np.zeros(len(points), dtype=bool)

        for i in range(len(points)):
            if used[i]:
                continue

            # Find all points within tolerance of this point
            distances = np.linalg.norm(points - points[i], axis=1)
            in_cluster = distances < tolerance

            # Mark as used
            used |= in_cluster

            # Compute cluster center (mean of all points in cluster)
            cluster_points = points[in_cluster]
            cluster_confs = confidences[in_cluster]

            center = np.mean(cluster_points, axis=0)
            avg_conf = np.mean(cluster_confs)
            count = np.sum(in_cluster)

            clusters.append({
                'position': center.tolist(),
                'count': int(count),
                'confidence': float(avg_conf)
            })

        return clusters

    def publish_ground_truth(self):
        """Publish calibrated loops as ground truth."""
        if not self.has_calibrated_data:
            return

        # Create LoopGroundTruth message
        msg = LoopGroundTruth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('camera_frame_id').value

        loop_radius = self.get_parameter('loop_radius').value

        for i, loop in enumerate(self.calibrated_loops):
            pos = Point()
            pos.x = loop['position'][0]
            pos.y = loop['position'][1]
            pos.z = loop['position'][2]
            msg.positions.append(pos)
            msg.radii.append(loop_radius)
            msg.loop_ids.append(i)

        msg.count = len(self.calibrated_loops)
        self.ground_truth_pub.publish(msg)

        # Publish visualization markers
        self.publish_markers()

    def publish_markers(self):
        """Publish RViz markers for calibrated loops."""
        if not self.has_calibrated_data:
            return

        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        frame_id = self.get_parameter('camera_frame_id').value
        loop_radius = self.get_parameter('loop_radius').value

        for i, loop in enumerate(self.calibrated_loops):
            # Sphere marker
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = frame_id
            marker.ns = 'calibrated_loops'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = loop['position'][0]
            marker.pose.position.y = loop['position'][1]
            marker.pose.position.z = loop['position'][2]
            marker.pose.orientation.w = 1.0

            marker.scale.x = loop_radius * 2
            marker.scale.y = loop_radius * 2
            marker.scale.z = loop_radius * 2

            # Blue color for ground truth
            marker.color.r = 0.2
            marker.color.g = 0.4
            marker.color.b = 0.9
            marker.color.a = 0.8

            marker.lifetime.sec = 0  # Persistent
            marker_array.markers.append(marker)

            # Text label
            text = Marker()
            text.header.stamp = stamp
            text.header.frame_id = frame_id
            text.ns = 'calibrated_loop_labels'
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD

            text.pose.position.x = loop['position'][0]
            text.pose.position.y = loop['position'][1]
            text.pose.position.z = loop['position'][2] + loop_radius * 3
            text.pose.orientation.w = 1.0

            text.text = f'L{i} ({loop["count"]})'
            text.scale.z = loop_radius * 1.5

            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0

            text.lifetime.sec = 0
            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)

    def save_calibration(self) -> bool:
        """Save calibrated loop positions to file."""
        if not self.has_calibrated_data:
            self.get_logger().warn('No calibration data to save')
            return False

        save_dir = self.get_parameter('save_directory').value
        os.makedirs(save_dir, exist_ok=True)

        # Create filename with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filepath = os.path.join(save_dir, f'loop_calibration_{timestamp}.json')

        # Also save as 'latest'
        latest_path = os.path.join(save_dir, 'latest.json')

        data = {
            'timestamp': timestamp,
            'frame_id': self.get_parameter('camera_frame_id').value,
            'parameters': {
                'collection_duration': self.get_parameter('collection_duration').value,
                'spatial_tolerance': self.get_parameter('spatial_tolerance').value,
                'min_detection_count': self.get_parameter('min_detection_count').value,
            },
            'raw_detection_count': len(self.raw_detections),
            'loops': self.calibrated_loops
        }

        try:
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)
            with open(latest_path, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f'Saved calibration to {filepath}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save calibration: {e}')
            return False

    def save_callback(self, request, response):
        """Service callback to save calibration."""
        if self.save_calibration():
            response.success = True
            response.message = 'Calibration saved'
        else:
            response.success = False
            response.message = 'Failed to save calibration'
        return response

    def load_callback(self, request, response):
        """Service callback to load calibration from file."""
        save_dir = self.get_parameter('save_directory').value
        latest_path = os.path.join(save_dir, 'latest.json')

        if not os.path.exists(latest_path):
            response.success = False
            response.message = f'No calibration file found at {latest_path}'
            return response

        try:
            with open(latest_path, 'r') as f:
                data = json.load(f)

            self.calibrated_loops = data['loops']
            self.has_calibrated_data = len(self.calibrated_loops) > 0

            self.get_logger().info(f'Loaded {len(self.calibrated_loops)} loops from {latest_path}')

            response.success = True
            response.message = f'Loaded {len(self.calibrated_loops)} calibrated loops'
        except Exception as e:
            response.success = False
            response.message = f'Failed to load calibration: {e}'

        return response


def main(args=None):
    rclpy.init(args=args)
    node = LoopCalibrationNode()

    # Use multi-threaded executor for async service handling
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
