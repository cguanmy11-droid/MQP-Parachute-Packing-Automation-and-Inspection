#!/usr/bin/env python3
"""
Loop Calibration Node
Calibrates loop positions from real camera detections.

This node:
1. Sends HOME_ALL command to home all 3 axes of side arm
2. Moves to calibration position (x=180mm) to view loops
3. Collects detections and transforms them to world frame
4. Clusters nearby detections using spatial tolerance
5. Terminates early once all detected loops hit the min_detection_count threshold
6. Publishes calibrated positions as ground truth in world frame
7. Returns side arm to home position

Early Termination:
- Calibration finishes early when ALL detected loop clusters have >= min_detection_count detections
- Requires at least min_loops_expected loops to be detected
- Waits stable_time seconds with no new loops appearing before terminating
- Falls back to max collection_duration if thresholds not reached

Usage:
    # Start calibration via service call
    ros2 service call /calibrate_loops std_srvs/srv/Trigger

    # Or with custom parameters
    ros2 run parachute_perception loop_calibration_node --ros-args \
        -p collection_duration:=30.0 \
        -p spatial_tolerance:=0.01 \
        -p min_detection_count:=15 \
        -p min_loops_expected:=5
"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from parachute_interfaces.msg import DetectedLoops, LoopGroundTruth
from parachute_interfaces.srv import MoveToPosition
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import numpy as np
import json
import os
from datetime import datetime
import time


class LoopCalibrationNode(Node):
    def __init__(self):
        super().__init__('loop_calibration_node')

        # Callback group for async service calls
        self.callback_group = ReentrantCallbackGroup()

        # ==================== PARAMETERS ====================
        # Calibration settings
        self.declare_parameter('collection_duration', 30.0)  # max seconds (can finish early)
        self.declare_parameter('spatial_tolerance', 0.008)  # meters (8mm)
        self.declare_parameter('min_detection_count', 15)   # minimum detections to be valid
        self.declare_parameter('home_before_calibration', True)
        self.declare_parameter('return_home_after', True)
        self.declare_parameter('early_termination', True)   # finish when all loops hit threshold
        self.declare_parameter('min_loops_expected', 1)     # minimum loops to detect before early termination
        self.declare_parameter('stable_time', 1.0)          # seconds with no new loops before early termination

        # Calibration position (where to move for best view of loops)
        self.declare_parameter('calibration_x_mm', 180.0)  # Move to end of X axis
        self.declare_parameter('calibration_y_mm', 0.0)
        self.declare_parameter('calibration_z_mm', 0.0)

        # Frame settings
        self.declare_parameter('camera_frame_id', 'camera_frame')
        self.declare_parameter('world_frame_id', 'world')

        # Output settings
        self.declare_parameter('save_to_file', True)
        self.declare_parameter('save_directory', '/tmp/loop_calibration')
        self.declare_parameter('loop_radius', 0.015)  # Default loop radius (meters)

        # Publishing
        self.declare_parameter('publish_rate', 10.0)  # Hz

        # ==================== TF2 ====================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ==================== STATE ====================
        self.is_calibrating = False
        self.calibration_start_time = None
        self.raw_detections = []  # List of (x, y, z, confidence) tuples IN WORLD FRAME
        self.calibrated_loops = []  # List of {'position': [x,y,z], 'count': n, 'confidence': avg}
        self.has_calibrated_data = False

        # Early termination tracking
        self.last_cluster_count = 0
        self.last_cluster_change_time = None

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
        # Publisher for HOME_ALL command
        self.command_pub = self.create_publisher(
            String,
            '/side_arm/command',
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
        self.get_logger().info(f'  Max collection duration: {self.get_parameter("collection_duration").value}s')
        self.get_logger().info(f'  Spatial tolerance: {self.get_parameter("spatial_tolerance").value*1000:.1f}mm')
        self.get_logger().info(f'  Min detection count: {self.get_parameter("min_detection_count").value}')
        self.get_logger().info(f'  Early termination: {self.get_parameter("early_termination").value}')
        self.get_logger().info(f'  Min loops expected: {self.get_parameter("min_loops_expected").value}')
        self.get_logger().info(f'  Calibration position: x={self.get_parameter("calibration_x_mm").value}mm')
        self.get_logger().info(f'  Output frame: {self.get_parameter("world_frame_id").value}')
        self.get_logger().info('  Call /calibrate_loops service to start calibration')

    def transform_to_world(self, point, source_frame: str):
        """Transform a point from source_frame to world frame."""
        world_frame = self.get_parameter('world_frame_id').value

        if source_frame == world_frame:
            return point, True

        try:
            transform = self.tf_buffer.lookup_transform(
                world_frame,
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

    def detection_callback(self, msg: DetectedLoops):
        """Collect detections during calibration, transform to world frame."""
        if not self.is_calibrating:
            return

        # Check if calibration time has elapsed
        elapsed = (self.get_clock().now() - self.calibration_start_time).nanoseconds / 1e9
        duration = self.get_parameter('collection_duration').value

        if elapsed >= duration:
            self.finish_calibration()
            return

        # Get source frame from message
        source_frame = msg.header.frame_id if msg.header.frame_id else self.get_parameter('camera_frame_id').value

        # Transform and store detections in world frame
        for loop in msg.loops:
            pos = loop.pose.pose.position
            world_pos, success = self.transform_to_world(pos, source_frame)

            if success:
                self.raw_detections.append((world_pos.x, world_pos.y, world_pos.z, loop.confidence))

        # Check for early termination
        if self.get_parameter('early_termination').value and len(self.raw_detections) > 0:
            if self.check_early_termination():
                self.get_logger().info('Early termination: all loops verified!')
                self.finish_calibration()
                return

        # Publish status with current cluster info
        tolerance = self.get_parameter('spatial_tolerance').value
        min_count = self.get_parameter('min_detection_count').value
        clusters = self.cluster_detections(self.raw_detections, tolerance)
        valid_clusters = [c for c in clusters if c['count'] >= min_count]

        remaining = duration - elapsed
        status_msg = String()
        status_msg.data = (f'Calibrating: {len(self.raw_detections)} detections, '
                          f'{len(valid_clusters)}/{len(clusters)} loops verified, '
                          f'{remaining:.1f}s remaining')
        self.status_pub.publish(status_msg)

    def check_early_termination(self) -> bool:
        """Check if calibration can terminate early (all loops verified)."""
        tolerance = self.get_parameter('spatial_tolerance').value
        min_count = self.get_parameter('min_detection_count').value
        min_loops = self.get_parameter('min_loops_expected').value
        stable_time = self.get_parameter('stable_time').value

        # Cluster current detections
        clusters = self.cluster_detections(self.raw_detections, tolerance)

        # Check if we have minimum number of loops
        if len(clusters) < min_loops:
            return False

        # Check if number of clusters has changed
        current_time = self.get_clock().now()
        if len(clusters) != self.last_cluster_count:
            self.last_cluster_count = len(clusters)
            self.last_cluster_change_time = current_time
            return False

        # Check if clusters have been stable for stable_time
        if self.last_cluster_change_time is None:
            self.last_cluster_change_time = current_time
            return False

        time_stable = (current_time - self.last_cluster_change_time).nanoseconds / 1e9
        if time_stable < stable_time:
            return False

        # Check if all clusters have reached minimum count
        all_verified = all(c['count'] >= min_count for c in clusters)

        if all_verified:
            self.get_logger().info(f'All {len(clusters)} loops have >= {min_count} detections')

        return all_verified

    def calibrate_callback(self, request, response):
        """Service callback to start calibration."""
        if self.is_calibrating:
            response.success = False
            response.message = 'Calibration already in progress'
            return response

        self.get_logger().info('='*50)
        self.get_logger().info('Starting loop calibration sequence...')
        self.get_logger().info('='*50)

        # Step 1: Home all 3 axes
        if self.get_parameter('home_before_calibration').value:
            self.get_logger().info('Step 1: Homing all 3 axes...')
            if not self.home_all_axes():
                response.success = False
                response.message = 'Failed to home side arm'
                return response
            self.get_logger().info('  Homing complete')

        # Step 2: Move to calibration position
        self.get_logger().info('Step 2: Moving to calibration position...')
        cal_x = self.get_parameter('calibration_x_mm').value
        cal_y = self.get_parameter('calibration_y_mm').value
        cal_z = self.get_parameter('calibration_z_mm').value

        if not self.move_to_position(cal_x, cal_y, cal_z):
            response.success = False
            response.message = 'Failed to move to calibration position'
            return response
        self.get_logger().info(f'  At calibration position: ({cal_x}, {cal_y}, {cal_z}) mm')

        # Wait for arm to settle and TF to update
        time.sleep(1.0)

        # Step 3: Collect detections
        self.get_logger().info('Step 3: Collecting detections...')
        self.is_calibrating = True
        self.calibration_start_time = self.get_clock().now()
        self.raw_detections = []

        # Reset early termination tracking
        self.last_cluster_count = 0
        self.last_cluster_change_time = None

        duration = self.get_parameter('collection_duration').value
        min_count = self.get_parameter('min_detection_count').value
        self.get_logger().info(f'  Collecting for up to {duration}s (or until all loops have {min_count}+ detections)...')

        # Wait for collection to complete
        while self.is_calibrating:
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)

        self.get_logger().info(f'  Collection complete: {len(self.raw_detections)} raw detections')

        # Step 4: Return home
        if self.get_parameter('return_home_after').value:
            self.get_logger().info('Step 4: Returning to home position...')
            self.home_all_axes()
            self.get_logger().info('  Home position reached')

        self.get_logger().info('='*50)
        self.get_logger().info(f'Calibration complete: {len(self.calibrated_loops)} loops found')
        self.get_logger().info('='*50)

        response.success = True
        response.message = f'Calibration complete: {len(self.calibrated_loops)} loops found in world frame'
        return response

    def home_all_axes(self) -> bool:
        """Send HOME_ALL command to home all 3 axes."""
        self.get_logger().info('  Sending HOME_ALL command...')

        # Send HOME_ALL command
        cmd = String()
        cmd.data = 'HOME_ALL'
        self.command_pub.publish(cmd)

        # Wait for homing to complete
        # HOME_ALL can take a while - wait up to 60 seconds
        self.get_logger().info('  Waiting for homing to complete...')
        time.sleep(15.0)  # Homing typically takes 10-15 seconds

        # Also move to (0,0,0) to ensure we're at home
        return self.move_to_position(0.0, 0.0, 0.0)

    def move_to_position(self, x_mm: float, y_mm: float, z_mm: float) -> bool:
        """Move side arm to specified position."""
        if not self.move_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Side arm move service not available')
            return False

        request = MoveToPosition.Request()
        request.x_mm = float(x_mm)
        request.y_mm = float(y_mm)
        request.z_mm = float(z_mm)
        request.speed_scale = 0.5

        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if future.result() is not None:
            result = future.result()
            if result.success:
                # Wait for motion to complete
                est_time = result.estimated_time_sec if hasattr(result, 'estimated_time_sec') else 5.0
                time.sleep(max(est_time, 2.0))
                return True
            else:
                self.get_logger().error(f'Move failed: {result.message}')
                return False
        else:
            self.get_logger().error('Move service call failed')
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

        # Cluster detections (already in world frame)
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
                    f'world pos=({cluster["position"][0]:.4f}, {cluster["position"][1]:.4f}, {cluster["position"][2]:.4f}), '
                    f'count={cluster["count"]}, conf={cluster["confidence"]:.2f}'
                )

        # Sort by X position (left to right in world frame)
        self.calibrated_loops.sort(key=lambda c: c['position'][0])

        self.has_calibrated_data = len(self.calibrated_loops) > 0

        # Log results
        self.get_logger().info(f'Found {len(self.calibrated_loops)} valid loops in world frame')
        self.get_logger().info(f'  (Rejected {len(clusters) - len(self.calibrated_loops)} clusters with < {min_count} detections)')

        # Publish status
        status_msg = String()
        status_msg.data = f'Calibration complete: {len(self.calibrated_loops)} loops (world frame)'
        self.status_pub.publish(status_msg)

        # Auto-save if configured
        if self.get_parameter('save_to_file').value and self.has_calibrated_data:
            self.save_calibration()

    def cluster_detections(self, detections: list, tolerance: float) -> list:
        """
        Cluster detections using simple distance-based grouping.

        Args:
            detections: List of (x, y, z, confidence) tuples in world frame
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
        """Publish calibrated loops as ground truth in world frame."""
        if not self.has_calibrated_data:
            return

        world_frame = self.get_parameter('world_frame_id').value

        # Create LoopGroundTruth message
        msg = LoopGroundTruth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = world_frame  # Always world frame

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
        """Publish RViz markers for calibrated loops in world frame."""
        if not self.has_calibrated_data:
            return

        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        world_frame = self.get_parameter('world_frame_id').value
        loop_radius = self.get_parameter('loop_radius').value

        for i, loop in enumerate(self.calibrated_loops):
            # Sphere marker
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = world_frame
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
            text.header.frame_id = world_frame
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

        world_frame = self.get_parameter('world_frame_id').value

        data = {
            'timestamp': timestamp,
            'frame_id': world_frame,
            'parameters': {
                'collection_duration': self.get_parameter('collection_duration').value,
                'spatial_tolerance': self.get_parameter('spatial_tolerance').value,
                'min_detection_count': self.get_parameter('min_detection_count').value,
                'calibration_position_mm': {
                    'x': self.get_parameter('calibration_x_mm').value,
                    'y': self.get_parameter('calibration_y_mm').value,
                    'z': self.get_parameter('calibration_z_mm').value,
                }
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
            self.get_logger().info(f'  Frame: {data.get("frame_id", "unknown")}')

            for i, loop in enumerate(self.calibrated_loops):
                self.get_logger().info(
                    f'  Loop {i}: pos=({loop["position"][0]:.4f}, {loop["position"][1]:.4f}, {loop["position"][2]:.4f})'
                )

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
