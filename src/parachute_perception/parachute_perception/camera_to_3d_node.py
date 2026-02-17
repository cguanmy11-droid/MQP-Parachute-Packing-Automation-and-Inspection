#!/usr/bin/env python3
"""
Camera to 3D Node
Converts 2D pixel detections from real camera (YOLO) to 3D positions in camera_frame.

This node bridges real camera output to the same format as detection_simulator,
allowing the rest of the system to work identically with real or simulated data.

Pipeline:
    YOLO detector → /yolo/centers (pixels) → THIS NODE → /detected_loops (3D camera_frame)

The conversion assumes loops are on a known plane at a fixed depth from the camera.
For more accurate 3D, you would need stereo vision or a depth camera.

Subscriptions:
- /yolo/centers (PoseArray): Pixel coordinates from YOLO detector
  - pose.position.x = pixel u (horizontal)
  - pose.position.y = pixel v (vertical)
  - pose.position.z = 0 (unused)

Publications:
- /detected_loops (DetectedLoops): 3D positions in camera_frame (same as detection_simulator)

Parameters:
- Camera intrinsics (resolution, FOV or focal length)
- Assumed depth for pixel-to-3D conversion
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from parachute_interfaces.msg import DetectedLoops, DetectedLoop
import math


class CameraTo3DNode(Node):
    def __init__(self):
        super().__init__('camera_to_3d_node')

        # ==================== CAMERA PARAMETERS ====================
        # Image resolution
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        # Camera intrinsics - Option 1: Specify FOV (simpler)
        self.declare_parameter('camera_fov_horizontal', 60.0)  # degrees

        # Camera intrinsics - Option 2: Specify focal length directly (more accurate)
        # If fx/fy are 0, they'll be computed from FOV
        self.declare_parameter('fx', 0.0)  # focal length x (pixels)
        self.declare_parameter('fy', 0.0)  # focal length y (pixels)
        self.declare_parameter('cx', 0.0)  # principal point x (0 = use image_width/2)
        self.declare_parameter('cy', 0.0)  # principal point y (0 = use image_height/2)

        # ==================== DEPTH ASSUMPTION ====================
        # How far loops are from the camera (meters)
        # This is the key assumption for pixel→3D conversion
        self.declare_parameter('assumed_depth', 0.12)  # meters

        # ==================== OUTPUT CONFIGURATION ====================
        self.declare_parameter('camera_frame_id', 'camera_frame')
        self.declare_parameter('input_topic', '/yolo/centers')
        self.declare_parameter('output_topic', '/detected_loops')

        # ==================== DETECTION PARAMETERS ====================
        self.declare_parameter('base_confidence', 0.85)
        self.declare_parameter('min_pixel_x', 0)      # Crop region (ignore detections outside)
        self.declare_parameter('max_pixel_x', 10000)
        self.declare_parameter('min_pixel_y', 0)
        self.declare_parameter('max_pixel_y', 10000)

        # Get parameters
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.assumed_depth = self.get_parameter('assumed_depth').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.base_confidence = self.get_parameter('base_confidence').value

        # Compute camera intrinsics
        self._compute_intrinsics()

        # Subscriber for YOLO pixel detections
        input_topic = self.get_parameter('input_topic').value
        self.pixel_sub = self.create_subscription(
            PoseArray,
            input_topic,
            self.pixel_callback,
            10
        )

        # Publisher for 3D detections
        output_topic = self.get_parameter('output_topic').value
        self.detection_pub = self.create_publisher(
            DetectedLoops,
            output_topic,
            10
        )

        self.get_logger().info('Camera to 3D Node initialized')
        self.get_logger().info(f'  Input: {input_topic} (pixels)')
        self.get_logger().info(f'  Output: {output_topic} (3D camera_frame)')
        self.get_logger().info(f'  Resolution: {self.image_width}x{self.image_height}')
        self.get_logger().info(f'  Intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}')
        self.get_logger().info(f'  Assumed depth: {self.assumed_depth:.3f}m')

    def _compute_intrinsics(self):
        """Compute camera intrinsics from parameters."""
        fx = self.get_parameter('fx').value
        fy = self.get_parameter('fy').value
        cx = self.get_parameter('cx').value
        cy = self.get_parameter('cy').value

        # Principal point defaults to image center
        self.cx = cx if cx > 0 else self.image_width / 2.0
        self.cy = cy if cy > 0 else self.image_height / 2.0

        # Focal length: use provided values or compute from FOV
        if fx > 0 and fy > 0:
            self.fx = fx
            self.fy = fy
        else:
            # Compute from horizontal FOV
            fov_h = math.radians(self.get_parameter('camera_fov_horizontal').value)
            self.fx = self.image_width / (2.0 * math.tan(fov_h / 2.0))
            # Assume square pixels (fy = fx)
            self.fy = self.fx

    def pixel_to_camera_frame(self, u: float, v: float, depth: float) -> tuple:
        """
        Convert pixel coordinates to 3D position in camera frame.

        Camera frame convention (standard):
        - X: right
        - Y: down
        - Z: forward (into scene)

        Args:
            u: pixel x coordinate (horizontal, 0 = left)
            v: pixel y coordinate (vertical, 0 = top)
            depth: assumed Z distance from camera (meters)

        Returns:
            (x, y, z) in camera frame (meters)
        """
        # Standard pinhole camera model inversion
        x = (u - self.cx) * depth / self.fx
        y = (v - self.cy) * depth / self.fy
        z = depth

        return x, y, z

    def pixel_callback(self, msg: PoseArray):
        """Convert pixel detections to 3D and publish."""
        if len(msg.poses) == 0:
            return

        # Get crop region parameters
        min_x = self.get_parameter('min_pixel_x').value
        max_x = self.get_parameter('max_pixel_x').value
        min_y = self.get_parameter('min_pixel_y').value
        max_y = self.get_parameter('max_pixel_y').value

        # Create output message
        detected_msg = DetectedLoops()
        detected_msg.header = Header()
        detected_msg.header.stamp = self.get_clock().now().to_msg()
        detected_msg.header.frame_id = self.camera_frame_id

        for i, pose in enumerate(msg.poses):
            # Extract pixel coordinates
            u = pose.position.x  # pixel x
            v = pose.position.y  # pixel y

            # Skip if outside crop region
            if u < min_x or u > max_x or v < min_y or v > max_y:
                continue

            # Convert to 3D
            x, y, z = self.pixel_to_camera_frame(u, v, self.assumed_depth)

            # Create detected loop
            detected_loop = DetectedLoop()
            detected_loop.header = detected_msg.header
            detected_loop.loop_id = i

            detected_loop.pose = PoseStamped()
            detected_loop.pose.header = detected_msg.header
            detected_loop.pose.pose.position = Point(x=x, y=y, z=z)
            detected_loop.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            # Confidence based on position in frame (center = higher confidence)
            dist_from_center = math.sqrt(
                ((u - self.cx) / self.cx) ** 2 +
                ((v - self.cy) / self.cy) ** 2
            )
            detected_loop.confidence = self.base_confidence * (1.0 - 0.2 * min(dist_from_center, 1.0))

            detected_msg.loops.append(detected_loop)

        detected_msg.total_count = len(detected_msg.loops)

        # Publish
        self.detection_pub.publish(detected_msg)

        if detected_msg.total_count > 0:
            self.get_logger().debug(f'Published {detected_msg.total_count} 3D detections')


def main(args=None):
    rclpy.init(args=args)
    node = CameraTo3DNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
