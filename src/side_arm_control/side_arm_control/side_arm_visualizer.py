#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion
from parachute_interfaces.msg import SideArmState
import math


def quaternion_from_euler(roll, pitch, yaw):
    """Convert euler angles (radians) to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class SideArmVisualizer(Node):
    def __init__(self):
        super().__init__('side_arm_visualizer')
        
        # Publisher for RViz marker
        self.marker_pub = self.create_publisher(Marker, 'side_arm_marker', 10)
        
        # Subscribe to side arm state
        self.state_sub = self.create_subscription(
            SideArmState,
            '/side_arm/parsed_state',
            self.state_callback,
            10
        )
        
        # Position (meters)
        self.x_m = 0.0
        self.y_m = 0.0
        self.z_m = 0.0
        self.is_homed = False
        
        # Orientation parameters (radians) - adjust these to align the STL
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)
        
        # Offset from hook tip to mesh origin (if STL origin isn't at the tip)
        self.declare_parameter('offset_x', 0.0)
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('offset_z', 0.0)
        
        # Scale (adjust if STL is in mm vs meters)
        self.declare_parameter('scale', 0.001)  # mm to meters
        
        # Publish marker at steady rate
        self.timer = self.create_timer(0.1, self.publish_marker)
        
        self.get_logger().info('Side arm visualizer started with hook mesh')

    def state_callback(self, msg: SideArmState):
        # Convert mm to meters
        self.x_m = msg.x_mm / 1000.0
        self.y_m = msg.y_mm / 1000.0
        self.z_m = msg.z_mm / 1000.0
        self.is_homed = msg.is_homed

    def publish_marker(self):
        m = Marker()
        m.header.frame_id = "side_arm_origin"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "side_arm_hook"
        m.id = 0
        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = "package://side_arm_control/meshes/hook.stl"
        m.action = Marker.ADD
        
        # Get parameters
        roll = self.get_parameter('roll').value
        pitch = self.get_parameter('pitch').value
        yaw = self.get_parameter('yaw').value
        offset_x = self.get_parameter('offset_x').value
        offset_y = self.get_parameter('offset_y').value
        offset_z = self.get_parameter('offset_z').value
        scale = self.get_parameter('scale').value
        
        # Position (with offset for mesh origin)
        m.pose.position.x = self.x_m + offset_x
        m.pose.position.y = self.y_m + offset_y
        m.pose.position.z = self.z_m + offset_z
        
        # Orientation
        m.pose.orientation = quaternion_from_euler(roll, pitch, yaw)
        
        # Scale (same for all axes)
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        
        # Color based on homed state
        if self.is_homed:
            m.color.r = 0.3
            m.color.g = 0.3
            m.color.b = 0.3  # Gray metal color
        else:
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0  # Orange when not homed
        m.color.a = 1.0
        
        self.marker_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = SideArmVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()