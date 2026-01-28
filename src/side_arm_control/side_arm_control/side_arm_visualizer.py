#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from parachute_interfaces.msg import SideArmState

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
        
        # Store latest position (default until first message)
        self.x_m = 0.0
        self.y_m = 0.0
        self.z_m = 0.0
        self.is_homed = False
        
        # Publish marker at steady rate even if no new state
        self.timer = self.create_timer(0.1, self.publish_marker)
        
        self.get_logger().info('Side arm visualizer started, waiting for /side_arm/parsed_state...')

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
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        
        m.pose.position.x = self.x_m
        m.pose.position.y = self.y_m
        m.pose.position.z = self.z_m
        m.pose.orientation.w = 1.0
        
        m.scale.x = m.scale.y = m.scale.z = 0.02  # 2cm sphere
        
        # Color based on homed state
        if self.is_homed:
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0  # Green when homed
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