#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion
from parachute_interfaces.msg import SideArmState
from std_msgs.msg import String
import math
import json


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


def multiply_quaternions(q1, q2):
    """Multiply two quaternions."""
    q = Quaternion()
    q.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
    q.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
    q.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
    q.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
    return q


class SideArmVisualizer(Node):
    def __init__(self):
        super().__init__('side_arm_visualizer')
        
        # Publisher for RViz marker
        self.marker_pub = self.create_publisher(Marker, 'side_arm_marker', 10)
        
        # Subscribe to parsed state (for x, y, z position)
        self.state_sub = self.create_subscription(
            SideArmState,
            '/side_arm/parsed_state',
            self.state_callback,
            10
        )
        
        # Subscribe to raw state (for servo angle)
        self.raw_state_sub = self.create_subscription(
            String,
            '/side_arm/state',
            self.raw_state_callback,
            10
        )
        
        # Position (meters)
        self.x_m = 0.0
        self.y_m = 0.0
        self.z_m = 0.0
        self.is_homed = False
        
        # Servo angle (from raw state)
        self.servo_offset_us = 0  # microseconds offset from neutral
        
        # Orientation parameters (radians) - base orientation to align STL
        self.declare_parameter('roll', 3.1416)
        self.declare_parameter('pitch', 1.5708)
        self.declare_parameter('yaw', 0.0)
        
        # Servo rotation axis and scaling
        self.declare_parameter('servo_axis', 'yaw')  # which axis servo rotates around
        self.declare_parameter('servo_scale', 0.002)  # radians per microsecond offset
        
        # Offset from hook tip to mesh origin
        self.declare_parameter('offset_x', 0.0)
        self.declare_parameter('offset_y', 0.05)
        self.declare_parameter('offset_z', 0.0)
        
        # Scale
        self.declare_parameter('scale', 0.001)
        
        # Test mode
        self.declare_parameter('test_mode', False)
        self.declare_parameter('test_x', 0.15)
        self.declare_parameter('test_y', 0.10)
        self.declare_parameter('test_z', 0.05)
        
        self.last_msg_time = self.get_clock().now()
        
        # Publish marker at steady rate
        self.timer = self.create_timer(0.1, self.publish_marker)
        
        self.get_logger().info('Side arm visualizer started with hook mesh and servo rotation')

    def state_callback(self, msg: SideArmState):
        """Handle parsed state (position)."""
        self.x_m = msg.x_mm / 1000.0
        self.y_m = msg.y_mm / 1000.0
        self.z_m = msg.z_mm / 1000.0
        self.is_homed = msg.is_homed
        self.last_msg_time = self.get_clock().now()

    def raw_state_callback(self, msg: String):
        """Handle raw state to extract servo angle."""
        try:
            # Parse: "STATE {"l1":0,"l2":0,"l3":0,"s1":123,"s2":456,"dc":0,"servo":50}"
            data = msg.data
            if data.startswith('STATE '):
                json_str = data[6:]  # Remove "STATE " prefix
                state = json.loads(json_str)
                if 'servo' in state:
                    self.servo_offset_us = state['servo']
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            pass  # Ignore parse errors

    def publish_marker(self):
        # Use test position if in test mode or no recent messages
        test_mode = self.get_parameter('test_mode').value
        time_since_msg = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        
        if test_mode or time_since_msg > 2.0:
            self.x_m = self.get_parameter('test_x').value
            self.y_m = self.get_parameter('test_y').value
            self.z_m = self.get_parameter('test_z').value
        
        m = Marker()
        m.header.frame_id = "side_arm_origin"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "side_arm_hook"
        m.id = 0
        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = "package://side_arm_control/meshes/hook.stl"
        m.action = Marker.ADD
        
        # Get parameters
        base_roll = self.get_parameter('roll').value
        base_pitch = self.get_parameter('pitch').value
        base_yaw = self.get_parameter('yaw').value
        offset_x = self.get_parameter('offset_x').value
        offset_y = self.get_parameter('offset_y').value
        offset_z = self.get_parameter('offset_z').value
        scale = self.get_parameter('scale').value
        servo_axis = self.get_parameter('servo_axis').value
        servo_scale = self.get_parameter('servo_scale').value
        
        # Position
        m.pose.position.x = self.x_m + offset_x
        m.pose.position.y = self.y_m + offset_y
        m.pose.position.z = self.z_m + offset_z
        
        # Calculate servo rotation angle
        servo_angle = self.servo_offset_us * servo_scale
        
        # Base orientation
        base_orientation = quaternion_from_euler(base_roll, base_pitch, base_yaw)
        
        # Servo rotation (applied on top of base orientation)
        if servo_axis == 'roll':
            servo_rotation = quaternion_from_euler(servo_angle, 0, 0)
        elif servo_axis == 'pitch':
            servo_rotation = quaternion_from_euler(0, servo_angle, 0)
        else:  # yaw
            servo_rotation = quaternion_from_euler(0, 0, servo_angle)
        
        # Combine: base orientation * servo rotation
        m.pose.orientation = multiply_quaternions(base_orientation, servo_rotation)
        
        # Scale
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        
        # Color based on homed state
        if self.is_homed:
            m.color.r = 0.3
            m.color.g = 0.3
            m.color.b = 0.3
        else:
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
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