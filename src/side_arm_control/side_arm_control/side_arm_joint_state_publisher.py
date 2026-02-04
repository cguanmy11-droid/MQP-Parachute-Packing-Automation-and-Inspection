#!/usr/bin/env python3
"""
Side Arm Joint State Publisher

Bridges between side arm state messages and URDF joint states.
Subscribes to /side_arm/parsed_state and /side_arm/state,
publishes /side_arm/joint_states for robot_state_publisher.

Joint mapping:
  - joint_x: x_mm / 1000.0 (horizontal, prismatic)
  - joint_y: y_mm / 1000.0 (vertical, prismatic)
  - joint_z: z_mm / 1000.0 (depth, prismatic)
  - joint_servo: servo_offset_us * servo_scale (rotation, revolute)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from parachute_interfaces.msg import SideArmState
import json


class SideArmJointStatePublisher(Node):
    def __init__(self):
        super().__init__('side_arm_joint_state_publisher')

        # Parameters
        self.declare_parameter('servo_scale', 0.001)  # rad per microsecond offset
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('test_mode', False)
        self.declare_parameter('test_x', 0.0)  # meters
        self.declare_parameter('test_y', 0.0)
        self.declare_parameter('test_z', 0.0)
        self.declare_parameter('test_servo', 0.0)  # radians

        # State storage
        self.x_m = 0.0
        self.y_m = 0.0
        self.z_m = 0.0
        self.servo_rad = 0.0
        self.last_state_time = self.get_clock().now()

        # Publisher for joint states
        self.joint_pub = self.create_publisher(
            JointState,
            'joint_states',  # Will be namespaced to /side_arm/joint_states
            10
        )

        # Subscriber for parsed state (x, y, z positions)
        self.state_sub = self.create_subscription(
            SideArmState,
            '/side_arm/parsed_state',
            self.state_callback,
            10
        )

        # Subscriber for raw state (servo angle)
        self.raw_state_sub = self.create_subscription(
            String,
            '/side_arm/state',
            self.raw_state_callback,
            10
        )

        # Timer for publishing
        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_joint_states)

        self.get_logger().info('Side Arm Joint State Publisher initialized')
        self.get_logger().info(f'  Servo scale: {self.get_parameter("servo_scale").value} rad/us')
        self.get_logger().info(f'  Publish rate: {publish_rate} Hz')

    def state_callback(self, msg: SideArmState):
        """Handle parsed state message (positions in mm)."""
        self.x_m = msg.x_mm / 1000.0  # Convert mm to meters
        self.y_m = msg.y_mm / 1000.0
        self.z_m = msg.z_mm / 1000.0
        self.last_state_time = self.get_clock().now()

    def raw_state_callback(self, msg: String):
        """Handle raw state message to extract servo angle."""
        try:
            # Parse: "STATE {"l1":0,"l2":0,"l3":0,"s1":123,"s2":456,"dc":0,"servo":50}"
            data = msg.data
            if data.startswith('STATE '):
                json_str = data[6:]  # Remove "STATE " prefix
                state = json.loads(json_str)
                if 'servo' in state:
                    servo_offset_us = state['servo']
                    servo_scale = self.get_parameter('servo_scale').value
                    self.servo_rad = servo_offset_us * servo_scale
        except (json.JSONDecodeError, KeyError, ValueError):
            pass  # Ignore parse errors

    def publish_joint_states(self):
        """Publish joint states for URDF."""
        test_mode = self.get_parameter('test_mode').value

        # Use test values if in test mode or no recent state messages
        time_since_state = (self.get_clock().now() - self.last_state_time).nanoseconds / 1e9
        if test_mode or time_since_state > 2.0:
            x = self.get_parameter('test_x').value
            y = self.get_parameter('test_y').value
            z = self.get_parameter('test_z').value
            servo = self.get_parameter('test_servo').value
        else:
            x = self.x_m
            y = self.y_m
            z = self.z_m
            servo = self.servo_rad

        # Create joint state message
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['joint_x', 'joint_y', 'joint_z', 'joint_servo']
        js.position = [x, y, z, servo]
        js.velocity = []
        js.effort = []

        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = SideArmJointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
