#!/usr/bin/env python3
"""
Side Arm Joint State Publisher

Bridges between side arm state messages and URDF joint states.
Subscribes to /side_arm/parsed_state and converts to joint_states
for the robot_state_publisher to move the URDF.

This node is agnostic to whether the state comes from real hardware
or simulation - it just converts whatever state it receives.

Joint mapping (with axis corrections for URDF):
  - joint_x: -x_mm / 1000.0 (horizontal, prismatic, negated for URDF axis)
  - joint_y: y_mm / 1000.0 (vertical, prismatic)
  - joint_z: -z_mm / 1000.0 (depth, prismatic, negated for URDF orientation)
  - joint_servo: servo_angle (rotation, revolute)
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

        # State storage (current position in meters)
        self.x_m = 0.0
        self.y_m = 0.0
        self.z_m = 0.0
        self.servo_rad = 0.0
        self.servo_offset_us = 0  # Track servo offset in microseconds for simulation
        self.last_state_time = self.get_clock().now()
        self.state_received = False

        # Publisher for joint states
        self.joint_pub = self.create_publisher(
            JointState,
            'joint_states',  # Will be namespaced to /side_arm/joint_states
            10
        )

        # Subscriber for parsed state (works for both hardware and simulation)
        self.state_sub = self.create_subscription(
            SideArmState,
            '/side_arm/parsed_state',
            self.state_callback,
            10
        )

        # Subscriber for raw state (servo angle from hardware)
        self.raw_state_sub = self.create_subscription(
            String,
            '/side_arm/state',
            self.raw_state_callback,
            10
        )

        # Subscriber for commands (to track servo in simulation mode)
        self.cmd_sub = self.create_subscription(
            String,
            '/side_arm/command',
            self.command_callback,
            10
        )

        # Timer for publishing joint states
        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_joint_states)

        self.get_logger().info('Side Arm Joint State Publisher initialized')
        self.get_logger().info('  Subscribing to /side_arm/parsed_state')
        self.get_logger().info('  Subscribing to /side_arm/command (for servo simulation)')
        self.get_logger().info(f'  Publishing at {publish_rate} Hz')

    def state_callback(self, msg: SideArmState):
        """Handle parsed state message (positions in mm)."""
        self.x_m = msg.x_mm / 1000.0  # Convert mm to meters
        self.y_m = msg.y_mm / 1000.0
        self.z_m = msg.z_mm / 1000.0
        self.last_state_time = self.get_clock().now()
        self.state_received = True

    def raw_state_callback(self, msg: String):
        """Handle raw state message to extract servo angle (hardware mode)."""
        try:
            # Parse: "STATE {"l1":0,"l2":0,"l3":0,"s1":123,"s2":456,"dc":0,"servo":50}"
            data = msg.data
            if data.startswith('STATE '):
                json_str = data[6:]  # Remove "STATE " prefix
                state = json.loads(json_str)
                if 'servo' in state:
                    self.servo_offset_us = state['servo']
                    servo_scale = self.get_parameter('servo_scale').value
                    self.servo_rad = self.servo_offset_us * servo_scale
        except (json.JSONDecodeError, KeyError, ValueError):
            pass  # Ignore parse errors

    def command_callback(self, msg: String):
        """Track SERVO commands for simulation mode."""
        cmd = msg.data.strip().upper()
        if cmd.startswith('SERVO,'):
            try:
                parts = msg.data.strip().split(',')
                if len(parts) >= 2:
                    self.servo_offset_us = int(parts[1])
                    servo_scale = self.get_parameter('servo_scale').value
                    self.servo_rad = self.servo_offset_us * servo_scale
            except (ValueError, IndexError):
                pass

    def publish_joint_states(self):
        """Publish joint states for URDF."""
        # Create joint state message
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['joint_x', 'joint_y', 'joint_z', 'joint_servo']
        # Apply axis corrections for URDF orientation:
        # - X is negated because URDF joint_x has axis="-1 0 0"
        # - Z is negated to match the side_arm_origin frame orientation
        js.position = [-self.x_m, self.y_m, -self.z_m, self.servo_rad]
        js.velocity = []
        js.effort = []

        self.joint_pub.publish(js)

        # Log status periodically
        if not self.state_received:
            time_waiting = (self.get_clock().now() - self.last_state_time).nanoseconds / 1e9
            if time_waiting > 5.0 and int(time_waiting) % 5 == 0:
                self.get_logger().warn(
                    'No state received yet. Waiting for /side_arm/parsed_state...')


def main(args=None):
    rclpy.init(args=args)
    node = SideArmJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
