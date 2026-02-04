#!/usr/bin/env python3
"""
Side Arm Joint State Publisher

Bridges between side arm state messages and URDF joint states.
Subscribes to /side_arm/parsed_state and /side_arm/state,
publishes /side_arm/joint_states for robot_state_publisher.

In test mode, also subscribes to /side_arm/coordinate_command to
simulate movement without hardware.

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
from parachute_interfaces.msg import SideArmState, SideArmCoordinateCommand
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
        self.declare_parameter('sim_move_speed', 0.05)  # m/s for simulated movement

        # State storage (current position)
        self.x_m = 0.0
        self.y_m = 0.0
        self.z_m = 0.0
        self.servo_rad = 0.0
        self.last_state_time = self.get_clock().now()

        # Target position for simulated movement
        self.target_x_m = 0.0
        self.target_y_m = 0.0
        self.target_z_m = 0.0

        # Publisher for joint states
        self.joint_pub = self.create_publisher(
            JointState,
            'joint_states',  # Will be namespaced to /side_arm/joint_states
            10
        )

        # Subscriber for parsed state (x, y, z positions) - hardware mode
        self.state_sub = self.create_subscription(
            SideArmState,
            '/side_arm/parsed_state',
            self.state_callback,
            10
        )

        # Subscriber for raw state (servo angle) - hardware mode
        self.raw_state_sub = self.create_subscription(
            String,
            '/side_arm/state',
            self.raw_state_callback,
            10
        )

        # Subscriber for coordinate commands - test mode simulation
        self.coord_cmd_sub = self.create_subscription(
            SideArmCoordinateCommand,
            '/side_arm/coordinate_command',
            self.coordinate_command_callback,
            10
        )

        # Timer for publishing and simulated movement
        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / publish_rate, self.update_and_publish)

        # Initialize position from test parameters
        test_mode = self.get_parameter('test_mode').value
        if test_mode:
            self.x_m = self.get_parameter('test_x').value
            self.y_m = self.get_parameter('test_y').value
            self.z_m = self.get_parameter('test_z').value
            self.target_x_m = self.x_m
            self.target_y_m = self.y_m
            self.target_z_m = self.z_m

        self.get_logger().info('Side Arm Joint State Publisher initialized')
        self.get_logger().info(f'  Test mode: {test_mode}')
        self.get_logger().info(f'  Servo scale: {self.get_parameter("servo_scale").value} rad/us')
        self.get_logger().info(f'  Publish rate: {publish_rate} Hz')

    def state_callback(self, msg: SideArmState):
        """Handle parsed state message (positions in mm) - hardware mode."""
        if not self.get_parameter('test_mode').value:
            self.x_m = msg.x_mm / 1000.0  # Convert mm to meters
            self.y_m = msg.y_mm / 1000.0
            self.z_m = msg.z_mm / 1000.0
            self.last_state_time = self.get_clock().now()

    def raw_state_callback(self, msg: String):
        """Handle raw state message to extract servo angle - hardware mode."""
        if self.get_parameter('test_mode').value:
            return
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

    def coordinate_command_callback(self, msg: SideArmCoordinateCommand):
        """Handle coordinate commands - update target position for simulation."""
        if self.get_parameter('test_mode').value:
            # In test mode, update target position (mm to meters)
            self.target_x_m = msg.x_mm / 1000.0
            self.target_y_m = msg.y_mm / 1000.0
            self.target_z_m = msg.z_mm / 1000.0
            self.get_logger().info(
                f'Simulating move to ({msg.x_mm:.1f}, {msg.y_mm:.1f}, {msg.z_mm:.1f}) mm')

    def update_and_publish(self):
        """Update simulated position and publish joint states."""
        test_mode = self.get_parameter('test_mode').value
        publish_rate = self.get_parameter('publish_rate').value
        dt = 1.0 / publish_rate

        if test_mode:
            # Simulate movement toward target
            move_speed = self.get_parameter('sim_move_speed').value
            max_move = move_speed * dt

            # Move each axis toward target
            for axis in ['x', 'y', 'z']:
                current = getattr(self, f'{axis}_m')
                target = getattr(self, f'target_{axis}_m')
                diff = target - current
                if abs(diff) > max_move:
                    new_val = current + max_move * (1 if diff > 0 else -1)
                else:
                    new_val = target
                setattr(self, f'{axis}_m', new_val)

        # Get current positions
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
