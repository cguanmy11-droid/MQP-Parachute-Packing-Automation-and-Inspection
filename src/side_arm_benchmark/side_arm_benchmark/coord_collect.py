#!/usr/bin/env python3
import csv
import os
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, JointState

class JointStateRecorder(Node):
    def __init__(self):
        super().__init__('joint_state_recorder')

        self.declare_parameter('record_button_index', 0)  # A button
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('joint_state_topic', '/joint_states')

        self.record_button_index = self.get_parameter(
            'record_button_index').value

        self.joy_sub = self.create_subscription(
            Joy,
            self.get_parameter('joy_topic').value,
            self.joy_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            self.get_parameter('joint_state_topic').value,
            self.joint_state_callback,
            10
        )

        self.recorded_joint_pub = self.create_publisher(
            JointState,
            '/main_arm/recorded_joint_state',
            10
        )

        self.last_buttons = []
        self.latest_joint_state = None
        self.recorded_samples = []

        self.get_logger().info('Joint State Recorder initialized')
        self.get_logger().info(
            f'Record button index: {self.record_button_index}')
        
        # *******************************************************
        # ******need to change to what path you want*************
        # ******not in workspace right now***********************
        # *******************************************************
        self.csv_path = os.path.expanduser(
            '~/recorded_joint_states.csv'
        )
	
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.get_logger().info(f'Saving joint states to {self.csv_path}')

    def joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def joy_callback(self, msg: Joy):
        #pos edge
        if not self.last_buttons:
            self.last_buttons = list(msg.buttons)
            return

        if (msg.buttons[self.record_button_index] == 1 and
                self.last_buttons[self.record_button_index] == 0):
            self.record_joint_state()

        self.last_buttons = list(msg.buttons)

    def record_joint_state(self):
        if self.latest_joint_state is None:
            self.get_logger().warn('No joint state received yet')
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        if self.csv_file.tell() == 0:
            header = ['timestamp'] + list(self.latest_joint_state.name)
            self.csv_writer.writerow(header)

        row = [now] + list(self.latest_joint_state.position)
        self.csv_writer.writerow(row)
        self.csv_file.flush()

        self.get_logger().info(
            f'Recorded joint state ({len(self.latest_joint_state.position)} joints)'
        )
        
    def destroy_node(self):
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
            self.get_logger().info('CSV file closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

