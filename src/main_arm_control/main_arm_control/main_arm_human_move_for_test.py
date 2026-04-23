#!/usr/bin/env python3
"""
Main Arm Human Move For Test
================================

Continuously prints / publishes the live state of the WidowX-200 arm.

By default, this node does not change torque state. If desired, torque can be
disabled explicitly via ROS parameters so the arm can be moved by hand.

It reports:
  - Each joint position (rad + deg)
  - End-effector position (x, y, z) in the base frame
  - End-effector orientation (roll, pitch, yaw)
  - End-effector pitch relative to the ground (e.g. +90 deg = pointing up,
    -90 deg = pointing down, 0 deg = horizontal)
  - Gripper finger position (m) and opening width (mm + percent of max)

Run:
  ros2 run main_arm_control main_arm_human_move_for_test

Optional parameters:
  print_rate         (float, Hz, default 2.0)   how often to print to terminal
  publish_rate       (float, Hz, default 20.0)  how often to publish topics
  torque_off_arm     (bool, default False)
  torque_off_gripper (bool, default False)
  torque_on_shutdown (bool, default True)       re-enable torque when exiting
  robot_model        (str, default 'wx200')
  robot_name         (str, default 'wx200')
"""

import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


class MainArmHumanMoveForTest(Node):
    def __init__(self):
        super().__init__('main_arm_human_move_for_test')

        self.declare_parameter('print_rate', 2.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('torque_off_arm', False)
        self.declare_parameter('torque_off_gripper', False)
        self.declare_parameter('torque_on_shutdown', True)
        self.declare_parameter('robot_model', 'wx200')
        self.declare_parameter('robot_name', 'wx200')

        self.print_rate = float(self.get_parameter('print_rate').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.torque_off_arm = bool(self.get_parameter('torque_off_arm').value)
        self.torque_off_gripper = bool(
            self.get_parameter('torque_off_gripper').value)
        self.torque_on_shutdown = bool(
            self.get_parameter('torque_on_shutdown').value)
        robot_model = self.get_parameter('robot_model').value
        robot_name = self.get_parameter('robot_name').value

        self.bot = InterbotixManipulatorXS(
            robot_model=robot_model,
            robot_name=robot_name,
            moving_time=2.0,
            accel_time=0.5,
        )

        try:
            self.gripper_lower = float(self.bot.gripper.left_finger_lower_limit)
            self.gripper_upper = float(self.bot.gripper.left_finger_upper_limit)
        except Exception:
            self.gripper_lower = 0.015
            self.gripper_upper = 0.037

        if self.torque_off_arm or self.torque_off_gripper:
            self.get_logger().warn(
                'Torque-off mode requested via parameters. '
                'Support the arm before moving it by hand.')
            self._set_torque(enable=False)

        self.joint_state_pub = self.create_publisher(
            JointState, '/main_arm/human_move/joint_state', 10)
        self.ee_pose_pub = self.create_publisher(
            Pose, '/main_arm/human_move/ee_pose', 10)
        self.ee_pitch_to_ground_pub = self.create_publisher(
            Float32, '/main_arm/human_move/ee_pitch_to_ground_deg', 10)
        self.gripper_opening_pub = self.create_publisher(
            Float32, '/main_arm/human_move/gripper_opening_mm', 10)
        self.summary_pub = self.create_publisher(
            String, '/main_arm/human_move/summary', 10)

        self.publish_timer = self.create_timer(
            1.0 / max(self.publish_rate, 0.1), self._publish_state)
        self.print_timer = self.create_timer(
            1.0 / max(self.print_rate, 0.1), self._print_state)

        if self.torque_off_arm or self.torque_off_gripper:
            self.get_logger().info(
                'main_arm_human_move_for_test ready. Torque-off mode is active.')
        else:
            self.get_logger().info(
                'main_arm_human_move_for_test ready. Monitoring only; torque unchanged.')

    def _set_torque(self, enable: bool) -> None:
        """Enable or disable torque on the arm group and (optionally) gripper."""
        verb = 'ENABLING' if enable else 'DISABLING'
        try:
            if self.torque_off_arm:
                self.get_logger().info(f'{verb} torque on group "arm"...')
                self.bot.core.robot_torque_enable('group', 'arm', enable)
            if self.torque_off_gripper:
                self.get_logger().info(f'{verb} torque on motor "gripper"...')
                self.bot.core.robot_torque_enable('single', 'gripper', enable)

            if enable:
                try:
                    self.bot.arm.capture_joint_positions()
                except Exception as e:
                    self.get_logger().warn(f'capture_joint_positions failed: {e}')
        except Exception as e:
            self.get_logger().error(f'Failed to {verb.lower()} torque: {e}')

    def _read_state(self):
        """Read the current state of the arm. Returns a dict or None on failure."""
        try:
            joint_names = list(self.bot.arm.group_info.joint_names)
            joint_positions = self.bot.arm.get_joint_positions()

            T_sb = self.bot.arm.get_ee_pose()
            x = float(T_sb[0, 3])
            y = float(T_sb[1, 3])
            z = float(T_sb[2, 3])

            R = T_sb[0:3, 0:3]

            roll = math.atan2(R[2, 1], R[2, 2])
            pitch = math.atan2(-R[2, 0], math.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
            yaw = math.atan2(R[1, 0], R[0, 0])

            forward_world = R[:, 0]
            pitch_to_ground = math.asin(
                max(-1.0, min(1.0, float(forward_world[2]))))

            try:
                finger_pos = float(self.bot.gripper.get_finger_position())
            except Exception:
                finger_pos = float('nan')

            opening_width_m = 2.0 * finger_pos if not math.isnan(finger_pos) else float('nan')
            max_opening_m = 2.0 * self.gripper_upper
            min_opening_m = 2.0 * self.gripper_lower
            span = max(max_opening_m - min_opening_m, 1e-6)
            if math.isnan(opening_width_m):
                opening_pct = float('nan')
            else:
                opening_pct = 100.0 * (opening_width_m - min_opening_m) / span
                opening_pct = max(0.0, min(100.0, opening_pct))

            return {
                'joint_names': joint_names,
                'joint_positions': list(joint_positions),
                'ee_xyz': (x, y, z),
                'ee_rpy_rad': (roll, pitch, yaw),
                'ee_pitch_to_ground_rad': pitch_to_ground,
                'rotation_matrix': R,
                'gripper_finger_m': finger_pos,
                'gripper_opening_m': opening_width_m,
                'gripper_opening_pct': opening_pct,
            }
        except Exception as e:
            self.get_logger().warn(
                f'Failed to read arm state: {e}', throttle_duration_sec=2.0)
            return None

    def _publish_state(self):
        s = self._read_state()
        if s is None:
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(s['joint_names'])
        js.position = list(s['joint_positions'])
        self.joint_state_pub.publish(js)

        x, y, z = s['ee_xyz']
        roll, pitch, yaw = s['ee_rpy_rad']
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose.orientation.z = cr * cp * sy - sr * sp * cy
        self.ee_pose_pub.publish(pose)

        self.ee_pitch_to_ground_pub.publish(
            Float32(data=float(math.degrees(s['ee_pitch_to_ground_rad']))))

        if not math.isnan(s['gripper_opening_m']):
            self.gripper_opening_pub.publish(
                Float32(data=float(s['gripper_opening_m'] * 1000.0)))

    def _print_state(self):
        s = self._read_state()
        if s is None:
            return

        lines = []
        lines.append('=' * 64)
        if self.torque_off_arm or self.torque_off_gripper:
            lines.append('Main Arm - Human Move (torque OFF)')
        else:
            lines.append('Main Arm - Human Move (torque ON / monitor only)')
        lines.append('-' * 64)

        lines.append('Joints:')
        for name, pos in zip(s['joint_names'], s['joint_positions']):
            lines.append(
                f'  {name:14s}  {pos:+8.4f} rad   {math.degrees(pos):+7.2f} deg')

        x, y, z = s['ee_xyz']
        roll, pitch, yaw = s['ee_rpy_rad']
        lines.append('End-effector position (base frame):')
        lines.append(f'  x = {x:+.4f} m   y = {y:+.4f} m   z = {z:+.4f} m')

        lines.append('End-effector orientation (roll-pitch-yaw):')
        lines.append(
            f'  roll  = {math.degrees(roll):+7.2f} deg   '
            f'pitch = {math.degrees(pitch):+7.2f} deg   '
            f'yaw   = {math.degrees(yaw):+7.2f} deg')

        ang = math.degrees(s['ee_pitch_to_ground_rad'])
        if ang > 1.0:
            tip = '(tip is pointing UP)'
        elif ang < -1.0:
            tip = '(tip is pointing DOWN)'
        else:
            tip = '(tip is roughly HORIZONTAL)'
        lines.append(
            f'EE forward axis vs ground: {ang:+7.2f} deg  {tip}')

        if math.isnan(s['gripper_finger_m']):
            lines.append('Gripper: <unavailable>')
        else:
            lines.append(
                f'Gripper finger pos: {s["gripper_finger_m"] * 1000.0:6.2f} mm')
            lines.append(
                f'Gripper opening:    {s["gripper_opening_m"] * 1000.0:6.2f} mm '
                f'({s["gripper_opening_pct"]:5.1f}% of max)')

        lines.append('=' * 64)

        self.get_logger().info('\n'.join(lines))

        summary = String()
        summary.data = (
            f'xyz=({x:+.3f},{y:+.3f},{z:+.3f}) m | '
            f'rpy=({math.degrees(roll):+.1f},{math.degrees(pitch):+.1f},'
            f'{math.degrees(yaw):+.1f}) deg | '
            f'ee_to_ground={ang:+.1f} deg | '
            f'gripper={s["gripper_opening_m"] * 1000.0:.1f} mm'
            if not math.isnan(s['gripper_opening_m']) else
            f'xyz=({x:+.3f},{y:+.3f},{z:+.3f}) m | '
            f'rpy=({math.degrees(roll):+.1f},{math.degrees(pitch):+.1f},'
            f'{math.degrees(yaw):+.1f}) deg | '
            f'ee_to_ground={ang:+.1f} deg | gripper=NaN'
        )
        self.summary_pub.publish(summary)

    def shutdown(self):
        try:
            if (self.torque_off_arm or self.torque_off_gripper) and self.torque_on_shutdown:
                self.get_logger().warn(
                    '*** Re-enabling torque on shutdown. '
                    'Make sure nothing is in the way. ***')
                self._set_torque(enable=True)
            elif self.torque_off_arm or self.torque_off_gripper:
                self.get_logger().warn(
                    'torque_on_shutdown=False: torque stays OFF, '
                    'the arm will fall if you let go!')
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MainArmHumanMoveForTest()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'main_arm_human_move_for_test failed: {e}', file=sys.stderr)
    finally:
        if node is not None:
            try:
                node.shutdown()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
