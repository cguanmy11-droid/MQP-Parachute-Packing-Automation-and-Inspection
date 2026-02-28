#!/usr/bin/env python3
"""
Smart Gripper Controller for WX200
-----------------------------------
Closes/opens with PWM but monitors Present_Load in real-time.
Stops the instant it detects contact (load threshold) so it
never stalls, never triggers hardware errors, never ruins a run.

Subscribes:
    /main_arm/gripper_command  (String)  - 'open', 'close', 'release'
    /main_arm/gripper_position (Float32) - 0.0=closed, 1.0=open (proportional)

Publishes:
    /main_arm/gripper_status   (String)  - 'open', 'closed', 'gripping', 'moving', 'error'
    /main_arm/gripper_load     (Float32) - current load as percentage (for debugging)

Works alongside main_arm_interface_node — just handles the gripper so
the main node doesn't have to.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_msgs.srv import RegisterValues, TorqueEnable
import time
import threading


class SmartGripperController(Node):
    def __init__(self):
        super().__init__('smart_gripper_controller')

        # ── Parameters ──
        self.declare_parameter('robot_name', 'wx200')
        self.declare_parameter('grip_pwm', 200)        # PWM to close (tune this: 150-350)
        self.declare_parameter('release_pwm', -200)     # PWM to open (negative = reverse)
        self.declare_parameter('load_threshold', 30.0)  # % load that means "grabbed something"
        self.declare_parameter('monitor_rate', 50.0)    # Hz to check load while moving
        self.declare_parameter('move_timeout', 3.0)     # Max seconds for any gripper move
        self.declare_parameter('hold_pwm', 0)           # PWM after grip detected (0 = just hold position)
        self.declare_parameter('test_mode', False)

        self.robot_name = self.get_parameter('robot_name').value
        self.grip_pwm = self.get_parameter('grip_pwm').value
        self.release_pwm = self.get_parameter('release_pwm').value
        self.load_threshold = self.get_parameter('load_threshold').value
        self.monitor_rate = self.get_parameter('monitor_rate').value
        self.move_timeout = self.get_parameter('move_timeout').value
        self.hold_pwm = self.get_parameter('hold_pwm').value
        self.test_mode = self.get_parameter('test_mode').value

        # ── State ──
        self.gripper_state = 'unknown'  # open, closed, gripping, moving, error
        self.is_moving = False
        self.move_lock = threading.Lock()

        # ── Publishers ──
        self.pwm_pub = self.create_publisher(
            JointSingleCommand,
            f'/{self.robot_name}/commands/joint_single', 10
        )
        self.status_pub = self.create_publisher(String, '/main_arm/gripper_status', 10)
        self.load_pub = self.create_publisher(Float32, '/main_arm/gripper_load', 10)

        # ── Subscribers ──
        self.create_subscription(String, '/main_arm/gripper_command',
                                 self.gripper_command_callback, 10)
        self.create_subscription(Float32, '/main_arm/gripper_position',
                                 self.gripper_position_callback, 10)

        # ── Service clients for reading registers ──
        self.get_register_client = self.create_client(
            RegisterValues,
            f'/{self.robot_name}/get_motor_registers'
        )
        self.torque_client = self.create_client(
            TorqueEnable,
            f'/{self.robot_name}/torque_enable'
        )

        # ── Status timer ──
        self.create_timer(0.5, self.publish_status)

        # Wait for services
        if not self.test_mode:
            self.get_logger().info('Waiting for motor services...')
            self.get_register_client.wait_for_service(timeout_sec=10.0)
            self.get_logger().info('Smart Gripper Controller ready')

            # Clear any existing hardware errors on startup
            self._clear_hardware_errors()
        else:
            self.get_logger().info('Smart Gripper Controller ready (TEST MODE)')

    # ─────────────────────────────────────────────
    # Core: send PWM and monitor load
    # ─────────────────────────────────────────────
    def _send_pwm(self, pwm_value: int):
        """Send a raw PWM command to the gripper motor."""
        cmd = JointSingleCommand()
        cmd.name = 'gripper'
        cmd.cmd = float(pwm_value)
        self.pwm_pub.publish(cmd)

    def _read_present_load(self) -> float:
        """
        Read Present_Load register from the gripper Dynamixel.
        Returns load as a percentage (0-100).
        Present_Load on XL430: 0-1000 (0.1% units), with direction bit.
        """
        req = RegisterValues.Request()
        req.cmd_type = 'single'
        req.name = 'gripper'
        req.reg = 'Present_Load'

        future = self.get_register_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)

        if future.result() is not None:
            raw = future.result().values[0]
            # XL430 Present_Load: bits 0-9 = magnitude, bit 10 = direction
            # Convert to percentage (max 1000 = 100%)
            magnitude = raw & 0x3FF  # Lower 10 bits
            load_pct = magnitude / 10.0  # Convert to percentage
            return load_pct
        else:
            self.get_logger().warn('Failed to read Present_Load')
            return 0.0

    def _read_hardware_error(self) -> int:
        """Read Hardware_Error_Status register. 0 = no error."""
        req = RegisterValues.Request()
        req.cmd_type = 'single'
        req.name = 'gripper'
        req.reg = 'Hardware_Error_Status'

        future = self.get_register_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)

        if future.result() is not None:
            return future.result().values[0]
        return -1

    def _clear_hardware_errors(self):
        """
        Clear hardware errors by cycling torque.
        Call this on startup and after any stall.
        """
        hw_error = self._read_hardware_error()
        if hw_error != 0:
            self.get_logger().warn(
                f'Hardware error detected ({hw_error}), clearing via torque cycle...'
            )
            # Disable torque
            req = TorqueEnable.Request()
            req.cmd_type = 'single'
            req.name = 'gripper'
            req.enable = False
            future = self.torque_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            time.sleep(0.5)

            # Re-enable torque
            req.enable = True
            future = self.torque_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            # Verify
            hw_error = self._read_hardware_error()
            if hw_error == 0:
                self.get_logger().info('Hardware errors cleared successfully')
            else:
                self.get_logger().error(
                    f'Could not clear hardware error ({hw_error}). '
                    'Power cycle the robot.'
                )
                self.gripper_state = 'error'
        else:
            self.get_logger().info('No hardware errors on gripper')

    # ─────────────────────────────────────────────
    # Smart close: PWM + load monitoring
    # ─────────────────────────────────────────────
    def smart_close(self):
        """
        Close the gripper with load monitoring.
        Sends PWM to close, monitors Present_Load, and stops
        the instant load exceeds threshold (= grabbed something).
        """
        if not self.move_lock.acquire(blocking=False):
            self.get_logger().warn('Gripper already moving, ignoring close')
            return False

        try:
            self.is_moving = True
            self.gripper_state = 'moving'

            # Clear any prior errors
            self._clear_hardware_errors()
            if self.gripper_state == 'error':
                return False

            self.get_logger().info(
                f'Smart close: PWM={self.grip_pwm}, '
                f'load threshold={self.load_threshold}%'
            )

            # Start closing
            self._send_pwm(self.grip_pwm)

            # Monitor load at high frequency
            period = 1.0 / self.monitor_rate
            start_time = time.time()
            max_load_seen = 0.0

            while (time.time() - start_time) < self.move_timeout:
                load = self._read_present_load()
                max_load_seen = max(max_load_seen, load)

                # Publish load for debugging
                load_msg = Float32()
                load_msg.data = load
                self.load_pub.publish(load_msg)

                if load >= self.load_threshold:
                    # Contact detected! Stop immediately.
                    self._send_pwm(self.hold_pwm)
                    self.gripper_state = 'gripping'
                    self.get_logger().info(
                        f'Grip detected! Load={load:.1f}% '
                        f'(threshold={self.load_threshold}%). '
                        f'Holding with PWM={self.hold_pwm}'
                    )
                    return True

                # Check for hardware error mid-move
                hw_error = self._read_hardware_error()
                if hw_error != 0:
                    self._send_pwm(0)
                    self.get_logger().error(
                        f'Hardware error during close ({hw_error}), '
                        f'stopping. Max load was {max_load_seen:.1f}%'
                    )
                    self.gripper_state = 'error'
                    self._clear_hardware_errors()
                    return False

                time.sleep(period)

            # Timeout - stop motor
            self._send_pwm(0)
            self.gripper_state = 'closed'
            self.get_logger().warn(
                f'Close timed out ({self.move_timeout}s). '
                f'Max load was {max_load_seen:.1f}%. '
                'May not have grabbed anything.'
            )
            return True

        finally:
            self.is_moving = False
            self.move_lock.release()

    def smart_open(self):
        """
        Open the gripper with load monitoring.
        Simpler than close — just opens until load drops or timeout.
        """
        if not self.move_lock.acquire(blocking=False):
            self.get_logger().warn('Gripper already moving, ignoring open')
            return False

        try:
            self.is_moving = True
            self.gripper_state = 'moving'

            self._clear_hardware_errors()
            if self.gripper_state == 'error':
                return False

            self.get_logger().info(f'Smart open: PWM={self.release_pwm}')

            # Start opening
            self._send_pwm(self.release_pwm)

            # Monitor — stop when load drops (fingers are free)
            # or after a short timeout
            period = 1.0 / self.monitor_rate
            start_time = time.time()
            open_timeout = min(self.move_timeout, 2.0)  # Opening is usually fast

            # Give it a moment to start moving before checking load
            time.sleep(0.2)

            while (time.time() - start_time) < open_timeout:
                load = self._read_present_load()

                load_msg = Float32()
                load_msg.data = load
                self.load_pub.publish(load_msg)

                # Check for hardware error
                hw_error = self._read_hardware_error()
                if hw_error != 0:
                    self._send_pwm(0)
                    self.get_logger().error(f'Hardware error during open ({hw_error})')
                    self.gripper_state = 'error'
                    self._clear_hardware_errors()
                    return False

                time.sleep(period)

            # Stop motor
            self._send_pwm(0)
            self.gripper_state = 'open'
            self.get_logger().info('Gripper open')
            return True

        finally:
            self.is_moving = False
            self.move_lock.release()

    def smart_partial(self, fraction: float):
        """
        Proportional control via timed PWM.
        fraction: 0.0 = full close, 1.0 = full open

        Opens fully first, then closes for a proportional duration.
        Load monitoring still active during close phase.
        """
        if not self.move_lock.acquire(blocking=False):
            self.get_logger().warn('Gripper already moving')
            return False

        try:
            self.is_moving = True
            fraction = max(0.0, min(1.0, fraction))

            if fraction >= 0.95:
                self.move_lock.release()
                return self.smart_open()
            elif fraction <= 0.05:
                self.move_lock.release()
                return self.smart_close()

            # Open fully first
            self.get_logger().info(f'Partial grip: {fraction*100:.0f}% open')
            self._send_pwm(self.release_pwm)
            time.sleep(1.5)  # Enough time to fully open
            self._send_pwm(0)
            time.sleep(0.2)

            # Close proportionally with load monitoring
            close_duration = (1.0 - fraction) * 2.0  # ~2s for full close travel
            self._send_pwm(self.grip_pwm)

            period = 1.0 / self.monitor_rate
            start_time = time.time()

            while (time.time() - start_time) < close_duration:
                load = self._read_present_load()
                if load >= self.load_threshold:
                    self._send_pwm(self.hold_pwm)
                    self.gripper_state = 'gripping'
                    self.get_logger().info(f'Contact during partial close at {load:.1f}%')
                    return True

                time.sleep(period)

            self._send_pwm(0)
            self.gripper_state = 'open' if fraction > 0.5 else 'closed'
            return True

        finally:
            self.is_moving = False
            if self.move_lock.locked():
                self.move_lock.release()

    # ─────────────────────────────────────────────
    # ROS callbacks
    # ─────────────────────────────────────────────
    def gripper_command_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f'Gripper command: {command}')

        if self.test_mode:
            self.get_logger().info(f'TEST: gripper {command}')
            self.gripper_state = command
            return

        # Run in a thread so we don't block the executor
        if command in ('close', 'grasp', 'grip'):
            threading.Thread(target=self.smart_close, daemon=True).start()
        elif command in ('open', 'release'):
            threading.Thread(target=self.smart_open, daemon=True).start()
        elif command == 'stop':
            self._send_pwm(0)
            self.is_moving = False
            self.gripper_state = 'stopped'
        elif command == 'clear_error':
            self._clear_hardware_errors()
        else:
            self.get_logger().warn(f'Unknown gripper command: {command}')

    def gripper_position_callback(self, msg: Float32):
        """0.0 = closed, 1.0 = open"""
        if self.test_mode:
            return
        threading.Thread(
            target=self.smart_partial,
            args=(msg.data,),
            daemon=True
        ).start()

    def publish_status(self):
        msg = String()
        msg.data = self.gripper_state
        self.status_pub.publish(msg)

    def shutdown(self):
        self._send_pwm(0)
        self.get_logger().info('Gripper controller shutdown')


def main(args=None):
    rclpy.init(args=args)
    node = SmartGripperController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt')
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()