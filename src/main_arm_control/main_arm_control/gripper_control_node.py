#!/usr/bin/env python3
"""
Smart Gripper Controller for WX200 (v2)
-----------------------------------------
PWM control with both load monitoring AND position tracking.

Fixes:
- Ignores load during initial settling period (static friction spike)
- Tracks encoder position to detect mechanical limits
- Stops based on whichever triggers first: load threshold OR position stall

Subscribes:
    /main_arm/gripper_command  (String)  - 'open', 'close', 'stop', 'clear_error'
    /main_arm/gripper_position (Float32) - 0.0=closed, 1.0=open (proportional)

Publishes:
    /main_arm/gripper_status   (String)  - 'open', 'closed', 'gripping', 'moving', 'error'
    /main_arm/gripper_load     (Float32) - current load % (for tuning)
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
        self.declare_parameter('grip_pwm', -200)         # PWM to close (tune: 150-350)
        self.declare_parameter('release_pwm', 200)      # PWM to open
        self.declare_parameter('load_threshold', 40.0)   # % load = "grabbed something"
        self.declare_parameter('monitor_rate', 50.0)     # Hz to check load/position
        self.declare_parameter('move_timeout', 3.0)      # Max seconds for any move
        self.declare_parameter('hold_pwm', 0)            # PWM after grip (0 = coast/hold)
        self.declare_parameter('test_mode', False)

        # Settling: ignore load readings for this long after starting a move.
        # Lets the motor get past static friction without false-triggering.
        self.declare_parameter('settle_time', 0.5)

        # Position stall detection: if encoder moves less than this (in raw units)
        # over stall_check_window seconds, we've hit a mechanical limit.
        self.declare_parameter('stall_position_threshold', 5)   # raw encoder ticks
        self.declare_parameter('stall_check_window', 0.3)       # seconds

        self.robot_name = self.get_parameter('robot_name').value
        self.grip_pwm = self.get_parameter('grip_pwm').value
        self.release_pwm = self.get_parameter('release_pwm').value
        self.load_threshold = self.get_parameter('load_threshold').value
        self.monitor_rate = self.get_parameter('monitor_rate').value
        self.move_timeout = self.get_parameter('move_timeout').value
        self.hold_pwm = self.get_parameter('hold_pwm').value
        self.test_mode = self.get_parameter('test_mode').value
        self.settle_time = self.get_parameter('settle_time').value
        self.stall_position_threshold = self.get_parameter('stall_position_threshold').value
        self.stall_check_window = self.get_parameter('stall_check_window').value

        # ── State ──
        self.gripper_state = 'unknown'
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

        # ── Service clients ──
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

        if not self.test_mode:
            self.get_logger().info('Waiting for motor services...')
            self.get_register_client.wait_for_service(timeout_sec=10.0)
            self._clear_hardware_errors()
            self.get_logger().info('Smart Gripper Controller v2 ready')
        else:
            self.get_logger().info('Smart Gripper Controller v2 ready (TEST MODE)')

    # ─────────────────────────────────────────────
    # Low-level motor helpers
    # ─────────────────────────────────────────────
    def _send_pwm(self, pwm_value: int):
        cmd = JointSingleCommand()
        cmd.name = 'gripper'
        cmd.cmd = float(pwm_value)
        self.pwm_pub.publish(cmd)

    def _read_register(self, reg_name: str) -> int:
        """Read a single register from the gripper motor."""
        req = RegisterValues.Request()
        req.cmd_type = 'single'
        req.name = 'gripper'
        req.reg = reg_name

        future = self.get_register_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)

        if future.result() is not None and len(future.result().values) > 0:
            return future.result().values[0]
        return -1

    def _read_present_load(self) -> float:
        """Read Present_Load as a percentage (0-100)."""
        raw = self._read_register('Present_Load')
        if raw < 0:
            return 0.0
        magnitude = raw & 0x3FF
        return magnitude / 10.0

    def _read_present_position(self) -> int:
        """Read Present_Position in raw encoder ticks."""
        return self._read_register('Present_Position')

    def _read_hardware_error(self) -> int:
        return self._read_register('Hardware_Error_Status')

    def _clear_hardware_errors(self):
        hw_error = self._read_hardware_error()
        if hw_error != 0 and hw_error != -1:
            self.get_logger().warn(f'Hardware error ({hw_error}), clearing...')

            req = TorqueEnable.Request()
            req.cmd_type = 'single'
            req.name = 'gripper'

            req.enable = False
            future = self.torque_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            time.sleep(0.5)

            req.enable = True
            future = self.torque_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            hw_error = self._read_hardware_error()
            if hw_error == 0:
                self.get_logger().info('Hardware errors cleared')
            else:
                self.get_logger().error(
                    f'Could not clear error ({hw_error}). Power cycle needed.'
                )
                self.gripper_state = 'error'
        else:
            self.get_logger().info('No hardware errors on gripper')

    # ─────────────────────────────────────────────
    # Position stall detection
    # ─────────────────────────────────────────────
    def _is_stalled(self, position_history: list, timestamps: list) -> bool:
        """
        Check if the motor has stopped moving by comparing recent
        encoder positions. Returns True if position hasn't changed
        significantly over the stall check window.
        """
        if len(position_history) < 2:
            return False

        now = timestamps[-1]
        window_start = now - self.stall_check_window
        recent = [
            (t, p) for t, p in zip(timestamps, position_history)
            if t >= window_start
        ]

        if len(recent) < 2:
            return False

        pos_range = abs(recent[-1][1] - recent[0][1])
        return pos_range < self.stall_position_threshold

    # ─────────────────────────────────────────────
    # Smart close
    # ─────────────────────────────────────────────
    def smart_close(self):
        """
        Close with load monitoring + position stall detection.
        Ignores load during settle_time to get past static friction.
        """
        if not self.move_lock.acquire(blocking=False):
            self.get_logger().warn('Gripper already moving, ignoring close')
            return False

        try:
            self.is_moving = True
            self.gripper_state = 'moving'

            self._clear_hardware_errors()
            if self.gripper_state == 'error':
                return False

            self.get_logger().info(
                f'Smart close: PWM={self.grip_pwm}, '
                f'load_thresh={self.load_threshold}%, '
                f'settle={self.settle_time}s'
            )

            self._send_pwm(self.grip_pwm)

            period = 1.0 / self.monitor_rate
            start_time = time.time()
            settled = False
            max_load_seen = 0.0
            position_history = []
            timestamps = []

            while (time.time() - start_time) < self.move_timeout:
                elapsed = time.time() - start_time
                now = time.time()

                load = self._read_present_load()
                position = self._read_present_position()
                max_load_seen = max(max_load_seen, load)

                position_history.append(position)
                timestamps.append(now)

                # Publish for tuning
                load_msg = Float32()
                load_msg.data = load
                self.load_pub.publish(load_msg)

                # After settling, start checking load
                if not settled and elapsed >= self.settle_time:
                    settled = True
                    self.get_logger().info(
                        f'Settled. Monitoring load (current={load:.1f}%, '
                        f'pos={position})'
                    )

                # Load-based grip detection (only after settling)
                if settled and load >= self.load_threshold:
                    self._send_pwm(self.hold_pwm)
                    self.gripper_state = 'gripping'
                    self.get_logger().info(
                        f'Grip detected! Load={load:.1f}% pos={position}. '
                        f'Holding PWM={self.hold_pwm}'
                    )
                    return True

                # Position stall detection (only after settling)
                if settled and self._is_stalled(position_history, timestamps):
                    self._send_pwm(self.hold_pwm)
                    self.gripper_state = 'closed'
                    self.get_logger().info(
                        f'Position stall at {position} (load={load:.1f}%). '
                        f'Stopping.'
                    )
                    return True

                # Hardware error check
                hw_error = self._read_hardware_error()
                if hw_error != 0 and hw_error != -1:
                    self._send_pwm(0)
                    self.get_logger().error(
                        f'Hardware error during close ({hw_error}). '
                        f'Max load={max_load_seen:.1f}%'
                    )
                    self.gripper_state = 'error'
                    self._clear_hardware_errors()
                    return False

                time.sleep(period)

            self._send_pwm(0)
            self.gripper_state = 'closed'
            self.get_logger().warn(
                f'Close timed out ({self.move_timeout}s). '
                f'Max load={max_load_seen:.1f}%'
            )
            return True

        finally:
            self.is_moving = False
            self.move_lock.release()

    # ─────────────────────────────────────────────
    # Smart open
    # ─────────────────────────────────────────────
    def smart_open(self):
        """
        Open with position stall detection.
        Stops when encoder stops changing (= fully open).
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

            self._send_pwm(self.release_pwm)

            period = 1.0 / self.monitor_rate
            start_time = time.time()
            position_history = []
            timestamps = []
            open_timeout = min(self.move_timeout, 2.5)

            while (time.time() - start_time) < open_timeout:
                elapsed = time.time() - start_time
                now = time.time()

                position = self._read_present_position()
                position_history.append(position)
                timestamps.append(now)

                # After settling, check if we've stopped
                if elapsed > self.settle_time:
                    if self._is_stalled(position_history, timestamps):
                        self._send_pwm(0)
                        self.gripper_state = 'open'
                        self.get_logger().info(
                            f'Fully open (stall at position={position})'
                        )
                        return True

                hw_error = self._read_hardware_error()
                if hw_error != 0 and hw_error != -1:
                    self._send_pwm(0)
                    self.get_logger().error(
                        f'Hardware error during open ({hw_error})'
                    )
                    self.gripper_state = 'error'
                    self._clear_hardware_errors()
                    return False

                time.sleep(period)

            self._send_pwm(0)
            self.gripper_state = 'open'
            self.get_logger().info('Open complete (timeout)')
            return True

        finally:
            self.is_moving = False
            self.move_lock.release()

    # ─────────────────────────────────────────────
    # Proportional control via timed PWM
    # ─────────────────────────────────────────────
    def smart_partial(self, fraction: float):
        """
        fraction: 0.0 = closed, 1.0 = open.
        Opens fully, then closes for a proportional duration.
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

            self.get_logger().info(f'Partial grip: {fraction*100:.0f}% open')

            # Open fully first
            self._send_pwm(self.release_pwm)
            time.sleep(1.5)
            self._send_pwm(0)
            time.sleep(0.2)

            # Close proportionally, still monitoring
            close_duration = (1.0 - fraction) * 2.0
            self._send_pwm(self.grip_pwm)

            period = 1.0 / self.monitor_rate
            start_time = time.time()
            position_history = []
            timestamps = []

            while (time.time() - start_time) < close_duration:
                elapsed = time.time() - start_time
                now = time.time()

                load = self._read_present_load()
                position = self._read_present_position()
                position_history.append(position)
                timestamps.append(now)

                if elapsed > self.settle_time and load >= self.load_threshold:
                    self._send_pwm(self.hold_pwm)
                    self.gripper_state = 'gripping'
                    self.get_logger().info(
                        f'Contact during partial at {load:.1f}%'
                    )
                    return True

                if elapsed > self.settle_time:
                    if self._is_stalled(position_history, timestamps):
                        self._send_pwm(self.hold_pwm)
                        self.gripper_state = 'closed'
                        self.get_logger().info('Stall during partial close')
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
            self.gripper_state = command
            return

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
            self.get_logger().warn(f'Unknown command: {command}')

    def gripper_position_callback(self, msg: Float32):
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