#!/usr/bin/env python3
"""
Motor Health Monitor for WX200
--------------------------------
Periodically reads Dynamixel registers for all arm motors to detect
hardware errors, overload, and overheating BEFORE they stall a run.

Publishes:
    /main_arm/motor_health   (String)  - JSON with per-motor status
    /main_arm/motor_alerts   (String)  - Human-readable alerts (only when problems found)

Subscribes:
    /main_arm/motor_health_cmd (String) - 'clear_all', 'clear <joint_name>', 'status'

Monitors:
    - Hardware_Error_Status  (0 = ok, nonzero = error bits)
    - Present_Load           (detect near-stall before it triggers hardware error)
    - Present_Temperature    (detect overheating trend)

Dynamixel Hardware Error Bits:
    Bit 0: Input Voltage Error
    Bit 2: Overheating Error
    Bit 3: Motor Encoder Error
    Bit 4: Electrical Shock Error
    Bit 5: Overload Error
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from interbotix_xs_msgs.srv import RegisterValues, TorqueEnable


# Hardware error bit definitions
ERROR_BITS = {
    0: 'Input Voltage',
    2: 'Overheating',
    3: 'Motor Encoder',
    4: 'Electrical Shock',
    5: 'Overload',
}

# WX200 arm joint names (excludes gripper, handled by smart_gripper_controller)
ARM_JOINTS = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']


class MotorHealthMonitor(Node):
    def __init__(self):
        super().__init__('motor_health_monitor')

        # ── Parameters ──
        self.declare_parameter('robot_name', 'wx200')
        self.declare_parameter('monitor_rate', 2.0)          # Hz - how often to check
        self.declare_parameter('load_warn_threshold', 60.0)  # % load to warn
        self.declare_parameter('load_critical_threshold', 80.0)  # % load to alert
        self.declare_parameter('temp_warn_threshold', 55)    # °C to warn
        self.declare_parameter('temp_critical_threshold', 65) # °C to alert
        self.declare_parameter('auto_clear', False)          # Auto-clear errors via torque cycle
        self.declare_parameter('monitor_gripper', False)     # Also monitor gripper motor
        self.declare_parameter('test_mode', False)

        self.robot_name = self.get_parameter('robot_name').value
        self.monitor_rate = self.get_parameter('monitor_rate').value
        self.load_warn = self.get_parameter('load_warn_threshold').value
        self.load_critical = self.get_parameter('load_critical_threshold').value
        self.temp_warn = self.get_parameter('temp_warn_threshold').value
        self.temp_critical = self.get_parameter('temp_critical_threshold').value
        self.auto_clear = self.get_parameter('auto_clear').value
        self.monitor_gripper = self.get_parameter('monitor_gripper').value
        self.test_mode = self.get_parameter('test_mode').value

        # Build joint list
        self.joints = list(ARM_JOINTS)
        if self.monitor_gripper:
            self.joints.append('gripper')

        # Track state per motor
        self.motor_status = {}
        for joint in self.joints:
            self.motor_status[joint] = {
                'hw_error': 0,
                'hw_error_names': [],
                'load': 0.0,
                'temperature': 0,
                'healthy': True,
                'warning': '',
                'consecutive_high_load': 0,
            }

        # Track overall system state
        self.system_healthy = True
        self.error_count = 0
        self.last_alert = ''
        self._services_ready = False
        self._service_check_logged = False

        # Callback group for service calls (allows concurrent execution)
        self._service_cb_group = MutuallyExclusiveCallbackGroup()

        # ── Publishers ──
        self.health_pub = self.create_publisher(String, '/main_arm/motor_health', 10)
        self.alert_pub = self.create_publisher(String, '/main_arm/motor_alerts', 10)

        # ── Subscribers ──
        self.create_subscription(
            String, '/main_arm/motor_health_cmd',
            self.health_cmd_callback, 10
        )

        # ── Service clients (in separate callback group) ──
        self.get_register_client = self.create_client(
            RegisterValues,
            f'/{self.robot_name}/get_motor_registers',
            callback_group=self._service_cb_group
        )
        self.torque_client = self.create_client(
            TorqueEnable,
            f'/{self.robot_name}/torque_enable',
            callback_group=self._service_cb_group
        )

        if self.test_mode:
            self._services_ready = True
            self.get_logger().info('Motor Health Monitor ready (TEST MODE)')
        else:
            self.get_logger().info('Motor Health Monitor starting - will wait for motor services...')

        # ── Monitor timer ──
        self.create_timer(1.0 / self.monitor_rate, self.monitor_tick)

        # ── Periodic summary (every 30s) ──
        self.create_timer(30.0, self.print_summary)

    # ─────────────────────────────────────────────
    # Service availability check
    # ─────────────────────────────────────────────
    def _check_services_ready(self) -> bool:
        """Check if motor services are available (non-blocking)."""
        if self._services_ready:
            return True

        if self.get_register_client.service_is_ready():
            self._services_ready = True
            self.get_logger().info(
                f'Motor Health Monitor ready - watching {len(self.joints)} joints '
                f'at {self.monitor_rate}Hz'
            )
            return True

        # Log waiting message once
        if not self._service_check_logged:
            self.get_logger().info('Waiting for motor services to become available...')
            self._service_check_logged = True

        return False

    # ─────────────────────────────────────────────
    # Register reading
    # ─────────────────────────────────────────────
    def _read_register(self, joint_name: str, reg_name: str) -> int:
        """Read a single register from a specific joint motor."""
        if self.test_mode:
            return 0

        if not self._services_ready:
            return -1

        req = RegisterValues.Request()
        req.cmd_type = 'single'
        req.name = joint_name
        req.reg = reg_name

        try:
            future = self.get_register_client.call_async(req)
            # Wait for result with timeout (works with MultiThreadedExecutor)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)

            if future.done() and future.result() is not None:
                if len(future.result().values) > 0:
                    return future.result().values[0]
        except Exception as e:
            self.get_logger().debug(f'Register read failed for {joint_name}/{reg_name}: {e}')

        return -1

    def _decode_hw_error(self, error_byte: int) -> list:
        """Decode hardware error byte into list of error names."""
        errors = []
        if error_byte <= 0:
            return errors
        for bit, name in ERROR_BITS.items():
            if error_byte & (1 << bit):
                errors.append(name)
        return errors

    def _read_load_percent(self, joint_name: str) -> float:
        """Read Present_Load as percentage."""
        raw = self._read_register(joint_name, 'Present_Load')
        if raw < 0:
            return 0.0
        magnitude = raw & 0x3FF
        return magnitude / 10.0

    # ─────────────────────────────────────────────
    # Error clearing
    # ─────────────────────────────────────────────
    def clear_error(self, joint_name: str) -> bool:
        """Clear hardware error on a joint by cycling torque."""
        if self.test_mode:
            return True

        if not self._services_ready:
            self.get_logger().warn('Cannot clear error - services not ready')
            return False

        self.get_logger().info(f'Clearing error on {joint_name}...')

        req = TorqueEnable.Request()
        req.cmd_type = 'single'
        req.name = joint_name

        try:
            # Disable torque
            req.enable = False
            future = self.torque_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            import time
            time.sleep(0.3)

            # Re-enable torque
            req.enable = True
            future = self.torque_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        except Exception as e:
            self.get_logger().error(f'Failed to cycle torque on {joint_name}: {e}')
            return False

        # Verify
        hw_error = self._read_register(joint_name, 'Hardware_Error_Status')
        if hw_error == 0:
            self.get_logger().info(f'{joint_name}: error cleared successfully')
            self.motor_status[joint_name]['hw_error'] = 0
            self.motor_status[joint_name]['hw_error_names'] = []
            self.motor_status[joint_name]['healthy'] = True
            self.motor_status[joint_name]['warning'] = ''
            return True
        else:
            self.get_logger().error(
                f'{joint_name}: error persists ({hw_error}). '
                'Power cycle may be needed.'
            )
            return False

    def clear_all_errors(self):
        """Clear errors on all joints that have them."""
        cleared = 0
        failed = 0
        for joint in self.joints:
            if self.motor_status[joint]['hw_error'] != 0:
                if self.clear_error(joint):
                    cleared += 1
                else:
                    failed += 1
        self.get_logger().info(
            f'Clear all: {cleared} cleared, {failed} failed'
        )

    # ─────────────────────────────────────────────
    # Main monitoring loop
    # ─────────────────────────────────────────────
    def monitor_tick(self):
        """Check all motors and publish health status."""
        # Check if services are ready (non-blocking)
        if not self._check_services_ready():
            return

        alerts = []

        for joint in self.joints:
            status = self.motor_status[joint]

            # Read hardware error
            hw_error = self._read_register(joint, 'Hardware_Error_Status')
            if hw_error < 0:
                # Read failed, skip this joint this tick
                continue

            status['hw_error'] = hw_error
            status['hw_error_names'] = self._decode_hw_error(hw_error)

            # Read load
            load = self._read_load_percent(joint)
            status['load'] = load

            # Read temperature
            temp = self._read_register(joint, 'Present_Temperature')
            if temp >= 0:
                status['temperature'] = temp

            # ── Evaluate health ──
            status['healthy'] = True
            status['warning'] = ''

            # Hardware error (critical)
            if hw_error != 0:
                status['healthy'] = False
                error_names = ', '.join(status['hw_error_names'])
                status['warning'] = f'HW ERROR: {error_names}'
                alerts.append(f'🔴 {joint}: {error_names}')

                if self.auto_clear:
                    self.clear_error(joint)

            # Load monitoring
            if load >= self.load_critical:
                status['consecutive_high_load'] += 1
                if status['consecutive_high_load'] >= 3:
                    status['healthy'] = False
                    status['warning'] = f'CRITICAL LOAD: {load:.1f}%'
                    alerts.append(f'🟠 {joint}: load {load:.1f}% (stall imminent)')
            elif load >= self.load_warn:
                status['consecutive_high_load'] += 1
                if status['consecutive_high_load'] >= 5:
                    status['warning'] = f'HIGH LOAD: {load:.1f}%'
                    alerts.append(f'🟡 {joint}: load {load:.1f}%')
            else:
                status['consecutive_high_load'] = 0

            # Temperature monitoring
            if temp >= self.temp_critical:
                status['healthy'] = False
                status['warning'] = f'OVERHEATING: {temp}°C'
                alerts.append(f'🔴 {joint}: temperature {temp}°C')
            elif temp >= self.temp_warn:
                if not status['warning']:
                    status['warning'] = f'WARM: {temp}°C'
                alerts.append(f'🟡 {joint}: temperature {temp}°C')

        # ── Update system health ──
        self.system_healthy = all(s['healthy'] for s in self.motor_status.values())
        self.error_count = sum(
            1 for s in self.motor_status.values() if s['hw_error'] != 0
        )

        # ── Publish health JSON ──
        health_msg = String()
        health_data = {
            'system_healthy': self.system_healthy,
            'error_count': self.error_count,
            'motors': {}
        }
        for joint in self.joints:
            s = self.motor_status[joint]
            health_data['motors'][joint] = {
                'healthy': s['healthy'],
                'hw_error': s['hw_error'],
                'load': round(s['load'], 1),
                'temp': s['temperature'],
                'warning': s['warning'],
            }
        health_msg.data = json.dumps(health_data)
        self.health_pub.publish(health_msg)

        # ── Publish alerts (only when there are problems) ──
        if alerts:
            alert_text = ' | '.join(alerts)
            # Only log if alert changed (avoid spam)
            if alert_text != self.last_alert:
                self.get_logger().warn(f'Motor alerts: {alert_text}')
                self.last_alert = alert_text

            alert_msg = String()
            alert_msg.data = alert_text
            self.alert_pub.publish(alert_msg)
        else:
            if self.last_alert:
                self.get_logger().info('All motors healthy')
                self.last_alert = ''

    def print_summary(self):
        """Periodic summary log."""
        if self.test_mode:
            return

        lines = ['Motor Health Summary:']
        for joint in self.joints:
            s = self.motor_status[joint]
            indicator = '✓' if s['healthy'] else '✗'
            line = (
                f'  {indicator} {joint:15s}  '
                f'load={s["load"]:5.1f}%  '
                f'temp={s["temperature"]:3d}°C  '
                f'err={s["hw_error"]}'
            )
            if s['warning']:
                line += f'  ⚠ {s["warning"]}'
            lines.append(line)

        for line in lines:
            self.get_logger().info(line)

    # ─────────────────────────────────────────────
    # Command handler
    # ─────────────────────────────────────────────
    def health_cmd_callback(self, msg):
        """Handle health commands."""
        cmd = msg.data.strip().lower()

        if cmd == 'clear_all':
            self.clear_all_errors()

        elif cmd.startswith('clear '):
            joint_name = cmd.split(' ', 1)[1].strip()
            if joint_name in self.joints:
                self.clear_error(joint_name)
            else:
                self.get_logger().warn(f'Unknown joint: {joint_name}')

        elif cmd == 'status':
            self.print_summary()

        else:
            self.get_logger().warn(f'Unknown health command: {cmd}')


def main(args=None):
    rclpy.init(args=args)
    node = MotorHealthMonitor()

    # Use MultiThreadedExecutor to allow service calls from timer callbacks
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()