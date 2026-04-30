import json
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    import serial
    from serial.serialutil import SerialException
except ImportError as exc:  # pragma: no cover - handled at runtime
    serial = None
    SerialException = Exception
    raise exc


class SideArmSerialBridge(Node):
    """Serial bridge node: ROS 2 <-> ESP32."""

    def __init__(self) -> None:
        super().__init__('side_arm_serial_bridge')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('command_topic', '/side_arm/command')
        self.declare_parameter('state_topic', '/side_arm/state')
        self.declare_parameter('state_request_hz', 5.0)
        self.declare_parameter('auto_request_state', True)
        self.declare_parameter('ignore_limit_switch', False)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.command_topic = self.get_parameter('command_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.state_request_hz = self.get_parameter('state_request_hz').value
        self.auto_request_state = self.get_parameter('auto_request_state').value
        self.ignore_limit_switch = self.get_parameter('ignore_limit_switch').value

        self._serial: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()
        self._shutdown_event = threading.Event()  # Signal for clean shutdown
        self._reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self._reader_active = threading.Event()
        self._reconnect_timer = None
        self._state_request_timer = None

        self._state_pub = self.create_publisher(String, self.state_topic, 10)
        self._cmd_sub = self.create_subscription(String, self.command_topic, self._command_callback, 10)
        self._trigger_srv = self.create_service(Trigger, 'side_arm/request_state', self._trigger_state_srv)

        if self.auto_request_state and self.state_request_hz > 0:
            period = 1.0 / self.state_request_hz
            self._state_request_timer = self.create_timer(period, self._request_state_timer)

        self._connect_serial()
        self._reader_active.set()
        self._reader_thread.start()

    # region Serial helpers
    def _connect_serial(self) -> None:
        try:
            self._serial = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0.1,
                write_timeout=0.1,
            )
            self.get_logger().info(f'Serial connected {self.serial_port} @ {self.baud_rate}')
            if self._reconnect_timer:
                self._reconnect_timer.cancel()
                self._reconnect_timer = None
        except SerialException as exc:
            self._serial = None
            self.get_logger().error(f'Serial connection failed: {exc}')
            if self._reconnect_timer is None:
                self._reconnect_timer = self.create_timer(2.0, self._reconnect_once)

    def _reconnect_once(self) -> None:
        if self._serial:
            return
        self._connect_serial()

    def _write_serial(self, line: str) -> bool:
        if not self._serial:
            self.get_logger().warn('Serial not connected, command dropped')
            return False

        payload = (line if line.endswith('\n') else f'{line}\n').encode('utf-8')

        with self._serial_lock:
            try:
                self._serial.write(payload)
                self._serial.flush()
                return True
            except SerialException as exc:
                self.get_logger().error(f'Serial write failed: {exc}')
                self._serial = None
                self._connect_serial()
                return False

    def _serial_reader(self) -> None:
        while not self._shutdown_event.is_set() and rclpy.ok():
            # Wait for active signal or shutdown (check every 100ms)
            self._reader_active.wait(timeout=0.1)
            if self._shutdown_event.is_set():
                break
            if not self._serial:
                time.sleep(0.1)
                continue
            try:
                line = self._serial.readline().decode('utf-8').strip()
            except SerialException as exc:
                if not self._shutdown_event.is_set():
                    self.get_logger().error(f'Serial read failed: {exc}')
                self._serial = None
                continue

            if line and not self._shutdown_event.is_set():
                if self.ignore_limit_switch and line.startswith("STATE"):
                    # zero out l1/l2/l3 if present
                    try:
                        payload = line.split("STATE", 1)[1].strip()
                        state = json.loads(payload)
                        state["l1"] = 0
                        state["l2"] = 0
                        state["l3"] = 0
                        line = "STATE " + json.dumps(state)
                    except Exception:
                        pass
                msg = String()
                msg.data = line
                if not self._shutdown_event.is_set() and self.context.ok():
                    self._state_pub.publish(msg)

    # endregion

    # region ROS callbacks
    def _command_callback(self, msg: String) -> None:
        success = self._write_serial(msg.data)
        if success:
            self.get_logger().debug(f'sent command: {msg.data}')

    def _request_state_timer(self) -> None:
        self._write_serial('REQUEST_STATE')

    def _trigger_state_srv(self, _, response: Trigger.Response) -> Trigger.Response:
        success = self._write_serial('REQUEST_STATE')
        response.success = success
        response.message = 'state requested' if success else 'request failed'
        return response

    # endregion

    def cleanup(self) -> None:
        """Clean shutdown of threads and resources."""
        self.get_logger().info('Shutting down serial bridge...')

        # Signal shutdown to reader thread
        self._shutdown_event.set()
        self._reader_active.set()  # Wake up thread if waiting

        # Cancel timers
        if self._reconnect_timer is not None:
            self._reconnect_timer.cancel()
        if self._state_request_timer is not None:
            self._state_request_timer.cancel()

        # Wait for reader thread to finish (with timeout)
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)

        # Close serial connection
        with self._serial_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None

        self.get_logger().info('Serial bridge shutdown complete')


def main() -> None:
    rclpy.init()
    node = SideArmSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

