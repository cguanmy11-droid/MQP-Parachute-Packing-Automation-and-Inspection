"""
gui_node.py — ROS2 bridge for operator GUI.

Subscriptions:
  /coordinator/state   String        - state name (may include " (PAUSED)")
  /coordinator/error   String        - error messages
  /detected_loops      DetectedLoops - all perception detections
  /target_loop         DetectedLoop  - current target selected by coordinator

Publishers:
  /stow/command        String        - start|pause|resume|retry|skip|abort|home
  /joystick_enabled    Bool          - joystick mode toggle
"""

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from parachute_interfaces.msg import DetectedLoops, DetectedLoop

from PyQt5.QtCore import QObject, pyqtSignal


class ROSBridge(QObject):
    state_changed         = pyqtSignal(str)
    error_received        = pyqtSignal(str)
    loops_updated         = pyqtSignal(list)   # list of dicts
    target_loop_updated   = pyqtSignal(int)    # loop_id of current target
    joystick_mode_changed = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        rclpy.init()
        self._node = Node('parachute_gui')

        self._node.create_subscription(
            String, '/coordinator/state', self._on_state, 10)
        self._node.create_subscription(
            String, '/coordinator/error', self._on_error, 10)
        self._node.create_subscription(
            DetectedLoops, '/detected_loops', self._on_loops, 10)
        self._node.create_subscription(
            DetectedLoop, '/target_loop', self._on_target_loop, 10)

        self._cmd_pub = self._node.create_publisher(String, '/stow/command', 10)
        self._joy_pub = self._node.create_publisher(Bool, '/joystick_enabled', 10)

        threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True
        ).start()

    # ── ROS2 callbacks ────────────────────────────────────────────────────────

    def _on_state(self, msg):
        self.state_changed.emit(msg.data)

    def _on_error(self, msg):
        self.error_received.emit(msg.data)

    def _on_loops(self, msg: DetectedLoops):
        loops = sorted([{
            'id':         l.loop_id,
            'confidence': l.confidence,
            'x':          l.pose.pose.position.x,
            'y':          l.pose.pose.position.y,
            'z':          l.pose.pose.position.z,
        } for l in msg.loops], key=lambda l: l['id'])
        self.loops_updated.emit(loops)

    def _on_target_loop(self, msg: DetectedLoop):
        self.target_loop_updated.emit(msg.loop_id)

    # ── GUI → ROS2 ────────────────────────────────────────────────────────────

    def send_command(self, command: str):
        self._cmd_pub.publish(String(data=command))

    def set_joystick_mode(self, enabled: bool):
        self._joy_pub.publish(Bool(data=enabled))
        self.joystick_mode_changed.emit(enabled)

    def shutdown(self):
        self._node.destroy_node()
        rclpy.shutdown()