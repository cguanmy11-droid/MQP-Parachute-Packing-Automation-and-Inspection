"""
perception_bridge.py
--------------------
ROS2 subscription helpers for the loop perception panel.

Add this to your existing gui_node.py or import it separately.
Connects the LoopPerceptionPanel to live ROS2 topics.

Usage in gui_node.py:
    from parachute_gui.perception_bridge import PerceptionBridge

    # In ROSBridge.__init__:
    self.perception = PerceptionBridge(self._node)

    # In main_window.py, after creating LoopPerceptionPanel:
    bridge.perception.connect_panel(perception_panel)
"""
from __future__ import annotations

from typing import Optional
from PyQt5.QtCore import QObject, pyqtSignal

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# Import with fallback for when parachute_interfaces isn't built yet
try:
    from parachute_interfaces.msg import LoopStateArray
    HAS_LOOP_STATE = True
except ImportError:
    HAS_LOOP_STATE = False


class PerceptionBridge(QObject):
    """
    Bridges ROS2 perception topics to Qt signals for the GUI.
    
    Signals emitted on the Qt thread (via QObject) so they're 
    safe to connect to widget slots directly.
    """
    # Signals carry the raw ROS message
    fused_states_received = pyqtSignal(object)
    top_cam_received = pyqtSignal(object)
    left_cam_received = pyqtSignal(object)
    right_cam_received = pyqtSignal(object)

    def __init__(self, node: Node):
        super().__init__()
        self._node = node

        # Subscribe to fused loop states
        if HAS_LOOP_STATE:
            self._node.create_subscription(
                LoopStateArray,
                '/fused_loop_states',
                self._fused_cb, 10,
            )
            self._node.get_logger().info('[GUI] Subscribed to /fused_loop_states')
        else:
            self._node.get_logger().warn(
                '[GUI] LoopStateArray not available — fused states disabled')

        # Subscribe to camera image topics
        self._node.create_subscription(
            Image, '/top_cam/image', self._top_cam_cb, 10)
        self._node.create_subscription(
            Image, '/side_arm_left/yolo/image', self._left_cam_cb, 10)
        self._node.create_subscription(
            Image, '/side_arm_right/yolo/image', self._right_cam_cb, 10)

        self._node.get_logger().info('[GUI] Subscribed to camera image topics')

    def _fused_cb(self, msg):
        self.fused_states_received.emit(msg)

    def _top_cam_cb(self, msg):
        self.top_cam_received.emit(msg)

    def _left_cam_cb(self, msg):
        self.left_cam_received.emit(msg)

    def _right_cam_cb(self, msg):
        self.right_cam_received.emit(msg)

    def connect_panel(self, panel):
        """
        Wire all signals to a LoopPerceptionPanel instance.
        
        Call this after creating the panel widget:
            perception_bridge.connect_panel(my_perception_panel)
        """
        self.fused_states_received.connect(
            lambda msg: panel.update_loops(msg.loops))
        self.top_cam_received.connect(panel.update_top_cam_image)
        self.left_cam_received.connect(panel.update_left_cam_image)
        self.right_cam_received.connect(panel.update_right_cam_image)