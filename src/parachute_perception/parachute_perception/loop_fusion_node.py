#!/usr/bin/env python3
"""
Loop Fusion Node

Fuses side camera detections (3D positions) with top camera classifications
(stow states) into a unified per-loop state.

Side cameras provide:  3D position of each loop (DetectedLoops)
Top camera provides:   stow state per loop - fully/partial/not (LoopStateArray)
Ground truth provides: expected positions as matching anchors (LoopGroundTruth)

Matching strategy:
  - Each side camera detection is matched to its nearest expected (ground truth)
    position within that side (L or R), using Euclidean distance.
  - Top camera loop IDs (L1, L2, R1, R2...) map directly to canonical IDs.
  - Once a loop is stowed and the side camera loses it, the fusion node
    retains the last known position and updates state from the top camera.

Subscriptions:
  /side_arm_left/detected_loops   (DetectedLoops)  - left side camera
  /side_arm_right/detected_loops  (DetectedLoops)  - right side camera
  /top_cam/loop_states            (LoopStateArray)  - top camera classification
  /loop_ground_truth              (LoopGroundTruth) - expected loop positions

Publications:
  /fused_loop_states  (LoopStateArray)  - unified output
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rclpy
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped

from parachute_interfaces.msg import (
    DetectedLoop,
    DetectedLoops,
    LoopGroundTruth,
    LoopState,
    LoopStateArray,
)


# ---------------------------------------------------------------------------
# Internal state for a single loop
# ---------------------------------------------------------------------------

@dataclass
class FusedLoop:
    """Tracks the fused state of a single physical loop."""
    canonical_id: str          # e.g. "L1", "R3"
    side: str                  # "L" or "R"
    index: int                 # 1-based index within side

    # From ground truth (anchor for matching)
    expected_position: Optional[Tuple[float, float, float]] = None

    # From side camera (updated on each detection)
    detected_position: Optional[Tuple[float, float, float]] = None
    detection_confidence: float = 0.0
    last_detected_stamp: float = 0.0

    # From top camera (updated on each classification)
    stow_state: str = "unknown"       # "fully", "partial", "not", "unknown"
    stow_confidence: float = 0.0
    last_classified_stamp: float = 0.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _euclidean(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def _match_detections_to_expected(
    detections: List[Tuple[int, Tuple[float, float, float], float]],
    expected: List[Tuple[str, Tuple[float, float, float]]],
    max_distance: float,
) -> Dict[str, Tuple[Tuple[float, float, float], float]]:
    """
    Greedy nearest-neighbor matching.

    Args:
        detections: list of (original_index, (x,y,z), confidence)
        expected:   list of (canonical_id, (x,y,z))
        max_distance: max allowable match distance

    Returns:
        dict mapping canonical_id -> (detected_position, confidence)
    """
    if not detections or not expected:
        return {}

    # Build distance matrix
    pairs: List[Tuple[float, int, int]] = []
    for di, (_, d_pos, _) in enumerate(detections):
        for ei, (_, e_pos) in enumerate(expected):
            dist = _euclidean(d_pos, e_pos)
            if dist <= max_distance:
                pairs.append((dist, di, ei))

    pairs.sort(key=lambda t: t[0])

    matched: Dict[str, Tuple[Tuple[float, float, float], float]] = {}
    used_detections = set()
    used_expected = set()

    for dist, di, ei in pairs:
        if di in used_detections or ei in used_expected:
            continue
        canon_id = expected[ei][0]
        _, d_pos, d_conf = detections[di]
        matched[canon_id] = (d_pos, d_conf)
        used_detections.add(di)
        used_expected.add(ei)

    return matched


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class LoopFusionNode(Node):

    def __init__(self) -> None:
        super().__init__('loop_fusion_node')

        # Parameters
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('max_match_distance', 0.03)  # meters
        self.declare_parameter('left_topic', '/side_arm_left/detected_loops')
        self.declare_parameter('right_topic', '/side_arm_right/detected_loops')
        self.declare_parameter('top_cam_topic', '/top_cam/loop_states')
        self.declare_parameter('ground_truth_topic', '/loop_ground_truth')
        self.declare_parameter('output_topic', '/fused_loop_states')
        self.declare_parameter('num_loops_per_side', 5)

        # Y-coordinate threshold to assign ground truth loops to L or R.
        # Loops with y > 0 are Left, y < 0 are Right (matching your launch config).
        self.declare_parameter('y_side_threshold', 0.0)

        publish_rate = self.get_parameter('publish_rate').value
        self.max_match_dist = self.get_parameter('max_match_distance').value
        left_topic = self.get_parameter('left_topic').value
        right_topic = self.get_parameter('right_topic').value
        top_cam_topic = self.get_parameter('top_cam_topic').value
        gt_topic = self.get_parameter('ground_truth_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.num_per_side = self.get_parameter('num_loops_per_side').value
        self.y_threshold = self.get_parameter('y_side_threshold').value

        # TF buffer for transforming side cam detections to world frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Internal state: canonical_id -> FusedLoop
        self.loops: Dict[str, FusedLoop] = {}
        self._init_loops()

        # Ground truth received flag
        self._gt_received = False

        # Subscriptions
        self.create_subscription(
            DetectedLoops, left_topic, self._left_cam_cb, 10)
        self.create_subscription(
            DetectedLoops, right_topic, self._right_cam_cb, 10)
        self.create_subscription(
            LoopStateArray, top_cam_topic, self._top_cam_cb, 10)
        self.create_subscription(
            LoopGroundTruth, gt_topic, self._ground_truth_cb, 10)

        # Publisher
        self.fused_pub = self.create_publisher(LoopStateArray, output_topic, 10)

        # Publish timer
        self.create_timer(1.0 / max(publish_rate, 1.0), self._publish_fused)

        self.get_logger().info(
            f'LoopFusionNode ready → {output_topic}  '
            f'({self.num_per_side} loops/side, '
            f'match threshold {self.max_match_dist:.3f}m)'
        )

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def _init_loops(self) -> None:
        """Pre-create FusedLoop entries for all expected loops."""
        for side in ('L', 'R'):
            for i in range(1, self.num_per_side + 1):
                cid = f'{side}{i}'
                self.loops[cid] = FusedLoop(
                    canonical_id=cid,
                    side=side,
                    index=i,
                )

    # ------------------------------------------------------------------
    # Ground truth callback
    # ------------------------------------------------------------------

    def _ground_truth_cb(self, msg: LoopGroundTruth) -> None:
        """
        Receive expected loop positions and assign them as anchors.

        Ground truth positions are split into L/R by y-coordinate,
        then sorted by x within each side and assigned L1..LN, R1..RN.
        """
        left_positions: List[Tuple[float, float, float]] = []
        right_positions: List[Tuple[float, float, float]] = []

        for pt in msg.positions:
            pos = (pt.x, pt.y, pt.z)
            if pt.y > self.y_threshold:
                left_positions.append(pos)
            else:
                right_positions.append(pos)

        # Sort each side by x (left-to-right on the parachute)
        left_positions.sort(key=lambda p: p[0])
        right_positions.sort(key=lambda p: p[0])

        for i, pos in enumerate(left_positions):
            cid = f'L{i + 1}'
            if cid in self.loops:
                self.loops[cid].expected_position = pos

        for i, pos in enumerate(right_positions):
            cid = f'R{i + 1}'
            if cid in self.loops:
                self.loops[cid].expected_position = pos

        if not self._gt_received:
            self._gt_received = True
            self.get_logger().info(
                f'Ground truth received: {len(left_positions)}L + '
                f'{len(right_positions)}R loops'
            )

    # ------------------------------------------------------------------
    # Side camera callbacks
    # ------------------------------------------------------------------

    def _side_cam_cb(self, msg: DetectedLoops, side: str) -> None:
        """
        Process side camera detections for one side.
        Match each detection to nearest expected position.
        """
        if not self._gt_received:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # Transform each detection to world frame, then match
        detections: List[Tuple[int, Tuple[float, float, float], float]] = []
        for i, loop in enumerate(msg.loops):
            p = loop.pose.pose.position
            frame_id = msg.header.frame_id or loop.pose.header.frame_id

            # Try to transform to world
            try:
                transform = self.tf_buffer.lookup_transform(
                    'world', frame_id, rclpy.time.Time(), 
                    timeout=rclpy.duration.Duration(seconds=0.1))
                
                pt = PointStamped()
                pt.header.frame_id = frame_id
                pt.point = loop.pose.pose.position
                
                from tf2_geometry_msgs import do_transform_point
                pt_world = do_transform_point(pt, transform)
                
                detections.append((i, (pt_world.point.x, pt_world.point.y, pt_world.point.z), loop.confidence))
            except Exception as e:
                self.get_logger().warning(f'TF transform failed: {e}', throttle_duration_sec=5.0)
                return

        # Gather expected positions for this side
        expected: List[Tuple[str, Tuple[float, float, float]]] = []
        for cid, fl in self.loops.items():
            if fl.side == side and fl.expected_position is not None:
                expected.append((cid, fl.expected_position))

        matches = _match_detections_to_expected(
            detections, expected, self.max_match_dist
        )

        for cid, (pos, conf) in matches.items():
            fl = self.loops[cid]
            fl.detected_position = pos
            fl.detection_confidence = conf
            fl.last_detected_stamp = now

    def _left_cam_cb(self, msg: DetectedLoops) -> None:
        self._side_cam_cb(msg, 'L')

    def _right_cam_cb(self, msg: DetectedLoops) -> None:
        self._side_cam_cb(msg, 'R')

    # ------------------------------------------------------------------
    # Top camera callback
    # ------------------------------------------------------------------

    def _top_cam_cb(self, msg: LoopStateArray) -> None:
        """
        Update stow state from top camera.
        Top camera uses IDs like L1, L2, R1, R2 — direct match to canonical.
        """
        now = self.get_clock().now().nanoseconds / 1e9

        for ls in msg.loops:
            cid = ls.loop_id  # Already "L1", "R2", etc.
            if cid in self.loops:
                fl = self.loops[cid]
                fl.stow_state = ls.state
                fl.stow_confidence = ls.confidence
                fl.last_classified_stamp = now

    # ------------------------------------------------------------------
    # Publish fused state
    # ------------------------------------------------------------------

    def _publish_fused(self) -> None:
        """Publish unified loop state at configured rate."""
        msg = LoopStateArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        for cid in sorted(self.loops.keys(), key=self._sort_key):
            fl = self.loops[cid]

            ls = LoopState()
            ls.loop_id = fl.canonical_id
            ls.state = fl.stow_state
            ls.confidence = fl.stow_confidence

            # Use detected position if available, else expected position
            pos = fl.detected_position or fl.expected_position
            if pos is not None:
                # Normalize to 0-1 for LoopState (or use raw — depends on consumer)
                # For now, store raw world coordinates in center_x/center_y
                ls.center_x = float(pos[0])
                ls.center_y = float(pos[1])

            msg.loops.append(ls)

        msg.total_count = len(msg.loops)
        msg.changed = False  # Could track changes if needed
        self.fused_pub.publish(msg)

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    @staticmethod
    def _sort_key(cid: str) -> Tuple[int, int]:
        side = 0 if cid.startswith('L') else 1
        num = int(cid[1:]) if cid[1:].isdigit() else 99
        return (side, num)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LoopFusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()