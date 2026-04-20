#!/usr/bin/env python3
"""
Top Camera Loop State Node

Camera orientation: top half of image = Left (L) side of parachute
                   bottom half of image = Right (R) side of parachute

ID assignment:
  - Top half    (cy <= height/2), sorted left-to-right by x: L1, L2, L3 ...
  - Bottom half (cy  > height/2), sorted left-to-right by x: R1, R2, R3 ...

Architecture: inference runs in a background thread so the display
              is never blocked by slow CPU inference.

ROS 2 Publications:
  /top_cam/loop_states  (parachute_interfaces/LoopStateArray)
  /top_cam/image        (sensor_msgs/Image) - annotated camera feed
"""
from __future__ import annotations

import threading
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

try:
    from ultralytics import YOLO
except ImportError as exc:
    raise RuntimeError("ultralytics not found. Run: pip install ultralytics") from exc

from parachute_interfaces.msg import LoopState, LoopStateArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger, SetBool


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _to_float(x) -> float:
    return float(x.item()) if hasattr(x, "item") else float(x)


def _normalize_label(label: str) -> str:
    s = str(label).strip().lower().replace("-", "_").replace(" ", "_")
    if "not" in s:
        return "not"
    if "partial" in s:
        return "partial"
    if "fully" in s or "full" in s:
        return "fully"
    return "unknown"


def _state_color(state: str) -> Tuple[int, int, int]:
    return {
        "fully":   (0, 255, 0),
        "partial": (0, 255, 255),
        "not":     (0, 0, 255),
    }.get(state, (200, 200, 200))


def _ensure_landscape(img: np.ndarray) -> np.ndarray:
    h, w = img.shape[:2]
    if h > w:
        return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    return img


def _letterbox(img: np.ndarray, out_h: int, out_w: int, fill: int = 114) -> np.ndarray:
    h, w = img.shape[:2]
    scale = min(out_w / w, out_h / h)
    nw, nh = max(1, int(round(w * scale))), max(1, int(round(h * scale)))
    img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((out_h, out_w, 3), fill, dtype=img.dtype)
    left, top = (out_w - nw) // 2, (out_h - nh) // 2
    canvas[top:top + nh, left:left + nw] = img
    return canvas


def _assign_ids(
    detections: List[Tuple[int, int, int, int]],
    img_height: int,
) -> List[str]:
    """
    Camera top half  (cy <= img_height/2) → L group, sorted by x (left→right) → L1, L2 ...
    Camera bottom half (cy > img_height/2) → R group, sorted by x (left→right) → R1, R2 ...
    """
    mid = img_height / 2.0
    left_idx:  List[Tuple[float, int]] = []   # (center_x, original_index)
    right_idx: List[Tuple[float, int]] = []

    for i, (x1, y1, x2, y2) in enumerate(detections):
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        if cy <= mid:
            left_idx.append((cx, i))   # top half → L
        else:
            right_idx.append((cx, i))  # bottom half → R

    left_idx.sort(key=lambda t: t[0])   # left-to-right
    right_idx.sort(key=lambda t: t[0])  # left-to-right

    ids: List[str] = [""] * len(detections)
    for rank, (_, orig) in enumerate(left_idx, start=1):
        ids[orig] = f"L{rank}"
    for rank, (_, orig) in enumerate(right_idx, start=1):
        ids[orig] = f"R{rank}"

    return ids


# ---------------------------------------------------------------------------
# Inference worker (runs in background thread)
# ---------------------------------------------------------------------------

class _InferenceWorker:
    """
    Runs YOLO detect + classify in a dedicated thread.
    The ROS node reads `latest_result` without blocking.
    """

    def __init__(self, det_model, cls_model, device, conf, iou, cls_h, cls_w):
        self.det_model = det_model
        self.cls_model = cls_model
        self.device    = device
        self.conf      = conf
        self.iou       = iou
        self.cls_h     = cls_h
        self.cls_w     = cls_w

        self._lock         = threading.Lock()
        self._new_frame    = threading.Event()
        self._stop         = threading.Event()
        self._pending: Optional[np.ndarray] = None
        self._result_ready = threading.Event()

        # Shared output: (annotated_frame, loop_states)
        self.latest_result: Optional[Tuple[np.ndarray, List[LoopState]]] = None
        self.infer_fps: float = 0.0

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, frame: np.ndarray) -> None:
        with self._lock:
            self._pending = frame.copy()
            self.latest_result = None
        self._result_ready.clear()         
        self._new_frame.set()

    def stop(self) -> None:
        self._stop.set()
        self._new_frame.set()

    def _run(self) -> None:
        while not self._stop.is_set():
            self._new_frame.wait()
            self._new_frame.clear()
            if self._stop.is_set():
                break
            with self._lock:
                frame = self._pending
            if frame is None:
                continue
            t0 = time.time()
            try:
                annotated, states = self._infer(frame)
            except Exception as e:
                # Log but don't die — thread must survive to service future submits
                print(f'[WORKER] Inference error: {e}', flush=True)
                import traceback; traceback.print_exc()
                with self._lock:
                    self.latest_result = None
                self._result_ready.set()      # unblock waiter with None → handler sees timeout semantics
                continue
            dt = time.time() - t0
            if dt > 0:
                self.infer_fps = 0.8 * self.infer_fps + 0.2 * (1.0 / dt)
            with self._lock:
                self.latest_result = (annotated, states)
            self._result_ready.set()           # ← signal waiters

    def _infer(self, frame: np.ndarray):
        img_h, img_w = frame.shape[:2]
        annotated = frame.copy()
        loop_states: List[LoopState] = []

        det_results = self.det_model.predict(
            source=frame,
            task="detect",
            imgsz=640,
            device=self.device,
            conf=self.conf,
            iou=self.iou,
            verbose=False,
        )

        boxes_raw = det_results[0].boxes if det_results else None
        if boxes_raw is None or len(boxes_raw) == 0:
            return annotated, loop_states

        box_coords: List[Tuple[int, int, int, int]] = []
        box_confs:  List[float] = []

        for box in boxes_raw:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            x1 = max(0, min(x1, img_w - 1))
            y1 = max(0, min(y1, img_h - 1))
            x2 = max(0, min(x2, img_w))
            y2 = max(0, min(y2, img_h))
            if x2 > x1 and y2 > y1:
                box_coords.append((x1, y1, x2, y2))
                box_confs.append(_to_float(box.conf[0]) if box.conf is not None else 0.0)

        ids = _assign_ids(box_coords, img_h)

        crops = []
        for (x1, y1, x2, y2) in box_coords:
            crop = frame[y1:y2, x1:x2]
            crop = _ensure_landscape(crop)
            crop = _letterbox(crop, self.cls_h, self.cls_w)
            crops.append(crop)

        cls_results = []
        if crops:
            cls_results = self.cls_model.predict(
                source=crops,
                task="classify",
                imgsz=max(self.cls_h, self.cls_w),
                device=self.device,
                batch=max(1, len(crops)),
                verbose=False,
            )

        for i, ((x1, y1, x2, y2), loop_id) in enumerate(zip(box_coords, ids)):
            cls_r = cls_results[i] if i < len(cls_results) else None
            if cls_r is not None and cls_r.probs is not None:
                top1 = int(cls_r.probs.top1)
                raw_label = str(
                    cls_r.names.get(top1, top1)
                    if isinstance(cls_r.names, dict)
                    else cls_r.names[top1]
                )
                cls_conf = _to_float(cls_r.probs.top1conf)
            else:
                raw_label, cls_conf = "unknown", 0.0

            state = _normalize_label(raw_label)
            color = _state_color(state)

            ls = LoopState()
            ls.loop_id    = loop_id
            ls.state      = state
            ls.confidence = cls_conf
            ls.center_x   = float((x1 + x2) / 2.0 / img_w)
            ls.center_y   = float((y1 + y2) / 2.0 / img_h)
            loop_states.append(ls)

            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 1)
            ty = max(16, y1 - 4)
            cv2.putText(annotated, f"{loop_id}: {state}", (x1, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)

        return annotated, loop_states


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class TopCamLoopStateNode(Node):

    def __init__(self) -> None:
        super().__init__("top_cam_loop_state")

        self.declare_parameter("camera_index",  "/dev/video2")
        self.declare_parameter("det_weights",   "")
        self.declare_parameter("cls_weights",   "")
        self.declare_parameter("conf_threshold", 0.35)
        self.declare_parameter("iou_threshold",  0.45)
        self.declare_parameter("frame_rate",     30.0)   # display FPS
        self.declare_parameter("cls_h",          512)
        self.declare_parameter("cls_w",          256)
        self.declare_parameter("display",        True)
        self.declare_parameter("output_topic",   "/top_cam/loop_states")
        self.declare_parameter("image_topic",    "/top_cam/image")
        self.declare_parameter("publish_image",  True)
        self.declare_parameter("continuous_mode", True)  # False = only capture on service call

        # Camera is pointing down, camera Y is in line w/ world Y
        self.declare_parameter("cam_x", 0.32)       
        self.declare_parameter("cam_y", 0.0)        
        self.declare_parameter("cam_z", 0.54)       
        self.declare_parameter("fov_width", 0.65)   
        self.declare_parameter("fov_height", 0.50)  
        self.declare_parameter("table_z", 0.0)      
        self.declare_parameter("flip_x", False)     
        self.declare_parameter("flip_y", True)      

        cam_raw      = self.get_parameter("camera_index").value
        # Support both device path ("/dev/video2") and integer index
        try:
            cam_idx = int(cam_raw)
        except (ValueError, TypeError):
            # Extract integer from /dev/videoN path if possible
            s = str(cam_raw)
            import re
            m = re.match(r'/dev/video(\d+)', s)
            cam_idx = int(m.group(1)) if m else s
        det_weights  = self.get_parameter("det_weights").value
        cls_weights  = self.get_parameter("cls_weights").value
        self.conf    = self.get_parameter("conf_threshold").value
        self.iou     = self.get_parameter("iou_threshold").value
        frame_rate   = self.get_parameter("frame_rate").value
        cls_h        = self.get_parameter("cls_h").value
        cls_w        = self.get_parameter("cls_w").value
        self.display       = self.get_parameter("display").value
        out_topic          = self.get_parameter("output_topic").value
        image_topic        = self.get_parameter("image_topic").value
        self.publish_image = self.get_parameter("publish_image").value

        # Camera projection params
        self.cam_x = self.get_parameter("cam_x").value
        self.cam_y = self.get_parameter("cam_y").value
        self.cam_z = self.get_parameter("cam_z").value
        self.fov_width = self.get_parameter("fov_width").value
        self.fov_height = self.get_parameter("fov_height").value
        self.table_z = self.get_parameter("table_z").value
        self.flip_x = self.get_parameter("flip_x").value
        self.flip_y = self.get_parameter("flip_y").value

        try:
            import torch
            self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
            torch.set_num_threads(os.cpu_count())
        except Exception:
            self.device = "cpu"
        self.get_logger().info(f"Inference device: {self.device}")

        if not det_weights:
            raise RuntimeError("Parameter 'det_weights' must be set.")
        if not cls_weights:
            raise RuntimeError("Parameter 'cls_weights' must be set.")

        self.get_logger().info(f"Loading detection model:      {det_weights}")
        det_model = YOLO(str(det_weights))
        det_model.to(self.device)

        self.get_logger().info(f"Loading classification model: {cls_weights}")
        cls_model = YOLO(str(cls_weights))
        cls_model.to(self.device)

        self.cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera: {cam_idx}")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # GPU warmup — first inference is always slow
        if self.device != "cpu":
            self.get_logger().info("GPU warmup...")
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            det_model.predict(source=dummy, imgsz=640, device=self.device, verbose=False)
            cls_model.predict(source=dummy, imgsz=max(cls_h, cls_w), device=self.device, verbose=False)
            self.get_logger().info("GPU warmup done")

        # Background inference worker
        self._worker = _InferenceWorker(
            det_model, cls_model, self.device,
            self.conf, self.iou, cls_h, cls_w
        )

        self.pub = self.create_publisher(LoopStateArray, out_topic, 10)
        self.image_pub = self.create_publisher(Image, image_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/top_cam/loop_markers", 10)

        self._prev_states: Dict[str, str] = {}
        self._last_display: Optional[np.ndarray] = None
        self._fps_tick_prev: float = time.time()
        self._fps_value: float = 0.0

        # On-demand capture service (always available)
        self.capture_srv = self.create_service(
            Trigger, '/top_cam/capture', self._capture_callback)

        # Enable/disable service for dynamic control
        self.enable_srv = self.create_service(
            SetBool, '/top_cam/enable', self._enable_callback)

        self.continuous_mode = self.get_parameter("continuous_mode").value
        self._processing_enabled = self.continuous_mode  # Start enabled if continuous

        if self.display:
            cv2.namedWindow("Top Cam Loop State", cv2.WINDOW_NORMAL)

        # Timer always runs for camera keep-alive, but only processes when enabled
        period = 1.0 / max(frame_rate, 1.0)
        self.timer = self.create_timer(period, self._tick)

        mode_str = "CONTINUOUS" if self.continuous_mode else "ON-DEMAND"
        self.get_logger().info(
            f"TopCamLoopStateNode ready → {out_topic}  "
            f"({mode_str} mode, processing={'ON' if self._processing_enabled else 'OFF'})"
        )
        self.get_logger().info(
            f"  Services: /top_cam/capture (single shot), /top_cam/enable (on/off)"
        )

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Camera read failed")
            return

        # Only submit to inference worker if processing is enabled
        if not self._processing_enabled:
            # Just keep camera alive, don't process
            if self.display:
                cv2.imshow("Top Cam Loop State", frame)
                cv2.waitKey(1)
            return

        # Submit new frame to inference worker (non-blocking)
        self._worker.submit(frame)

        # Grab latest inference result (may be from a previous frame)
        with self._worker._lock:
            result = self._worker.latest_result

        if result is not None:
            annotated, loop_states = result

            # Compute 3D world positions for each loop
            for ls in loop_states:
                ls.world_position = self._compute_world_position(ls.center_x, ls.center_y)

            # Publish & terminal print on state change
            current = {ls.loop_id: ls.state for ls in loop_states}
            changed = current != self._prev_states

            msg = LoopStateArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"  # Now in world frame since we have 3D positions
            msg.loops = loop_states
            msg.changed = changed
            msg.total_count = len(loop_states)
            self.pub.publish(msg)

            # Publish RViz markers for visualization
            self._publish_markers(loop_states)

            if changed:
                self._prev_states = dict(current)
                self._print_states(loop_states)

            self._last_display = annotated
        else:
            # No inference result yet — show raw frame
            self._last_display = frame.copy()

        # FPS calculation
        now = time.time()
        dt = now - self._fps_tick_prev
        if dt > 0:
            self._fps_value = 0.9 * self._fps_value + 0.1 * (1.0 / dt)
        self._fps_tick_prev = now

        if self._last_display is not None:
            img_w = self._last_display.shape[1]
            # Display FPS (top-right)
            cv2.putText(self._last_display, f"Display: {self._fps_value:.1f} FPS",
                        (img_w - 180, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
            # Inference FPS (top-right, second line)
            cv2.putText(self._last_display, f"Infer:   {self._worker.infer_fps:.1f} FPS",
                        (img_w - 180, 38),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1, cv2.LINE_AA)

            # Publish annotated image to ROS2
            if self.publish_image:
                img_msg = self._cv2_to_imgmsg(self._last_display)
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "top_camera"
                self.image_pub.publish(img_msg)

            if self.display:
                cv2.imshow("Top Cam Loop State", self._last_display)
                if cv2.waitKey(1) & 0xFF == 27:
                    self.get_logger().info("ESC — shutting down.")
                    rclpy.shutdown()

    # ------------------------------------------------------------------
    def _cv2_to_imgmsg(self, cv_image: np.ndarray) -> Image:
        """Convert OpenCV image to ROS Image message (without cv_bridge)."""
        msg = Image()
        msg.height = cv_image.shape[0]
        msg.width = cv_image.shape[1]
        if len(cv_image.shape) == 3:
            msg.encoding = 'bgr8'
            msg.step = cv_image.shape[1] * 3
        else:
            msg.encoding = 'mono8'
            msg.step = cv_image.shape[1]
        msg.data = cv_image.tobytes()
        return msg

    # ------------------------------------------------------------------
    def _compute_world_position(self, center_x: float, center_y: float) -> Point:
        """
        Convert image coordinates to 3D world position, where cam position defined by params

        Args:
            center_x: Normalized x in image (0.0 = left, 1.0 = right)
            center_y: Normalized y in image (0.0 = top, 1.0 = bottom)

        Returns:
            Point with world x, y, z coordinates
        """
        # Convert from normalized [0,1] to centered [-0.5, 0.5]
        nx = center_x - 0.5
        ny = center_y - 0.5

        # Apply axis flips if needed
        if self.flip_x:
            nx = -nx
        if self.flip_y:
            ny = -ny

        # Project to world coordinates
        # Camera at (cam_x, cam_y, cam_z) looking down at table_z
        world_x = self.cam_x + nx * self.fov_width
        world_y = self.cam_y + ny * self.fov_height
        world_z = self.table_z

        return Point(x=world_x, y=world_y, z=world_z)

    # ------------------------------------------------------------------
    def _publish_markers(self, loop_states: List[LoopState]) -> None:
        """Publish RViz markers for loop visualization."""
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Color mapping for stow states
        colors = {
            "fully":   (0.0, 1.0, 0.0, 1.0),  # Green
            "partial": (1.0, 1.0, 0.0, 1.0),  # Yellow
            "not":     (0.5, 0.5, 0.5, 1.0),  # Gray
            "unknown": (1.0, 0.5, 0.5, 1.0),  # Orange?
        }

        for i, ls in enumerate(loop_states):
            # Sphere marker for loop position
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = "world"
            marker.ns = "top_cam_loops"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = ls.world_position
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.02  # 20mm diameter
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            r, g, b, a = colors.get(ls.state, (0.5, 0.5, 0.5, 1.0))
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = a
            marker_array.markers.append(marker)

            # Text marker for loop ID
            text_marker = Marker()
            text_marker.header.stamp = stamp
            text_marker.header.frame_id = "world"
            text_marker.ns = "top_cam_loop_labels"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = ls.world_position.x
            text_marker.pose.position.y = ls.world_position.y
            text_marker.pose.position.z = ls.world_position.z + 0.03  # Above sphere
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.015  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = ls.loop_id
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

    # ------------------------------------------------------------------
    def _print_states(self, loop_states: List[LoopState]) -> None:
        if not loop_states:
            print("[TopCam] No loops detected")
            return

        def sort_key(ls):
            side = 0 if ls.loop_id.startswith("L") else 1
            num  = int(ls.loop_id[1:]) if ls.loop_id[1:].isdigit() else 99
            return (side, num)

        lines = ["[TopCam] Loop states updated:"]
        row: List[str] = []
        for ls in sorted(loop_states, key=sort_key):
            row.append(f"  {ls.loop_id}: {ls.state:<8}")
            if len(row) == 4:
                lines.append("".join(row))
                row = []
        if row:
            lines.append("".join(row))
        print("\n".join(lines))

    # ------------------------------------------------------------------
    def _draw_status_panel(self, frame, loop_states, changed):
        if not loop_states:
            return

        def sort_key(ls):
            side = 0 if ls.loop_id.startswith("L") else 1
            num  = int(ls.loop_id[1:]) if ls.loop_id[1:].isdigit() else 99
            return (side, num)

        sorted_ls = sorted(loop_states, key=sort_key)

        # Build compact single-line text: "L1:fully L2:partial R1:not"
        parts = [f"{ls.loop_id}:{ls.state}" for ls in sorted_ls]
        text = "  ".join(parts)

        # Background bar at bottom
        h, w = frame.shape[:2]
        bar_h = 24
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, h - bar_h), (w, h), (30, 30, 30), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

        # Draw each part with its own color
        x_cursor = 8
        y_pos = h - 7
        for ls in sorted_ls:
            color = _state_color(ls.state)
            label = f"{ls.loop_id}:{ls.state}"
            cv2.putText(frame, label, (x_cursor, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
            x_cursor += len(label) * 11 + 12

    # ------------------------------------------------------------------
    def _enable_callback(self, request, response):
        """Enable or disable continuous processing."""
        self._processing_enabled = request.data
        state = "ENABLED" if request.data else "DISABLED"
        self.get_logger().info(f'[TOP_CAM] Processing {state}')
        response.success = True
        response.message = f'Processing {state}'
        return response

    # ------------------------------------------------------------------
    def _capture_callback(self, request, response):
        """
        Service callback for on-demand capture.
        Captures a single frame, runs inference, publishes result, and returns.
        """
        self.get_logger().info('[CAPTURE] Capturing frame...')

        # Read a fresh frame
        ret, frame = self.cap.read()
        if not ret:
            response.success = False
            response.message = 'Camera read failed'
            return response

        # Submit to worker and wait for result
        self._worker.submit(frame)
        self._worker.submit(frame)

        # Wait for inference to complete (with timeout)
        if not self._worker._result_ready.wait(timeout=10.0):
            response.success = False
            response.message = 'Inference timeout'
            return response

        with self._worker._lock:
            result = self._worker.latest_result

        if result is None:
            response.success = False
            response.message = 'Inference failed (see logs)'
            return response

        annotated, loop_states = result

        # Compute 3D world positions
        for ls in loop_states:
            ls.world_position = self._compute_world_position(ls.center_x, ls.center_y)

        # Publish the results
        msg = LoopStateArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.loops = loop_states
        msg.changed = True
        msg.total_count = len(loop_states)
        self.pub.publish(msg)

        # Publish markers
        self._publish_markers(loop_states)

        # Publish image if enabled
        if self.publish_image:
            self.image_pub.publish(self._cv2_to_imgmsg(annotated))

        self._print_states(loop_states)

        response.success = True
        response.message = f'Captured {len(loop_states)} loops'
        self.get_logger().info(f'[CAPTURE] {response.message}')
        return response

    # ------------------------------------------------------------------
    def destroy_node(self):
        self._worker.stop()
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        if self.display and self.continuous_mode:
            cv2.destroyAllWindows()
        return super().destroy_node()


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TopCamLoopStateNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
