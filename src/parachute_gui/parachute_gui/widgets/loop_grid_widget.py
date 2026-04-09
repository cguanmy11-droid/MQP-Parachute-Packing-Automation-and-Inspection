"""
loop_grid_widget.py
-------------------
Central loop visualization panel for the operator GUI.

Shows a top-down grid of all loops (L1-L9, R1-R9) with:
  - Color-coded stow states (green=fully, yellow=partial, gray=not)
  - Camera view indicator that slides between L/R sides
  - 3D perspective tilt for visual depth
  - Side camera feeds on left/right with toggle

Subscribes to:
  /fused_loop_states  (LoopStateArray)
  /top_cam/image      (Image)
  /side_arm_left/yolo/image   (Image) — if available
  /side_arm_right/yolo/image  (Image) — if available
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import numpy as np
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QFrame, QSizePolicy,
)
from PyQt5.QtCore import Qt, QTimer, QRectF, QPointF, pyqtSignal, QPropertyAnimation, QEasingCurve
from PyQt5.QtGui import (
    QPainter, QPen, QBrush, QColor, QFont, QLinearGradient,
    QImage, QPixmap, QTransform, QPainterPath,
)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

STATE_COLORS = {
    'fully':   QColor(16, 185, 129),    # green
    'partial': QColor(245, 158, 11),     # amber
    'not':     QColor(107, 114, 128),    # gray
    'unknown': QColor(55, 65, 81),       # dark gray
}

STATE_GLOW = {
    'fully':   QColor(16, 185, 129, 100),
    'partial': QColor(245, 158, 11, 100),
    'not':     QColor(107, 114, 128, 50),
    'unknown': QColor(55, 65, 81, 50),
}

BG_DARK = QColor(13, 13, 20)
BG_PANEL = QColor(15, 23, 42)
BORDER_DIM = QColor(255, 255, 255, 15)
BORDER_ACTIVE = QColor(59, 130, 246, 128)
BLUE_ACCENT = QColor(59, 130, 246)
BLUE_LIGHT = QColor(147, 197, 253)
TEXT_DIM = QColor(107, 114, 128)
TEXT_LIGHT = QColor(209, 213, 219)


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

class LoopInfo:
    """Display state for a single loop."""
    __slots__ = ('loop_id', 'side', 'index', 'state', 'confidence', 'cx', 'cy')

    def __init__(self, loop_id: str = '', side: str = 'L', index: int = 0,
                 state: str = 'unknown', confidence: float = 0.0,
                 cx: float = 0.0, cy: float = 0.0):
        self.loop_id = loop_id
        self.side = side
        self.index = index
        self.state = state
        self.confidence = confidence
        self.cx = cx
        self.cy = cy


# ---------------------------------------------------------------------------
# Camera feed display
# ---------------------------------------------------------------------------

class CameraFeedWidget(QWidget):
    """Displays a camera feed image with overlay label."""
    clicked = pyqtSignal()

    def __init__(self, label: str, parent=None):
        super().__init__(parent)
        self._label = label
        self._pixmap: Optional[QPixmap] = None
        self._active = False
        self._fps = 0.0
        self.setMinimumSize(200, 150)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setCursor(Qt.PointingHandCursor)

    def set_active(self, active: bool):
        self._active = active
        self.update()

    def set_fps(self, fps: float):
        self._fps = fps

    def update_image(self, img: np.ndarray):
        """Update from OpenCV BGR image."""
        if img is None:
            return
        h, w = img.shape[:2]
        if len(img.shape) == 3:
            bytes_per_line = w * 3
            qimg = QImage(img.data, w, h, bytes_per_line, QImage.Format_BGR888)
        else:
            qimg = QImage(img.data, w, h, w, QImage.Format_Grayscale8)
        self._pixmap = QPixmap.fromImage(qimg)
        self.update()

    def update_from_ros_image(self, msg):
        """Update from sensor_msgs/Image."""
        try:
            if msg.encoding == 'bgr8':
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif msg.encoding == 'rgb8':
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                arr = arr[:, :, ::-1].copy()  # RGB -> BGR
            elif msg.encoding == 'mono8':
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            else:
                return
            self.update_image(arr)
        except Exception:
            pass

    def mousePressEvent(self, event):
        self.clicked.emit()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        # Background
        p.fillRect(0, 0, w, h, QColor(8, 8, 16))

        # Border
        border_color = BORDER_ACTIVE if self._active else BORDER_DIM
        p.setPen(QPen(border_color, 1))
        p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(0, 0, w - 1, h - 1, 8, 8)

        # Image
        if self._pixmap:
            scaled = self._pixmap.scaled(w - 4, h - 4, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            x_off = (w - scaled.width()) // 2
            y_off = (h - scaled.height()) // 2
            opacity = 1.0 if self._active else 0.35
            p.setOpacity(opacity)
            p.drawPixmap(x_off, y_off, scaled)
            p.setOpacity(1.0)
        else:
            # No image - draw placeholder
            p.setOpacity(0.3 if not self._active else 0.6)
            p.setPen(QPen(TEXT_DIM, 1))
            p.setFont(QFont('Courier New', 9))
            p.drawText(QRectF(0, 0, w, h), Qt.AlignCenter, 'NO FEED')
            p.setOpacity(1.0)

        # Label overlay
        dot_color = QColor(16, 185, 129) if self._active else TEXT_DIM
        p.setBrush(QBrush(dot_color))
        p.setPen(Qt.NoPen)
        p.drawEllipse(QPointF(14, 14), 3, 3)

        label_color = TEXT_LIGHT if self._active else TEXT_DIM
        p.setPen(QPen(label_color))
        p.setFont(QFont('Courier New', 8, QFont.Bold))
        p.drawText(22, 17, self._label)

        # FPS
        p.setPen(QPen(TEXT_DIM))
        p.setFont(QFont('Courier New', 7))
        fps_text = f'{self._fps:.0f} FPS' if self._active and self._fps > 0 else '---'
        p.drawText(w - 55, h - 8, fps_text)

        p.end()


# ---------------------------------------------------------------------------
# Loop grid (the center 3D-perspective view)
# ---------------------------------------------------------------------------

class LoopGridWidget(QWidget):
    """
    Renders all loops in a 2-row grid (L row, R row) with perspective tilt,
    a sliding camera indicator, and side highlighting.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._loops: Dict[str, LoopInfo] = {}
        self._active_side: str = 'right'
        self._camera_y: float = 0.68  # animated position of camera indicator
        self.setMinimumSize(400, 280)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Initialize default loops
        for side in ('L', 'R'):
            for i in range(1, 10):
                lid = f'{side}{i}'
                self._loops[lid] = LoopInfo(
                    loop_id=lid, side=side, index=i,
                    cx=0.06 + (i - 1) * 0.105,
                    cy=0.32 if side == 'L' else 0.68,
                )

        # Animation timer for smooth camera indicator movement
        self._anim_timer = QTimer(self)
        self._anim_timer.timeout.connect(self._animate_camera)
        self._anim_timer.start(16)  # ~60fps
        self._camera_target_y = 0.68

    def set_active_side(self, side: str):
        """'left' or 'right'."""
        self._active_side = side
        self._camera_target_y = 0.25 if side == 'left' else 0.72
        self.update()

    def update_loops(self, loop_states: list):
        """
        Update from fused loop state data.
        Each item should have: loop_id, state, confidence, center_x, center_y
        """
        for ls in loop_states:
            lid = ls.loop_id if hasattr(ls, 'loop_id') else ls.get('loop_id', '')
            if lid in self._loops:
                info = self._loops[lid]
                info.state = ls.state if hasattr(ls, 'state') else ls.get('state', 'unknown')
                info.confidence = ls.confidence if hasattr(ls, 'confidence') else ls.get('confidence', 0.0)
        self.update()

    def _animate_camera(self):
        diff = self._camera_target_y - self._camera_y
        if abs(diff) > 0.002:
            self._camera_y += diff * 0.12
            self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.TextAntialiasing)
        w, h = self.width(), self.height()

        # Background
        grad = QLinearGradient(0, 0, 0, h)
        grad.setColorAt(0, QColor(15, 23, 42, 200))
        grad.setColorAt(1, QColor(15, 23, 42, 80))
        p.fillRect(0, 0, w, h, QBrush(grad))

        # Border
        p.setPen(QPen(BORDER_DIM, 1))
        p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(0, 0, w - 1, h - 1, 12, 12)

        # --- Perspective grid ---
        p.save()
        p.setOpacity(0.1)
        grid_pen = QPen(BLUE_ACCENT, 0.5)
        p.setPen(grid_pen)
        for i in range(11):
            frac = i / 10.0
            # Horizontal lines with slight perspective convergence
            y = int(h * frac)
            x_squeeze = int(w * 0.02 * (0.5 - abs(frac - 0.5)))
            p.drawLine(x_squeeze, y, w - x_squeeze, y)
            # Vertical lines
            x = int(w * frac)
            p.drawLine(x, 0, x, h)
        p.restore()

        # --- Active side highlight band ---
        p.save()
        if self._active_side == 'left':
            highlight_rect = QRectF(0, 0, w, h * 0.5)
        else:
            highlight_rect = QRectF(0, h * 0.5, w, h * 0.5)
        highlight_grad = QLinearGradient(0, highlight_rect.top(), w, highlight_rect.top())
        highlight_grad.setColorAt(0, QColor(59, 130, 246, 15))
        highlight_grad.setColorAt(1, QColor(59, 130, 246, 0))
        p.fillRect(highlight_rect, QBrush(highlight_grad))
        # Left accent bar
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(59, 130, 246, 60)))
        p.drawRect(QRectF(0, highlight_rect.top(), 2, highlight_rect.height()))
        p.restore()

        # --- Divider line ---
        p.save()
        p.setOpacity(0.2)
        div_grad = QLinearGradient(w * 0.06, 0, w * 0.94, 0)
        div_grad.setColorAt(0, QColor(0, 0, 0, 0))
        div_grad.setColorAt(0.5, BLUE_ACCENT)
        div_grad.setColorAt(1, QColor(0, 0, 0, 0))
        p.setPen(QPen(QBrush(div_grad), 1))
        mid_y = h * 0.5
        p.drawLine(QPointF(w * 0.06, mid_y), QPointF(w * 0.94, mid_y))
        p.restore()

        # --- Side labels ---
        p.setFont(QFont('Courier New', 9, QFont.Bold))
        left_active = self._active_side == 'left'
        right_active = self._active_side == 'right'

        p.setPen(QPen(BLUE_ACCENT if left_active else QColor(96, 165, 250, 60)))
        p.drawText(10, int(h * 0.28), 'LEFT ▸')
        p.setPen(QPen(BLUE_ACCENT if right_active else QColor(96, 165, 250, 60)))
        p.drawText(10, int(h * 0.73), 'RIGHT ▸')

        # --- Loop dots ---
        margin_l = 55
        margin_r = 65
        margin_t = 30
        margin_b = 30
        draw_w = w - margin_l - margin_r
        draw_h = h - margin_t - margin_b

        for lid, info in self._loops.items():
            is_highlighted = (
                (self._active_side == 'left' and info.side == 'L') or
                (self._active_side == 'right' and info.side == 'R')
            )
            self._draw_loop_dot(p, info, is_highlighted,
                                margin_l, margin_t, draw_w, draw_h)

        # --- Camera indicator ---
        self._draw_camera_indicator(p, w, h)

        # --- Legend ---
        self._draw_legend(p, w, h)

        p.end()

    def _draw_loop_dot(self, p: QPainter, info: LoopInfo, highlighted: bool,
                       ox: int, oy: int, dw: int, dh: int):
        x = ox + info.cx * dw
        y = oy + info.cy * dh
        radius = 14 if highlighted else 10
        color = STATE_COLORS.get(info.state, STATE_COLORS['unknown'])
        glow = STATE_GLOW.get(info.state, STATE_GLOW['unknown'])

        p.save()
        opacity = 1.0 if highlighted else 0.4
        p.setOpacity(opacity)

        # Glow
        if highlighted:
            glow_brush = QBrush(glow)
            p.setPen(Qt.NoPen)
            p.setBrush(glow_brush)
            p.drawEllipse(QPointF(x, y), radius + 6, radius + 6)

        # Dot
        border = QColor(255, 255, 255, 200) if highlighted else QColor(255, 255, 255, 40)
        p.setPen(QPen(border, 1.5))
        p.setBrush(QBrush(color))
        p.drawEllipse(QPointF(x, y), radius, radius)

        # Label
        p.setPen(QPen(QColor(255, 255, 255, 230 if highlighted else 150)))
        font_size = 8 if highlighted else 7
        p.setFont(QFont('Courier New', font_size, QFont.Bold))
        p.drawText(QRectF(x - radius, y - radius, radius * 2, radius * 2),
                   Qt.AlignCenter, info.loop_id)

        p.restore()

    def _draw_camera_indicator(self, p: QPainter, w: int, h: int):
        """Draw the sliding camera icon near the active row."""
        cam_x = w - 50
        cam_y = h * self._camera_y

        p.save()
        p.setOpacity(0.85)

        # Camera body
        body = QRectF(cam_x - 12, cam_y - 8, 24, 16)
        cam_grad = QLinearGradient(body.topLeft(), body.bottomRight())
        cam_grad.setColorAt(0, QColor(59, 130, 246, 180))
        cam_grad.setColorAt(1, QColor(139, 92, 246, 130))
        p.setPen(QPen(BLUE_LIGHT, 0.8))
        p.setBrush(QBrush(cam_grad))
        p.drawRoundedRect(body, 3, 3)

        # Lens
        p.setPen(QPen(BLUE_LIGHT, 1.2))
        p.setBrush(QBrush(QColor(147, 197, 253, 100)))
        p.drawEllipse(QPointF(cam_x, cam_y), 4, 4)
        p.setBrush(QBrush(QColor(147, 197, 253, 160)))
        p.drawEllipse(QPointF(cam_x, cam_y), 2, 2)

        # FOV lines pointing down
        p.setPen(QPen(QColor(59, 130, 246, 80), 0.5, Qt.DashLine))
        fov_bottom = cam_y + 18
        p.drawLine(QPointF(cam_x - 10, cam_y + 8), QPointF(cam_x - 18, fov_bottom))
        p.drawLine(QPointF(cam_x + 10, cam_y + 8), QPointF(cam_x + 18, fov_bottom))

        # Label
        p.setPen(QPen(BLUE_LIGHT))
        p.setFont(QFont('Courier New', 6))
        side_label = 'L CAM' if self._active_side == 'left' else 'R CAM'
        p.drawText(QRectF(cam_x - 15, cam_y - 18, 30, 10), Qt.AlignCenter, side_label)

        p.restore()

    def _draw_legend(self, p: QPainter, w: int, h: int):
        p.save()
        p.setFont(QFont('Courier New', 7))
        legend_items = [
            ('fully', STATE_COLORS['fully']),
            ('partial', STATE_COLORS['partial']),
            ('not', STATE_COLORS['not']),
        ]
        total_w = len(legend_items) * 70
        start_x = (w - total_w) // 2
        y = h - 14

        for i, (label, color) in enumerate(legend_items):
            x = start_x + i * 70
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(color))
            p.drawEllipse(QPointF(x, y), 4, 4)
            p.setPen(QPen(TEXT_DIM))
            p.drawText(x + 8, y + 4, label)

        p.restore()


# ---------------------------------------------------------------------------
# Composite panel: cameras + grid + toggle
# ---------------------------------------------------------------------------

class LoopPerceptionPanel(QWidget):
    """
    Full perception panel combining:
      - Top camera feed (above)
      - Left cam | Loop grid | Right cam (middle)
      - L/R toggle buttons (below)
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._active_side = 'right'
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        # Top camera feed
        self.top_cam = CameraFeedWidget('TOP CAMERA (OVERVIEW)')
        self.top_cam.set_active(True)
        self.top_cam.setFixedHeight(140)
        layout.addWidget(self.top_cam)

        # Middle row: left cam | grid | right cam
        mid = QHBoxLayout()
        mid.setSpacing(8)

        self.left_cam = CameraFeedWidget('LEFT SIDE CAM')
        self.left_cam.clicked.connect(lambda: self.set_active_side('left'))
        mid.addWidget(self.left_cam, stretch=1)

        self.loop_grid = LoopGridWidget()
        mid.addWidget(self.loop_grid, stretch=2)

        self.right_cam = CameraFeedWidget('RIGHT SIDE CAM')
        self.right_cam.clicked.connect(lambda: self.set_active_side('right'))
        mid.addWidget(self.right_cam, stretch=1)

        layout.addLayout(mid, stretch=1)

        # Toggle buttons
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(8)
        btn_layout.addStretch()

        self._left_btn = QPushButton('◀  LEFT CAM')
        self._left_btn.setFixedSize(140, 32)
        self._left_btn.setCursor(Qt.PointingHandCursor)
        self._left_btn.clicked.connect(lambda: self.set_active_side('left'))
        btn_layout.addWidget(self._left_btn)

        self._right_btn = QPushButton('RIGHT CAM  ▶')
        self._right_btn.setFixedSize(140, 32)
        self._right_btn.setCursor(Qt.PointingHandCursor)
        self._right_btn.clicked.connect(lambda: self.set_active_side('right'))
        btn_layout.addWidget(self._right_btn)

        btn_layout.addStretch()
        layout.addLayout(btn_layout)

        # Initial state
        self.set_active_side('right')

    def set_active_side(self, side: str):
        self._active_side = side
        self.left_cam.set_active(side == 'left')
        self.right_cam.set_active(side == 'right')
        self.loop_grid.set_active_side(side)

        # Button styling
        active_style = '''
            QPushButton {
                background: rgba(59, 130, 246, 0.15);
                border: 1px solid #3B82F6;
                color: #93C5FD;
                border-radius: 6px;
                font-family: Courier New;
                font-size: 10px;
                font-weight: bold;
                letter-spacing: 1px;
            }
        '''
        inactive_style = '''
            QPushButton {
                background: rgba(255, 255, 255, 0.03);
                border: 1px solid rgba(255, 255, 255, 0.1);
                color: #6B7280;
                border-radius: 6px;
                font-family: Courier New;
                font-size: 10px;
                font-weight: bold;
                letter-spacing: 1px;
            }
            QPushButton:hover {
                border: 1px solid rgba(59, 130, 246, 0.3);
                color: #93C5FD;
            }
        '''
        self._left_btn.setStyleSheet(active_style if side == 'left' else inactive_style)
        self._right_btn.setStyleSheet(active_style if side == 'right' else inactive_style)

    def update_loops(self, loop_states):
        """Pass fused loop state data to the grid."""
        self.loop_grid.update_loops(loop_states)

    def update_top_cam_image(self, msg):
        """Update top camera from ROS Image message."""
        self.top_cam.update_from_ros_image(msg)

    def update_left_cam_image(self, msg):
        """Update left side camera from ROS Image message."""
        self.left_cam.update_from_ros_image(msg)

    def update_right_cam_image(self, msg):
        """Update right side camera from ROS Image message."""
        self.right_cam.update_from_ros_image(msg)