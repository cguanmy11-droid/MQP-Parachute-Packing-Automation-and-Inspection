"""
camera_widget.py
----------------
PyQt5 widget for displaying camera feeds with loop detection overlay.

Displays ROS2 Image messages and overlays detected loop positions.
"""

import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QSizePolicy
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap, QFont, QPainter, QPen, QColor


class CameraWidget(QWidget):
    """Widget that displays a camera feed with optional loop detection overlay."""

    def __init__(self, title: str = "Camera Feed", parent=None):
        super().__init__(parent)
        self._title = title
        self._current_image = None
        self._detected_loops = []  # List of (x, y, confidence, loop_id) in pixel coords
        self._target_loop_id = -1

        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)

        # Title label
        title_label = QLabel(self._title)
        title_label.setFont(QFont('Courier New', 10, QFont.Bold))
        title_label.setStyleSheet('color: #00b4d8;')
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)

        # Image display label
        self._image_label = QLabel()
        self._image_label.setAlignment(Qt.AlignCenter)
        self._image_label.setMinimumSize(320, 240)
        self._image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._image_label.setStyleSheet('''
            background-color: #0d0d1a;
            border: 1px solid #0f3460;
            border-radius: 4px;
        ''')
        layout.addWidget(self._image_label)

        # Status label (shows detection count)
        self._status_label = QLabel("No feed")
        self._status_label.setFont(QFont('Courier New', 9))
        self._status_label.setStyleSheet('color: #666;')
        self._status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self._status_label)

    @pyqtSlot(object)
    def update_image(self, image_data: dict):
        """
        Update the displayed image.

        Args:
            image_data: dict with keys:
                - 'data': bytes or numpy array
                - 'width': int
                - 'height': int
                - 'encoding': str ('bgr8', 'rgb8', 'mono8')
        """
        try:
            width = image_data['width']
            height = image_data['height']
            encoding = image_data.get('encoding', 'bgr8')
            data = image_data['data']

            # Convert bytes to numpy array if needed
            if isinstance(data, bytes):
                if encoding in ('bgr8', 'rgb8'):
                    np_img = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
                else:  # mono8
                    np_img = np.frombuffer(data, dtype=np.uint8).reshape((height, width))
            else:
                np_img = data

            # Convert BGR to RGB for Qt
            if encoding == 'bgr8':
                np_img = np_img[:, :, ::-1].copy()  # BGR to RGB

            # Create QImage
            if len(np_img.shape) == 3:
                qimg = QImage(np_img.data, width, height, width * 3, QImage.Format_RGB888)
            else:
                qimg = QImage(np_img.data, width, height, width, QImage.Format_Grayscale8)

            # Scale to fit widget while maintaining aspect ratio
            pixmap = QPixmap.fromImage(qimg)
            scaled = pixmap.scaled(
                self._image_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )

            # Draw loop overlays if we have detections
            if self._detected_loops:
                scaled = self._draw_loop_overlay(scaled, width, height)

            self._image_label.setPixmap(scaled)
            self._current_image = np_img
            self._status_label.setText(f"{width}x{height} | {len(self._detected_loops)} loops")

        except Exception as e:
            self._status_label.setText(f"Error: {str(e)[:30]}")

    def _draw_loop_overlay(self, pixmap: QPixmap, orig_width: int, orig_height: int) -> QPixmap:
        """Draw circle overlays on detected loops."""
        # Calculate scale factors
        scale_x = pixmap.width() / orig_width
        scale_y = pixmap.height() / orig_height

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)

        for loop in self._detected_loops:
            x, y, confidence, loop_id = loop

            # Scale coordinates
            sx = int(x * scale_x)
            sy = int(y * scale_y)
            radius = int(15 * min(scale_x, scale_y))

            # Color: green for normal, red for target
            if loop_id == self._target_loop_id:
                pen = QPen(QColor(255, 50, 50), 3)
            else:
                pen = QPen(QColor(50, 255, 50), 2)

            painter.setPen(pen)
            painter.drawEllipse(sx - radius, sy - radius, radius * 2, radius * 2)

            # Draw loop ID label
            painter.setPen(QPen(QColor(255, 255, 255), 1))
            painter.setFont(QFont('Courier New', 8, QFont.Bold))
            painter.drawText(sx + radius + 2, sy + 4, f"L{loop_id}")

        painter.end()
        return pixmap

    @pyqtSlot(list)
    def update_loop_detections(self, loops: list):
        """
        Update the detected loop positions for overlay.

        Args:
            loops: List of dicts with 'pixel_x', 'pixel_y', 'confidence', 'loop_id'
        """
        self._detected_loops = [
            (l.get('pixel_x', 0), l.get('pixel_y', 0),
             l.get('confidence', 0), l.get('loop_id', -1))
            for l in loops
        ]

    @pyqtSlot(int)
    def set_target_loop(self, loop_id: int):
        """Set the target loop ID to highlight in red."""
        self._target_loop_id = loop_id

    def clear(self):
        """Clear the display."""
        self._image_label.clear()
        self._current_image = None
        self._detected_loops = []
        self._status_label.setText("No feed")
