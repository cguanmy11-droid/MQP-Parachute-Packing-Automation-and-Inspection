"""
state_machine_widget.py
Matches stow_transitions.yaml exactly: IDLE, HOMING, AT_LOOP, INSERT,
HANDOFF, RETRACT, RELEASE, COMPLETE, ERROR
"""

import math
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt, QRectF, QPointF
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont, QPainterPath

# (display_name, x, y) normalized. Main flow down center-left, ERROR right.
STATES = {
    'IDLE':     ('IDLE',     0.38, 0.05),
    'HOMING':   ('HOMING',   0.38, 0.17),
    'AT_LOOP':  ('AT_LOOP',  0.38, 0.30),
    'INSERT':   ('INSERT',   0.38, 0.43),
    'HANDOFF':  ('HANDOFF',  0.38, 0.56),
    'RETRACT':  ('RETRACT',  0.38, 0.68),
    'RELEASE':  ('RELEASE',  0.38, 0.80),
    'COMPLETE': ('COMPLETE', 0.38, 0.92),
    'ERROR':    ('ERROR',    0.82, 0.35),
}

# (from, to, routing)  routing: 'straight' | 'left'
TRANSITIONS = [
    ('IDLE',     'HOMING',   'straight'),
    ('HOMING',   'AT_LOOP',  'straight'),
    ('AT_LOOP',  'INSERT',   'straight'),
    ('INSERT',   'HANDOFF',  'straight'),
    ('HANDOFF',  'RETRACT',  'straight'),
    ('RETRACT',  'RELEASE',  'straight'),
    ('RELEASE',  'COMPLETE', 'straight'),
    ('RELEASE',  'AT_LOOP',  'left'),
    ('COMPLETE', 'IDLE',     'left'),
    ('HOMING',   'ERROR',    'straight'),
    ('AT_LOOP',  'ERROR',    'straight'),
    ('INSERT',   'ERROR',    'straight'),
    ('HANDOFF',  'ERROR',    'straight'),
    ('RETRACT',  'ERROR',    'straight'),
    ('RELEASE',  'ERROR',    'straight'),
    ('ERROR',    'IDLE',     'straight'),
    ('ERROR',    'AT_LOOP',  'straight'),
]

COLOR_BG        = QColor('#1a1a2e')
COLOR_STATE     = QColor('#16213e')
COLOR_BORDER    = QColor('#0f3460')
COLOR_ACTIVE    = QColor('#e94560')
COLOR_ACTIVE_BG = QColor('#3d0016')
COLOR_DONE      = QColor('#00b4d8')
COLOR_ERROR_BG  = QColor('#1a0e00')
COLOR_ERROR_BDR = QColor('#ff6b35')
COLOR_TEXT      = QColor('#eaeaea')
COLOR_ARROW     = QColor('#2a3a4a')
COLOR_ARROW_ACT = QColor('#e94560')

NODE_W = 100
NODE_H = 34


class StateMachineWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._active_state = 'IDLE'
        self._completed_states = set()
        self.setMinimumSize(300, 500)
        self.setStyleSheet(f'background-color: {COLOR_BG.name()};')

    def set_state(self, state_name: str):
        if state_name != self._active_state:
            self._completed_states.add(self._active_state)
            self._active_state = state_name
            if state_name in ('IDLE', 'COMPLETE'):
                self._completed_states.clear()
            self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, COLOR_BG)
        positions = {k: QPointF(nx * w, ny * h) for k, (_, nx, ny) in STATES.items()}
        self._draw_edges(p, positions, w, h)
        for key, (label, nx, ny) in STATES.items():
            cx, cy = positions[key].x(), positions[key].y()
            self._draw_node(p, QRectF(cx - NODE_W/2, cy - NODE_H/2, NODE_W, NODE_H), label, key)

    def _draw_node(self, p, rect, label, key):
        is_active = key == self._active_state
        is_done   = key in self._completed_states
        is_error  = key == 'ERROR'

        if is_active:
            bg, border, bw = COLOR_ACTIVE_BG, COLOR_ACTIVE, 2.5
        elif is_done:
            bg, border, bw = QColor('#0d2137'), COLOR_DONE, 1.5
        elif is_error:
            bg, border, bw = COLOR_ERROR_BG, COLOR_ERROR_BDR, 1.5
        else:
            bg, border, bw = COLOR_STATE, COLOR_BORDER, 1.0

        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(0, 0, 0, 50)))
        p.drawRoundedRect(rect.translated(2, 2), 6, 6)
        p.setBrush(QBrush(bg))
        p.setPen(QPen(border, bw))
        p.drawRoundedRect(rect, 6, 6)

        if is_active:
            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(COLOR_ACTIVE, 1, Qt.DashLine))
            p.drawRoundedRect(rect.adjusted(-4, -4, 4, 4), 9, 9)

        p.setFont(QFont('Courier New', 8, QFont.Bold if is_active else QFont.Normal))
        p.setPen(QPen(COLOR_ACTIVE if is_active else COLOR_TEXT))
        p.drawText(rect, Qt.AlignCenter, label)

    def _edge_point(self, center, toward):
        dx = toward.x() - center.x()
        dy = toward.y() - center.y()
        dist = math.sqrt(dx*dx + dy*dy) or 1
        ux, uy = dx/dist, dy/dist
        if abs(ux) * NODE_H > abs(uy) * NODE_W:
            t = (NODE_W/2) / abs(ux) if ux else 1e9
        else:
            t = (NODE_H/2) / abs(uy) if uy else 1e9
        return QPointF(center.x() + ux*t, center.y() + uy*t)

    def _arrowhead(self, p, src, dst, color):
        dx = dst.x() - src.x()
        dy = dst.y() - src.y()
        dist = math.sqrt(dx*dx + dy*dy) or 1
        ux, uy = dx/dist, dy/dist
        sz = 6
        l = QPointF(dst.x() - ux*sz + uy*sz*0.45, dst.y() - uy*sz - ux*sz*0.45)
        r = QPointF(dst.x() - ux*sz - uy*sz*0.45, dst.y() - uy*sz + ux*sz*0.45)
        path = QPainterPath()
        path.moveTo(dst); path.lineTo(l); path.lineTo(r); path.closeSubpath()
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(color))
        p.drawPath(path)

    def _draw_edges(self, p, positions, w, h):
        for (src, dst, routing) in TRANSITIONS:
            if src not in positions or dst not in positions:
                continue
            sp, ep = positions[src], positions[dst]
            active = src == self._active_state or dst == self._active_state
            color  = COLOR_ARROW_ACT if active else COLOR_ARROW
            lw     = 1.5 if active else 0.8
            p.setBrush(QBrush(color))

            if routing == 'left':
                sx = sp.x() - NODE_W/2 - 18
                ex = ep.x() - NODE_W/2
                path = QPainterPath()
                path.moveTo(sp.x() - NODE_W/2, sp.y())
                path.lineTo(sx, sp.y())
                path.lineTo(sx, ep.y())
                path.lineTo(ex, ep.y())
                p.setBrush(Qt.NoBrush)
                p.setPen(QPen(color, lw))
                p.drawPath(path)
                self._arrowhead(p, QPointF(sx, ep.y()), QPointF(ex, ep.y()), color)
            else:
                start = self._edge_point(sp, ep)
                end   = self._edge_point(ep, sp)
                p.setPen(QPen(color, lw))
                p.drawLine(start, end)
                self._arrowhead(p, start, end, color)