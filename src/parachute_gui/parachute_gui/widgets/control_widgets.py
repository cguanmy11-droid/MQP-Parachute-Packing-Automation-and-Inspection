"""
control_widgets.py
Loop selector (read-only display + target highlight) and arm control panel.
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QFrame, QScrollArea
)
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QFont


class LoopSelectorWidget(QWidget):
    """
    Shows all detected loops with confidence and position.
    Highlights whichever loop the coordinator has selected as target.
    Loop override is future work — display only for now.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._loops = []
        self._target_id = None
        self._buttons = {}   # loop_id -> QPushButton
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        header = QLabel('Detected Loops')
        header.setFont(QFont('Courier New', 11, QFont.Bold))
        header.setStyleSheet('color: #00b4d8;')
        layout.addWidget(header)

        self._status_label = QLabel('Waiting for perception...')
        self._status_label.setStyleSheet(
            'color: #888; font-size: 10px; font-family: Courier New;')
        layout.addWidget(self._status_label)

        # Target indicator
        self._target_label = QLabel('')
        self._target_label.setStyleSheet(
            'color: #69f000; font-size: 10px; font-family: Courier New;')
        layout.addWidget(self._target_label)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet('QScrollArea { border: none; background: transparent; }')
        self._btn_container = QWidget()
        self._btn_layout = QVBoxLayout(self._btn_container)
        self._btn_layout.setSpacing(4)
        self._btn_layout.addStretch()
        scroll.setWidget(self._btn_container)
        layout.addWidget(scroll)

        note = QLabel('Target selected automatically by coordinator.\nLoop override: future work.')
        note.setStyleSheet('color: #444; font-size: 9px; font-family: Courier New;')
        note.setWordWrap(True)
        layout.addWidget(note)

    def update_loops(self, loops: list):
        self._loops = loops
        self._buttons.clear()

        # Clear old buttons
        while self._btn_layout.count() > 1:
            item = self._btn_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        if not loops:
            self._status_label.setText('No loops detected')
            return

        self._status_label.setText(f'{len(loops)} loop(s) detected')

        for loop in loops:
            lid = loop['id']
            conf = loop['confidence']
            x, y = loop['x'], loop['y']
            btn = QPushButton(
                f"Loop {lid}   {conf:.0%} conf   ({x:.3f}, {y:.3f})"
            )
            btn.setEnabled(False)   # display only
            btn.setStyleSheet(self._btn_style(lid == self._target_id))
            self._buttons[lid] = btn
            # Insert before the stretch
            self._btn_layout.insertWidget(self._btn_layout.count() - 1, btn)

    def set_target_loop(self, loop_id: int):
        """Highlight whichever loop the coordinator is targeting."""
        old = self._target_id
        self._target_id = loop_id
        self._target_label.setText(f'▶ Targeting: Loop {loop_id}')

        # Re-style old and new buttons
        if old in self._buttons:
            self._buttons[old].setStyleSheet(self._btn_style(False))
        if loop_id in self._buttons:
            self._buttons[loop_id].setStyleSheet(self._btn_style(True))

    def _btn_style(self, is_target: bool) -> str:
        if is_target:
            return '''QPushButton {
                background: #0d2e1a; color: #69f000;
                border: 1.5px solid #69f000; border-radius: 6px;
                padding: 6px 10px; text-align: left;
                font-family: Courier New; font-size: 10px;
            }'''
        return '''QPushButton {
            background: #16213e; color: #666;
            border: 1px solid #0f3460; border-radius: 6px;
            padding: 6px 10px; text-align: left;
            font-family: Courier New; font-size: 10px;
        }'''


class ArmControlWidget(QWidget):
    command_requested = pyqtSignal(str)
    joystick_toggled  = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._joystick_on = False
        self._paused = False
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        header = QLabel('Arm Control')
        header.setFont(QFont('Courier New', 11, QFont.Bold))
        header.setStyleSheet('color: #00b4d8;')
        layout.addWidget(header)

        # ── Joystick toggle ──────────────────────────────────────────────────
        self._joystick_btn = QPushButton('⊙  Enable Joystick Control')
        self._joystick_btn.setCheckable(True)
        self._joystick_btn.setStyleSheet(self._joy_style(False))
        self._joystick_btn.clicked.connect(self._toggle_joystick)
        layout.addWidget(self._joystick_btn)

        hint = QLabel('When active: joystick drives main arm.\nDisable to return to coordinator control.')
        hint.setStyleSheet('color: #555; font-size: 9px; font-family: Courier New;')
        hint.setWordWrap(True)
        layout.addWidget(hint)

        layout.addWidget(self._separator())

        # ── Sequence control ─────────────────────────────────────────────────
        layout.addWidget(self._section('Sequence Control'))

        self._start_btn = self._make_btn('▶  Start', 'start', '#0d2e1a', '#00c853')
        layout.addWidget(self._start_btn)

        # Pause / Resume share the same spot — toggle visibility
        self._pause_btn  = self._make_btn('⏸  Pause',  'pause',  '#1a1a00', '#f0c040')
        self._resume_btn = self._make_btn('▶  Resume', 'resume', '#0d2e1a', '#00c853')
        self._resume_btn.setVisible(False)
        layout.addWidget(self._pause_btn)
        layout.addWidget(self._resume_btn)

        layout.addWidget(self._separator())

        # ── Error recovery ────────────────────────────────────────────────────
        layout.addWidget(self._section('Error Recovery'))

        row = QHBoxLayout()
        self._retry_btn = self._make_btn('↻  Retry',     'retry', '#0f3460', '#4a90d9')
        self._skip_btn  = self._make_btn('↩  Skip Loop', 'skip',  '#0f3460', '#00b4d8')
        row.addWidget(self._retry_btn)
        row.addWidget(self._skip_btn)
        layout.addLayout(row)

        self._abort_btn = self._make_btn('✕  Abort → IDLE', 'abort', '#2e0d0d', '#e94560')
        layout.addWidget(self._abort_btn)

        layout.addStretch()

        self._context_label = QLabel('')
        self._context_label.setStyleSheet(
            'color: #555; font-size: 9px; font-family: Courier New;')
        self._context_label.setWordWrap(True)
        layout.addWidget(self._context_label)

        # Initial state: only Start is active
        self.update_state('IDLE')

    def update_state(self, state: str):
        # Strip paused suffix for logic
        base_state = state.replace(' (PAUSED)', '').strip()
        is_paused  = '(PAUSED)' in state
        in_idle    = base_state == 'IDLE'
        in_error   = base_state == 'ERROR'
        running    = not in_idle and not in_error

        self._start_btn.setEnabled(in_idle)
        self._pause_btn.setEnabled(running and not is_paused)
        self._resume_btn.setEnabled(is_paused)
        self._pause_btn.setVisible(not is_paused)
        self._resume_btn.setVisible(is_paused)
        self._retry_btn.setEnabled(in_error)
        self._skip_btn.setEnabled(in_error)
        self._abort_btn.setEnabled(in_error)

        hints = {
            'IDLE':     'Send "start" to begin sequence',
            'HOMING':   'Homing side arm to known position',
            'AT_LOOP':  'Vision-guided positioning at next loop',
            'INSERT':   'Hook insertion through loop',
            'HANDOFF':  'Executing stow trajectory',
            'RETRACT':  'Retracting hook with line seating',
            'RELEASE':  'Verifying stow, preparing next loop',
            'COMPLETE': 'All loops stowed',
            'ERROR':    'Fault — use Retry, Skip, or Abort',
        }
        base = hints.get(base_state, '')
        self._context_label.setText(
            f'PAUSED — {base}' if is_paused else base
        )

    def _toggle_joystick(self, checked):
        self._joystick_on = checked
        self._joystick_btn.setStyleSheet(self._joy_style(checked))
        self._joystick_btn.setText(
            '⊙  Joystick Active — tap to release' if checked
            else '⊙  Enable Joystick Control'
        )
        self.joystick_toggled.emit(checked)

    def set_joystick_state(self, enabled: bool):
        self._joystick_btn.setChecked(enabled)
        self._toggle_joystick(enabled)

    def _make_btn(self, label, cmd, bg, border):
        btn = QPushButton(label)
        btn.setStyleSheet(f'''
            QPushButton {{
                background: {bg}; color: #eaeaea;
                border: 1px solid {border}; border-radius: 6px;
                padding: 7px; font-family: Courier New; font-size: 10px;
            }}
            QPushButton:hover {{ background: {border}44; }}
            QPushButton:disabled {{ color: #333; border-color: #222; background: #111; }}
        ''')
        btn.clicked.connect(lambda: self.command_requested.emit(cmd))
        return btn

    def _separator(self):
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet('color: #1e2d50;')
        return sep

    def _section(self, text):
        lbl = QLabel(text)
        lbl.setStyleSheet('color: #888; font-size: 9px; font-family: Courier New;')
        return lbl

    def _joy_style(self, active):
        if active:
            return '''QPushButton {
                background: #1a3a00; color: #69f000;
                border: 2px solid #69f000; border-radius: 6px;
                padding: 10px; font-family: Courier New;
                font-size: 11px; font-weight: bold;
            }'''
        return '''QPushButton {
            background: #16213e; color: #aaa;
            border: 1px solid #4a5568; border-radius: 6px;
            padding: 10px; font-family: Courier New; font-size: 11px;
        }
        QPushButton:hover { background: #1e2d50; color: #eaeaea; }'''