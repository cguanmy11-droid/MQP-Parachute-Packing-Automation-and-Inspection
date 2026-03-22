"""
main_window.py
--------------
Top-level Qt window. Wires GUI widgets ↔ ROSBridge.

Layout:
  ┌─────────────────────────────────────────────────────────┐
  │  PARACHUTE PACKING OPERATOR CONSOLE          [state tag]│
  ├──────────────────┬──────────────────┬───────────────────┤
  │  State Machine   │  Loop Selector   │  Arm Control      │
  │  (diagram)       │  (buttons)       │  (joystick/cmds)  │
  ├──────────────────┴──────────────────┴───────────────────┤
  │  Error / Message Log                                    │
  └─────────────────────────────────────────────────────────┘
"""

import sys
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QTextEdit, QApplication, QFrame
)
from PyQt5.QtCore import Qt, QDateTime
from PyQt5.QtGui import QFont, QColor, QPalette

from parachute_gui.gui_node import ROSBridge
from parachute_gui.widgets.state_machine_widget import StateMachineWidget
from parachute_gui.widgets.control_widgets import LoopSelectorWidget, ArmControlWidget

# Max log lines shown
MAX_LOG_LINES = 200


class MainWindow(QMainWindow):
    def __init__(self, ros_bridge: ROSBridge):
        super().__init__()
        self._bridge = ros_bridge
        self._log_lines = []

        self.setWindowTitle('Parachute Packing Operator Console')
        self.resize(1200, 750)
        self.setStyleSheet('background-color: #1a1a2e; color: #eaeaea;')

        self._build_ui()
        self._connect_signals()

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        main_layout = QVBoxLayout(root)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(8)

        # Header bar
        main_layout.addWidget(self._build_header())

        # Three-panel content area
        content = QHBoxLayout()
        content.setSpacing(8)

        self._state_widget = StateMachineWidget()
        self._state_widget.setMinimumWidth(320)

        self._loop_widget = LoopSelectorWidget()
        self._loop_widget.setMinimumWidth(240)

        self._arm_widget = ArmControlWidget()
        self._arm_widget.setMinimumWidth(240)

        for w in [self._state_widget, self._loop_widget, self._arm_widget]:
            w.setStyleSheet(w.styleSheet() + '''
                border: 1px solid #0f3460; border-radius: 8px;
            ''')
            content.addWidget(w)

        main_layout.addLayout(content, stretch=4)

        # Log panel
        main_layout.addWidget(self._build_log_panel(), stretch=1)

    def _build_header(self) -> QWidget:
        bar = QWidget()
        bar.setFixedHeight(48)
        bar.setStyleSheet('background: #16213e; border-radius: 8px;')
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(16, 0, 16, 0)

        title = QLabel('PARACHUTE PACKING  —  OPERATOR CONSOLE')
        title.setFont(QFont('Courier New', 13, QFont.Bold))
        title.setStyleSheet('color: #00b4d8; letter-spacing: 2px;')
        layout.addWidget(title)

        layout.addStretch()

        self._state_badge = QLabel('STATE: IDLE')
        self._state_badge.setFont(QFont('Courier New', 11, QFont.Bold))
        self._state_badge.setStyleSheet('''
            color: #e94560;
            background: #3d0016;
            border: 1px solid #e94560;
            border-radius: 4px;
            padding: 4px 12px;
        ''')
        layout.addWidget(self._state_badge)

        return bar

    def _build_log_panel(self) -> QWidget:
        panel = QWidget()
        panel.setStyleSheet('background: #16213e; border-radius: 8px; border: 1px solid #0f3460;')
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 6, 8, 6)

        label = QLabel('System Log')
        label.setStyleSheet('color: #888; font-size: 9px; font-family: Courier New;')
        layout.addWidget(label)

        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setFont(QFont('Courier New', 9))
        self._log.setStyleSheet('''
            QTextEdit {
                background: #0d0d1a; color: #888;
                border: none; border-radius: 4px;
            }
        ''')
        self._log.setFixedHeight(120)
        layout.addWidget(self._log)

        return panel

    # ── Signal wiring ─────────────────────────────────────────────────────────

    def _connect_signals(self):
        # ROS2 → GUI
        self._bridge.state_changed.connect(self._on_state_changed)
        self._bridge.error_received.connect(self._on_error)
        self._bridge.loops_updated.connect(self._loop_widget.update_loops)
        self._bridge.target_loop_updated.connect(self._loop_widget.set_target_loop)
        self._bridge.joystick_mode_changed.connect(self._arm_widget.set_joystick_state)

        # GUI → ROS2
        self._arm_widget.command_requested.connect(self._bridge.send_command)
        self._arm_widget.joystick_toggled.connect(self._bridge.set_joystick_mode)

    # ── Handlers ──────────────────────────────────────────────────────────────

    def _on_state_changed(self, state: str):
        self._state_badge.setText(f'STATE: {state}')
        self._state_widget.set_state(state)
        self._arm_widget.update_state(state)
        self._log_message(f'State → {state}', color='#00b4d8')

    def _on_error(self, error: str):
        self._log_message(f'ERROR: {error}', color='#e94560')

    def _log_message(self, msg: str, color: str = '#888'):
        ts = QDateTime.currentDateTime().toString('hh:mm:ss')
        html = f'<span style="color:#444">[{ts}]</span> <span style="color:{color}">{msg}</span>'
        self._log_lines.append(html)
        if len(self._log_lines) > MAX_LOG_LINES:
            self._log_lines.pop(0)
        self._log.setHtml('<br>'.join(self._log_lines))
        self._log.verticalScrollBar().setValue(
            self._log.verticalScrollBar().maximum()
        )

    def closeEvent(self, event):
        self._bridge.shutdown()
        event.accept()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    # Dark palette fallback
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor('#1a1a2e'))
    palette.setColor(QPalette.WindowText, QColor('#eaeaea'))
    app.setPalette(palette)

    bridge = ROSBridge()
    window = MainWindow(bridge)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()