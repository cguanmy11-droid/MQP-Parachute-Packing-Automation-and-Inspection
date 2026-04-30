"""
handoff_confirm_dialog.py
-------------------------
Modal popup that asks the operator to verify handoff completion
before the coordinator transitions to RETRACT.
"""

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QFrame
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont


class HandoffConfirmDialog(QDialog):
    """Blocking popup shown when coordinator is AWAITING_CONFIRM."""

    confirm_clicked = pyqtSignal()
    retry_clicked = pyqtSignal()

    def __init__(self, arm: str = 'unknown', loop_id: str = '?', parent=None):
        super().__init__(parent)
        self.setWindowTitle('Handoff Confirmation Required')
        self.setModal(True)
        self.setFixedSize(480, 240)
        self.setStyleSheet('background: #1a1a2e; color: #eaeaea;')

        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(16)

        # Title
        title = QLabel('HANDOFF COMPLETE')
        title.setFont(QFont('Courier New', 14, QFont.Bold))
        title.setStyleSheet('color: #00b4d8; letter-spacing: 2px;')
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet('color: #0f3460;')
        layout.addWidget(sep)

        # Context info
        info = QLabel(
            f'Arm: <b>{arm.upper()}</b><br>'
            f'Loop: <b>{loop_id}</b><br><br>'
            'Main arm has completed the hole-center sequence.<br>'
            'Please verify the line has been passed correctly.'
        )
        info.setFont(QFont('Courier New', 10))
        info.setStyleSheet('color: #eaeaea;')
        info.setAlignment(Qt.AlignCenter)
        info.setWordWrap(True)
        layout.addWidget(info)

        # Buttons
        button_row = QHBoxLayout()
        button_row.setSpacing(12)

        retry_btn = QPushButton('⟲  RETRY HANDOFF')
        retry_btn.setStyleSheet('''
            QPushButton {
                background: #3d0016; color: #e94560;
                border: 1px solid #e94560; border-radius: 6px;
                padding: 10px 16px; font-weight: bold;
                font-family: Courier New;
            }
            QPushButton:hover { background: #5d002a; }
        ''')
        retry_btn.clicked.connect(self._on_retry)

        confirm_btn = QPushButton('✓  ACCEPT & CONTINUE')
        confirm_btn.setStyleSheet('''
            QPushButton {
                background: #0d3d1f; color: #69f000;
                border: 1px solid #69f000; border-radius: 6px;
                padding: 10px 16px; font-weight: bold;
                font-family: Courier New;
            }
            QPushButton:hover { background: #155a2f; }
        ''')
        confirm_btn.clicked.connect(self._on_confirm)
        confirm_btn.setDefault(True)

        button_row.addWidget(retry_btn)
        button_row.addWidget(confirm_btn)
        layout.addLayout(button_row)

    def _on_confirm(self):
        self.confirm_clicked.emit()
        self.accept()

    def _on_retry(self):
        self.retry_clicked.emit()
        self.reject()