from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, QWidget, QSizePolicy)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont, QKeyEvent
from std_msgs.msg import String

class _KeyWidget(QWidget):
    """Single key cap with label and subtitle."""
 
    def __init__(self, letter, subtitle, bg, border, text_color, sub_color, parent=None):
        super().__init__(parent)
        self.setFixedSize(64, 64)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 6, 0, 6)
        layout.setSpacing(2)
 
        lbl = QLabel(letter)
        lbl.setFont(QFont('monospace', 16, QFont.Bold))
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet(f'color: {text_color}; background: transparent;')
        layout.addWidget(lbl)
 
        sub = QLabel(subtitle)
        sub.setFont(QFont('monospace', 10))
        sub.setAlignment(Qt.AlignCenter)
        sub.setStyleSheet(f'color: {sub_color}; background: transparent;')
        layout.addWidget(sub)
 
        self.setStyleSheet(
            f'background: {bg}; border: 1px solid {border}; border-radius: 6px;'
        )
 
 
class _Spacer(QWidget):
    """Invisible spacer the same size as a key."""
 
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(64, 64)

class ManualJogDialog(QDialog):
    """GUI popup for manual jogging the side arm via keyboard."""
 
    # Color scheme
    _BLUE_BG = '#CADFF5'
    _BLUE_BD = '#85B7EB'
    _BLUE_TX = '#0C447C'
    _BLUE_SB = '#185FA5'
 
    _GREEN_BG = '#B5E8D4'
    _GREEN_BD = '#5DCAA5'
    _GREEN_TX = '#085041'
    _GREEN_SB = '#0F6E56'
 
    _PINK_BG = '#F4C0D1'
    _PINK_BD = '#ED93B1'
    _PINK_TX = '#72243E'
    _PINK_SB = '#993556'
 
    _GRAY_BG = '#D3D1C7'
    _GRAY_BD = '#B4B2A9'
    _GRAY_TX = '#2C2C2A'
    _GRAY_SB = '#444441'
 
    def __init__(self, ros_node, arm_ns='side_arm_right', parent=None):
        super().__init__(parent)
        self.ros_node = ros_node
        self.arm_ns = arm_ns
        self.pub = ros_node.create_publisher(String, f'/{arm_ns}/command', 10)
 
        # Jog parameters
        self.step_size = 1000
        self.speed = 1500
        self.dc_speed = 50
        self.servo_step = 50
        self.servo_offset = 0
 
        self.setWindowTitle(f'Manual Jog: {arm_ns}')
        self.setMinimumSize(520, 420)
        self.setFocusPolicy(Qt.StrongFocus)
        self._build_ui()
 
    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(12)
 
        # Status bar
        self._status = QLabel('Ready — press keys to jog')
        self._status.setFont(QFont('monospace', 11))
        self._status.setStyleSheet(
            'background: #111; color: #0f0; padding: 8px 12px; border-radius: 6px;'
        )
        layout.addWidget(self._status)
 
        # Main content: keys left, adjustments right
        content = QHBoxLayout()
        content.setSpacing(16)
 
        # Left side: keyboard layout
        left = QVBoxLayout()
        left.setSpacing(12)
 
        # Homing row
        left.addWidget(self._section_label('Homing'))
        home_row = QHBoxLayout()
        home_row.setSpacing(4)
        home_row.addWidget(_KeyWidget('H', 'all', self._GRAY_BG, self._GRAY_BD, self._GRAY_TX, self._GRAY_SB))
        home_row.addWidget(_KeyWidget('0', 'DC', self._GRAY_BG, self._GRAY_BD, self._GRAY_TX, self._GRAY_SB))
        home_row.addWidget(_KeyWidget('1', 'stp1', self._GRAY_BG, self._GRAY_BD, self._GRAY_TX, self._GRAY_SB))
        home_row.addWidget(_KeyWidget('2', 'stp2', self._GRAY_BG, self._GRAY_BD, self._GRAY_TX, self._GRAY_SB))
        home_row.addStretch()
        left.addLayout(home_row)
 
        # Movement + depth + servo
        left.addWidget(self._section_label('Movement + depth + servo'))
 
        # Row 1: Q W E
        row1 = QHBoxLayout()
        row1.setSpacing(4)
        row1.addWidget(_KeyWidget('Q', 'depth in', self._GREEN_BG, self._GREEN_BD, self._GREEN_TX, self._GREEN_SB))
        row1.addWidget(_KeyWidget('W', 'up', self._BLUE_BG, self._BLUE_BD, self._BLUE_TX, self._BLUE_SB))
        row1.addWidget(_KeyWidget('E', 'depth out', self._GREEN_BG, self._GREEN_BD, self._GREEN_TX, self._GREEN_SB))
        row1.addStretch()
        left.addLayout(row1)
 
        # Row 2: A S D
        row2 = QHBoxLayout()
        row2.setSpacing(4)
        row2.addWidget(_KeyWidget('A', 'left', self._BLUE_BG, self._BLUE_BD, self._BLUE_TX, self._BLUE_SB))
        row2.addWidget(_KeyWidget('S', 'down', self._BLUE_BG, self._BLUE_BD, self._BLUE_TX, self._BLUE_SB))
        row2.addWidget(_KeyWidget('D', 'right', self._BLUE_BG, self._BLUE_BD, self._BLUE_TX, self._BLUE_SB))
        row2.addStretch()
        left.addLayout(row2)
 
        # Row 3: Z [spacer] C
        row3 = QHBoxLayout()
        row3.setSpacing(4)
        row3.addWidget(_KeyWidget('Z', 'rot \u2190', self._PINK_BG, self._PINK_BD, self._PINK_TX, self._PINK_SB))
        row3.addWidget(_Spacer())
        row3.addWidget(_KeyWidget('C', 'rot \u2192', self._PINK_BG, self._PINK_BD, self._PINK_TX, self._PINK_SB))
        row3.addStretch()
        left.addLayout(row3)
 
        # Emergency stop + space
        stop_row = QHBoxLayout()
        stop_row.setSpacing(6)
 
        estop = QLabel('Enter/X — Stop All (emergency)')
        estop.setFont(QFont('monospace', 11, QFont.Bold))
        estop.setAlignment(Qt.AlignCenter)
        estop.setStyleSheet(
            'background: #FCEBEB; border: 1px solid #A32D2D; border-radius: 6px;'
            'color: #791F1F; padding: 8px 12px;'
        )
        stop_row.addWidget(estop, stretch=1)
 
        space = QLabel('Space — stop DC')
        space.setFont(QFont('monospace', 11, QFont.Bold))
        space.setAlignment(Qt.AlignCenter)
        space.setStyleSheet(
            'background: #FAEEDA; border: 1px solid #854F0B; border-radius: 6px;'
            'color: #633806; padding: 8px 12px;'
        )
        stop_row.addWidget(space, stretch=1)
 
        left.addLayout(stop_row)
        left.addStretch()
        content.addLayout(left, stretch=1)
 
        # Right side: adjustment panels
        right = QVBoxLayout()
        right.setSpacing(8)
        right.addWidget(self._section_label('Adjustments'))
 
        self._step_label = self._param_card('Step size', str(self.step_size), '+ / \u2212 to adjust')
        right.addWidget(self._step_label)
 
        self._speed_label = self._param_card('Speed', str(self.speed), '[ / ] to adjust')
        right.addWidget(self._speed_label)
 
        self._dc_label = self._param_card('DC speed', f'{self.dc_speed}%', '')
        right.addWidget(self._dc_label)
 
        self._servo_label = self._param_card('Servo step', f'{self.servo_step}us', '< / > to adjust')
        right.addWidget(self._servo_label)
 
        # Misc keys
        misc = QWidget()
        misc.setStyleSheet('background: #F1EFE8; border-radius: 6px;')
        misc_layout = QVBoxLayout(misc)
        misc_layout.setContentsMargins(10, 8, 10, 8)
        misc_layout.setSpacing(2)
        for line in ['R — request state', 'Esc — close']:
            lbl = QLabel(line)
            lbl.setFont(QFont('monospace', 10))
            lbl.setStyleSheet('color: #5F5E5A; background: transparent;')
            misc_layout.addWidget(lbl)
        right.addWidget(misc)
 
        right.addStretch()
        content.addLayout(right)
 
        layout.addLayout(content)
 
    def _section_label(self, text):
        lbl = QLabel(text.upper())
        lbl.setFont(QFont('monospace', 10))
        lbl.setStyleSheet('color: #888780; letter-spacing: 0.5px;')
        return lbl
 
    def _param_card(self, title, value, hint):
        card = QWidget()
        card.setStyleSheet('background: #D3D1C7; border-radius: 6px;')
        card.setMinimumWidth(140)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(2)
 
        t = QLabel(title)
        t.setFont(QFont('monospace', 9))
        t.setStyleSheet('color: #888780; background: transparent;')
        layout.addWidget(t)
 
        v = QLabel(value)
        v.setFont(QFont('monospace', 14, QFont.Bold))
        v.setStyleSheet('color: #2C2C2A; background: transparent;')
        v.setObjectName('value')
        layout.addWidget(v)
 
        if hint:
            h = QLabel(hint)
            h.setFont(QFont('monospace', 9))
            h.setStyleSheet('color: #5F5E5A; background: transparent;')
            layout.addWidget(h)
 
        return card
 
    def _update_param(self, card, value_text):
        lbl = card.findChild(QLabel, 'value')
        if lbl:
            lbl.setText(value_text)
 
    def _send(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.pub.publish(msg)
        self._status.setText(f'Sent: {cmd}')
        self._status.setStyleSheet(
            'background: #111; color: #0f0; padding: 8px 12px; border-radius: 6px;'
        )
    
    def keyPressEvent(self, event: QKeyEvent):
        key = event.key()
        text = event.text().lower()
        
        # Movement
        if text == 'a':
            self._send(f'STEPPER_MOVE,2,-{self.step_size},{self.speed}')
        elif text == 'd':
            self._send(f'STEPPER_MOVE,2,{self.step_size},{self.speed}')
        elif text == 'w':
            self._send(f'STEPPER_MOVE,1,{self.step_size},{self.speed}')
        elif text == 's':
            self._send(f'STEPPER_MOVE,1,-{self.step_size},{self.speed}')
        elif text == 'q':
            self._send(f'DC_SPEED,-{self.dc_speed}')
        elif text == 'e':
            self._send(f'DC_SPEED,{self.dc_speed}')
        
        # Servo
        elif text == 'z':
            self.servo_offset -= self.servo_step
            self._send(f'SERVO,{self.servo_offset}')
        elif text == 'c':
            self.servo_offset += self.servo_step
            self._send(f'SERVO,{self.servo_offset}')
        
        # Stop
        elif key == Qt.Key_Space:
            self._send('DC_SPEED,0')
        elif key == Qt.Key_Return:
            self._send('STOP_ALL')
        
        # Homing
        elif text == 'h':
            self._send('HOME')
        elif text == '0':
            self._send('HOME_DC')
        elif text == '1':
            self._send('HOME_STEPPER1')
        elif text == '2':
            self._send('HOME_STEPPER2')
        
        # Emergency stop
        elif text == 'x':
            self._send('STOP_NOW')
            self._status.setStyleSheet('background: #8b0000; color: white; padding: 8px; border-radius: 4px;')
            self._status.setText('EMERGENCY STOP SENT')
        
        # Parameter adjustments
        elif text in ('+', '='):
            self.step_size += 100
            self._update_params_label()
        elif text == '-':
            self.step_size = max(100, self.step_size - 100)
            self._update_params_label()
        elif text == ']':
            self.speed += 100
            self._update_params_label()
        elif text == '[':
            self.speed = max(100, self.speed - 100)
            self._update_params_label()
        elif text == '.':  # > without shift
            self.servo_step += 10
            self._update_params_label()
        elif text == ',':  # < without shift
            self.servo_step = max(10, self.servo_step - 10)
            self._update_params_label()
        
        # State request
        elif text == 'r':
            self._send('REQUEST_STATE')
        
        # Close
        elif key == Qt.Key_Escape:
            self.close()
        
        else:
            super().keyPressEvent(event)