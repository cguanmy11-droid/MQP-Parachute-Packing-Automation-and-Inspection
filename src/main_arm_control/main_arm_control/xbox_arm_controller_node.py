#!/usr/bin/env python3
"""
Xbox One Controller for WidowX-200 Arm
Publishes to MainArmInterfaceNode topics for:
  - Continuous EE movement (sticks/triggers -> incremental commands)
  - Named poses (Start/LB -> pose_command)
  - Gripper control (A/B/X/Y -> gripper commands)
  - Waypoint recording (Select -> save, hold -> continuous record)
"""

import json
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist


# ── Xbox One Button Indices (verified: 11 buttons, 0-10) ────────────
BUTTONS = {
    'A':          0,   # Gripper Close
    'B':          1,   # Gripper Open
    'X':          2,   # Decrease Gripper Effort
    'Y':          3,   # Increase Gripper Effort
    'LB':         4,   # Smart Pose Toggle (Home <-> Sleep)
    'RB':         5,   # EE -Y direction
    'SELECT':     6,   # Back button - Waypoint save / toggle recording
    'START':      7,   # Start button - Home Pose
    'GUIDE':      8,   # Xbox button - Print current position
    'L3':         9,   # Left Stick Press - Flip EE X direction
    'R3':        10,   # Right Stick Press - Flip EE Roll direction
}

# ── Xbox One Axis Indices (verified from /joy topic at rest) ─────────
AXES = {
    'LEFT_X':     0,   # Left Stick X  -> EE X (horizontal)
    'LEFT_Y':     1,   # Left Stick Y  -> EE Z (vertical)
    'LT':         2,   # Left Trigger  -> Waist CCW (rests at 1.0)
    'RIGHT_X':    3,   # Right Stick X -> EE Roll
    'RIGHT_Y':    4,   # Right Stick Y -> EE Pitch
    'RT':         5,   # Right Trigger -> Waist CW (rests at 1.0)
    'DPAD_X':     6,   # D-pad X: Left=1.0 (coarse), Right=-1.0 (fine)
    'DPAD_Y':     7,   # D-pad Y: Up=1.0 (speed+), Down=-1.0 (speed-)
}

# Triggers rest at 1.0, fully pressed = -1.0
TRIGGER_DEADZONE = 0.0

# ── Speed Presets ────────────────────────────────────────────────────────
# Tuned for joy_moving_time=0.08s at ~10Hz control rate
SPEED_SETTINGS = {
    'coarse': {'ee_step': 0.01, 'rot_step': 0.04, 'waist_step': 0.03},
    'fine':   {'ee_step': 0.003, 'rot_step': 0.01, 'waist_step': 0.008},
}


class XboxArmController(Node):
    def __init__(self):
        super().__init__('xbox_arm_controller')

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter('threshold', 0.20)  # Deadzone for proportional control
        self.declare_parameter('control_rate', 10.0)       # Hz
        self.declare_parameter('record_interval', 0.5)     # seconds for continuous recording
        self.declare_parameter('waypoint_dir', os.path.expanduser('~/waypoints'))

        self.threshold = self.get_parameter('threshold').value
        control_rate = self.get_parameter('control_rate').value
        self.record_interval = self.get_parameter('record_interval').value
        self.waypoint_dir = self.get_parameter('waypoint_dir').value

        os.makedirs(self.waypoint_dir, exist_ok=True)

        # ── State Tracking ───────────────────────────────────────────
        self.latest_joy = None
        self.last_buttons = []
        self.axis_baseline = [0.0] * 8  # updated on first joy message

        # Flip toggles (L3/R3)
        self.flip_ee_x = False
        self.flip_ee_x_last = False
        self.flip_ee_roll = False
        self.flip_ee_roll_last = False

        # Triggers start at 0.0 on Linux before first touch, then rest at 1.0
        # We ignore them until they report a non-zero value
        self.lt_initialized = False
        self.rt_initialized = False

        # Safety: don't publish any movement until user presses a button,
        # confirming they're actually holding the controller.
        # Prevents phantom commands from uninitialized axes/stick drift at startup.
        self.controller_armed = False

        # Pose toggle state
        self.current_pose = 'unknown'   # 'home', 'sleep', 'unknown'
        self.moved_since_pose = False

        # Speed control
        self.speed_mode = 'coarse'
        self.speed_multiplier = 1.0     # adjusted by D-pad up/down

        # Recording state
        self.waypoints = []
        self.is_recording = False
        self.record_timer = None
        self.select_press_time = None   # for long-press detection

        # Latest robot state (from subscriptions)
        self.current_ee_pose = None     # Pose msg
        self.current_joint_positions = None  # dict {name: position}

        # ── Publishers (to MainArmInterfaceNode) ─────────────────────
        self.pose_cmd_pub = self.create_publisher(
            String, '/main_arm/pose_command', 10)
        self.gripper_cmd_pub = self.create_publisher(
            String, '/main_arm/gripper_command', 10)
        self.ee_increment_pub = self.create_publisher(
            Twist, '/main_arm/ee_increment', 10)

        # ── Subscribers ──────────────────────────────────────────────
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.pose_sub = self.create_subscription(
            Pose, '/main_arm/current_pose', self.current_pose_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/wx200/joint_states', self.joint_states_callback, 10)

        # ── Control Loop Timer ───────────────────────────────────────
        self.control_timer = self.create_timer(
            1.0 / control_rate, self.control_loop)

        self._print_controls()

    # ─────────────────────────────────────────────────────────────────
    # Startup Info
    # ─────────────────────────────────────────────────────────────────
    def _print_controls(self):
        self.get_logger().info('🎮 Xbox One Arm Controller initialized')
        self.get_logger().info('── Movement ──')
        self.get_logger().info('  Left Stick:   EE X/Z (horizontal/vertical)')
        self.get_logger().info('  Right Stick:  Wrist Rotate / Wrist Angle')
        self.get_logger().info('  RB:           EE -Y direction')
        self.get_logger().info('  LT/RT:        Waist CCW/CW')
        self.get_logger().info('── Gripper ──')
        self.get_logger().info('  A: Snap Close   B: Snap Open')
        self.get_logger().info('  X: Close step   Y: Open step')
        self.get_logger().info('── Poses ──')
        self.get_logger().info('  START: Home   LB: Smart Toggle (Home<->Sleep)')
        self.get_logger().info('── Speed ──')
        self.get_logger().info('  D-pad Up/Down: Speed +/-')
        self.get_logger().info('  D-pad Left: Coarse   Right: Fine')
        self.get_logger().info('── Recording ──')
        self.get_logger().info('  SELECT tap:  Save waypoint')
        self.get_logger().info('  SELECT hold: Toggle continuous recording')
        self.get_logger().info('  GUIDE:       Print current position')
        self.get_logger().info('  L3/R3: Flip EE X / Roll direction')

    # ─────────────────────────────────────────────────────────────────
    # Subscriber Callbacks
    # ─────────────────────────────────────────────────────────────────
    def joy_callback(self, msg: Joy):
        """Store latest joy and detect button edges."""
        buttons = list(msg.buttons)

        # Pad last_buttons on first message
        if not self.last_buttons:
            self.last_buttons = [0] * len(buttons)
            # Capture resting axis values as baseline to subtract drift
            self.axis_baseline = list(msg.axes)
            self.get_logger().info(
                '🎮 Controller connected. Press any button to arm.',
                throttle_duration_sec=5.0)

        # Detect rising edges
        for i in range(min(len(buttons), len(self.last_buttons))):
            if buttons[i] == 1 and self.last_buttons[i] == 0:
                if not self.controller_armed:
                    self.controller_armed = True
                    # Re-capture baseline now that controller is being held
                    self.axis_baseline = list(msg.axes)
                    self.get_logger().info('✅ Controller armed! Movement enabled.')
                self.handle_button_press(i)
            elif buttons[i] == 0 and self.last_buttons[i] == 1:
                self.handle_button_release(i)

        # Handle flip toggles (L3)
        l3 = buttons[BUTTONS['L3']] if len(buttons) > BUTTONS['L3'] else 0
        if l3 == 1 and not self.flip_ee_x_last:
            self.flip_ee_x = not self.flip_ee_x
            self.get_logger().info(f'EE X flip: {"inverted" if self.flip_ee_x else "normal"}')
        if l3 == 0:
            self.flip_ee_x_last = self.flip_ee_x

        # Handle flip toggles (R3)
        r3 = buttons[BUTTONS['R3']] if len(buttons) > BUTTONS['R3'] else 0
        if r3 == 1 and not self.flip_ee_roll_last:
            self.flip_ee_roll = not self.flip_ee_roll
            self.get_logger().info(f'EE Roll flip: {"inverted" if self.flip_ee_roll else "normal"}')
        if r3 == 0:
            self.flip_ee_roll_last = self.flip_ee_roll

        self.last_buttons = buttons
        self.latest_joy = msg

    def current_pose_callback(self, msg: Pose):
        self.current_ee_pose = msg

    def joint_states_callback(self, msg: JointState):
        self.current_joint_positions = dict(zip(msg.name, msg.position))

    # ─────────────────────────────────────────────────────────────────
    # Button Press Handlers
    # ─────────────────────────────────────────────────────────────────
    def handle_button_press(self, idx):
        # ── Gripper ──
        if idx == BUTTONS['A']:
            self.get_logger().info('Gripper CLOSE')
            msg = String()
            msg.data = 'close'
            self.gripper_cmd_pub.publish(msg)

        elif idx == BUTTONS['B']:
            self.get_logger().info('Gripper OPEN')
            msg = String()
            msg.data = 'open'
            self.gripper_cmd_pub.publish(msg)

        elif idx == BUTTONS['X']:
            self.get_logger().info('Gripper close step')
            msg = String()
            msg.data = 'dec'
            self.gripper_cmd_pub.publish(msg)

        elif idx == BUTTONS['Y']:
            self.get_logger().info('Gripper open step')
            msg = String()
            msg.data = 'inc'
            self.gripper_cmd_pub.publish(msg)

        # ── Poses ──
        elif idx == BUTTONS['START']:
            self.get_logger().info('Going to HOME')
            msg = String()
            msg.data = 'home'
            self.pose_cmd_pub.publish(msg)
            self.current_pose = 'home'
            self.moved_since_pose = False

        elif idx == BUTTONS['LB']:
            self._smart_pose_toggle()

        # ── Recording ──
        elif idx == BUTTONS['SELECT']:
            self.select_press_time = self.get_clock().now().nanoseconds / 1e9

        # ── Print Position ──
        elif idx == BUTTONS['GUIDE']:
            self.print_current_position()

    def handle_button_release(self, idx):
        """Handle button release — used for SELECT long-press detection."""
        if idx == BUTTONS['SELECT'] and self.select_press_time is not None:
            held = (self.get_clock().now().nanoseconds / 1e9) - self.select_press_time
            self.select_press_time = None

            if held >= 1.5:
                self.toggle_continuous_recording()
            else:
                self.save_waypoint()

    def _smart_pose_toggle(self):
        """LB: smart toggle between Home and Sleep."""
        if self.current_pose == 'home' and not self.moved_since_pose:
            target = 'sleep'
        else:
            target = 'home'

        self.get_logger().info(f'LB toggle -> {target}')
        msg = String()
        msg.data = target
        self.pose_cmd_pub.publish(msg)
        self.current_pose = target
        self.moved_since_pose = False

    # ─────────────────────────────────────────────────────────────────
    # Control Loop (runs at control_rate Hz)
    # ─────────────────────────────────────────────────────────────────
    def control_loop(self):
        """Read sticks/triggers and publish incremental EE commands."""
        if self.latest_joy is None:
            return
        if not self.controller_armed:
            return

        axes = list(self.latest_joy.axes)
        buttons = self.latest_joy.buttons
        if len(axes) < 8:
            return

        # Subtract baseline drift from stick axes (not triggers or d-pad)
        for i in [AXES['LEFT_X'], AXES['LEFT_Y'], AXES['RIGHT_X'], AXES['RIGHT_Y']]:
            if i < len(self.axis_baseline):
                axes[i] = axes[i] - self.axis_baseline[i]

        speed = SPEED_SETTINGS[self.speed_mode]
        mult = self.speed_multiplier

        dx = dy = dz = droll = dpitch = dwaist = 0.0

        # ── Left Stick -> EE X / Z ──
        lx = axes[AXES['LEFT_X']]
        ly = axes[AXES['LEFT_Y']]

        if abs(lx) >= self.threshold:
            direction = 1.0 if not self.flip_ee_x else -1.0
            # Proportional: remap [threshold..1.0] to [0.2..1.0] for smooth speed
            scale = (abs(lx) - self.threshold) / (1.0 - self.threshold)
            scale = 0.2 + 0.8 * scale  # minimum 20% speed at threshold
            dx = direction * (1.0 if lx > 0 else -1.0) * speed['ee_step'] * mult * scale
            self.moved_since_pose = True

        if abs(ly) >= self.threshold:
            scale = (abs(ly) - self.threshold) / (1.0 - self.threshold)
            scale = 0.2 + 0.8 * scale
            dz = (1.0 if ly > 0 else -1.0) * speed['ee_step'] * mult * scale
            self.moved_since_pose = True

        # ── RB -> EE -Y ──
        if len(buttons) > BUTTONS['RB'] and buttons[BUTTONS['RB']] == 1:
            dy = -speed['ee_step'] * mult
            self.moved_since_pose = True

        # ── Right Stick -> Wrist Joints (direct) ──
        rx = axes[AXES['RIGHT_X']]
        ry = axes[AXES['RIGHT_Y']]

        if abs(rx) >= self.threshold:
            direction = 1.0 if not self.flip_ee_roll else -1.0
            scale = (abs(rx) - self.threshold) / (1.0 - self.threshold)
            scale = 0.2 + 0.8 * scale
            droll = direction * (1.0 if rx > 0 else -1.0) * speed['rot_step'] * mult * scale

        if abs(ry) >= self.threshold:
            scale = (abs(ry) - self.threshold) / (1.0 - self.threshold)
            scale = 0.2 + 0.8 * scale
            dpitch = (1.0 if ry > 0 else -1.0) * speed['rot_step'] * mult * scale

        # ── Triggers -> Waist ──
        # Linux quirk: triggers report 0.0 before first touch, then rest at 1.0
        # Once touched, they range from 1.0 (released) to -1.0 (fully pressed)
        rt = axes[AXES['RT']]
        lt = axes[AXES['LT']]

        if rt != 0.0:
            self.rt_initialized = True
        if lt != 0.0:
            self.lt_initialized = True

        if self.rt_initialized and rt <= TRIGGER_DEADZONE:
            dwaist = -speed['waist_step'] * mult   # CW
        elif self.lt_initialized and lt <= TRIGGER_DEADZONE:
            dwaist = speed['waist_step'] * mult    # CCW

        # ── D-pad -> Speed Control ──
        dpad_y = axes[AXES['DPAD_Y']]
        dpad_x = axes[AXES['DPAD_X']]

        if dpad_y == 1.0:
            self.speed_multiplier = min(3.0, self.speed_multiplier + 0.1)
            self.get_logger().info(
                f'Speed multiplier: {self.speed_multiplier:.1f}x', throttle_duration_sec=0.5)
        elif dpad_y == -1.0:
            self.speed_multiplier = max(0.2, self.speed_multiplier - 0.1)
            self.get_logger().info(
                f'Speed multiplier: {self.speed_multiplier:.1f}x', throttle_duration_sec=0.5)

        if dpad_x == 1.0 and self.speed_mode != 'coarse':
            self.speed_mode = 'coarse'
            self.get_logger().info('Speed mode: COARSE')
        elif dpad_x == -1.0 and self.speed_mode != 'fine':
            self.speed_mode = 'fine'
            self.get_logger().info('Speed mode: FINE')

        # ── Publish if any movement ──
        if any(v != 0.0 for v in [dx, dy, dz, droll, dpitch, dwaist]):
            twist = Twist()
            twist.linear.x = dx
            twist.linear.y = dy
            twist.linear.z = dz
            twist.angular.x = droll    # wrist_rotate increment
            twist.angular.y = dpitch   # wrist_angle increment
            twist.angular.z = dwaist   # waist increment
            self.ee_increment_pub.publish(twist)

    # ─────────────────────────────────────────────────────────────────
    # Position Display
    # ─────────────────────────────────────────────────────────────────
    def print_current_position(self):
        """Print current EE pose and joint positions to terminal."""
        self.get_logger().info('═' * 50)
        self.get_logger().info('📍 CURRENT POSITION')
        self.get_logger().info('═' * 50)

        if self.current_ee_pose is not None:
            p = self.current_ee_pose
            self.get_logger().info(
                f'  EE Position:  x={p.position.x:.4f}  '
                f'y={p.position.y:.4f}  z={p.position.z:.4f}')
            self.get_logger().info(
                f'  EE Quaternion: x={p.orientation.x:.4f}  '
                f'y={p.orientation.y:.4f}  z={p.orientation.z:.4f}  '
                f'w={p.orientation.w:.4f}')
        else:
            self.get_logger().warn(
                '  EE Pose: not available (is interface publishing /main_arm/current_pose?)')

        if self.current_joint_positions is not None:
            self.get_logger().info('  Joint Positions:')
            for name, pos in self.current_joint_positions.items():
                self.get_logger().info(
                    f'    {name:20s}: {pos:+.4f} rad  ({math.degrees(pos):+.1f}°)')
        else:
            self.get_logger().warn(
                '  Joints: not available (is /wx200/joint_states publishing?)')

        self.get_logger().info(f'  Waypoints saved: {len(self.waypoints)}')
        self.get_logger().info('═' * 50)

    # ─────────────────────────────────────────────────────────────────
    # Waypoint Recording
    # ─────────────────────────────────────────────────────────────────
    def save_waypoint(self):
        """Capture current EE pose + joint positions as a waypoint."""
        waypoint = {
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'ee_pose': None,
            'joint_positions': None,
        }

        if self.current_ee_pose is not None:
            p = self.current_ee_pose
            waypoint['ee_pose'] = {
                'x': p.position.x,
                'y': p.position.y,
                'z': p.position.z,
                'qx': p.orientation.x,
                'qy': p.orientation.y,
                'qz': p.orientation.z,
                'qw': p.orientation.w,
            }

        if self.current_joint_positions is not None:
            waypoint['joint_positions'] = dict(self.current_joint_positions)

        self.waypoints.append(waypoint)
        count = len(self.waypoints)
        self.get_logger().info(f'📍 Waypoint {count} saved')

        if waypoint['ee_pose']:
            ep = waypoint['ee_pose']
            self.get_logger().info(
                f'   EE: x={ep["x"]:.4f} y={ep["y"]:.4f} z={ep["z"]:.4f}')
        if waypoint['joint_positions']:
            joints_str = ', '.join(
                f'{k}={v:.3f}' for k, v in waypoint['joint_positions'].items())
            self.get_logger().info(f'   Joints: {joints_str}')

        # Auto-save after each waypoint
        self._write_waypoints_to_file()

    def toggle_continuous_recording(self):
        """Start or stop recording at fixed intervals."""
        if self.is_recording:
            # Stop
            if self.record_timer is not None:
                self.record_timer.cancel()
                self.record_timer = None
            self.is_recording = False
            self._write_waypoints_to_file()
            self.get_logger().info(
                f'⏹ Recording stopped. {len(self.waypoints)} waypoints saved.')
        else:
            # Start
            self.is_recording = True
            self.record_timer = self.create_timer(
                self.record_interval, self._record_tick)
            self.get_logger().info(
                f'⏺ Recording started (every {self.record_interval}s)')

    def _record_tick(self):
        """Timer callback for continuous recording."""
        self.save_waypoint()

    def _write_waypoints_to_file(self):
        """Write current waypoints list to a JSON file."""
        if not self.waypoints:
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.waypoint_dir, f'waypoints_{timestamp}.json')

        # Convert any float32/float64 values to plain floats for JSON
        clean_waypoints = []
        for wp in self.waypoints:
            clean_wp = {'timestamp': float(wp['timestamp'])}

            if wp['ee_pose'] is not None:
                clean_wp['ee_pose'] = {k: float(v) for k, v in wp['ee_pose'].items()}

            if wp['joint_positions'] is not None:
                clean_wp['joint_positions'] = {
                    k: float(v) for k, v in wp['joint_positions'].items()
                }

            clean_waypoints.append(clean_wp)

        data = {
            'created': timestamp,
            'num_waypoints': len(clean_waypoints),
            'speed_mode': self.speed_mode,
            'waypoints': clean_waypoints,
        }

        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(f'💾 Saved to {filename}')

    def clear_waypoints(self):
        """Clear all recorded waypoints."""
        self.waypoints.clear()
        self.get_logger().info('🗑 Waypoints cleared')


def main(args=None):
    rclpy.init(args=args)
    node = XboxArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Xbox controller')
        # Final save if recording
        if node.is_recording:
            node.toggle_continuous_recording()
        elif node.waypoints:
            node._write_waypoints_to_file()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()