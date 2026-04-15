#!/usr/bin/env python3
"""
Manual Jog Script for Side Arm Calibration and Testing

Keyboard controls for manually moving the side arm motors.
Useful for calibration and testing motor directions.

Usage:
    ros2 run side_arm_control manual_jog

Controls:
    a/d - Horizontal movement (stepper2, left/right)
    w/s - Vertical movement (stepper1, up/down)
    q/e - Depth movement (DC motor, in/out)
    z/c - Servo rotation (left/right)
    SPACE - Stop DC motor and servo
    h - Home all axes
    0/1/2 - Home DC/Stepper1/Stepper2 individually
    x - Emergency stop all motors
    +/- - Adjust step size
    [/] - Adjust speed
    r - Request current state
    ESC - Quit
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import termios
import tty


class ManualJog(Node):
    def __init__(self):
        super().__init__('manual_jog')

        self.pub = self.create_publisher(String, '/side_arm_right/command', 10)
        self.sub = self.create_subscription(
            String, '/side_arm_right/state', self._state_callback, 10)

        self.step_size = 1000    # steps per keypress
        self.speed = 1500        # steps/second
        self.dc_speed = 50      # DC motor speed percent
        self.servo_step = 50    # servo offset increment (microseconds)
        self.servo_offset = 0   # current servo offset from neutral

        self._last_state = None
        self._running = True

    def _state_callback(self, msg: String):
        """Store latest state for display."""
        self._last_state = msg.data

    def send(self, cmd: str):
        """Send command to ESP32."""
        msg = String()
        msg.data = cmd
        self.pub.publish(msg)
        print(f'\rSent: {cmd:<50}', end='', flush=True)

    def print_help(self):
        """Print control instructions."""
        print('\n' + '=' * 60)
        print('SIDE ARM MANUAL JOG')
        print('=' * 60)
        print(f'Step size: {self.step_size} steps | Speed: {self.speed} steps/s')
        print(f'DC speed: {self.dc_speed}% | Servo step: {self.servo_step}us')
        print('-' * 60)
        print('Controls:')
        print('  a/d  - Horizontal (stepper2 left/right)')
        print('  w/s  - Vertical (stepper1 up/down)')
        print('  q/e  - Depth (DC motor in/out)')
        print('  z/c  - Servo rotation (left/right)')
        print('  SPACE - Stop DC motor and center servo')
        print('  h    - Home all axes | Steppers and DC')
        print('  0/1/2- Home DC/Stepper1/Stepper2 individually')
        print('  x    - EMERGENCY STOP')
        print('  +/-  - Step size | [/] - Speed')
        print('  </> - Servo step size')
        print('  r    - Request state')
        print('  ESC  - Quit')
        print('=' * 60)
        print()

    def run(self):
        """Main loop - read keyboard input and send commands."""
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)

        try:
            # Set terminal to raw mode
            tty.setraw(sys.stdin.fileno())
            self.print_help()

            while self._running and rclpy.ok():
                # Check if input is available (non-blocking)
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)

                    if ch == '\x1b':  # ESC
                        print('\n\rQuitting...')
                        self.send('STOP_ALL')
                        self.send('SERVO,0')  # Center servo on exit
                        break

                    elif ch == 'a':
                        self.send(f'STEPPER_MOVE,2,-{self.step_size},{self.speed}')
                    elif ch == 'd':
                        self.send(f'STEPPER_MOVE,2,{self.step_size},{self.speed}')

                    elif ch == 'w':
                        self.send(f'STEPPER_MOVE,1,{self.step_size},{self.speed}')
                    elif ch == 's':
                        self.send(f'STEPPER_MOVE,1,-{self.step_size},{self.speed}')

                    elif ch == 'q':
                        self.send(f'DC_SPEED,-{self.dc_speed}')
                    elif ch == 'e':
                        self.send(f'DC_SPEED,{self.dc_speed}')

                    # Servo controls
                    elif ch == 'z':
                        self.servo_offset -= self.servo_step
                        self.servo_offset = max(-1000, self.servo_offset)  # Limit range
                        self.send(f'SERVO,{self.servo_offset}')
                        print(f'\n\rServo offset: {self.servo_offset}us')
                    elif ch == 'c':
                        self.servo_offset += self.servo_step
                        self.servo_offset = min(1000, self.servo_offset)  # Limit range
                        self.send(f'SERVO,{self.servo_offset}')
                        print(f'\n\rServo offset: {self.servo_offset}us')

                    elif ch == ' ':
                        self.send('DC_SPEED,0')
                        # self.servo_offset = 0
                        # self.send('SERVO,0')  # Center servo
                        print('\n\rDC stopped, servo centered')

                    elif ch == 'h':
                        print('\n\rHoming all axes...')
                        self.send('HOME_ALL')
                    elif ch == '0':
                        self.send('HOME,0')
                    elif ch == '1':
                        self.send('HOME,1')
                    elif ch == '2':
                        self.send('HOME,2')

                    elif ch == 'x':
                        print('\n\rEMERGENCY STOP!')
                        self.send('STOP_NOW')
                        self.servo_offset = 0
                        self.send('SERVO,0')
                    elif ch == '\r':
                        print('\n\r STOP ALL')
                        self.send('STOP_ALL')

                    elif ch == '+' or ch == '=':
                        self.step_size += 100
                        print(f'\n\rStep size: {self.step_size}')
                    elif ch == '-':
                        self.step_size = max(100, self.step_size - 100)
                        print(f'\n\rStep size: {self.step_size}')

                    elif ch == ']':
                        self.speed += 100
                        print(f'\n\rSpeed: {self.speed}')
                    elif ch == '[':
                        self.speed = max(100, self.speed - 100)
                        print(f'\n\rSpeed: {self.speed}')

                    # Servo step size adjustment
                    elif ch == '>':
                        self.servo_step += 10
                        print(f'\n\rServo step: {self.servo_step}us')
                    elif ch == '<':
                        self.servo_step = max(10, self.servo_step - 10)
                        print(f'\n\rServo step: {self.servo_step}us')

                    elif ch == 'r':
                        self.send('REQUEST_STATE')
                        if self._last_state:
                            print(f'\n\rState: {self._last_state}')

                    elif ch == '?':
                        # Temporarily restore terminal for help display
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                        self.print_help()
                        tty.setraw(sys.stdin.fileno())

                # Spin ROS to process callbacks
                rclpy.spin_once(self, timeout_sec=0)

        except Exception as e:
            print(f'\n\rError: {e}')
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print('\n\rManual jog ended.')


def main(args=None):
    rclpy.init(args=args)
    node = ManualJog()

    print('Manual Jog Script for Side Arm')
    print('Make sure the serial bridge is running!')
    print('Press any key to start (ESC to quit)...\n')

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
