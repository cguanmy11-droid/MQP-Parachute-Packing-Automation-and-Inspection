#!/usr/bin/env python3
"""
Coordinate System Node for Side Arm

Converts (x, y, z) mm coordinates to motor commands for the ESP32.

Coordinate mapping:
- X axis = Stepper 2 (horizontal belt rails)
- Y axis = Stepper 1 (vertical lead screw)
- Z axis = DC motor (depth lead screw, timed movement)
"""

import json
import time
import threading
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from parachute_interfaces.msg import SideArmState
from parachute_interfaces.srv import MoveToPosition
from parachute_interfaces.action import MoveToCoordinate


class SideArmCoordinateNode(Node):
    """
    Coordinate system node that:
    - Subscribes to /side_arm/state (raw ESP32 JSON)
    - Publishes /side_arm/parsed_state (parsed with mm coordinates)
    - Provides service /side_arm/move_to_position
    - Provides action /side_arm/move_to_coordinate
    - Publishes commands to /side_arm/command
    """

    def __init__(self):
        super().__init__('side_arm_coordinate_node')

        # ========== CALIBRATION PARAMETERS (to be tuned) ==========
        self.declare_parameter('steps_per_mm_horizontal', 80.0)   # Stepper2 - belt drive
        self.declare_parameter('steps_per_mm_vertical', 200.0)    # Stepper1 - lead screw
        self.declare_parameter('dc_mm_per_second', 10.0)          # DC motor travel rate
        self.declare_parameter('dc_speed_percent', 50)            # Default DC speed

        # Speed defaults
        self.declare_parameter('default_speed_horizontal', 800.0)  # steps/s
        self.declare_parameter('default_speed_vertical', 400.0)    # steps/s

        # Workspace limits (soft limits in mm)
        self.declare_parameter('max_x_mm', 300.0)
        self.declare_parameter('max_y_mm', 200.0)
        self.declare_parameter('max_z_mm', 150.0)

        self._load_parameters()

        # State tracking
        self._current_state: Optional[Dict[str, Any]] = None
        self._is_homed = False
        self._position_x_mm = 0.0
        self._position_y_mm = 0.0
        self._position_z_mm = 0.0  # DC motor - estimated from timed moves
        self._dc_move_start: Optional[float] = None
        self._dc_target_z_mm: Optional[float] = None
        self._dc_start_z_mm: float = 0.0      
        self._dc_duration: float = 0.0        
        self._dc_direction: int = 0           
        self._state_lock = threading.Lock()

        # Callback group for concurrent callbacks
        self._cb_group = ReentrantCallbackGroup()

        # Publishers
        self._cmd_pub = self.create_publisher(String, '/side_arm/command', 10)
        self._state_pub = self.create_publisher(SideArmState, '/side_arm/parsed_state', 10)

        # Subscribers
        self._raw_state_sub = self.create_subscription(
            String, '/side_arm/state', self._raw_state_callback, 10)

        # Subscribe to commands to track DC movements from any source
        self._cmd_sub = self.create_subscription(
            String, '/side_arm/command', self._command_callback, 10)

        # Services
        self._move_srv = self.create_service(
            MoveToPosition, '/side_arm/move_to_position',
            self._move_to_position_callback,
            callback_group=self._cb_group)

        # Action server
        self._move_action = ActionServer(
            self, MoveToCoordinate, '/side_arm/move_to_coordinate',
            self._move_to_coordinate_callback,
            callback_group=self._cb_group)

        # Timer to publish parsed state
        self.create_timer(0.1, self._publish_parsed_state)

        # Timer to monitor DC motor and stop after duration (50ms interval)
        self.create_timer(0.05, self._dc_motor_monitor_callback)

        self.get_logger().info(
            f'Coordinate node initialized. Calibration: '
            f'H={self.steps_per_mm_horizontal} steps/mm, '
            f'V={self.steps_per_mm_vertical} steps/mm, '
            f'DC={self.dc_mm_per_second} mm/s'
        )

    def _load_parameters(self):
        """Load ROS parameters into instance variables."""
        self.steps_per_mm_horizontal = self.get_parameter('steps_per_mm_horizontal').value
        self.steps_per_mm_vertical = self.get_parameter('steps_per_mm_vertical').value
        self.dc_mm_per_second = self.get_parameter('dc_mm_per_second').value
        self.dc_speed_percent = self.get_parameter('dc_speed_percent').value
        self.default_speed_horizontal = self.get_parameter('default_speed_horizontal').value
        self.default_speed_vertical = self.get_parameter('default_speed_vertical').value
        self.max_x_mm = self.get_parameter('max_x_mm').value
        self.max_y_mm = self.get_parameter('max_y_mm').value
        self.max_z_mm = self.get_parameter('max_z_mm').value

    def _raw_state_callback(self, msg: String):
        """Parse raw STATE JSON from ESP32."""
        data = msg.data.strip()
        if not data.startswith('STATE'):
            return

        try:
            payload = data.split('STATE', 1)[1].strip()
            state = json.loads(payload)

            with self._state_lock:
                self._current_state = state

                # Convert step positions to mm
                s1 = int(state.get('s1', 0))  # Vertical (stepper1)
                s2 = int(state.get('s2', 0))  # Horizontal (stepper2)

                self._position_y_mm = s1 / self.steps_per_mm_vertical
                self._position_x_mm = s2 / self.steps_per_mm_horizontal

                # Check homing status (all limits triggered and positions zeroed)
                l1 = bool(int(state.get('l1', 0)))  # Depth limit
                l2 = bool(int(state.get('l2', 0)))  # Horizontal limit
                l3 = bool(int(state.get('l3', 0)))  # Vertical limit

                # Reset Z position to 0 when depth limit is hit
                if l1:
                    self._position_z_mm = 0.0
                    self._dc_move_start = None
                    self._dc_target_z_mm = None
                    self._dc_start_z_mm = 0.0
                    self._dc_duration = 0.0
                    self._dc_direction = 0

                # Consider homed if all positions are at zero (after limits hit)
                if s1 == 0 and s2 == 0 and l1:
                    self._is_homed = True

        except Exception as e:
            self.get_logger().warn(f'Failed to parse state: {e}')

    def _publish_parsed_state(self):
        """Publish parsed state with mm coordinates."""
        if self._current_state is None:
            return

        with self._state_lock:
            msg = SideArmState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.limit_horizontal = bool(int(self._current_state.get('l2', 0)))
            msg.limit_vertical = bool(int(self._current_state.get('l3', 0)))
            msg.limit_depth = bool(int(self._current_state.get('l1', 0)))
            msg.steps_horizontal = int(self._current_state.get('s2', 0))
            msg.steps_vertical = int(self._current_state.get('s1', 0))
            msg.dc_percent = int(self._current_state.get('dc', 0))
            msg.x_mm = self._position_x_mm
            msg.y_mm = self._position_y_mm
            msg.z_mm = self._position_z_mm
            msg.is_homed = self._is_homed

        self._state_pub.publish(msg)

    def _dc_motor_monitor_callback(self):
        """Monitor DC motor movement and update position estimate."""
        with self._state_lock:
            if self._dc_move_start is None:
                return

            elapsed = time.time() - self._dc_move_start

            # Interpolate Z position during movement
            distance_moved = elapsed * self.dc_mm_per_second
            
            if self._dc_direction > 0:
                new_z = self._dc_start_z_mm + distance_moved
                if self._dc_target_z_mm is not None:
                    new_z = min(new_z, self._dc_target_z_mm)
                self._position_z_mm = min(new_z, self.max_z_mm)
            elif self._dc_direction < 0:
                new_z = self._dc_start_z_mm - distance_moved
                if self._dc_target_z_mm is not None:
                    new_z = max(new_z, self._dc_target_z_mm)
                self._position_z_mm = max(new_z, 0.0)

            # Only auto-stop if we have a target and duration (service/action initiated)
            if self._dc_target_z_mm is not None and self._dc_duration < 900:
                if elapsed >= self._dc_duration:
                    self._send_command('DC_SPEED,0')
                    self._position_z_mm = self._dc_target_z_mm

                    self._dc_move_start = None
                    self._dc_target_z_mm = None
                    self._dc_duration = 0.0
                    self._dc_direction = 0

                    self.get_logger().info(
                        f'DC motor stopped at Z={self._position_z_mm:.1f}mm'
                    )

    def _command_callback(self, msg: String):
        """Track DC motor commands from any source (including manual_jog)."""
        cmd = msg.data.strip().upper()
        
        if cmd.startswith('DC_SPEED,'):
            try:
                # Parse the speed percent
                parts = cmd.split(',')
                if len(parts) >= 2:
                    dc_percent = int(parts[1])
                    
                    with self._state_lock:
                        if dc_percent == 0:
                            # Motor stopped - finalize position
                            if self._dc_move_start is not None:
                                elapsed = time.time() - self._dc_move_start
                                distance = elapsed * self.dc_mm_per_second
                                
                                if self._dc_direction > 0:
                                    self._position_z_mm = min(
                                        self._dc_start_z_mm + distance,
                                        self.max_z_mm
                                    )
                                elif self._dc_direction < 0:
                                    self._position_z_mm = max(
                                        self._dc_start_z_mm - distance,
                                        0.0
                                    )
                                
                                self.get_logger().info(
                                    f'DC stopped (external): Z={self._position_z_mm:.1f}mm'
                                )
                            
                            # Clear tracking
                            self._dc_move_start = None
                            self._dc_target_z_mm = None
                            self._dc_duration = 0.0
                            self._dc_direction = 0
                        
                        else:
                            # Motor starting - only start tracking if not already tracking
                            if self._dc_move_start is None:
                                self._dc_start_z_mm = self._position_z_mm
                                self._dc_move_start = time.time()
                                self._dc_target_z_mm = None  # Unknown target for manual moves
                                self._dc_duration = 999.0  # Will run until stopped
                                self._dc_direction = -1 if dc_percent > 0 else 1  # Negative speed = positive Z
                                
                                self.get_logger().info(
                                    f'DC started (external): from Z={self._dc_start_z_mm:.1f}mm, '
                                    f'direction={self._dc_direction}'
                                )
            
            except (ValueError, IndexError) as e:
                self.get_logger().warn(f'Failed to parse DC command: {e}')
    
    # def _dc_motor_monitor_callback(self): 
    #     """Monitor DC motor movement and stop after calculated duration.""" 
    #     should_stop = False 
    #     final_z = 0.0 
        
    #     with self._state_lock: 
    #         # Debug: log when tracking is active 
    #         if self._dc_move_start is not None: 
    #             self.get_logger().debug( 
    #                 f'DC monitor: start={self._dc_move_start}, ' 
    #                 f'duration={self._dc_duration}, dir={self._dc_direction}' 
    #             ) 
            
    #         if self._dc_move_start is None or self._dc_duration <= 0: 
    #         return 
            
    #         elapsed = time.time() - self._dc_move_start 
    #         self.get_logger().info( 
    #             f'DC moving: elapsed={elapsed:.2f}s / {self._dc_duration:.2f}s, ' 
    #             f'Z={self._position_z_mm:.1f}mm' 
    #         ) 
            
    #         # Interpolate Z position during movement 
    #         distance_moved = elapsed * self.dc_mm_per_second 
    #         if self._dc_direction > 0: 
    #             self._position_z_mm = min( 
    #                 self._dc_start_z_mm + distance_moved, 
    #                 self._dc_target_z_mm if self._dc_target_z_mm else self.max_z_mm 
    #             )
    #         elif self._dc_direction < 0: 
    #             self._position_z_mm = max( 
    #                 self._dc_start_z_mm - distance_moved, 
    #                 self._dc_target_z_mm if self._dc_target_z_mm is not None else 0.0 
    #             ) 
            
    #         # Check if duration has elapsed - time to stop 
    #         if elapsed >= self._dc_duration: 
    #         should_stop = True 
            
    #         # Set final position to target 
    #         if self._dc_target_z_mm is not None: 
    #             self._position_z_mm = self._dc_target_z_mm 
    #             final_z = self._position_z_mm 
            
    #         # Clear tracking variables 
    #         self._dc_move_start = None 
    #         self._dc_target_z_mm = None 
    #         self._dc_duration = 0.0 
    #         self._dc_direction = 0 
            
    #         # Send stop command OUTSIDE the lock 
    #         if should_stop: 
    #             self._send_command('DC_SPEED,0') 
    #             self.get_logger().info(f'DC motor stopped at Z={final_z:.1f}mm') 

    def _send_command(self, cmd: str):
        """Send command to ESP32 via serial bridge."""
        msg = String()
        msg.data = cmd
        self._cmd_pub.publish(msg)
        self.get_logger().debug(f'Sent: {cmd}')

    def _mm_to_steps_horizontal(self, mm: float) -> int:
        return int(mm * self.steps_per_mm_horizontal)

    def _mm_to_steps_vertical(self, mm: float) -> int:
        return int(mm * self.steps_per_mm_vertical)

    def _mm_to_dc_duration(self, delta_mm: float) -> float:
        """Calculate duration in seconds to move DC motor given distance in mm."""
        return abs(delta_mm) / self.dc_mm_per_second

    def _move_to_position_callback(self, request, response):
        """Service callback: move to absolute position."""
        try:
            # Validate bounds
            x = max(0, min(request.x_mm, self.max_x_mm))
            y = max(0, min(request.y_mm, self.max_y_mm))
            z = max(0, min(request.z_mm, self.max_z_mm))

            speed_scale = max(0.1, min(1.0, request.speed_scale))

            # Calculate deltas
            with self._state_lock:
                dx = x - self._position_x_mm
                dy = y - self._position_y_mm
                dz = z - self._position_z_mm

            # Convert to motor commands
            steps_x = self._mm_to_steps_horizontal(dx)
            steps_y = self._mm_to_steps_vertical(dy)

            speed_x = int(self.default_speed_horizontal * speed_scale)
            speed_y = int(self.default_speed_vertical * speed_scale)

            estimated_time = 0.0

            # Send stepper commands
            if abs(steps_x) > 0:
                self._send_command(f'STEPPER_MOVE,2,{steps_x},{speed_x}')
                estimated_time = max(
                    estimated_time,
                    abs(dx) / (speed_x / self.steps_per_mm_horizontal)
                )

            if abs(steps_y) > 0:
                self._send_command(f'STEPPER_MOVE,1,{steps_y},{speed_y}')
                estimated_time = max(
                    estimated_time,
                    abs(dy) / (speed_y / self.steps_per_mm_vertical)
                )

            # DC motor: timed movement
            if abs(dz) > 0.5:  # Threshold to avoid tiny moves
                dc_duration = self._mm_to_dc_duration(dz)
                dc_percent = -self.dc_speed_percent if dz > 0 else self.dc_speed_percent
                dc_percent = int(dc_percent * speed_scale)

                with self._state_lock:
                    self._dc_start_z_mm = self._position_z_mm
                    self._dc_move_start = time.time()
                    self._dc_target_z_mm = z
                    self._dc_duration = dc_duration
                    self._dc_direction = 1 if dz > 0 else -1

                self._send_command(f'DC_SPEED,{dc_percent}')
                estimated_time = max(estimated_time, dc_duration)
                self.get_logger().info(
                    f'DC motor moving Z: {self._dc_start_z_mm:.1f} -> {z:.1f}mm '
                    f'(duration={dc_duration:.2f}s)'
                )

            response.success = True
            response.message = f'Moving to ({x:.1f}, {y:.1f}, {z:.1f}) mm'
            response.estimated_time_sec = estimated_time

        except Exception as e:
            response.success = False
            response.message = str(e)
            response.estimated_time_sec = 0.0

        return response

    def _move_to_coordinate_callback(self, goal_handle):
        """Action callback: move with feedback and DC motor timing."""
        request = goal_handle.request
        feedback = MoveToCoordinate.Feedback()

        # Validate bounds
        x = max(0, min(request.x_mm, self.max_x_mm))
        y = max(0, min(request.y_mm, self.max_y_mm))
        z = max(0, min(request.z_mm, self.max_z_mm))
        speed_scale = max(0.1, min(1.0, request.speed_scale))

        start_time = time.time()

        with self._state_lock:
            start_x = self._position_x_mm
            start_y = self._position_y_mm
            start_z = self._position_z_mm

        # Calculate moves
        dx = x - start_x
        dy = y - start_y
        dz = z - start_z

        steps_x = self._mm_to_steps_horizontal(dx)
        steps_y = self._mm_to_steps_vertical(dy)
        speed_x = int(self.default_speed_horizontal * speed_scale)
        speed_y = int(self.default_speed_vertical * speed_scale)

        self.get_logger().info(
            f'Moving from ({start_x:.1f}, {start_y:.1f}, {start_z:.1f}) '
            f'to ({x:.1f}, {y:.1f}, {z:.1f}) mm'
        )

        # Start stepper movements
        if abs(steps_x) > 0:
            self._send_command(f'STEPPER_MOVE,2,{steps_x},{speed_x}')
        if abs(steps_y) > 0:
            self._send_command(f'STEPPER_MOVE,1,{steps_y},{speed_y}')

        # DC motor with timed stop
        dc_duration = 0.0
        if abs(dz) > 0.5:
            dc_duration = self._mm_to_dc_duration(dz)
            dc_percent = -self.dc_speed_percent if dz > 0 else self.dc_speed_percent
            dc_percent = int(dc_percent * speed_scale)
            self._send_command(f'DC_SPEED,{dc_percent}')
            with self._state_lock:
                self._dc_start_z_mm = start_z
                self._dc_move_start = time.time()
                self._dc_target_z_mm = z
                self._dc_duration = dc_duration
                self._dc_direction = 1 if dz > 0 else -1

        # Calculate max duration for timeout
        max_duration = max(
            abs(dx) / (speed_x / self.steps_per_mm_horizontal) if steps_x else 0,
            abs(dy) / (speed_y / self.steps_per_mm_vertical) if steps_y else 0,
            dc_duration
        ) + 2.0  # Buffer

        dc_stopped = dc_duration == 0.0

        # Monitor progress
        while rclpy.ok():
            elapsed = time.time() - start_time

            with self._state_lock:
                current_x = self._position_x_mm
                current_y = self._position_y_mm
                current_z = self._position_z_mm

            # Stop DC motor after duration
            if not dc_stopped and elapsed >= dc_duration:
                self._send_command('DC_SPEED,0')
                with self._state_lock:
                    self._position_z_mm = z
                    current_z = z
                    # Clear tracking variables
                    self._dc_move_start = None
                    self._dc_target_z_mm = None
                    self._dc_duration = 0.0
                    self._dc_direction = 0
                dc_stopped = True

            # Calculate progress
            total_dist = abs(dx) + abs(dy) + abs(dz)
            if total_dist > 0:
                moved = (
                    abs(current_x - start_x) +
                    abs(current_y - start_y) +
                    abs(current_z - start_z)
                )
                progress = min(1.0, moved / total_dist)
            else:
                progress = 1.0

            feedback.current_x_mm = current_x
            feedback.current_y_mm = current_y
            feedback.current_z_mm = current_z
            feedback.progress = progress
            goal_handle.publish_feedback(feedback)

            # Check completion
            at_target = (
                abs(current_x - x) < 2.0 and
                abs(current_y - y) < 2.0 and
                abs(current_z - z) < 2.0
            )

            if at_target or elapsed > max_duration:
                break

            time.sleep(0.1)

        # Result
        goal_handle.succeed()
        result = MoveToCoordinate.Result()
        result.success = True
        result.final_x_mm = current_x
        result.final_y_mm = current_y
        result.final_z_mm = current_z
        result.execution_time_sec = time.time() - start_time
        result.message = 'Move complete'

        self.get_logger().info(
            f'Move complete in {result.execution_time_sec:.2f}s, '
            f'final position: ({current_x:.1f}, {current_y:.1f}, {current_z:.1f}) mm'
        )

        return result


def main(args=None):
    rclpy.init(args=args)
    node = SideArmCoordinateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
