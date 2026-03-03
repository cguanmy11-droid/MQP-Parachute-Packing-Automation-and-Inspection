#!/usr/bin/env python3
"""
Packing Coordinator Node

State machine coordinator for the automated parachute line stowing system.
Orchestrates two robotic arms through vision-guided manipulation using
event-driven state transitions loaded from config/stow_transitions.yaml.

States (matching paper Figure 6):
    IDLE       - System ready, arms homed
    AT_LOOP    - Vision-guided positioning at next loop
    INSERT     - Hook insertion with collision detection
    HANDOFF    - Dual-arm line transfer
    RETRACT    - Hook withdrawal with line seating
    RELEASE    - Cycle completion and verification
    COMPLETE   - All loops stowed
    ERROR      - Fault handling with operator recovery

The coordinator is intentionally thin — it sends high-level goals
and reacts to results. Arm nodes own IK, orientation, and execution.

Usage:
    ros2 run parachute_coordinator packing_coordinator_node

    # Start the sequence:
    ros2 topic pub --once /stow/command std_msgs/String "data: start"

    # Change pattern:
    ros2 topic pub --once /stow/command std_msgs/String "data: pattern:square_stow"

    # Recovery commands (in ERROR state):
    ros2 topic pub --once /stow/command std_msgs/String "data: retry"
    ros2 topic pub --once /stow/command std_msgs/String "data: skip"
    ros2 topic pub --once /stow/command std_msgs/String "data: abort"
"""

import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory

from parachute_interfaces.srv import RequestNextTarget, RotateHook, CaptureLoops
from parachute_interfaces.action import InsertHook, ExecuteTrajectory
from parachute_interfaces.msg import HookStatus, SideArmState
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
import time

from .state_machine import StateMachine, StowState
from .motion_pattern_manager import MotionPatternManager, BUILTIN_PATTERNS


class PackingCoordinatorNode(Node):
    """
    State machine coordinator for automated parachute line stowing.

    Implements on_enter_<state> methods that the StateMachine calls
    on each transition. State exits are driven by action/service
    callbacks that emit events back to the state machine.
    """

    def __init__(self):
        super().__init__('packing_coordinator_node')

        # ==================== PARAMETERS ====================
        self.declare_parameter('test_mode', False)
        self.declare_parameter('stow_pattern', 'recorded_stow')
        self.declare_parameter('pattern_dir', '')
        self.declare_parameter('action_timeout', 30.0)
        self.declare_parameter('expected_loop_count', 0)  # 0 = skip count verification

        self.test_mode = self.get_parameter('test_mode').value
        self.current_pattern = self.get_parameter('stow_pattern').value
        self.action_timeout = self.get_parameter('action_timeout').value
        self.expected_loop_count = self.get_parameter('expected_loop_count').value

        # ==================== STATE MACHINE ====================
        config_path = os.path.join(
            get_package_share_directory('parachute_coordinator'),
            'config', 'stow_transitions.yaml'
        )
        self.sm = StateMachine(
            config_path=config_path,
            handler_object=self,
            logger=self.get_logger()
        )

        # ==================== MOTION PATTERNS ====================
        pattern_dir = self.get_parameter('pattern_dir').value or None
        self.pattern_manager = MotionPatternManager(
            pattern_dir=pattern_dir,
            logger=self.get_logger()
        )
        for name, pattern in BUILTIN_PATTERNS.items():
            self.pattern_manager.patterns[name] = pattern

        # ==================== TRACKING ====================
        self.current_target_loop = None
        self.completed_loops = 0
        self.total_loops = 0
        self._error_message = ''
        self._active_goal_handle = None

        # Side arm homing tracking
        self._side_arm_is_homed = False
        self._has_homed_once = False  # Only home once at startup
        self._homing_timer = None
        self._homing_start_time = None

        # ==================== CALLBACK GROUP ====================
        self._cb_group = ReentrantCallbackGroup()

        # ==================== SERVICE CLIENTS ====================
        self.target_client = self.create_client(
            RequestNextTarget, '/request_next_target',
            callback_group=self._cb_group
        )
        self.rotate_client = self.create_client(
            RotateHook, '/side_arm/rotate_hook',
            callback_group=self._cb_group
        )
        self.capture_client = self.create_client(
            CaptureLoops, '/capture_loops',
            callback_group=self._cb_group
        )

        # ==================== ACTION CLIENTS ====================
        self.hook_action_client = ActionClient(
            self, InsertHook, '/side_arm/insert_hook',
            callback_group=self._cb_group
        )
        self.arm_action_client = ActionClient(
            self, ExecuteTrajectory, '/main_arm/execute_trajectory',
            callback_group=self._cb_group
        )

        # ==================== SUBSCRIBERS ====================
        self.cmd_sub = self.create_subscription(
            String, '/stow/command',
            self._command_callback, 10
        )
        self.hook_status_sub = self.create_subscription(
            HookStatus, '/side_arm/status',
            self._hook_status_callback, 10
        )
        self.side_arm_state_sub = self.create_subscription(
            SideArmState, '/side_arm/parsed_state',
            self._side_arm_state_callback, 10
        )

        # ==================== PUBLISHERS ====================
        self.status_pub = self.create_publisher(String, '/stow/status', 10)
        self.side_arm_cmd_pub = self.create_publisher(String, '/side_arm/command', 10)
        self.status_timer = self.create_timer(1.0, self._publish_status)

        # ==================== SERVICE CHECK ====================
        self._check_services()

        self.get_logger().info('=' * 50)
        self.get_logger().info('PACKING COORDINATOR')
        self.get_logger().info(f'  State: {self.sm.state_name}')
        self.get_logger().info(f'  Pattern: {self.current_pattern}')
        self.get_logger().info(f'  Available: {self.pattern_manager.list_patterns()}')
        self.get_logger().info('  Publish to /stow/command to control')
        self.get_logger().info('=' * 50)

    # ================================================================
    #  SETUP
    # ================================================================

    def _check_services(self):
        """Check which services and actions are available (non-blocking)."""
        services = {
            '/request_next_target': self.target_client,
            '/side_arm/rotate_hook': self.rotate_client,
            '/capture_loops': self.capture_client,
        }
        actions = {
            '/side_arm/insert_hook': self.hook_action_client,
            '/main_arm/execute_trajectory': self.arm_action_client,
        }

        self.get_logger().info('Checking services...')
        for name, client in services.items():
            ready = client.wait_for_service(timeout_sec=2.0)
            self.get_logger().info(f'  {name}: {"✓" if ready else "✗"}')

        for name, client in actions.items():
            ready = client.wait_for_server(timeout_sec=2.0)
            self.get_logger().info(f'  {name}: {"✓" if ready else "✗"}')

    # ================================================================
    #  OPERATOR COMMANDS
    # ================================================================

    def _command_callback(self, msg: String):
        """Handle operator commands from /stow/command."""
        cmd = msg.data.strip().lower()
        state = self.sm.state

        self.get_logger().info(f'Command received: "{cmd}" (state: {state.name})')

        # Commands that work from any state
        if cmd == 'stop':
            self._halt_all()
            self.sm.reset()
            self.get_logger().info('Sequence stopped, reset to IDLE')
            return

        if cmd == 'home':
            self._halt_all()
            self.sm.transition('home')
            return

        if cmd.startswith('pattern:'):
            pattern_name = cmd.split(':', 1)[1]
            if self.pattern_manager.get_pattern(pattern_name):
                self.current_pattern = pattern_name
                self.get_logger().info(f'Pattern set to: {pattern_name}')
            else:
                self.get_logger().error(
                    f'Unknown pattern: {pattern_name}. '
                    f'Available: {self.pattern_manager.list_patterns()}'
                )
            return

        if cmd == 'status':
            self._log_status()
            return

        if cmd == 'patterns':
            self.get_logger().info(
                f'Available patterns: {self.pattern_manager.list_patterns()}'
            )
            return

        # State-specific commands — forward as events to the state machine
        if self.sm.can_transition(cmd):
            self.sm.transition(cmd)
        else:
            valid = self.sm.get_valid_events()
            self.get_logger().warn(
                f'"{cmd}" not valid in {state.name}. Valid: {valid}'
            )

    # ================================================================
    #  SUBSCRIBER CALLBACKS
    # ================================================================

    def _hook_status_callback(self, msg: HookStatus):
        """Monitor side arm hook status."""
        # Could be used for safety monitoring during HANDOFF/RETRACT
        pass

    def _side_arm_state_callback(self, msg: SideArmState):
        """Track side arm homing status."""
        self._side_arm_is_homed = msg.is_homed

    def _home_side_arm(self, timeout: float = 60.0) -> bool:
        """Home the side arm and wait for completion."""
        self.get_logger().info('Homing side arm (HOME_ALL)...')

        # Send HOME_ALL command
        cmd = String()
        cmd.data = 'HOME_ALL'
        self.side_arm_cmd_pub.publish(cmd)

        # Wait for is_homed to become True
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._side_arm_is_homed:
                self.get_logger().info(f'Side arm homed in {time.time() - start_time:.1f}s')
                self._has_homed_once = True
                return True

            # Log progress every 10 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 10 == 0 and int(elapsed) > 0:
                self.get_logger().info(f'  Still homing... ({elapsed:.0f}s)')

        self.get_logger().error(f'Side arm homing timeout after {timeout}s')
        return False

    # ================================================================
    #  STATUS PUBLISHING
    # ================================================================

    def _publish_status(self):
        """Publish current coordinator status."""
        msg = String()
        msg.data = (
            f'{self.sm.state_name}|'
            f'loop={self.completed_loops}/{self.total_loops}|'
            f'pattern={self.current_pattern}'
        )
        self.status_pub.publish(msg)

    def _log_status(self):
        """Log detailed status."""
        self.get_logger().info(f'  State: {self.sm.state_name}')
        self.get_logger().info(f'  Loop: {self.completed_loops}/{self.total_loops}')
        self.get_logger().info(f'  Pattern: {self.current_pattern}')
        self.get_logger().info(f'  Valid events: {self.sm.get_valid_events()}')
        if self.sm.state == StowState.ERROR:
            self.get_logger().info(f'  Error source: {self.sm.error_source}')
            self.get_logger().info(f'  Error: {self._error_message}')

    # ================================================================
    #  SAFETY
    # ================================================================

    def _halt_all(self):
        """Emergency stop — cancel active goals, halt arms."""
        self.get_logger().warn('HALTING ALL MOTION')

        # Cancel any active action goal
        if self._active_goal_handle is not None:
            try:
                self._active_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._active_goal_handle = None

        # TODO: Send stop commands to both arms
        # self._send_main_arm_stop()
        # self._send_side_arm_stop()

    def _enter_error(self, event: str, message: str):
        """Common error entry — halts motion, records diagnostics."""
        self._halt_all()
        self._error_message = message
        self.sm.transition(event)

    # ================================================================
    #  STATE HANDLERS — on_enter_<state>
    #
    #  Each method is called by the StateMachine when entering that
    #  state. The method kicks off async work (action goals, service
    #  calls). When the async work completes, the callback emits an
    #  event that triggers the next transition.
    # ================================================================

    def on_enter_idle(self, state: StowState, event: str):
        """Entered IDLE — system is ready."""
        self.current_target_loop = None
        self._active_goal_handle = None
        self.get_logger().info('System ready. Send "start" to begin.')

    def on_enter_homing(self, state: StowState, event: str):
        """Entered HOMING — home the side arm."""
        self.get_logger().info('[HOMING] Starting side arm homing sequence...')

        # Check if already homed (skip homing but still capture loops)
        if self._side_arm_is_homed and self._has_homed_once:
            self.get_logger().info('[HOMING] Side arm already homed, skipping to capture')
            self._capture_loops_after_homing()
            return

        # Send HOME_ALL command
        self.get_logger().info('[HOMING] Sending HOME_ALL command...')
        cmd = String()
        cmd.data = 'HOME_ALL'
        self.side_arm_cmd_pub.publish(cmd)

        # Start a timer to check homing status
        self._homing_start_time = time.time()
        self._homing_timer = self.create_timer(0.5, self._check_homing_status)

    def _check_homing_status(self):
        """Timer callback to check if homing is complete."""
        timeout = 60.0  # seconds

        if self._side_arm_is_homed:
            # Homing complete
            self._homing_timer.cancel()
            self._homing_timer = None
            self._has_homed_once = True
            elapsed = time.time() - self._homing_start_time
            self.get_logger().info(f'[HOMING] Side arm homed in {elapsed:.1f}s')

            # Capture loop positions after homing
            self._capture_loops_after_homing()
            return

        # Check timeout
        elapsed = time.time() - self._homing_start_time
        if elapsed >= timeout:
            self._homing_timer.cancel()
            self._homing_timer = None
            self.get_logger().error(f'[HOMING] Timeout after {timeout}s')
            self._enter_error('homing_failed', 'Side arm homing timeout')
            return

        # Log progress every 10 seconds
        if int(elapsed) % 10 == 0 and int(elapsed) > 0:
            self.get_logger().info(f'[HOMING] Still homing... ({elapsed:.0f}s)')

    def _capture_loops_after_homing(self):
        """Capture and lock loop positions after homing completes."""
        self.get_logger().info('[HOMING] Capturing loop positions...')

        if not self.capture_client.service_is_ready():
            self.get_logger().warn(
                '[HOMING] Capture service not available - proceeding without lock'
            )
            self.sm.transition('homed')
            return

        request = CaptureLoops.Request()
        request.expected_count = self.expected_loop_count
        request.timeout_sec = 2.0

        future = self.capture_client.call_async(request)
        future.add_done_callback(self._on_capture_response)

    def _on_capture_response(self, future):
        """Callback when loop capture completes."""
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'[HOMING] Capture service error: {e}')
            self._enter_error('homing_failed', f'Loop capture failed: {e}')
            return

        if response.success:
            self.total_loops = response.captured_count
            self.completed_loops = 0  # Reset for new sequence
            self.get_logger().info(
                f'[HOMING] Captured {response.captured_count} loops - ready to stow'
            )
            self.sm.transition('homed')
        else:
            self.get_logger().error(f'[HOMING] Capture failed: {response.message}')
            self._enter_error('homing_failed', response.message)

    def on_enter_at_loop(self, state: StowState, event: str):
        """Entered AT_LOOP — request next target from perception."""
        self.get_logger().info('[AT_LOOP] Requesting next target loop...')

        if not self.target_client.service_is_ready():
            self.get_logger().warn(
                '[AT_LOOP] Target service not available — '
                'using current target or waiting'
            )
            # TODO: Could retry or wait for service
            # For now, transition to error
            self._enter_error('vision_failure', 'Target service not available')
            return

        request = RequestNextTarget.Request()
        future = self.target_client.call_async(request)
        future.add_done_callback(self._on_target_response)

    def _on_target_response(self, future):
        """Callback when target selection completes."""
        try:
            response = future.result()
        except Exception as e:
            self._enter_error('vision_failure', f'Target service error: {e}')
            return

        if not response.target_available:
            self.get_logger().info('[AT_LOOP] No more targets available')
            self.sm.transition('no_targets')
            return

        self.current_target_loop = response.target_loop
        pos = self.current_target_loop.pose.pose.position
        self.get_logger().info(
            f'[AT_LOOP] Target: Loop {self.current_target_loop.loop_id} '
            f'at ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})'
        )

        # TODO: Command both arms to position at target
        # For now, go straight to INSERT (assumes arms are positioned)
        # Eventually:
        #   1. Command side arm to approach position
        #   2. Command main arm to stow-ready config
        #   3. Wait for both to report success
        #   4. Then transition 'positioned'

        self.sm.transition('positioned')

    def on_enter_insert(self, state: StowState, event: str):
        """Entered INSERT — send hook through the target loop."""
        retry = self.sm.retry_count
        if retry > 0:
            config = self.sm.get_state_config(StowState.INSERT)
            offset_mm = config.get('retry_offset_mm', 5.0)
            self.get_logger().info(
                f'[INSERT] Retry {retry} with ±{offset_mm}mm offset'
            )
            # TODO: Apply position offset to target loop

        self.get_logger().info('[INSERT] Inserting hook through loop...')

        if not self.hook_action_client.server_is_ready():
            self._enter_error('timeout', 'Hook action server not available')
            return

        goal = InsertHook.Goal()
        goal.target_loop = self.current_target_loop

        send_future = self.hook_action_client.send_goal_async(
            goal, feedback_callback=self._on_insert_feedback
        )
        send_future.add_done_callback(self._on_insert_goal_response)

    def _on_insert_feedback(self, feedback_msg):
        """Monitor hook insertion progress."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'  [INSERT] {int(feedback.progress * 100)}% - '
            f'state: {feedback.current_state}'
        )
        # TODO: Check for collision via motor current in feedback
        # if feedback.motor_current > COLLISION_THRESHOLD:
        #     self.sm.transition('collision')

    def _on_insert_goal_response(self, future):
        """Handle goal acceptance for hook insertion."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._enter_error('timeout', 'Hook insertion goal rejected')
            return

        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_insert_result)

    def _on_insert_result(self, future):
        """Handle hook insertion result."""
        self._active_goal_handle = None
        result = future.result().result

        if result.success:
            self.get_logger().info('[INSERT] Hook inserted successfully')
            # TODO: Verify depth with forward kinematics
            self.sm.transition('inserted')
        else:
            self.get_logger().warn(f'[INSERT] Failed: {result.message}')
            # Distinguish collision from other failures
            # TODO: Check result for collision flag
            self.sm.transition('collision')  # will retry with offset

    def on_enter_handoff(self, state: StowState, event: str):
        """Entered HANDOFF — rotate hook and execute stow trajectory."""
        self.get_logger().info('[HANDOFF] Rotating hook to 90°...')

        if not self.rotate_client.service_is_ready():
            self._enter_error('timeout', 'Rotate service not available')
            return

        request = RotateHook.Request()
        request.angle_degrees = 90.0

        future = self.rotate_client.call_async(request)
        future.add_done_callback(self._on_pre_stow_rotate_done)

    def _on_pre_stow_rotate_done(self, future):
        """After rotating hook, execute the stow trajectory."""
        try:
            response = future.result()
            if not response.success:
                self._enter_error('trajectory_failure', 'Pre-stow rotation failed')
                return
        except Exception as e:
            self._enter_error('trajectory_failure', f'Rotation error: {e}')
            return

        self.get_logger().info(
            f'[HANDOFF] Executing stow trajectory (pattern: {self.current_pattern})'
        )

        # Generate waypoints from pattern
        target_pos = self.current_target_loop.pose.pose.position
        waypoints = self.pattern_manager.apply_pattern(
            self.current_pattern, target_pos
        )

        if not waypoints:
            self._enter_error(
                'trajectory_failure',
                f'Pattern "{self.current_pattern}" produced no waypoints'
            )
            return

        self.get_logger().info(f'[HANDOFF] {len(waypoints)} waypoints generated')

        # Get speed from pattern config
        pattern = self.pattern_manager.get_pattern(self.current_pattern)
        speed_factor = pattern.speed_factor if pattern else 0.5

        # Send trajectory goal
        # TODO: Once main_arm accepts Point + pattern name instead of Pose,
        # switch to that interface and remove Pose construction from here
        if not self.arm_action_client.server_is_ready():
            self._enter_error('timeout', 'Arm action server not available')
            return

        goal = ExecuteTrajectory.Goal()
        goal.waypoints = waypoints
        goal.speed_factor = speed_factor

        send_future = self.arm_action_client.send_goal_async(
            goal, feedback_callback=self._on_trajectory_feedback
        )
        send_future.add_done_callback(self._on_trajectory_goal_response)

    def _on_trajectory_feedback(self, feedback_msg):
        """Monitor trajectory execution."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'  [HANDOFF] Trajectory: {int(feedback.progress * 100)}%'
        )
        # TODO: Check vision alignment
        # if not vision_aligned:
        #     self.sm.transition('alignment_lost')

    def _on_trajectory_goal_response(self, future):
        """Handle trajectory goal acceptance."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._enter_error('trajectory_failure', 'Trajectory goal rejected')
            return

        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_trajectory_result)

    def _on_trajectory_result(self, future):
        """Handle trajectory execution result."""
        self._active_goal_handle = None
        result = future.result().result

        if result.success:
            self.get_logger().info('[HANDOFF] Stow trajectory complete')
            self.sm.transition('trajectory_complete')
        else:
            self.get_logger().error(f'[HANDOFF] Trajectory failed: {result.message}')
            self._enter_error('trajectory_failure', result.message)

    def on_enter_retract(self, state: StowState, event: str):
        """Entered RETRACT — rotate hook again, then withdraw."""
        self.get_logger().info('[RETRACT] Rotating hook to capture line...')

        request = RotateHook.Request()
        request.angle_degrees = 90.0  # Second 90° rotation to capture

        future = self.rotate_client.call_async(request)
        future.add_done_callback(self._on_capture_rotate_done)

    def _on_capture_rotate_done(self, future):
        """After capture rotation, retract the hook."""
        try:
            response = future.result()
            if not response.success:
                self._enter_error('excessive_force', 'Capture rotation failed')
                return
        except Exception as e:
            self._enter_error('excessive_force', f'Rotation error: {e}')
            return

        self.get_logger().info('[RETRACT] Retracting hook...')

        # TODO: Send retraction command (reversed insertion path)
        # For now, rotate hook back to neutral as a placeholder
        request = RotateHook.Request()
        request.angle_degrees = 0.0

        future = self.rotate_client.call_async(request)
        future.add_done_callback(self._on_retract_done)

    def _on_retract_done(self, future):
        """Handle retraction completion."""
        try:
            response = future.result()
        except Exception as e:
            self._enter_error('excessive_force', f'Retract error: {e}')
            return

        self.get_logger().info('[RETRACT] Hook retracted')
        # TODO: Verify line position using vision
        self.sm.transition('retracted')

    def on_enter_release(self, state: StowState, event: str):
        """Entered RELEASE — verify stow and prepare for next loop."""
        self.get_logger().info('[RELEASE] Verifying stow quality...')

        # TODO: Vision verification of stow quality
        # For now, assume success

        self.completed_loops += 1
        self.get_logger().info(
            f'[RELEASE] Loop {self.completed_loops}/{self.total_loops} stowed'
        )

        # Reset for next cycle
        self.current_target_loop = None

        # TODO: Return arms to ready positions

        # Check if more loops remain
        if self.total_loops > 0 and self.completed_loops >= self.total_loops:
            self.sm.transition('all_complete')
        else:
            self.sm.transition('loops_remaining')

    def on_enter_complete(self, state: StowState, event: str):
        """Entered COMPLETE — all loops stowed."""
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'ALL {self.completed_loops} LOOPS STOWED')
        self.get_logger().info('=' * 50)
        # TODO: Return arms to home

    def on_enter_error(self, state: StowState, event: str):
        """Entered ERROR — halt all motion, log diagnostics, await operator."""
        self._halt_all()

        source = self.sm.error_source
        self.get_logger().error('=' * 50)
        self.get_logger().error(f'ERROR from {source.name if source else "unknown"}')
        self.get_logger().error(f'  {self._error_message}')
        self.get_logger().error('  Commands: retry | skip | abort')
        self.get_logger().error('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = PackingCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Coordinator interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()