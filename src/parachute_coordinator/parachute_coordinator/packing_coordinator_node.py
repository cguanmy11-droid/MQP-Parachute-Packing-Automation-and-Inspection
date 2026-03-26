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
        # Dual arm configuration - set both to True for alternating mode
        # Set only one to True for single-arm backwards compatibility
        self.declare_parameter('enable_left_arm', True)
        self.declare_parameter('enable_right_arm', True)

        self.test_mode = self.get_parameter('test_mode').value
        self.current_pattern = self.get_parameter('stow_pattern').value
        self.action_timeout = self.get_parameter('action_timeout').value
        self.expected_loop_count = self.get_parameter('expected_loop_count').value
        self.enable_left_arm = self.get_parameter('enable_left_arm').value
        self.enable_right_arm = self.get_parameter('enable_right_arm').value
        self.dual_arm_mode = self.enable_left_arm and self.enable_right_arm

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

        self.state_pub = self.create_publisher(String, '/coordinator/state', 10)
        self.error_pub = self.create_publisher(String, '/coordinator/error', 10)
        def _on_state_change(old, new):
            if self._paused:
                # Roll back — don't allow transitions while paused
                self.get_logger().warn(f'Transition {old}→{new} blocked (paused)')
                self.sm.current_state = self.sm._states[old]  # revert
                return
            self.state_pub.publish(String(data=new))

        self.sm.on_transition = _on_state_change
        self._paused = False
        self._pending_event = None
        self.state_pub.publish(String(data=self.sm.state_name))

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

        # Dual arm tracking - alternates between 'left' and 'right'
        # Start with whichever arm is enabled (prefer left if both)
        if self.enable_left_arm:
            self.current_arm = 'left'
        elif self.enable_right_arm:
            self.current_arm = 'right'
        else:
            self.get_logger().error('No arms enabled! Set enable_left_arm or enable_right_arm')
            self.current_arm = 'left'  # Fallback
        self._left_arm_homed = False
        self._right_arm_homed = False

        # Side arm homing tracking (legacy, now per-arm)
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
        self.capture_client = self.create_client(
            CaptureLoops, '/capture_loops',
            callback_group=self._cb_group
        )

        # Dual arm rotate hook clients (only create for enabled arms)
        self.left_rotate_client = None
        self.right_rotate_client = None
        if self.enable_left_arm:
            self.left_rotate_client = self.create_client(
                RotateHook, '/side_arm_left/rotate_hook',
                callback_group=self._cb_group
            )
        if self.enable_right_arm:
            self.right_rotate_client = self.create_client(
                RotateHook, '/side_arm_right/rotate_hook',
                callback_group=self._cb_group
            )
        # Legacy alias for compatibility (use whichever is available)
        self.rotate_client = self.left_rotate_client or self.right_rotate_client

        # ==================== ACTION CLIENTS ====================
        # Dual arm insert hook clients (only create for enabled arms)
        self.left_hook_client = None
        self.right_hook_client = None
        if self.enable_left_arm:
            self.left_hook_client = ActionClient(
                self, InsertHook, '/side_arm_left/insert_hook',
                callback_group=self._cb_group
            )
        if self.enable_right_arm:
            self.right_hook_client = ActionClient(
                self, InsertHook, '/side_arm_right/insert_hook',
                callback_group=self._cb_group
            )
        # Legacy alias for compatibility (use whichever is available)
        self.hook_action_client = self.left_hook_client or self.right_hook_client

        self.arm_action_client = ActionClient(
            self, ExecuteTrajectory, '/main_arm/execute_trajectory',
            callback_group=self._cb_group
        )

        # ==================== SUBSCRIBERS ====================
        self.cmd_sub = self.create_subscription(
            String, '/stow/command',
            self._command_callback, 10
        )
        # Dual arm status subscribers (only subscribe to enabled arms)
        if self.enable_left_arm:
            self.left_hook_status_sub = self.create_subscription(
                HookStatus, '/side_arm_left/status',
                lambda msg: self._hook_status_callback(msg, 'left'), 10
            )
            self.left_state_sub = self.create_subscription(
                SideArmState, '/side_arm_left/parsed_state',
                lambda msg: self._side_arm_state_callback(msg, 'left'), 10
            )
        if self.enable_right_arm:
            self.right_hook_status_sub = self.create_subscription(
                HookStatus, '/side_arm_right/status',
                lambda msg: self._hook_status_callback(msg, 'right'), 10
            )
            self.right_state_sub = self.create_subscription(
                SideArmState, '/side_arm_right/parsed_state',
                lambda msg: self._side_arm_state_callback(msg, 'right'), 10
            )

        # ==================== PUBLISHERS ====================
        self.status_pub = self.create_publisher(String, '/stow/status', 10)
        # Dual arm command publishers (only create for enabled arms)
        self.left_cmd_pub = None
        self.right_cmd_pub = None
        if self.enable_left_arm:
            self.left_cmd_pub = self.create_publisher(String, '/side_arm_left/command', 10)
        if self.enable_right_arm:
            self.right_cmd_pub = self.create_publisher(String, '/side_arm_right/command', 10)
        # Current arm publisher (for GUI)
        self.current_arm_pub = self.create_publisher(String, '/coordinator/current_arm', 10)
        self.status_timer = self.create_timer(1.0, self._publish_status)

        # ==================== SERVICE CHECK ====================
        self._check_services()

        arm_mode = 'DUAL (alternating)' if self.dual_arm_mode else f'SINGLE ({self.current_arm})'
        self.get_logger().info('=' * 50)
        self.get_logger().info('PACKING COORDINATOR')
        self.get_logger().info(f'  State: {self.sm.state_name}')
        self.get_logger().info(f'  Arm mode: {arm_mode}')
        self.get_logger().info(f'  Pattern: {self.current_pattern}')
        self.get_logger().info(f'  Available: {self.pattern_manager.list_patterns()}')
        self.get_logger().info('  Publish to /stow/command to control')
        self.get_logger().info('=' * 50)

    # ================================================================
    #  SETUP
    # ================================================================

    def _check_services(self):
        """Check which services and actions are available (non-blocking)."""
        services = {'/request_next_target': self.target_client}
        if self.left_rotate_client:
            services['/side_arm_left/rotate_hook'] = self.left_rotate_client
        if self.right_rotate_client:
            services['/side_arm_right/rotate_hook'] = self.right_rotate_client
        services['/capture_loops'] = self.capture_client

        actions = {}
        if self.left_hook_client:
            actions['/side_arm_left/insert_hook'] = self.left_hook_client
        if self.right_hook_client:
            actions['/side_arm_right/insert_hook'] = self.right_hook_client
        actions['/main_arm/execute_trajectory'] = self.arm_action_client

        mode = 'dual arm' if self.dual_arm_mode else f'single arm ({self.current_arm})'
        self.get_logger().info(f'Checking services ({mode} mode)...')
        for name, client in services.items():
            if client:
                ready = client.wait_for_service(timeout_sec=2.0)
                self.get_logger().info(f'  {name}: {"✓" if ready else "✗"}')

        for name, client in actions.items():
            if client:
                ready = client.wait_for_server(timeout_sec=2.0)
                self.get_logger().info(f'  {name}: {"✓" if ready else "✗"}')

    # ================================================================
    #  DUAL ARM HELPERS
    # ================================================================

    def get_current_hook_client(self) -> ActionClient:
        """Get the insert_hook action client for the current arm."""
        if self.current_arm == 'left' and self.left_hook_client:
            return self.left_hook_client
        elif self.current_arm == 'right' and self.right_hook_client:
            return self.right_hook_client
        # Fallback to whichever is available
        return self.left_hook_client or self.right_hook_client

    def get_current_rotate_client(self):
        """Get the rotate_hook service client for the current arm."""
        if self.current_arm == 'left' and self.left_rotate_client:
            return self.left_rotate_client
        elif self.current_arm == 'right' and self.right_rotate_client:
            return self.right_rotate_client
        # Fallback to whichever is available
        return self.left_rotate_client or self.right_rotate_client

    def get_current_cmd_pub(self):
        """Get the command publisher for the current arm."""
        if self.current_arm == 'left' and self.left_cmd_pub:
            return self.left_cmd_pub
        elif self.current_arm == 'right' and self.right_cmd_pub:
            return self.right_cmd_pub
        # Fallback to whichever is available
        return self.left_cmd_pub or self.right_cmd_pub

    def switch_arm(self):
        """Switch to the other arm for the next operation (only in dual arm mode)."""
        if not self.dual_arm_mode:
            # Single arm mode - no switching
            return
        old_arm = self.current_arm
        self.current_arm = 'right' if self.current_arm == 'left' else 'left'
        self.get_logger().info(f'Switched arm: {old_arm} → {self.current_arm}')
        self.current_arm_pub.publish(String(data=self.current_arm))

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
            self._transition('home')
            return
        
        if cmd == 'pause':
            self._paused = True
            self.get_logger().info('Sequence PAUSED by operator')
            self.state_pub.publish(String(data=f'{self.sm.state_name} (PAUSED)'))
            return

        if cmd == 'resume':
            self._paused = False
            self.get_logger().info('Sequence RESUMED')
            self.state_pub.publish(String(data=self.sm.state_name))
            # Replay any event that was blocked while paused
            if self._pending_event:
                event = self._pending_event
                self._pending_event = None
                self.get_logger().info(f'Replaying queued event "{event}"')
                self.sm.transition(event)
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
            self._transition(cmd)
        else:
            valid = self.sm.get_valid_events()
            self.get_logger().warn(
                f'"{cmd}" not valid in {state.name}. Valid: {valid}'
            )

    # ================================================================
    #  SUBSCRIBER CALLBACKS
    # ================================================================

    def _hook_status_callback(self, msg: HookStatus, arm: str):
        """Monitor side arm hook status for specified arm."""
        # Could be used for safety monitoring during HANDOFF/RETRACT
        pass

    def _side_arm_state_callback(self, msg: SideArmState, arm: str):
        """Track side arm homing status for specified arm."""
        if arm == 'left':
            self._left_arm_homed = msg.is_homed
        else:
            self._right_arm_homed = msg.is_homed
        # Legacy compatibility
        self._side_arm_is_homed = self._left_arm_homed and self._right_arm_homed

    def _home_both_arms(self, timeout: float = 60.0) -> bool:
        """Home both side arms and wait for completion."""
        self.get_logger().info('Homing both side arms (HOME_ALL)...')

        # Send HOME_ALL command to both arms
        cmd = String()
        cmd.data = 'HOME_ALL'
        self.left_cmd_pub.publish(cmd)
        self.right_cmd_pub.publish(cmd)

        # Wait for both arms to be homed
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._left_arm_homed and self._right_arm_homed:
                self.get_logger().info(f'Both arms homed in {time.time() - start_time:.1f}s')
                self._has_homed_once = True
                return True

            # Log progress every 10 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 10 == 0 and int(elapsed) > 0:
                left_status = '✓' if self._left_arm_homed else '...'
                right_status = '✓' if self._right_arm_homed else '...'
                self.get_logger().info(f'  Homing: left={left_status}, right={right_status} ({elapsed:.0f}s)')

        self.get_logger().error(f'Arm homing timeout after {timeout}s')
        return False

    def _home_side_arm(self, timeout: float = 60.0) -> bool:
        """Home the current side arm only (for single-arm fallback)."""
        arm = self.current_arm
        self.get_logger().info(f'Homing {arm} arm (HOME_ALL)...')

        cmd = String()
        cmd.data = 'HOME_ALL'
        self.get_current_cmd_pub().publish(cmd)

        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            is_homed = self._left_arm_homed if arm == 'left' else self._right_arm_homed
            if is_homed:
                self.get_logger().info(f'{arm.capitalize()} arm homed in {time.time() - start_time:.1f}s')
                return True

            elapsed = time.time() - start_time
            if int(elapsed) % 10 == 0 and int(elapsed) > 0:
                self.get_logger().info(f'  Still homing {arm}... ({elapsed:.0f}s)')

        self.get_logger().error(f'{arm.capitalize()} arm homing timeout after {timeout}s')
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
        self._transition(event)

    # ================================================================
    #  STATE HANDLERS — on_enter_<state>
    #
    #  Each method is called by the StateMachine when entering that
    #  state. The method kicks off async work (action goals, service
    #  calls). When the async work completes, the callback emits an
    #  event that triggers the next transition.
    # ================================================================

    def _transition(self, event: str) -> bool:
        if self._paused:
            self._pending_event = event   # store it
            self.get_logger().info(f'Paused — queued transition "{event}"')
            return False
        self._pending_event = None
        return self.sm.transition(event)

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

        # Send HOME_ALL command to current arm
        self.get_logger().info(f'[HOMING] Sending HOME_ALL command to {self.current_arm} arm...')
        cmd = String()
        cmd.data = 'HOME_ALL'
        cmd_pub = self.get_current_cmd_pub()
        if cmd_pub:
            cmd_pub.publish(cmd)
        else:
            self.get_logger().error('[HOMING] No command publisher available!')
            self._transition('error')
            return

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
            self._transition('homed')
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
            self._transition('homed')
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
            self._transition('no_targets')
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

        self._transition('positioned')

    def on_enter_insert(self, state: StowState, event: str):
        """Entered INSERT — send hook through the target loop using current arm."""
        retry = self.sm.retry_count
        if retry > 0:
            config = self.sm.get_state_config(StowState.INSERT)
            offset_mm = config.get('retry_offset_mm', 5.0)
            self.get_logger().info(
                f'[INSERT] Retry {retry} with ±{offset_mm}mm offset'
            )
            # TODO: Apply position offset to target loop

        self.get_logger().info(f'[INSERT] Using {self.current_arm} arm to insert hook...')

        hook_client = self.get_current_hook_client()
        if not hook_client.server_is_ready():
            self._enter_error('timeout', f'{self.current_arm} hook action server not available')
            return

        goal = InsertHook.Goal()
        goal.target_loop = self.current_target_loop

        send_future = hook_client.send_goal_async(
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
        #     self._transition('collision')

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
            self._transition('inserted')
        else:
            self.get_logger().warn(f'[INSERT] Failed: {result.message}')
            # Distinguish collision from other failures
            # TODO: Check result for collision flag
            self._transition('collision')  # will retry with offset

    def on_enter_handoff(self, state: StowState, event: str):
        """Entered HANDOFF — rotate hook and execute stow trajectory using current arm."""
        self.get_logger().info(f'[HANDOFF] Rotating {self.current_arm} hook to 90°...')

        rotate_client = self.get_current_rotate_client()
        if not rotate_client.service_is_ready():
            self._enter_error('timeout', f'{self.current_arm} rotate service not available')
            return

        request = RotateHook.Request()
        request.angle_degrees = 90.0

        future = rotate_client.call_async(request)
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
        #     self._transition('alignment_lost')

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
            self._transition('trajectory_complete')
        else:
            self.get_logger().error(f'[HANDOFF] Trajectory failed: {result.message}')
            self._enter_error('trajectory_failure', result.message)

    def on_enter_retract(self, state: StowState, event: str):
        """Entered RETRACT — rotate hook again, then withdraw using current arm."""
        self.get_logger().info(f'[RETRACT] Rotating {self.current_arm} hook to capture line...')

        rotate_client = self.get_current_rotate_client()
        request = RotateHook.Request()
        request.angle_degrees = 90.0  # Second 90° rotation to capture

        future = rotate_client.call_async(request)
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

        self.get_logger().info(f'[RETRACT] Retracting {self.current_arm} hook...')

        # TODO: Send retraction command (reversed insertion path)
        # For now, rotate hook back to neutral as a placeholder
        rotate_client = self.get_current_rotate_client()
        request = RotateHook.Request()
        request.angle_degrees = 0.0

        future = rotate_client.call_async(request)
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
        self._transition('retracted')

    def on_enter_release(self, state: StowState, event: str):
        """Entered RELEASE — verify stow and prepare for next loop."""
        self.get_logger().info(f'[RELEASE] {self.current_arm} arm: Verifying stow quality...')

        # TODO: Vision verification of stow quality
        # For now, assume success

        self.completed_loops += 1
        self.get_logger().info(
            f'[RELEASE] Loop {self.completed_loops}/{self.total_loops} stowed by {self.current_arm} arm'
        )

        # Reset for next cycle
        self.current_target_loop = None

        # Switch to the other arm for the next loop (alternating pattern)
        self.switch_arm()

        # TODO: Return arms to ready positions

        # Check if more loops remain
        if self.total_loops > 0 and self.completed_loops >= self.total_loops:
            self._transition('all_complete')
        else:
            self._transition('loops_remaining')

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

        self.error_pub.publish(String(data=msg))

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