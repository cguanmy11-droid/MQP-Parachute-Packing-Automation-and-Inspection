#!/usr/bin/env python3
"""
Main Arm Planner Node
Handles advanced motion planning with coordinates, trajectories, and joint angles
Provides smooth motion planning and execution
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import JointState
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np


# Auto-sequence target end-effector pose.
# Tune these values directly instead of guessing joint angles.
# For this 5-DOF arm, yaw is derived from x/y, so we only command roll + pitch here.
AUTO_SEQ_TARGET_X_M = 0.4508
AUTO_SEQ_TARGET_Y_M = -0.0270
AUTO_SEQ_TARGET_Z_M = 0.0506
AUTO_SEQ_TARGET_ROLL_DEG = -0.88
AUTO_SEQ_TARGET_PITCH_DEG = 69.22

# Final return pose after gripping.
AUTO_SEQ_RETURN_X_M = 0.3015
AUTO_SEQ_RETURN_Y_M = -0.0016
AUTO_SEQ_RETURN_Z_M = 0.0946
AUTO_SEQ_RETURN_ROLL_DEG = -0.26
AUTO_SEQ_RETURN_PITCH_DEG = 84.02
# Gripper left-finger position to use at HOME pose before moving to the target pose [m]
AUTO_SEQ_HOME_OPEN_FINGER_M = 0.03797
# Final gripper left-finger position after reaching the target pose [m]
AUTO_SEQ_TARGET_FINGER_M = 0.01766

# ----------------------------------------------------------------------
# Hole-center sequence (left / right hole)
# ----------------------------------------------------------------------
# 4 EE waypoints expressed in the HOLE-CENTER frame.
# When the sequence is triggered with a (hole_center pose in base frame),
# each waypoint is transformed back to the base frame and executed in order.
#
#   dx, dy, dz are translations in the hole-center frame [m].
#   droll, dpitch, dyaw are intrinsic XYZ Euler angles in the hole-center
#   frame [deg]. The full waypoint orientation in the base frame is:
#       R_wp_base = R_holecenter_base @ R_rel
#   finger_m is the desired left-finger position [m] at this waypoint
#   (total opening = 2 * finger_m). Set to None to leave the gripper
#   untouched at that waypoint.
#
# LEFT_HOLE_WAYPOINTS are the originally-measured 4 poses.
# RIGHT_HOLE_WAYPOINTS are derived by mirroring LEFT across the local
# XZ-plane of the hole-center frame: dy, droll, dyaw all flip sign;
# dx, dz, dpitch and finger_m stay the same. This is the standard
# left/right mirror for a symmetric workpiece.
#dx height

LEFT_HOLE_WAYPOINTS = [
    {  # pose 4
        'dx': +0.0151, 'dy': -0.0895, 'dz': +0.0623,
        'droll_deg': +26.35, 'dpitch_deg': -21.76, 'dyaw_deg': -0.54,
        'finger_m': 0.01559,
    },
    {  # pose 3
        'dx': -0.010, 'dy': +0.01, 'dz': +0.0968,
        'droll_deg': -47.22, 'dpitch_deg': -65.13, 'dyaw_deg': +25.84,
        'finger_m': 0.01357,
    },
    {  # pose 2
        'dx': +0.0050, 'dy': +0.0, 'dz': -0.0507,
        'droll_deg': -57.01, 'dpitch_deg': -60.61, 'dyaw_deg': +26.36,
        'finger_m': 0.01657,
    },
    {  # pose 1
        'dx': -0.0367, 'dy': -0.0486, 'dz': -0.0653,
        'droll_deg': 60.03, 'dpitch_deg': -10, 'dyaw_deg': -0.62,
        'finger_m': 0.01657,
    },
    {  # pose 2nd to last
        'dx': -0.0567, 'dy': -0.1586, 'dz': +0.0053,
        'droll_deg': 60.03, 'dpitch_deg': -10, 'dyaw_deg': -0.62,
        'finger_m': 0.01657,
    },
    {  # pose last
        'dx': -0.0467, 'dy': -0.1286, 'dz': +0.0053,
        'droll_deg': 60.03, 'dpitch_deg': -40, 'dyaw_deg': -0.62,
        'finger_m': 0.01657,
    },
]


def _mirror_waypoint_lr(wp: dict) -> dict:
    """Mirror a hole-center-frame waypoint across the local XZ-plane (left<->right)."""
    return {
        'dx':         +wp['dx'],
        'dy':         -wp['dy'],
        'dz':         +wp['dz'],
        'droll_deg':  -wp['droll_deg'],
        'dpitch_deg': +wp['dpitch_deg'],
        'dyaw_deg':   -wp['dyaw_deg'],
        'finger_m':    wp.get('finger_m', None),
    }


# Right-hole waypoints, auto-mirrored from LEFT_HOLE_WAYPOINTS.
# Override individual entries here if the right hole needs its own tuning.
RIGHT_HOLE_WAYPOINTS = [_mirror_waypoint_lr(w) for w in LEFT_HOLE_WAYPOINTS]

# Backward-compat alias for older code that still references HOLE_SEQ_WAYPOINTS.
HOLE_SEQ_WAYPOINTS = LEFT_HOLE_WAYPOINTS


class MainArmPlannerNode(Node):
    def __init__(self):
        super().__init__('main_arm_planner_node')
        
        # Parameters
        self.declare_parameter('robot_model', 'wx200')
        self.declare_parameter('robot_name', 'wx200')
        robot_model = self.get_parameter('robot_model').value
        robot_name = self.get_parameter('robot_name').value
        
        # Initialize robot
        self.get_logger().info(f'Initializing {robot_model} arm planner...')
        self.bot = InterbotixManipulatorXS(
            robot_model=robot_model,
            robot_name=robot_name,
            moving_time=3.0,
            accel_time=1.0
        )
        self._gripper_linear_mode_ready = False
        self._ensure_gripper_linear_position_mode()
        
        # Subscribers for different command types
        self.point_sub = self.create_subscription(Point, '/main_arm/target_point', self.target_point_callback, 10)
        
        self.pose_sub = self.create_subscription(Pose, '/main_arm/target_pose', self.target_pose_callback, 10)
        
        self.joint_angles_sub = self.create_subscription(JointState, '/main_arm/target_joint_angles', self.target_joint_angles_callback, 10)

        # Trigger for the automatic sequence (sleep -> home -> target -> grip)
        self.auto_seq_sub = self.create_subscription(
            Empty, '/main_arm/run_auto_sequence', self.auto_sequence_callback, 10)

        # Trigger the hole-center waypoint sequence. The Pose payload is the
        # hole center pose expressed in the main-arm base frame.
        # /main_arm/hole_center_sequence is kept as an alias for the LEFT hole.
        self.hole_seq_sub = self.create_subscription(
            Pose, '/main_arm/hole_center_sequence',
            lambda m: self.hole_center_sequence_callback(m, side='left'), 10)
        self.left_hole_seq_sub = self.create_subscription(
            Pose, '/main_arm/left_hole_center_sequence',
            lambda m: self.hole_center_sequence_callback(m, side='left'), 10)
        self.right_hole_seq_sub = self.create_subscription(
            Pose, '/main_arm/right_hole_center_sequence',
            lambda m: self.hole_center_sequence_callback(m, side='right'), 10)

        # Publishers
        self.status_pub = self.create_publisher(String, '/main_arm/planner_status', 10)

        self.current_joints_pub = self.create_publisher(JointState, '/main_arm/current_joint_angles', 10)

        self._auto_seq_thread = None
        self._auto_seq_lock = threading.Lock()
        
        # Workspace limits (meters)
        self.workspace_limits = {
            'x': (0.1, 0.45),
            'y': (-0.3, 0.3),
            'z': (0.05, 0.4)
        }
        
        # Timer for publishing current state
        self.create_timer(0.1, self.publish_current_state)
        
        self.get_logger().info('Main Arm Planner Node ready!')
        self.get_logger().info('Subscribed to:')
        self.get_logger().info('  - /main_arm/target_point (Point: x,y,z)')
        self.get_logger().info('  - /main_arm/target_pose (Pose: position + orientation)')
        self.get_logger().info('  - /main_arm/target_joint_angles (JointState)')
        self.get_logger().info('  - /main_arm/run_auto_sequence (Empty: trigger auto demo sequence)')
        self.get_logger().info('  - /main_arm/hole_center_sequence (Pose: alias of left_hole_center_sequence)')
        self.get_logger().info('  - /main_arm/left_hole_center_sequence (Pose: hole-center pose in base frame, LEFT waypoints)')
        self.get_logger().info('  - /main_arm/right_hole_center_sequence (Pose: hole-center pose in base frame, RIGHT waypoints, mirrored)')
    
    def target_point_callback(self, msg):
        """Move end-effector to target XYZ point"""
        x, y, z = msg.x, msg.y, msg.z
        self.get_logger().info(f'Moving to point: ({x:.3f}, {y:.3f}, {z:.3f})')
        
        # Check workspace limits
        if not self.check_workspace_limits(x, y, z):
            self.get_logger().error('Target point outside workspace limits!')
            self.publish_status_msg('error: out_of_workspace')
            return
        
        try:
            self.publish_status_msg('planning')
            
            # Move to target point (maintain current orientation)
            success = self.bot.arm.set_ee_pose_components(x=x, y=y, z=z)
            
            if success:
                self.get_logger().info('Successfully reached target point')
                self.publish_status_msg('success: point_reached')
            else:
                self.get_logger().warn('Could not reach target point (IK failed)')
                self.publish_status_msg('warning: ik_failed')
                
        except Exception as e:
            self.get_logger().error(f'Failed to move to point: {e}')
            self.publish_status_msg('error: motion_failed')
    
    def target_pose_callback(self, msg):
        """Move end-effector to target pose (position + orientation)"""
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        
        # Extract orientation (quaternion)
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        self.get_logger().info(f'Moving to pose: pos({x:.3f}, {y:.3f}, {z:.3f})')
        
        if not self.check_workspace_limits(x, y, z):
            self.get_logger().error('Target pose outside workspace limits!')
            self.publish_status_msg('error: out_of_workspace')
            return
        
        try:
            self.publish_status_msg('planning')
            
            # Convert quaternion to roll, pitch, yaw
            roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
            
            # Move to target pose
            success = self.bot.arm.set_ee_pose_components(
                x=x, y=y, z=z,
                roll=roll, pitch=pitch
            )
            
            if success:
                self.get_logger().info('Successfully reached target pose')
                self.publish_status_msg('success: pose_reached')
            else:
                self.get_logger().warn('Could not reach target pose (IK failed)')
                self.publish_status_msg('warning: ik_failed')
                
        except Exception as e:
            self.get_logger().error(f'Failed to move to pose: {e}')
            self.publish_status_msg('error: motion_failed')
    
    def target_joint_angles_callback(self, msg):
        """Move to target joint angles directly"""
        joint_positions = list(msg.position)
        
        self.get_logger().info(f'Moving to joint angles: {[f"{j:.3f}" for j in joint_positions]}')
        
        try:
            self.publish_status_msg('executing')
            
            # Move joints to target positions
            self.bot.arm.set_joint_positions(joint_positions)
            
            self.get_logger().info('Successfully reached target joint angles')
            self.publish_status_msg('success: joints_reached')
            
        except Exception as e:
            self.get_logger().error(f'Failed to move joints: {e}')
            self.publish_status_msg('error: joint_motion_failed')
    
    def auto_sequence_callback(self, msg):
        """Trigger the automatic sleep -> home -> target -> grip sequence.

        We run it in a background thread so this callback (and rclpy.spin) are not blocked.
        """
        with self._auto_seq_lock:
            if self._auto_seq_thread is not None and self._auto_seq_thread.is_alive():
                self.get_logger().warn('Auto sequence already running, ignoring trigger.')
                return
            self._auto_seq_thread = threading.Thread(
                target=self.run_auto_sequence, daemon=True)
            self._auto_seq_thread.start()

    def run_auto_sequence(self):
        """
        Auto demo sequence:
            1. Go to SLEEP pose (regardless of current pose).
            2. Go to HOME pose and open the gripper to a fixed finger position.
            3. Move to a fixed end-effector pose (the "test" pose).
            4. Close the gripper until the left finger reaches AUTO_SEQ_TARGET_FINGER_M.
            5. Move to the final return pose.
        """
        try:
            self.get_logger().info('=== Auto sequence: START ===')
            self.publish_status_msg('auto_seq: start')

            self.get_logger().info('[1/5] Going to SLEEP pose...')
            self.bot.arm.go_to_sleep_pose()

            self.get_logger().info(
                '[2/5] Going to HOME pose and opening gripper to '
                f'finger={AUTO_SEQ_HOME_OPEN_FINGER_M * 1000:.2f} mm '
                f'(opening={2.0 * AUTO_SEQ_HOME_OPEN_FINGER_M * 1000:.2f} mm)...')
            self.bot.arm.go_to_home_pose()
            self._set_gripper_finger_position(AUTO_SEQ_HOME_OPEN_FINGER_M)
            time.sleep(0.5)

            self.get_logger().info(
                '[3/5] Moving to target EE pose: '
                f'x={AUTO_SEQ_TARGET_X_M:.4f}, '
                f'y={AUTO_SEQ_TARGET_Y_M:.4f}, '
                f'z={AUTO_SEQ_TARGET_Z_M:.4f}, '
                f'roll={AUTO_SEQ_TARGET_ROLL_DEG:.2f} deg, '
                f'pitch={AUTO_SEQ_TARGET_PITCH_DEG:.2f} deg')
            success = self.bot.arm.set_ee_pose_components(
                x=AUTO_SEQ_TARGET_X_M,
                y=AUTO_SEQ_TARGET_Y_M,
                z=AUTO_SEQ_TARGET_Z_M,
                roll=float(np.deg2rad(AUTO_SEQ_TARGET_ROLL_DEG)),
                pitch=float(np.deg2rad(AUTO_SEQ_TARGET_PITCH_DEG)),
            )
            if not success:
                raise RuntimeError('Failed to reach auto-sequence target EE pose (IK failed).')
            time.sleep(0.3)

            self.get_logger().info(
                f'[4/5] Closing gripper to finger position '
                f'{AUTO_SEQ_TARGET_FINGER_M * 1000:.2f} mm...')
            self._set_gripper_finger_position(AUTO_SEQ_TARGET_FINGER_M)

            self.get_logger().info(
                '[5/5] Moving to final return pose: '
                f'x={AUTO_SEQ_RETURN_X_M:.4f}, '
                f'y={AUTO_SEQ_RETURN_Y_M:.4f}, '
                f'z={AUTO_SEQ_RETURN_Z_M:.4f}, '
                f'roll={AUTO_SEQ_RETURN_ROLL_DEG:.2f} deg, '
                f'pitch={AUTO_SEQ_RETURN_PITCH_DEG:.2f} deg')
            success = self.bot.arm.set_ee_pose_components(
                x=AUTO_SEQ_RETURN_X_M,
                y=AUTO_SEQ_RETURN_Y_M,
                z=AUTO_SEQ_RETURN_Z_M,
                roll=float(np.deg2rad(AUTO_SEQ_RETURN_ROLL_DEG)),
                pitch=float(np.deg2rad(AUTO_SEQ_RETURN_PITCH_DEG)),
            )
            if not success:
                raise RuntimeError('Failed to reach auto-sequence final return pose (IK failed).')

            self.get_logger().info('=== Auto sequence: DONE ===')
            self.publish_status_msg('auto_seq: done')

        except Exception as e:
            self.get_logger().error(f'Auto sequence failed: {e}')
            self.publish_status_msg(f'auto_seq: error: {e}')
    
    # ------------------------------------------------------------------
    # Hole-center waypoint sequence
    # ------------------------------------------------------------------
    @staticmethod
    def _rpy_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Intrinsic XYZ Euler (roll-pitch-yaw, all in rad) -> 3x3 rotation."""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    @staticmethod
    def _R_to_rpy(R: np.ndarray):
        """3x3 rotation -> roll, pitch, yaw [rad] (intrinsic XYZ)."""
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
        yaw = np.arctan2(R[1, 0], R[0, 0])
        return float(roll), float(pitch), float(yaw)

    @staticmethod
    def _quat_to_rpy(qx: float, qy: float, qz: float, qw: float):
        """Quaternion -> roll, pitch, yaw [rad] (intrinsic XYZ)."""
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (qw * qy - qz * qx)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = np.arcsin(sinp)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return float(roll), float(pitch), float(yaw)

    def hole_center_sequence_callback(self, msg: Pose, side: str = 'left'):
        """
        Trigger the 4-waypoint sequence relative to a hole center.

        msg.position    -> hole center XYZ in main-arm base frame [m]
        msg.orientation -> hole center orientation as quaternion in base frame
        side            -> 'left' or 'right'; selects which waypoint set to use.
        """
        roll, pitch, yaw = self._quat_to_rpy(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w)

        # If user publishes a default/zero quaternion (w=0), fall back to identity.
        norm = (msg.orientation.x ** 2 + msg.orientation.y ** 2 +
                msg.orientation.z ** 2 + msg.orientation.w ** 2)
        if norm < 1e-6:
            self.get_logger().warn(
                'Hole-center quaternion is zero, using identity orientation.')
            roll = pitch = yaw = 0.0

        with self._auto_seq_lock:
            if self._auto_seq_thread is not None and self._auto_seq_thread.is_alive():
                self.get_logger().warn(
                    'Another sequence is running, ignoring hole-center trigger.')
                return
            self._auto_seq_thread = threading.Thread(
                target=self.run_hole_center_sequence,
                kwargs=dict(
                    hc_x=msg.position.x,
                    hc_y=msg.position.y,
                    hc_z=msg.position.z,
                    hc_roll_deg=float(np.degrees(roll)),
                    hc_pitch_deg=float(np.degrees(pitch)),
                    hc_yaw_deg=float(np.degrees(yaw)),
                    side=side,
                ),
                daemon=True,
            )
            self._auto_seq_thread.start()

    def run_hole_center_sequence(
        self,
        hc_x: float,
        hc_y: float,
        hc_z: float,
        hc_roll_deg: float,
        hc_pitch_deg: float,
        hc_yaw_deg: float,
        waypoints=None,
        side: str = 'left',
    ) -> bool:
        """
        Move the EE through the 4 waypoints defined in the HOLE-CENTER frame.

        If `waypoints` is None, choose LEFT_HOLE_WAYPOINTS or RIGHT_HOLE_WAYPOINTS
        based on `side` ('left' or 'right'). Each waypoint dict has keys:
        dx, dy, dz, droll_deg, dpitch_deg, dyaw_deg, finger_m, all expressed in
        the hole-center frame. They are transformed back to the base frame here.

        For this 5-DOF arm only roll + pitch are commanded (yaw is implicit
        from the waist angle), so the resulting yaw of each waypoint is
        recomputed by the IK from x/y.
        """
        if waypoints is None:
            side_lc = (side or 'left').lower()
            if side_lc == 'right':
                wps = RIGHT_HOLE_WAYPOINTS
            elif side_lc == 'left':
                wps = LEFT_HOLE_WAYPOINTS
            else:
                self.get_logger().warn(
                    f"Unknown side '{side}', defaulting to LEFT.")
                side_lc = 'left'
                wps = LEFT_HOLE_WAYPOINTS
        else:
            side_lc = (side or 'custom').lower()
            wps = waypoints

        try:
            self.get_logger().info(
                f'=== Hole-center sequence ({side_lc.upper()}): START ('
                f'hc xyz=[{hc_x:+.4f}, {hc_y:+.4f}, {hc_z:+.4f}] m, '
                f'rpy=[{hc_roll_deg:+.2f}, {hc_pitch_deg:+.2f}, {hc_yaw_deg:+.2f}] deg) ===')
            self.publish_status_msg(f'hole_seq[{side_lc}]: start')

            t_hc = np.array([hc_x, hc_y, hc_z], dtype=float)
            R_hc = self._rpy_to_R(
                np.deg2rad(hc_roll_deg),
                np.deg2rad(hc_pitch_deg),
                np.deg2rad(hc_yaw_deg),
            )

            for i, wp in enumerate(wps, start=1):
                t_rel = np.array([wp['dx'], wp['dy'], wp['dz']], dtype=float)
                R_rel = self._rpy_to_R(
                    np.deg2rad(wp['droll_deg']),
                    np.deg2rad(wp['dpitch_deg']),
                    np.deg2rad(wp['dyaw_deg']),
                )

                t_abs = t_hc + R_hc @ t_rel
                R_abs = R_hc @ R_rel
                roll_abs, pitch_abs, yaw_abs = self._R_to_rpy(R_abs)

                finger_m = wp.get('finger_m', None)
                finger_str = (
                    f', finger={finger_m * 1000:.2f} mm '
                    f'(opening={2.0 * finger_m * 1000:.2f} mm)'
                    if finger_m is not None else ', finger=skip')
                self.get_logger().info(
                    f'[{i}/{len(wps)}] -> base xyz='
                    f'[{t_abs[0]:+.4f}, {t_abs[1]:+.4f}, {t_abs[2]:+.4f}] m, '
                    f'rpy=[{np.degrees(roll_abs):+.2f}, '
                    f'{np.degrees(pitch_abs):+.2f}, '
                    f'{np.degrees(yaw_abs):+.2f}] deg'
                    f'{finger_str}')

                # Step A: move EE position first, keeping the current
                # roll/pitch so the wrist orientation does not change yet.
                # Only roll/pitch are commanded on this 5-DOF arm (yaw is
                # derived from waist), so "current roll/pitch" fully pins the
                # wrist attitude for this phase.
                T_cur = np.asarray(self.bot.arm.T_sb, dtype=float)
                cur_roll, cur_pitch, _ = self._R_to_rpy(T_cur[:3, :3])
                self.get_logger().info(
                    f'[{i}/{len(wps)}] step A: move position only '
                    f'(hold rpy=[{np.degrees(cur_roll):+.2f}, '
                    f'{np.degrees(cur_pitch):+.2f}] deg)')
                success = self.bot.arm.set_ee_pose_components(
                    x=float(t_abs[0]),
                    y=float(t_abs[1]),
                    z=float(t_abs[2]),
                    roll=float(cur_roll),
                    pitch=float(cur_pitch),
                )
                if not success:
                    raise RuntimeError(
                        f'IK failed on hole-center waypoint {i}/{len(wps)} '
                        'during position phase.')

                # Step B: now rotate to the target orientation at the same
                # position. Splitting the motion prevents position and
                # orientation from interpolating together.
                self.get_logger().info(
                    f'[{i}/{len(wps)}] step B: rotate to target rpy '
                    f'[{np.degrees(roll_abs):+.2f}, '
                    f'{np.degrees(pitch_abs):+.2f}] deg')
                success = self.bot.arm.set_ee_pose_components(
                    x=float(t_abs[0]),
                    y=float(t_abs[1]),
                    z=float(t_abs[2]),
                    roll=float(roll_abs),
                    pitch=float(pitch_abs),
                )
                if not success:
                    raise RuntimeError(
                        f'IK failed on hole-center waypoint {i}/{len(wps)} '
                        'during orientation phase.')

                if finger_m is not None:
                    self._set_gripper_finger_position(float(finger_m))

                time.sleep(0.2)

            self.get_logger().info(
                f'=== Hole-center sequence ({side_lc.upper()}): DONE ===')
            self.publish_status_msg(f'hole_seq[{side_lc}]: done')
            return True

        except Exception as e:
            self.get_logger().error(
                f'Hole-center sequence ({side_lc.upper()}) failed: {e}')
            self.publish_status_msg(f'hole_seq[{side_lc}]: error: {e}')
            return False

    def _ensure_gripper_linear_position_mode(self) -> None:
        """
        Use the official Interbotix 'linear_position' mode for the gripper.

        In this mode, commanding joint 'gripper' expects the total opening width [m]
        between the two fingers, not a raw PWM value.
        """
        if self._gripper_linear_mode_ready:
            return

        self.get_logger().info(
            'Switching gripper to official linear_position mode...')
        self.bot.core.robot_set_operating_modes(
            cmd_type='single',
            name='gripper',
            mode='linear_position',
            profile_type='time',
            profile_velocity=800,
            profile_acceleration=0,
        )
        self._gripper_linear_mode_ready = True
        time.sleep(0.2)

    def _set_gripper_finger_position(
        self,
        target_finger_m: float,
        timeout_s: float = 3.0,
        tol_m: float = 0.0008,
        poll_period_s: float = 0.05,
    ) -> bool:
        """
        Command the gripper to a desired left-finger position using the official
        linear_position mode.

        The SDK expects the total opening width [m], while get_finger_position()
        returns the left-finger position [m], so we convert with opening = 2 * finger.
        """
        self._ensure_gripper_linear_position_mode()

        target_opening_m = max(0.0, 2.0 * float(target_finger_m))
        self.bot.core.robot_write_joint_command('gripper', target_opening_m)

        start = time.time()
        last = None
        while time.time() - start < timeout_s:
            try:
                last = float(self.bot.gripper.get_finger_position())
            except Exception:
                last = None

            if last is not None and abs(last - target_finger_m) <= tol_m:
                self.get_logger().info(
                    f'Gripper reached target. finger_pos = {last * 1000:.2f} mm '
                    f'(target {target_finger_m * 1000:.2f} mm, '
                    f'opening {target_opening_m * 1000:.2f} mm)')
                return True

            time.sleep(poll_period_s)

        if last is None:
            self.get_logger().warn(
                f'Gripper position feedback unavailable after commanding '
                f'opening {target_opening_m * 1000:.2f} mm.')
        else:
            self.get_logger().warn(
                f'Gripper did not settle within timeout. '
                f'last finger_pos = {last * 1000:.2f} mm, '
                f'target = {target_finger_m * 1000:.2f} mm.')
        return False

    def check_workspace_limits(self, x, y, z):
        """Check if point is within workspace limits"""
        x_ok = self.workspace_limits['x'][0] <= x <= self.workspace_limits['x'][1]
        y_ok = self.workspace_limits['y'][0] <= y <= self.workspace_limits['y'][1]
        z_ok = self.workspace_limits['z'][0] <= z <= self.workspace_limits['z'][1]
        
        return x_ok and y_ok and z_ok
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp) if abs(sinp) <= 1 else np.pi / 2
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def publish_current_state(self):
        """Publish current joint angles"""
        try:
            joint_positions = self.bot.arm.get_joint_commands()
            
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
            joint_state.position = joint_positions
            
            self.current_joints_pub.publish(joint_state)
        except:
            pass
    
    def publish_status_msg(self, status):
        """Publish status message"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down Main Arm Planner')
        try:
            self.bot.arm.go_to_sleep_pose()
            self.bot.shutdown()
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MainArmPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
