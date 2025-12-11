#!/usr/bin/env python3
"""
Test script to explore robot kinematics and coordinate frames
"""

import rclpy
from rclpy.node import Node
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import time

def main():
    print("Initializing robot for kinematics testing...")
    
    bot = InterbotixManipulatorXS(
        robot_model='wx200',
        robot_name='wx200',
        moving_time=2.0,
        accel_time=0.5
    )
    
    print("\n" + "="*70)
    print("WIDOWX-200 COORDINATE FRAME EXPLORATION")
    print("="*70)
    
    # Test different poses
    test_poses = [
        ('home', lambda: bot.arm.go_to_home_pose()),
        ('sleep', lambda: bot.arm.go_to_sleep_pose()),
        ('upright', lambda: bot.arm.set_joint_positions([0, 0, 0, 0, 0])),
    ]
    
    for pose_name, pose_func in test_poses:
        print(f"\n{'='*70}")
        print(f"Moving to {pose_name.upper()} pose...")
        print("="*70)
        
        pose_func()
        time.sleep(1.0)
        
        # Get FK
        T_sb = bot.arm.get_ee_pose()
        x, y, z = T_sb[0, 3], T_sb[1, 3], T_sb[2, 3]
        
        # Get orientation
        R = T_sb[0:3, 0:3]
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
        yaw = np.arctan2(R[1, 0], R[0, 0])
        
        print(f"\nEnd-Effector Position (base frame):")
        print(f"  X: {x:7.4f} m  (forward/back)")
        print(f"  Y: {y:7.4f} m  (left/right)")
        print(f"  Z: {z:7.4f} m  (up/down)")
        print(f"  Distance from base: {np.sqrt(x**2 + y**2 + z**2):.4f} m")
        
        print(f"\nEnd-Effector Orientation:")
        print(f"  Roll:  {np.degrees(roll):7.2f}° ({roll:7.4f} rad)")
        print(f"  Pitch: {np.degrees(pitch):7.2f}° ({pitch:7.4f} rad)")
        print(f"  Yaw:   {np.degrees(yaw):7.2f}° ({yaw:7.4f} rad)")
        
        # Get joints
        joints = bot.arm.get_joint_commands()
        joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
        
        print(f"\nJoint Positions:")
        for name, pos in zip(joint_names, joints):
            print(f"  {name:15s}: {np.degrees(pos):7.2f}° ({pos:7.4f} rad)")
    
    # Test custom positions
    print(f"\n{'='*70}")
    print("TESTING CUSTOM CARTESIAN POSITIONS")
    print("="*70)
    
    custom_positions = [
        (0.3, 0.0, 0.2, "Forward, center, low"),
        (0.2, 0.1, 0.3, "Forward-left, medium"),
        (0.2, -0.1, 0.3, "Forward-right, medium"),
    ]
    
    for x_target, y_target, z_target, description in custom_positions:
        print(f"\nMoving to: {description}")
        print(f"  Target: X={x_target}, Y={y_target}, Z={z_target}")
        
        success = bot.arm.set_ee_pose_components(
            x=x_target,
            y=y_target,
            z=z_target
        )
        
        if success:
            time.sleep(1.0)
            T_sb = bot.arm.get_ee_pose()
            x_actual = T_sb[0, 3]
            y_actual = T_sb[1, 3]
            z_actual = T_sb[2, 3]
            
            print(f"  Actual: X={x_actual:.4f}, Y={y_actual:.4f}, Z={z_actual:.4f}")
            print(f"  Error:  X={abs(x_actual-x_target):.4f}, Y={abs(y_actual-y_target):.4f}, Z={abs(z_actual-z_target):.4f}")
        else:
            print("  ⚠️  Position unreachable (IK failed)")
    
    print(f"\n{'='*70}")
    print("WORKSPACE INFORMATION")
    print("="*70)
    print("Typical workspace for WidowX-200:")
    print("  Radius: ~0.05m to ~0.50m from base")
    print("  Height: ~0.00m (table level) to ~0.50m")
    print("  Note: 5-DOF arm has orientation constraints")
    print("="*70)
    
    # Return to sleep
    bot.arm.go_to_sleep_pose()
    bot.shutdown()

if __name__ == '__main__':
    main()