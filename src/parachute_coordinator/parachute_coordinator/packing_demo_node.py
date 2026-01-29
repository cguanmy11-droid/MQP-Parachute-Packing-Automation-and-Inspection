#!/usr/bin/env python3
"""
Demo Node for Main Arm Testing
Moves arm to home, then cycles through request_next_target positions
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from parachute_interfaces.msg import ArmStatus
from geometry_msgs.msg import Pose
import time

class PackingDemoCoordinator(Node):
    def __init__(self):
        super().__init__('packing_demo_coordinator')
        
        # Publishers
        self.pose_cmd_pub = self.create_publisher(
            String, 
            '/main_arm/pose_command', 
            10
        )
        
        self.test_position_pub = self.create_publisher(
            Pose,
            '/main_arm/test_position',
            10
        )
        
        # Subscribers
        self.status_sub = self.create_subscription(
            ArmStatus,
            '/main_arm/status',
            self.status_callback,
            10
        )
        
        self.simple_status_sub = self.create_subscription(
            String,
            '/main_arm/simple_status',
            self.simple_status_callback,
            10
        )
        
        # Subscribe to detected loops topic
        from parachute_interfaces.msg import DetectedLoops
        self.loops_sub = self.create_subscription(
            DetectedLoops,
            '/loop_positions',
            self.loops_callback,
            10
        )
        
        # State tracking
        self.current_arm_status = None
        self.arm_is_moving = False
        self.last_simple_status = None
        
        # Loop detection state
        self.loop_positions = []
        self.num_detected_loops = 0
        self.demo_active = False
        self.current_position_index = 0
        self.at_home = True  # Start assuming we're at home
        self.waiting_for_completion = False
        
        # Timer for demo progression (check every 1.0 seconds for slower operation)
        self.demo_timer = self.create_timer(1.0, self.demo_step)
        
        self.get_logger().info('Arm Demo Node initialized')
        self.get_logger().info('Waiting for /detected_loops messages...')
        self.get_logger().info('Demo will start when 4+ loops are detected')
        
    def status_callback(self, msg):
        """Track detailed arm status"""
        self.current_arm_status = msg
        self.arm_is_moving = msg.is_moving
        
    def simple_status_callback(self, msg):
        """Track simple status updates"""
        if msg.data != self.last_simple_status:
            self.last_simple_status = msg.data
            self.get_logger().info(f'Arm status: {msg.data}')
    
    def loops_callback(self, msg):
        """
        Handle detected loops message
        msg type: DetectedLoops
          - header
          - loops[] (array of DetectedLoop)
          - total_count (int)
        """
        self.num_detected_loops = msg.total_count
        
        # Only start demo if we have 4+ loops and haven't started yet
        if self.num_detected_loops >= 4 and not self.demo_active:
            self.get_logger().info('='*60)
            self.get_logger().info(f'DETECTED {self.num_detected_loops} LOOPS!')
            self.get_logger().info('Starting demo sequence...')
            self.get_logger().info('='*60)
            
            # Extract positions from the DetectedLoop array
            self.loop_positions = []
            for i, detected_loop in enumerate(msg.loops):
                # Each detected_loop has a PoseStamped in the pose field
                # Extract just the Pose (without the header)
                pose = detected_loop.pose.pose
                self.loop_positions.append(pose)
                
                self.get_logger().info(f'Loop {i+1} (ID: {detected_loop.loop_id}):')
                self.get_logger().info(f'  Position: ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})')
                self.get_logger().info(f'  Confidence: {detected_loop.confidence:.2f}')
            
            self.get_logger().info(f'Stored {len(self.loop_positions)} loop positions')
            self.demo_active = True
            self.at_home = False  # Will go to home first
        
        elif self.num_detected_loops < 4:
            # self.get_logger().info(f'Detected {self.num_detected_loops} loops (need 4+ to start demo)')
            pass
        # If demo is already active, update positions but don't restart
        elif self.demo_active:
            # Update positions in case new loops were detected
            self.loop_positions = []
            for detected_loop in msg.loops:
                pose = detected_loop.pose.pose
                self.loop_positions.append(pose)
            self.get_logger().info(f'Updated to {len(self.loop_positions)} loop positions')
    
    def demo_step(self):
        """Execute demo loop: alternate between home and detected positions"""
        if not self.demo_active:
            return
        
        if len(self.loop_positions) == 0:
            self.get_logger().warn('No loop positions available!')
            return
        
        # If waiting for arm to finish moving, check status
        if self.waiting_for_completion:
            if self.current_arm_status is None:
                return  # Still waiting for first status message
            
            # Check if arm has stopped moving
            if not self.arm_is_moving:
                self.get_logger().info(f'  ✓ Movement complete')
                self.get_logger().info(f'  Pausing for 3 seconds...')
                time.sleep(3.0)  # Longer pause so you can see the position
                self.waiting_for_completion = False
            return
        
        # Alternate between home and loop positions
        if not self.at_home:
            # Go to home
            self.get_logger().info('')
            self.get_logger().info('='*60)
            self.get_logger().info('RETURNING TO HOME POSITION')
            self.get_logger().info('='*60)
            
            msg = String()
            msg.data = 'home'
            self.pose_cmd_pub.publish(msg)
            
            self.at_home = True
            self.waiting_for_completion = True
            
        else:
            # Go to next loop position
            position = self.loop_positions[self.current_position_index]
            
            self.get_logger().info('')
            self.get_logger().info('='*60)
            self.get_logger().info(f'MOVING TO LOOP POSITION {self.current_position_index + 1}/{len(self.loop_positions)}')
            self.get_logger().info('='*60)
            self.get_logger().info(f'  Position:')
            self.get_logger().info(f'    X: {position.position.x:.4f} m')
            self.get_logger().info(f'    Y: {position.position.y:.4f} m')
            self.get_logger().info(f'    Z: {position.position.z:.4f} m')
            self.get_logger().info(f'  Orientation (quaternion):')
            self.get_logger().info(f'    x: {position.orientation.x:.4f}')
            self.get_logger().info(f'    y: {position.orientation.y:.4f}')
            self.get_logger().info(f'    z: {position.orientation.z:.4f}')
            self.get_logger().info(f'    w: {position.orientation.w:.4f}')
            
            # Convert quaternion to euler for easier visualization
            from scipy.spatial.transform import Rotation as R
            quat = [position.orientation.x, position.orientation.y, 
                    position.orientation.z, position.orientation.w]
            if sum([abs(q) for q in quat]) > 0.01:
                euler = R.from_quat(quat).as_euler('xyz', degrees=True)
                self.get_logger().info(f'  Orientation (euler angles):')
                self.get_logger().info(f'    Roll:  {euler[0]:.2f}°')
                self.get_logger().info(f'    Pitch: {euler[1]:.2f}°')
                self.get_logger().info(f'    Yaw:   {euler[2]:.2f}°')
            
            self.get_logger().info('='*60)
            self.get_logger().info(f'→ Publishing to /main_arm/test_position')
            
            # Publish to test_position topic
            self.test_position_pub.publish(position)
            
            # Move to next position for next iteration
            self.current_position_index = (self.current_position_index + 1) % len(self.loop_positions)
            
            self.at_home = False
            self.waiting_for_completion = True

def main(args=None):
    rclpy.init(args=args)
    node = PackingDemoCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Demo interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()