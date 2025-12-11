#!/usr/bin/env python3
"""
Loop Detector Node
Detects parachute loops in camera feed and publishes their positions
"""
import rclpy
from rclpy.node import Node
from parachute_interfaces.msg import DetectedLoops, DetectedLoop
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, PoseArray
from std_msgs.msg import Header 

class LoopDetectorNode(Node):
    def __init__(self):
        super().__init__('loop_detector_node')
        self.get_logger().info('Loop Detector Node initialized')

        # Declare and get test_mode parameter
        self.declare_parameter('test_mode', True)
        self.test_mode = self.get_parameter('test_mode').value
        
        # In real mode, Create subscribers for camera feed
        self.subscriber = self.create_subscription(PoseArray, '/yolo/centers', self.publish_detected_loops, 10)
        # Create publisher for detected loops
        self.publisher = self.create_publisher(DetectedLoops, '/detected_loops', 10)

        # TODO: Parameters for detection thresholds

        if self.test_mode:
            self.get_logger().info('TEST MODE: Loop Detector Node initialized to publish test loop locations')
            # In test mode, publish test loop messages every 2 seconds
            self.timer = self.create_timer(2.0, self.publish_test_loops)
        else:
            self.get_logger().info('Loop Detector Node initialized')
            # In real mode, Create subscribers for camera feed
            # self.subscriber = self.create_subscription(PoseArray, '/yolo/centers', self.publish_detected_loops, 10)
        
    def camera_callback(self, msg):
        """Process camera image and detect loops"""
        # TODO: Implement loop detection
        pass
        
    def publish_detected_loops(self, loops):
        """Publish detected loop positions"""
        # Check is poses is empty
        if loops.poses == []:
            return

        msg = DetectedLoops()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"

        # Loop over poses in PoseArray
        for loop in loops.poses:
            # Set up detected loop header
            detected_loop = DetectedLoop()
            detected_loop.header = Header()
            detected_loop.pose = PoseStamped()

            # Pass pose info from recieved message
            detected_loop.pose.header = msg.header
            detected_loop.pose.pose = Pose()
            detected_loop.pose.pose.position = Point(
                x=loop.position.x, 
                y=-0.11,   
                z=loop.position.z
            )
            detected_loop.pose.pose.orientation = Quaternion(x= 0.0, y= 0.907, z= 0.0, w= 0.907)

            # Add to the list of detected loops
            msg.loops.append(detected_loop)

        # Publish detected loops message
        msg.total_count = len(msg.loops)
        self.publisher.publish(msg)
        # self.get_logger().info(f'Published {msg.total_count} test loops')


    def publish_test_loops(self):
        """Test function: Publish fake data to simulate detected loops"""
        # self.get_logger().info('TEST: Detecting loops...')
        
        # Simulate processing delay
        import time
        time.sleep(0.5)
        
        msg = DetectedLoops()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        
        # Create 3 dummy loops at different x positions
        for i in range(3):
            loop = DetectedLoop()
            loop.header = msg.header
            loop.loop_id = i
            loop.confidence = 0.95 - (i * 0.05)
            
            # Position loops - rightmost has highest x value
            loop.pose = PoseStamped()
            loop.pose.header = msg.header
            loop.pose.pose.position = Point(
                x=0.3 + (i * 0.15),  # 0.3, 0.45, 0.6
                y=0.0, 
                z=0.5
            )
            loop.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            msg.loops.append(loop)
        
        msg.total_count = len(msg.loops)
        
        self.publisher.publish(msg)
        # self.get_logger().info(f'TEST: Published {msg.total_count} test loops')

def main(args=None):
    rclpy.init(args=args)
    node = LoopDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
