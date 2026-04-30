#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from parachute_interfaces.srv import RequestNextTarget, MoveToWorldPose


class AlternatingTest(Node):
    def __init__(self):
        super().__init__('alternating_test')

        self.target_client = self.create_client(
            RequestNextTarget, '/request_next_target')
        self.left_move = self.create_client(
            MoveToWorldPose, '/side_arm_left/move_to_world_pose')
        self.right_move = self.create_client(
            MoveToWorldPose, '/side_arm_right/move_to_world_pose')

        self.current_arm = 'left'
        self.loop_count = 0
        self._next_timer = None

        self.get_logger().info('Waiting for services...')
        self.target_client.wait_for_service(timeout_sec=10.0)
        self.left_move.wait_for_service(timeout_sec=10.0)
        self.right_move.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('Services ready. Starting alternating test.')

        self.request_next()

    def request_next(self):
        future = self.target_client.call_async(RequestNextTarget.Request())
        future.add_done_callback(self.on_target_received)

    def on_target_received(self, future):
        result = future.result()

        if not result.target_available:
            self.get_logger().info(
                f'No more targets. Test complete after {self.loop_count} loops.')
            raise SystemExit

        # Pass the pose straight through — already in world frame
        target_pose = result.target_loop.pose
        pos = target_pose.pose.position
        side = 'LEFT' if pos.y >= 0 else 'RIGHT'

        self.get_logger().info(
            f'Loop {self.loop_count + 1}: {self.current_arm} arm <- '
            f'world ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) [{side} side loop]'
        )

        if side == 'LEFT' and self.current_arm != 'left':
            self.get_logger().warn('Side mismatch — selector gave LEFT loop to right arm')
        elif side == 'RIGHT' and self.current_arm != 'right':
            self.get_logger().warn('Side mismatch — selector gave RIGHT loop to left arm')

        req = MoveToWorldPose.Request()
        req.target_pose = target_pose
        req.target_pose.header.frame_id = 'world'
        req.speed_scale = 0.5

        client = self.left_move if self.current_arm == 'left' else self.right_move
        future = client.call_async(req)
        future.add_done_callback(self.on_move_complete)

    def on_move_complete(self, future):
        result = future.result()
        self.loop_count += 1
        self.get_logger().info(
            f'  Move complete: success={result.success} '
            f'arm-local=({result.final_x_mm:.1f}, {result.final_y_mm:.1f}, {result.final_z_mm:.1f})mm'
        )
        if not result.success:
            self.get_logger().warn(f'  Move failed: {result.message}')

        self.current_arm = 'right' if self.current_arm == 'left' else 'left'
        self._next_timer = self.create_timer(1.0, self._fire_next)

    def _fire_next(self):
        if self._next_timer:
            self._next_timer.cancel()
            self._next_timer = None
        self.request_next()


def main():
    rclpy.init()
    node = AlternatingTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()