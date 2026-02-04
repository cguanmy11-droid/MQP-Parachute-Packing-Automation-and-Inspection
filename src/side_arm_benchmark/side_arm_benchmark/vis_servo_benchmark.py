import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
from typing import Optional


class VisionCenteringNode(Node):
    def __init__(self):
        super().__init__("vision_centering_node")


        self.declare_parameter("image_width_px", 640)
        self.declare_parameter("image_height_px", 480)
        self.declare_parameter("kp_x", 1.2)
        self.declare_parameter("kp_y", 1.2)
        self.declare_parameter("deadband_px", 5.0)
        self.declare_parameter("control_rate", 15.0)

        self.image_width = self.get_parameter("image_width_px").value
        self.image_height = self.get_parameter("image_height_px").value
        self.kp_x = self.get_parameter("kp_x").value
        self.kp_y = self.get_parameter("kp_y").value
        self.deadband_x = self.get_parameter("deadband_px").value
        self.deadband_y = self.get_parameter("deadband_px").value
        self.control_rate = self.get_parameter("control_rate").value

        # speed limits
        self.min_speed_x = 400
        self.max_speed_x = 1100
        self.min_speed_y = 600
        self.max_speed_y = 1000


        self.stepper_x = 2 
        self.stepper_y = 1  

        # ROS I/O
        self.sub = self.create_subscription(
            PoseArray, "/yolo/centers", self._centers_cb, 10
        )
        self.pub = self.create_publisher(
            String, "/side_arm/command", 10
        )

        # latest tracked positions
        self.latest_x: Optional[float] = None
        self.latest_y: Optional[float] = None
        self.moving = {self.stepper_x: False, self.stepper_y: False}

        # timer
        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self._control_loop
        )

        self.get_logger().info(
            "Vision centering active (stepper X & Y, STOP_ALL for stop)"
        )


    def _centers_cb(self, msg: PoseArray):
        if not msg.poses:
            self.latest_x = None
            self.latest_y = None
            return

        # track rightmost object
        rightmost = max(msg.poses, key=lambda p: p.position.x)
        self.latest_x = rightmost.position.x
        self.latest_y = rightmost.position.y


    def _control_loop(self):
        center_x = self.image_width / 2.0
        center_y = self.image_height / 2.0


        if self.latest_x is None:
            self._stop_if_needed(self.stepper_x)
        else:
            error_x = self.latest_x - center_x
            if abs(error_x) < self.deadband_x:
                self._stop_if_needed(self.stepper_x)
            else:
                velocity_x = int(max(self.min_speed_x, min(self.max_speed_x, abs(self.kp_x * error_x))) * (1 if error_x > 0 else -1))
                self._send_velocity(self.stepper_x, velocity_x)


        if self.latest_y is None:
            self._stop_if_needed(self.stepper_y)
        else:
            error_y = self.latest_y - center_y
            if abs(error_y) < self.deadband_y:
                self._stop_if_needed(self.stepper_y)
            else:
                velocity_y = int(max(self.min_speed_y, min(self.max_speed_y, abs(self.kp_y * error_y))) * (1 if error_y > 0 else -1))
                self._send_velocity(self.stepper_y, velocity_y)


    def _send_velocity(self, stepper_id: int, velocity: int):
        msg = String()
        msg.data = f"STEPPER_MOVE,{stepper_id},{velocity}"
        self.pub.publish(msg)
        self.moving[stepper_id] = True
        self.get_logger().debug(f"send: {msg.data}")

    def _stop_if_needed(self, stepper_id: int):
        if self.moving.get(stepper_id, False):
            msg = String()
            msg.data = "STOP_ALL"
            self.pub.publish(msg)
            self.moving[stepper_id] = False
            self.get_logger().debug("send: STOP_ALL")


def main():
    rclpy.init()
    node = VisionCenteringNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

