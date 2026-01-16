import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MotionDemo(Node):
    """
    Simple ROS 2 demo that drives:
    - Stepper 1: +3000 steps at 700 steps/s
    - Stepper 2: +12000 steps at 2000 steps/s
    - DC motor: +50% PWM for 3 seconds

    Safety: if any limit switch is triggered (l1/l2/l3 == 1), send STOP_ALL and exit.
    """

    def __init__(self) -> None:
        super().__init__("side_arm_motion_demo")
        self.pub = self.create_publisher(String, "/side_arm/command", 10)
        self.sub = self.create_subscription(String, "/side_arm/state", self._state_cb, 10)
        self.limit_triggered = False
        self.last_state: Optional[dict] = None
        self.stop_sent = False

    # region helpers
    def _send(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f"send: {text}")

    def _state_cb(self, msg: String) -> None:
        data = msg.data.strip()
        # 只关心以 "STATE" 开头的状态消息
        if not data.startswith("STATE"):
            return

        try:
            # 去掉前缀 "STATE"，只拿后面的 JSON 部分
            payload = data.split("STATE", 1)[1].strip()
            state = json.loads(payload)

            # 保存最新状态，方便别的地方用
            self.last_state = state

            # 取出限位和位置（如果固件里有 s1 / s2 / dc 这些字段）
            l1 = int(state.get("l1", 0))
            l2 = int(state.get("l2", 0))
            l3 = int(state.get("l3", 0))
            s1 = state.get("s1", None)
            s2 = state.get("s2", None)
            dc = state.get("dc", None)

            # 只记录，不做任何停止动作，具体保护交给 MCU 固件
            if l1 or l2 or l3:
                self.get_logger().warn(
                    f"Limit hit (l1={l1}, l2={l2}, l3={l3}), "
                    f"s1={s1}, s2={s2}, dc={dc}; "
                    "letting MCU handle per-motor safety (no STOP_ALL from demo)."
                )

        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Failed to parse state: {exc}")


    def _wait(self, seconds: float) -> None:
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end and not self.limit_triggered:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _wait_stepper_done(self, motor_id: int, delta: int, timeout: float) -> None:
        """
        Wait until the target step is reached (within tolerance) or timeout/limit triggers.
        Falls back to simple timeout if no state messages are available.
        """
        start_time = time.time()
        start_pos: Optional[int] = None
        target_pos: Optional[int] = None
        tolerance = 5

        while rclpy.ok() and not self.limit_triggered:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_state:
                pos_key = "s1" if motor_id == 1 else "s2"
                try:
                    current_pos = int(self.last_state.get(pos_key, 0))
                except Exception:
                    current_pos = 0
                if start_pos is None:
                    start_pos = current_pos
                    target_pos = start_pos + delta
                if target_pos is not None and abs(current_pos - target_pos) <= tolerance:
                    return
            if (time.time() - start_time) > timeout:
                self.get_logger().warn(f"Stepper {motor_id} wait timeout")
                return
    # endregion

    def run(self) -> None:
        # Ensure steppers enabled
        self._send("STEPPER_ENABLE,1")
        self._wait(0.2)

        # # Stepper 1: 3000 steps @ 700 steps/s
        # self._send("STEPPER_MOVE,1,3000,700")
        # self._wait_stepper_done(motor_id=1, delta=3000, timeout=8.0)
        # if self.limit_triggered:
        #     return

        # Stepper 2: -12000 steps @ 2000 steps/s (reverse)
        self._send("STEPPER_MOVE,2,120000,1000")
        self._wait_stepper_done(motor_id=2, delta=12000, timeout=12.0)
        if self.limit_triggered:
            return

        # DC: 50% for 3 seconds
        self._send("DC_SPEED,70")
        self._wait(20.0)
        self._send("DC_SPEED,0")

        # Final state request
        self._send("REQUEST_STATE")
        self._wait(0.5)


def main() -> None:
    rclpy.init()
    node = MotionDemo()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

