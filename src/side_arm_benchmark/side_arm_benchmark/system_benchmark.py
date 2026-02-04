import json
import time
import statistics
from typing import Optional, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SystemBenchmark(Node):
    """
    Benchmarks:
    1) Command/state round-trip latency
    2) Servo response under no-load
    3) Stepper position accuracy
    4) Stability during mixed actuator motion
    """

    def __init__(self) -> None:
        super().__init__("side_arm_system_benchmark")

        self.pub = self.create_publisher(String, "/side_arm/command", 10)
        self.sub = self.create_subscription(
            String, "/side_arm/state", self._state_cb, 10
        )

        self.last_state: Optional[dict] = None
        self.last_state_time: Optional[float] = None

        self._pending_latency_ts: Optional[float] = None
        self.latency_samples: List[float] = []

        self.limit_triggered = False

    # helpers
    def _send(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.pub.publish(msg)
        self.get_logger().info(f"send: {cmd}")

    def _wait(self, seconds: float) -> None:
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end and not self.limit_triggered:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _state_cb(self, msg: String) -> None:
        data = msg.data.strip()
        if not data.startswith("STATE"):
            return

        now = time.time()

        try:
            payload = data.split("STATE", 1)[1].strip()
            state = json.loads(payload)

            self.last_state = state
            self.last_state_time = now

            #latency measurement
            if self._pending_latency_ts is not None:
                rtt_ms = (now - self._pending_latency_ts) * 1000.0
                self.latency_samples.append(rtt_ms)
                self._pending_latency_ts = None

            # 
            if any(int(state.get(k, 0)) for k in ("l1", "l2", "l3")):
                self.get_logger().error("LIMIT TRIGGERED — STOPPING BENCHMARK")
                self.limit_triggered = True
                self._send("STOP_ALL")

        except Exception as exc:
            self.get_logger().warn(f"Failed to parse STATE: {exc}")

    # benchmarks
    def benchmark_latency(self, samples: int = 20) -> None:
        self.get_logger().info("=== Benchmark: command/state latency ===")

        self.latency_samples.clear()

        for _ in range(samples):
            self._pending_latency_ts = time.time()
            self._send("REQUEST_STATE")
            self._wait(0.3)

        if not self.latency_samples:
            self.get_logger().error("No latency samples collected")
            return

        avg = statistics.mean(self.latency_samples)
        worst = max(self.latency_samples)
        jitter = statistics.stdev(self.latency_samples) if len(self.latency_samples) > 1 else 0.0

        self.get_logger().info(
            f"Latency: avg={avg:.2f} ms, max={worst:.2f} ms, jitter={jitter:.2f} ms "
            f"(n={len(self.latency_samples)})"
        )


    def benchmark_servo(self) -> None:
        self.get_logger().info("=== Benchmark: servo sweep ===")

        for angle in (0, 45, 90, 135, 180, 90, 0):
            self._send(f"SERVO,{angle}")
            self._wait(0.6)
            if self.limit_triggered:
                return

    def benchmark_stepper_accuracy(self, motor_id: int, delta: int) -> None:
        self.get_logger().info(f"=== Benchmark: stepper {motor_id} accuracy ===")

        start_pos = None
        key = "s1" if motor_id == 1 else "s2"

        # Capture starting position
        self._send("REQUEST_STATE")
        self._wait(0.3)
        if self.last_state:
            start_pos = int(self.last_state.get(key, 0))

        self._send(f"STEPPER_MOVE,{motor_id},{delta},800")
        self._wait(3.0)

        self._send(f"STEPPER_MOVE,{motor_id},{-delta},800")
        self._wait(3.0)

        self._send("REQUEST_STATE")
        self._wait(0.3)

        if self.last_state and start_pos is not None:
            end_pos = int(self.last_state.get(key, 0))
            error = abs(end_pos - start_pos)
            self.get_logger().info(
                f"Stepper {motor_id} error after round-trip: {error} steps"
            )

    def benchmark_mixed_load(self) -> None:
        self.get_logger().info("=== Benchmark: mixed actuator load ===")

        self._send("DC_SPEED,50")

        for angle in (0, 90, 180, 90):
            self._send(f"SERVO,{angle}")
            self._send("STEPPER_MOVE,1,500,600")
            self._wait(1.5)
            if self.limit_triggered:
                return

        self._send("DC_SPEED,0")
        
    def benchmark_throughput(self, duration_s: float = 5.0) -> None:
        self.get_logger().info(f"=== Benchmark: message throughput ({duration_s}s) ===")

        self.latency_samples.clear()  # reuse for counting responses
        start_time = time.time()
        end_time = start_time + duration_s

        messages_sent = 0
        messages_received = 0

        self._pending_latency_ts = None

        while rclpy.ok() and time.time() < end_time and not self.limit_triggered:
            # send command as fast as possible
            self._send("REQUEST_STATE")
            messages_sent += 1
            rclpy.spin_once(self, timeout_sec=0.01)  # short spin to process incoming messages

            # count received states
            if self.last_state_time and self.last_state_time > start_time:
                messages_received += 1
                self.last_state_time = None  # reset to avoid double-counting

        total_time = time.time() - start_time
        self.get_logger().info(
            f"Messages sent: {messages_sent}, messages received: {messages_received} "
            f"in {total_time:.2f}s"
        )

        if total_time > 0:
            throughput = messages_received / total_time
            self.get_logger().info(f"Estimated throughput: {throughput:.2f} msgs/sec")

    def benchmark_stepper_repeatability(self, motor_id: int, delta: int = 100, repeats: int = 5) -> None:
    
        self.get_logger().info(f"=== Benchmark: stepper {motor_id} micro-move repeatability ===")

        key = "s1" if motor_id == 1 else "s2"

        # Capture starting position
        self._send("REQUEST_STATE")
        self._wait(0.3)
        if not self.last_state:
            self.get_logger().error("No initial state for stepper")
            return

        start_pos = int(self.last_state.get(key, 0))
        positions = []

        for i in range(repeats):
            # move forward
            self._send(f"STEPPER_MOVE,{motor_id},{delta},800")
            self._wait(0.5)

            # move backward
            self._send(f"STEPPER_MOVE,{motor_id},{-delta},800")
            self._wait(0.5)

            # record end position
            self._send("REQUEST_STATE")
            self._wait(0.3)
            if self.last_state:
                pos = int(self.last_state.get(key, 0))
                positions.append(pos)

        # compute maximum deviation
        if positions:
            deviations = [abs(pos - start_pos) for pos in positions]
            max_dev = max(deviations)
            self.get_logger().info(
                f"Stepper {motor_id} repeatability: max deviation after {repeats} micro-moves = {max_dev} steps"
            )
        else:
            self.get_logger().warn("No positions recorded for repeatability test")


    # ---------------- run sequence ----------------
    def run(self) -> None:
        self.get_logger().info("=== SYSTEM BENCHMARK START ===")

        self._send("STOP_ALL")
        self._wait(0.5)
        self._send("STEPPER_ENABLE,1")
        self._wait(0.5)

        # self.benchmark_latency()
        # self.benchmark_throughput(duration_s=5.0)
        self.benchmark_stepper_repeatability(motor_id=1, delta=100, repeats=5)
        self.benchmark_stepper_repeatability(motor_id=2, delta=500, repeats=5)

        # self.benchmark_servo()
        # self.benchmark_stepper_accuracy(motor_id=1, delta=2000)
        # self.benchmark_stepper_accuracy(motor_id=2, delta=2000)
        # self.benchmark_mixed_load()

        self._send("STOP_ALL")
        self._send("REQUEST_STATE")

        self.get_logger().info("=== SYSTEM BENCHMARK COMPLETE ===")


def main() -> None:
    rclpy.init()
    node = SystemBenchmark()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
