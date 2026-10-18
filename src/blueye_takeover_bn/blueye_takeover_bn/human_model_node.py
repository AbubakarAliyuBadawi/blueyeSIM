#!/usr/bin/env python3
"""
Synthetic human model publisher for simulation.

Publishes the operator's fatigue and stress, both starting low (a rested, calm operator) and increasing
gradually with time on task: a linear ramp from the start to the end value over ramp_duration_s, then constant,
plus small noise.

  /blueye/human/fatigue  (std_msgs/Float32)  0.0 = rested,  1.0 = exhausted
  /blueye/human/stress   (std_msgs/Float32)  0.0 = calm,    1.0 = stressed

Time on task is counted from the mission start (the first /mission_state > 0), so every run sees the same
profile however long the node ran before the mission. Before the start the start values are published. When
/mission_state has been silent for mission_restart_gap_s (behaviour tree stopped), the next mission starts the
clock again.
"""

import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32


class HumanModelNode(Node):
    def __init__(self):
        super().__init__("human_model")

        self.declare_parameter("publish_rate_hz",       2.0)
        self.declare_parameter("ramp_duration_s",       900.0)
        self.declare_parameter("fatigue_start",         0.15)
        self.declare_parameter("fatigue_end",           0.55)
        self.declare_parameter("stress_start",          0.15)
        self.declare_parameter("stress_end",            0.50)
        self.declare_parameter("noise_std",             0.02)
        self.declare_parameter("mission_state_topic",   "/mission_state")
        self.declare_parameter("mission_restart_gap_s", 5.0)

        self._mission_start = None
        self._last_state_msg = None

        self._fatigue_pub = self.create_publisher(Float32, "/blueye/human/fatigue", 10)
        self._stress_pub  = self.create_publisher(Float32, "/blueye/human/stress",  10)
        self.create_subscription(Int32, self.get_parameter("mission_state_topic").value, self._mission_state_cb, 10)

        self.create_timer(1.0 / float(self.get_parameter("publish_rate_hz").value), self._tick)
        self.get_logger().info(
            "Human model started — fatigue and stress rise from the mission start on "
            "/blueye/human/fatigue and /blueye/human/stress"
        )

    def _mission_state_cb(self, msg: Int32):
        now = time.monotonic()
        gap = float(self.get_parameter("mission_restart_gap_s").value)
        if self._last_state_msg is not None and now - self._last_state_msg > gap:
            self._mission_start = None                      # behaviour tree was stopped: a new mission follows
        self._last_state_msg = now
        if msg.data > 0 and self._mission_start is None:
            self._mission_start = now
            self.get_logger().info("Mission started — operator fatigue and stress now increase with time on task")

    def _tick(self):
        ramp  = float(self.get_parameter("ramp_duration_s").value)
        noise = float(self.get_parameter("noise_std").value)
        t = 0.0 if self._mission_start is None else time.monotonic() - self._mission_start
        progress = min(t / ramp, 1.0) if ramp > 0 else 1.0

        values = []
        for name in ("fatigue", "stress"):
            start = float(self.get_parameter(f"{name}_start").value)
            end   = float(self.get_parameter(f"{name}_end").value)
            value = start + (end - start) * progress + random.gauss(0.0, noise)
            values.append(float(max(0.0, min(1.0, value))))

        for pub, value in zip((self._fatigue_pub, self._stress_pub), values):
            msg = Float32()
            msg.data = value
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HumanModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
