#!/usr/bin/env python3
"""
Synthetic human model publisher for simulation.

Publishes slowly-decreasing fatigue and stress values over the mission duration,
simulating an operator who starts anxious at mission begin and gradually relaxes
as the mission proceeds without incidents.

  /blueye/human/fatigue  (std_msgs/Float32)  0.0 = rested,  1.0 = exhausted
  /blueye/human/stress   (std_msgs/Float32)  0.0 = calm,    1.0 = stressed

Both start around 0.65–0.70 and decay toward 0.20–0.30, with small noise and
a gentle oscillation on the stress channel to reflect varying workload phases.
"""

import math
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class HumanModelNode(Node):
    def __init__(self):
        super().__init__("human_model")

        self.declare_parameter("publish_rate_hz",    2.0)
        self.declare_parameter("mission_duration_s", 450.0)
        self.declare_parameter("fatigue_start",      0.65)
        self.declare_parameter("fatigue_end",        0.20)
        self.declare_parameter("stress_start",       0.70)
        self.declare_parameter("stress_end",         0.25)
        self.declare_parameter("noise_std",          0.02)

        rate     = float(self.get_parameter("publish_rate_hz").value)
        self._dt = 1.0 / rate
        self._t  = 0.0

        self._fatigue_pub = self.create_publisher(Float32, "/blueye/human/fatigue", 10)
        self._stress_pub  = self.create_publisher(Float32, "/blueye/human/stress",  10)

        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            "Human model started — publishing fatigue and stress on "
            "/blueye/human/fatigue and /blueye/human/stress"
        )

    def _tick(self):
        dur   = float(self.get_parameter("mission_duration_s").value)
        noise = float(self.get_parameter("noise_std").value)
        f0    = float(self.get_parameter("fatigue_start").value)
        f1    = float(self.get_parameter("fatigue_end").value)
        s0    = float(self.get_parameter("stress_start").value)
        s1    = float(self.get_parameter("stress_end").value)

        # Exponential decay: fast drop early, slow tail toward end
        alpha   = max(self._t / dur, 0.0)
        decay   = math.exp(-3.0 * alpha)

        fatigue = f0 * decay + f1 * (1.0 - decay) + random.gauss(0.0, noise)

        stress_base = s0 * decay + s1 * (1.0 - decay)
        stress_osc  = (0.06 * math.sin(2.0 * math.pi * self._t / 55.0)
                       + 0.04 * math.sin(2.0 * math.pi * self._t / 23.0))
        stress = stress_base + stress_osc + random.gauss(0.0, noise)

        fatigue = float(max(0.0, min(1.0, fatigue)))
        stress  = float(max(0.0, min(1.0, stress)))

        msg_f = Float32(); msg_f.data = fatigue
        msg_s = Float32(); msg_s.data = stress
        self._fatigue_pub.publish(msg_f)
        self._stress_pub.publish(msg_s)

        self._t += self._dt


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
