#!/usr/bin/env python3
"""
Simulated USBL strength on /blueye/usbl_strength, from the age and range of the last Stonefish USBL fix.

Stonefish does not model signal strength, and its USBL noise is fixed, so the rule is the one the operator HUD uses
on its USBL tab (GOOD / FAIR / POOR / NO LOCK); the operator display and the BN evidence therefore always agree:

  fix older than lost_age_s                           -> Lost      (HUD: NO LOCK)
  fix younger than good_age_s and closer than good_range_m -> Strong    (HUD: GOOD)
  fix younger than fair_age_s and closer than fair_range_m -> Moderate  (HUD: FAIR)
  otherwise                                           -> Weak      (HUD: POOR)
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from stonefish_ros2.msg import BeaconInfo


class UsblStrengthMonitor(Node):
    def __init__(self):
        super().__init__("usbl_strength_monitor")

        self.declare_parameter("beacon_topic", "/blueye/usbl/beacon_info")
        self.declare_parameter("strength_topic", "/blueye/usbl_strength")
        self.declare_parameter("publish_rate_hz", 2.0)
        # Same thresholds as operator_hud.USBLPanel._signal_state
        self.declare_parameter("good_age_s", 1.2)
        self.declare_parameter("good_range_m", 15.0)
        self.declare_parameter("fair_age_s", 2.0)
        self.declare_parameter("fair_range_m", 50.0)
        self.declare_parameter("lost_age_s", 3.0)

        self._last_fix = None
        self._range = None

        self._pub = self.create_publisher(String, self.get_parameter("strength_topic").value, 10)
        self.create_subscription(BeaconInfo, self.get_parameter("beacon_topic").value, self._beacon_cb, 10)
        rate = max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.create_timer(1.0 / rate, self._publish)

    def _beacon_cb(self, msg: BeaconInfo):
        self._last_fix = time.monotonic()                  # wall clock, as in the HUD
        self._range = float(msg.range)

    def _state(self) -> str:
        age = float("inf") if self._last_fix is None else time.monotonic() - self._last_fix
        if age > self._param("lost_age_s"):
            return "Lost"
        if age < self._param("good_age_s") and self._range < self._param("good_range_m"):
            return "Strong"
        if age < self._param("fair_age_s") and self._range < self._param("fair_range_m"):
            return "Moderate"
        return "Weak"

    def _param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _publish(self):
        msg = String()
        msg.data = self._state()
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UsblStrengthMonitor()
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
