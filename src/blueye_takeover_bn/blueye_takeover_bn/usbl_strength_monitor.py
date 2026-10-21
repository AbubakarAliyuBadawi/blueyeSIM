#!/usr/bin/env python3
"""
Simulated USBL strength on /blueye/usbl_strength, from the age and range of the last Stonefish USBL fix.

Stonefish does not model signal strength, and its USBL noise is fixed, so the rule is the one the operator HUD uses
on its USBL tab (GOOD / FAIR / POOR / NO LOCK); the operator display and the BN evidence therefore always agree:

  fix older than lost_age_s                           -> Lost      (HUD: NO LOCK)
  fix younger than good_age_s and closer than good_range_m -> Strong    (HUD: GOOD)
  fix younger than fair_age_s and closer than fair_range_m -> Moderate  (HUD: FAIR)
  otherwise                                           -> Weak      (HUD: POOR)

USBL loss can be injected with /blueye/usbl/set_beacon_enabled (std_srvs/SetBool, e.g. from the Simulation tab of the
operator HUD): while the beacon is off its fixes are ignored, as if it stopped replying, so the strength degrades by the
rule above and reaches Lost after lost_age_s. The switch is published on /blueye/usbl/beacon_enabled (the HUD's USBL tab
follows it) and every change on /scenario/events.
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
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
        self.declare_parameter("beacon_service", "/blueye/usbl/set_beacon_enabled")
        self.declare_parameter("beacon_state_topic", "/blueye/usbl/beacon_enabled")
        self.declare_parameter("events_topic", "/scenario/events")

        self._last_fix = None
        self._range = None
        self._beacon_enabled = True
        self._beacon_pub = self.create_publisher(Bool, self.get_parameter("beacon_state_topic").value, 10)
        self._event_pub = self.create_publisher(String, self.get_parameter("events_topic").value, 10)
        self.create_service(SetBool, self.get_parameter("beacon_service").value, self._beacon_service)

        self._pub = self.create_publisher(String, self.get_parameter("strength_topic").value, 10)
        self.create_subscription(BeaconInfo, self.get_parameter("beacon_topic").value, self._beacon_cb, 10)
        rate = max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.create_timer(1.0 / rate, self._publish)

    def _beacon_cb(self, msg: BeaconInfo):
        if not self._beacon_enabled:                       # injected USBL loss: the beacon does not reply
            return
        self._last_fix = time.monotonic()                  # wall clock, as in the HUD
        self._range = float(msg.range)

    def _beacon_service(self, request, response):
        if request.data != self._beacon_enabled:
            self._beacon_enabled = request.data
            self.get_logger().info("USBL beacon " + ("ON: fixes accepted again" if request.data
                                                     else "OFF: fixes ignored (injected USBL loss)"))
            event = String()
            event.data = json.dumps({"event": "usbl_beacon", "enabled": request.data})
            self._event_pub.publish(event)
        response.success = True
        response.message = "USBL beacon " + ("on" if request.data else "off")
        return response

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
        beacon = Bool()
        beacon.data = self._beacon_enabled
        self._beacon_pub.publish(beacon)


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
