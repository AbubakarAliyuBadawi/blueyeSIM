#!/usr/bin/env python3
"""
The USBL changes of the closed-loop field run (pier_faults_run01_closedloop_20261025), replayed in the simulator twin of
that run (scenario SF: scenario:=blueye_field_twin, two rocks on the transit leg). The sonar is not injected: the rocks
give the simulated sonar its Near and Critical readings, and the operator avoids them with the joystick.

  start                   USBL Strong (the field's virtual USBL: no USBL fitted, nominal unless injected)
  F1, rock A              nothing injected: the sonar reads Critical on the approach
  F2, rock B              USBL Weak once the vehicle is within weak_distance_m of rock B in Transit: 3.5 m gives, at
                          the transit speed of about 0.1 m/s, Weak about 11 s before sonar Near (2.4 m) and 23 s before
                          Critical (1.0 m), as in the field. USBL stays Weak, as in the field
  F3, Inspection          USBL Lost lost_delay_s after Inspection starts (field: 20 s), Strong restore_after_s later
                          (field: 32 s); hand back once it is Strong
  nominal:=true           USBL Strong for the whole mission, nothing else (the simulator twin of the nominal field run)

The USBL goes to usbl_strength_monitor on /blueye/virtual/usbl_command, which logs each change on /scenario/events.
The sonar readings the rocks cause are logged there too ({"event": "sonar", "state": ..., "source": "simulated"}), so
the field analysis finds every change of the run in the same place.

Start it before the behaviour tree, after the takeover network (it starts usbl_strength_monitor):
  ros2 run blueye_takeover_bn field_twin_events [--ros-args -p nominal:=true]
"""

import json
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32, String

TRANSIT, INSPECTION = 2, 3


class FieldTwinEvents(Node):
    def __init__(self):
        super().__init__("field_twin_events")
        self.declare_parameter("nominal", False)
        self.declare_parameter("scenario_config", os.path.join(
            get_package_share_directory("stonefish_sim"), "config", "blueye_field_twin_config.yaml"))
        self.declare_parameter("weak_distance_m", 3.5)
        self.declare_parameter("lost_delay_s", 20.0)
        self.declare_parameter("restore_after_s", 32.0)

        config = yaml.safe_load(open(self.get_parameter("scenario_config").value))
        self.rock_b = [float(v) for v in str(config["transit_rock_position"]).split()[:2]]
        self.nominal = bool(self.get_parameter("nominal").value)
        self.state = None               # mission state
        self.usbl = "Strong"
        self.weak_done = False
        self.inspection_start = None
        self.sonar = None

        self.usbl_pub = self.create_publisher(String, "/blueye/virtual/usbl_command", 10)
        self.event_pub = self.create_publisher(String, "/scenario/events", 10)
        self.create_subscription(Int32, "/mission_state", self._state_cb, 10)
        self.create_subscription(Odometry, "/blueye/odom", self._odom_cb, 10)
        self.create_subscription(String, "/blueye/takeover_request/evidence", self._evidence_cb, 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info("Field twin " + ("(nominal): USBL Strong throughout" if self.nominal else
                               f"(F1-F3): USBL Strong; Weak within {self.get_parameter('weak_distance_m').value:.1f} m "
                               f"of rock B {self.rock_b}; Lost {self.get_parameter('lost_delay_s').value:.0f} s into "
                               f"Inspection, Strong {self.get_parameter('restore_after_s').value:.0f} s later"))

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _set_usbl(self, state, why):
        if state != self.usbl:
            self.get_logger().info(f"USBL {state}: {why}")
            self.usbl = state
            self._send_usbl()                      # at once: Weak must come before the sonar sees rock B

    def _send_usbl(self):
        out = String()
        out.data = self.usbl
        self.usbl_pub.publish(out)

    def _state_cb(self, msg: Int32):
        self.state = msg.data
        if msg.data == INSPECTION and self.inspection_start is None:
            self.inspection_start = self._now()

    def _odom_cb(self, msg: Odometry):
        if self.nominal or self.weak_done or self.state != TRANSIT:
            return
        p = msg.pose.pose.position
        distance = math.hypot(p.x - self.rock_b[0], p.y - self.rock_b[1])
        if distance <= float(self.get_parameter("weak_distance_m").value):
            self.weak_done = True
            self._set_usbl("Weak", f"{distance:.1f} m from rock B (F2)")

    def _evidence_cb(self, msg: String):
        sonar = json.loads(msg.data).get("evidence", {}).get("SonarRange")
        if sonar is not None and sonar != self.sonar:
            if self.sonar is not None:
                event = String()
                event.data = json.dumps({"event": "sonar", "state": sonar, "source": "simulated"})
                self.event_pub.publish(event)
            self.sonar = sonar

    def _tick(self):
        if not self.nominal and self.inspection_start is not None:
            elapsed = self._now() - self.inspection_start
            lost = float(self.get_parameter("lost_delay_s").value)
            restore = lost + float(self.get_parameter("restore_after_s").value)
            if lost <= elapsed < restore:
                self._set_usbl("Lost", "Inspection (F3)")
            elif elapsed >= restore:
                self._set_usbl("Strong", "restored after F3")
        self._send_usbl()                          # also every 0.5 s: the monitor logs only the changes


def main(args=None):
    rclpy.init(args=args)
    node = FieldTwinEvents()
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
