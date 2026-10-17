#!/usr/bin/env python3
"""
Simulated water current, published on /blueye/current like the manual mode of the real current publisher.

The current is the uniform ocean current of the Stonefish scenario, so the scenario file is the only place to set it.
Stonefish keeps currents off until /stonefish_ros2/enable_currents is called. This node switches them on
`enable_after_s` seconds after it starts (0 = at once) and publishes the horizontal current speed while they are on,
0.0 while they are off.

  ros2 param set /sim_current enabled false   # switch the current off during a run (true: on again)
"""

import math
import os
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger


class SimCurrent(Node):
    def __init__(self):
        super().__init__("sim_current")

        self.declare_parameter("scenario_file", "")
        self.declare_parameter("current_topic", "/blueye/current")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("enabled", True)
        self.declare_parameter("enable_after_s", 0.0)

        self.speed = self._scenario_current_speed()
        self.active = None                                 # unknown until the first service call succeeds
        self.pending = None
        self.start = self.get_clock().now()

        self.enable_client = self.create_client(Trigger, "/stonefish_ros2/enable_currents")
        self.disable_client = self.create_client(Trigger, "/stonefish_ros2/disable_currents")
        self.pub = self.create_publisher(Float32, self.get_parameter("current_topic").value, 10)

        rate = max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.create_timer(1.0 / rate, self._tick)

    def _scenario_current_speed(self) -> float:
        path = str(self.get_parameter("scenario_file").value) or os.path.join(
            get_package_share_directory("stonefish_sim"), "scenarios", "blueye.scn")
        vx = vy = 0.0
        for current in ET.parse(path).getroot().findall("environment/ocean/current"):
            if current.get("type") != "uniform":
                self.get_logger().warning(f"Ignoring '{current.get('type')}' current in {path}: only uniform is read")
                continue
            x, y, _ = (float(v) for v in current.find("velocity").get("xyz").split())
            vx, vy = vx + x, vy + y
        speed = math.hypot(vx, vy)
        self.get_logger().info(f"Scenario current: ({vx:.2f}, {vy:.2f}) m/s, horizontal speed {speed:.2f} m/s "
                               f"from {path}")
        return speed

    def _tick(self):
        elapsed = (self.get_clock().now() - self.start).nanoseconds / 1e9
        want = bool(self.get_parameter("enabled").value) and \
            elapsed >= float(self.get_parameter("enable_after_s").value)
        if want != self.active and self.pending is None:
            client = self.enable_client if want else self.disable_client
            if client.service_is_ready():                  # the simulator may start after this node
                self.pending = client.call_async(Trigger.Request())
                self.pending.add_done_callback(lambda future, want=want: self._switched(future, want))

        msg = Float32()
        msg.data = self.speed if self.active else 0.0
        self.pub.publish(msg)

    def _switched(self, future, want: bool):
        self.pending = None
        result = future.result()
        if result is not None and result.success:
            self.active = want
            self.get_logger().info(f"Ocean current {'on' if want else 'off'} ({self.speed:.2f} m/s)")
        else:
            self.get_logger().warning(f"Switching the ocean current {'on' if want else 'off'} failed; retrying")


def main(args=None):
    rclpy.init(args=args)
    node = SimCurrent()
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
