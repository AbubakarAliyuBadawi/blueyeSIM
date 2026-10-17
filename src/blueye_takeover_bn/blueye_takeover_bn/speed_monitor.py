#!/usr/bin/env python3
"""Vehicle speed on /blueye/speed: norm of the linear velocity in the simulator odometry."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


class SpeedMonitor(Node):
    def __init__(self):
        super().__init__("speed_monitor")

        self.declare_parameter("odometry_topic", "/blueye/odom")
        self.declare_parameter("speed_topic", "/blueye/speed")
        self.declare_parameter("publish_rate_hz", 2.0)

        self._speed = None

        self._pub = self.create_publisher(Float32, self.get_parameter("speed_topic").value, 10)
        self.create_subscription(Odometry, self.get_parameter("odometry_topic").value, self._odometry_cb, 10)
        rate = max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.create_timer(1.0 / rate, self._publish)

    def _odometry_cb(self, msg: Odometry):
        v = msg.twist.twist.linear
        self._speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

    def _publish(self):
        if self._speed is None:
            return
        msg = Float32()
        msg.data = float(self._speed)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedMonitor()
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
