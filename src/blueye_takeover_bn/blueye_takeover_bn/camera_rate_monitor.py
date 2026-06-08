"""Monitors /blueye/cam/image_color publish rate and maps it to CameraQuality states."""

import collections
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class CameraRateMonitor(Node):
    def __init__(self):
        super().__init__("camera_rate_monitor")

        self.declare_parameter("camera_topic", "/blueye/cam/image_color")
        self.declare_parameter("camera_quality_topic", "/blueye/camera_quality")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("window_size", 20)
        # Thresholds (Hz) for state transitions
        self.declare_parameter("excellent_threshold_hz", 8.0)
        self.declare_parameter("good_threshold_hz", 5.0)
        self.declare_parameter("poor_threshold_hz", 1.0)

        self._timestamps = collections.deque(
            maxlen=int(self.get_parameter("window_size").value)
        )

        self._pub = self.create_publisher(
            String,
            self.get_parameter("camera_quality_topic").value,
            10,
        )

        self.create_subscription(
            Image,
            self.get_parameter("camera_topic").value,
            self._image_cb,
            rclpy.qos.qos_profile_sensor_data,
        )

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(period, self._publish_quality)

        self.get_logger().info(
            f"Camera rate monitor started on '{self.get_parameter('camera_topic').value}'"
        )

    def _image_cb(self, _msg: Image):
        self._timestamps.append(time.monotonic())

    def _compute_rate(self) -> float:
        now = time.monotonic()
        window = 2.0  # seconds to look back
        recent = [t for t in self._timestamps if now - t <= window]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / (recent[-1] - recent[0])

    def _rate_to_state(self, rate: float) -> str:
        excellent = float(self.get_parameter("excellent_threshold_hz").value)
        good = float(self.get_parameter("good_threshold_hz").value)
        poor = float(self.get_parameter("poor_threshold_hz").value)
        if rate >= excellent:
            return "Excellent"
        if rate >= good:
            return "Good"
        if rate >= poor:
            return "Poor"
        return "Failed"

    def _publish_quality(self):
        rate = self._compute_rate()
        state = self._rate_to_state(rate)
        msg = String()
        msg.data = state
        self._pub.publish(msg)
        self.get_logger().debug(f"Camera rate: {rate:.2f} Hz → CameraQuality: {state}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraRateMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
