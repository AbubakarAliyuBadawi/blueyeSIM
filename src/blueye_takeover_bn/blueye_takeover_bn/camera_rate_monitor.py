"""Monitors the camera frame rate on /blueye/cam/image_color/compressed and maps it to CameraQuality states.

The compressed stream is counted because it is small: subscribing to the raw 1920x1080 images made this node fall
behind under load and report a camera failure while the camera itself kept running at 10 Hz.

The published state is the best state seen over the last min_degraded_s, so a worse state is only
reported once every sample in that window was at least that bad. Short frame-rate dips of the simulator
(render stalls, start-up) are therefore not reported as a camera failure; a better state is published at once.
"""

import collections
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class CameraRateMonitor(Node):
    QUALITY_ORDER = ("Excellent", "Good", "Poor", "Failed")

    def __init__(self):
        super().__init__("camera_rate_monitor")

        self.declare_parameter("camera_topic", "/blueye/cam/image_color/compressed")
        self.declare_parameter("camera_quality_topic", "/blueye/camera_quality")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("window_size", 20)
        # Thresholds (Hz) for state transitions
        self.declare_parameter("excellent_threshold_hz", 8.0)
        self.declare_parameter("good_threshold_hz", 5.0)
        self.declare_parameter("poor_threshold_hz", 1.0)
        self.declare_parameter("min_degraded_s", 3.0)

        self._history = collections.deque()   # (time, raw state) over the last min_degraded_s
        self._started = None
        self._timestamps = collections.deque(
            maxlen=int(self.get_parameter("window_size").value)
        )

        self._pub = self.create_publisher(
            String,
            self.get_parameter("camera_quality_topic").value,
            10,
        )

        self.create_subscription(
            CompressedImage,
            self.get_parameter("camera_topic").value,
            self._image_cb,
            rclpy.qos.qos_profile_sensor_data,
        )

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(period, self._publish_quality)

        self.get_logger().info(
            f"Camera rate monitor started on '{self.get_parameter('camera_topic').value}'"
        )

    def _image_cb(self, _msg: CompressedImage):
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
        now = time.monotonic()
        hold = float(self.get_parameter("min_degraded_s").value)
        if self._started is None:
            self._started = now
        self._history.append((now, state))
        while now - self._history[0][0] > hold:
            self._history.popleft()
        best = min((s for _, s in self._history), key=self.QUALITY_ORDER.index)
        if best not in ("Excellent", "Good") and now - self._started < hold:
            return                                        # start-up: nothing known yet
        msg = String()
        msg.data = best
        self._pub.publish(msg)
        self.get_logger().debug(f"Camera rate: {rate:.2f} Hz → {state}, published: {best}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraRateMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
