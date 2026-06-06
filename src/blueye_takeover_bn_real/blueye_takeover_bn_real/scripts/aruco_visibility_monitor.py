#!/usr/bin/env python3
from typing import Optional

import rclpy
from blueye_interfaces.msg import ArucoDetectionStatus
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String


class ArucoVisibilityMonitor(Node):
    """Convert real ArUco detection status into BN-style visibility states."""

    def __init__(self):
        super().__init__("aruco_visibility_monitor")
        self.declare_parameter("aruco_status_topic", "/blueye/aruco/status")
        self.declare_parameter("visibility_topic", "/blueye/aruco_visibility")
        self.declare_parameter("marker_count_topic", "/blueye/aruco_marker_count")
        self.declare_parameter("docking_detected_topic", "/blueye/docking_station_detected")
        self.declare_parameter("expected_markers", 6)
        self.declare_parameter("min_some_markers", 1)
        self.declare_parameter("timeout_s", 1.5)

        self.last_status_time = None
        self.last_visibility: Optional[str] = None
        self.last_marker_count = 0

        self.visibility_pub = self.create_publisher(
            String,
            str(self.get_parameter("visibility_topic").value),
            10,
        )
        self.marker_count_pub = self.create_publisher(
            Int32,
            str(self.get_parameter("marker_count_topic").value),
            10,
        )
        self.docking_detected_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("docking_detected_topic").value),
            10,
        )

        self.create_subscription(
            ArucoDetectionStatus,
            str(self.get_parameter("aruco_status_topic").value),
            self.status_callback,
            10,
        )
        self.create_timer(0.5, self.timeout_callback)

        self.get_logger().info(
            f"Listening to ArUco status on {self.get_parameter('aruco_status_topic').value}"
        )

    def status_callback(self, msg: ArucoDetectionStatus):
        self.last_status_time = self.get_clock().now()
        marker_count = int(msg.num_tags)
        if not msg.detection_valid or marker_count <= 0:
            marker_count = 0

        self.last_marker_count = marker_count
        visibility = self._visibility_from_count(marker_count)
        self.last_visibility = visibility
        self._publish(visibility, marker_count)

    def timeout_callback(self):
        if self.last_status_time is None:
            self._publish("None", 0)
            return

        age = (self.get_clock().now() - self.last_status_time).nanoseconds / 1e9
        if age > float(self.get_parameter("timeout_s").value):
            self._publish("None", 0)

    def _visibility_from_count(self, marker_count: int) -> str:
        expected = int(self.get_parameter("expected_markers").value)
        min_some = int(self.get_parameter("min_some_markers").value)
        if marker_count >= expected:
            return "All"
        if marker_count >= min_some:
            return "Some"
        return "None"

    def _publish(self, visibility: str, marker_count: int):
        visibility_msg = String()
        visibility_msg.data = visibility
        self.visibility_pub.publish(visibility_msg)

        count_msg = Int32()
        count_msg.data = int(marker_count)
        self.marker_count_pub.publish(count_msg)

        detected_msg = Bool()
        detected_msg.data = visibility != "None"
        self.docking_detected_pub.publish(detected_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoVisibilityMonitor()
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
