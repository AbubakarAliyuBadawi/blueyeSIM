import re
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class UsblQuality:
    rssi_db: int
    integrity: int


class UsblStrengthMonitor(Node):
    """Convert raw Evologics RECVIM packets into a simple USBL strength state."""

    RECVIM_PATTERN = re.compile(
        r"RECVIM,[^,]*,[^,]*,[^,]*,[^,]*,[^,]*,"
        r"(?P<rssi>-?\d+),(?P<integrity>\d+),"
    )

    def __init__(self):
        super().__init__("usbl_strength_monitor")
        self.declare_parameter("raw_data_topic", "/blueye/evologics/raw_data")
        self.declare_parameter("strength_topic", "/blueye/usbl_strength")
        self.declare_parameter("strong_rssi_db", -55)
        self.declare_parameter("moderate_rssi_db", -65)
        self.declare_parameter("weak_rssi_db", -75)
        self.declare_parameter("strong_integrity", 100)
        self.declare_parameter("moderate_integrity", 70)
        self.declare_parameter("weak_integrity", 40)
        self.declare_parameter("strong_state", "Strong")
        self.declare_parameter("moderate_state", "Moderate")
        self.declare_parameter("weak_state", "Weak")
        self.declare_parameter("lost_state", "Lost")

        raw_topic = self.get_parameter("raw_data_topic").value
        strength_topic = self.get_parameter("strength_topic").value

        self.publisher = self.create_publisher(String, strength_topic, 10)
        self.create_subscription(String, raw_topic, self.raw_data_callback, 10)

        self.get_logger().info(f"Listening to raw USBL data on {raw_topic}")
        self.get_logger().info(f"Publishing USBL strength on {strength_topic}")

    def raw_data_callback(self, msg: String):
        quality = self._parse_quality(msg.data)
        if quality is None:
            self._publish_state(str(self.get_parameter("lost_state").value))
            return

        strong_rssi = int(self.get_parameter("strong_rssi_db").value)
        moderate_rssi = int(self.get_parameter("moderate_rssi_db").value)
        weak_rssi = int(self.get_parameter("weak_rssi_db").value)
        strong_integrity = int(self.get_parameter("strong_integrity").value)
        moderate_integrity = int(self.get_parameter("moderate_integrity").value)
        weak_integrity = int(self.get_parameter("weak_integrity").value)

        if quality.rssi_db >= strong_rssi and quality.integrity >= strong_integrity:
            state = str(self.get_parameter("strong_state").value)
        elif quality.rssi_db >= moderate_rssi and quality.integrity >= moderate_integrity:
            state = str(self.get_parameter("moderate_state").value)
        elif quality.rssi_db >= weak_rssi and quality.integrity >= weak_integrity:
            state = str(self.get_parameter("weak_state").value)
        else:
            state = str(self.get_parameter("lost_state").value)

        self._publish_state(state)
        self.get_logger().info(
            f"USBL {state}: RSSI={quality.rssi_db} dB, integrity={quality.integrity}"
        )

    def _parse_quality(self, raw_data: str) -> Optional[UsblQuality]:
        match = self.RECVIM_PATTERN.search(raw_data)
        if match is None:
            return None
        return UsblQuality(
            rssi_db=int(match.group("rssi")),
            integrity=int(match.group("integrity")),
        )

    def _publish_state(self, state: str):
        msg = String()
        msg.data = state
        self.publisher.publish(msg)


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
