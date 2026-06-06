#!/usr/bin/env python3
import math
import random
from typing import Optional

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    import requests
except ImportError:
    requests = None


class RealCurrentPublisher(Node):
    """Publish real-mission water current speed on /blueye/current."""

    def __init__(self):
        super().__init__("real_current_publisher")
        self.declare_parameter("current_topic", "/blueye/current")
        self.declare_parameter("gps_topic", "/blueye/gps")
        self.declare_parameter("mode", "manual")
        self.declare_parameter("manual_current_mps", 0.2)
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("api_refresh_s", 1800.0)
        self.declare_parameter("stormglass_api_key", "")
        self.declare_parameter("default_lat", 63.4414548287786)
        self.declare_parameter("default_lon", 10.3482882678509)
        self.declare_parameter("noise_mps", 0.005)

        self.current_mps = float(self.get_parameter("manual_current_mps").value)
        self.lat = float(self.get_parameter("default_lat").value)
        self.lon = float(self.get_parameter("default_lon").value)

        current_topic = str(self.get_parameter("current_topic").value)
        gps_topic = str(self.get_parameter("gps_topic").value)

        self.publisher = self.create_publisher(Float32, current_topic, 10)
        self.create_subscription(Point, gps_topic, self.gps_callback, 10)

        publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        api_refresh_s = max(float(self.get_parameter("api_refresh_s").value), 60.0)

        self.create_timer(1.0 / publish_rate_hz, self.publish_current)
        self.create_timer(api_refresh_s, self.fetch_current_from_api)

        if self._api_enabled():
            self.fetch_current_from_api()

        self.get_logger().info(f"Publishing current speed on {current_topic}")
        self.get_logger().info(
            f"Mode={self.get_parameter('mode').value}, current={self.current_mps:.3f} m/s"
        )

    def gps_callback(self, msg: Point):
        self.lat = float(msg.x)
        self.lon = float(msg.y)

    def publish_current(self):
        noise = float(self.get_parameter("noise_mps").value)
        current = max(0.0, self.current_mps + random.uniform(-noise, noise))

        msg = Float32()
        msg.data = current
        self.publisher.publish(msg)

    def fetch_current_from_api(self):
        if not self._api_enabled():
            return
        if requests is None:
            self.get_logger().warning("requests is not installed; using manual current value")
            return

        api_key = str(self.get_parameter("stormglass_api_key").value)
        url = "https://api.stormglass.io/v2/weather/point"
        params = {
            "lat": self.lat,
            "lng": self.lon,
            "params": "currentSpeed",
        }
        headers = {"Authorization": api_key}

        try:
            response = requests.get(url, params=params, headers=headers, timeout=15)
            if response.status_code != 200:
                self.get_logger().warning(f"Stormglass API error: {response.status_code}")
                return

            data = response.json()
            hour = data["hours"][0]
            current = self._best_current_value(hour.get("currentSpeed", {}))
            if current is None or not math.isfinite(current):
                self.get_logger().warning("Stormglass response did not contain currentSpeed")
                return

            self.current_mps = max(0.0, float(current))
            self.get_logger().info(f"Updated current from API: {self.current_mps:.3f} m/s")
        except Exception as exc:
            self.get_logger().warning(f"Stormglass fetch failed: {exc}")

    def _api_enabled(self) -> bool:
        return (
            str(self.get_parameter("mode").value).lower() == "stormglass"
            and bool(str(self.get_parameter("stormglass_api_key").value))
        )

    @staticmethod
    def _best_current_value(current_speed_by_source) -> Optional[float]:
        preferred_sources = ("noaa", "sg", "meteo")
        for source in preferred_sources:
            value = current_speed_by_source.get(source)
            if value is not None:
                return float(value)
        for value in current_speed_by_source.values():
            if value is not None:
                return float(value)
        return None


def main(args=None):
    rclpy.init(args=args)
    node = RealCurrentPublisher()
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
