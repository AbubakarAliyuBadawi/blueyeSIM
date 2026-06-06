#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32
from std_msgs.msg import String


class SonarRangeMonitor(Node):
    """Classify real 3D sonar returns as Critical, Near, or Clear."""

    def __init__(self):
        super().__init__("sonar_range_monitor")
        self.declare_parameter("pointcloud_topic", "/blueye/sonar_3d/pointcloud_intensity")
        self.declare_parameter("range_state_topic", "/blueye/sonar_range")
        self.declare_parameter("critical_range_m", 0.7)
        self.declare_parameter("clear_range_m", 2.0)
        self.declare_parameter("min_x_m", 0.05)
        self.declare_parameter("max_x_m", 5.0)
        self.declare_parameter("max_abs_y_m", 1.0)
        self.declare_parameter("max_abs_z_m", 0.8)
        self.declare_parameter("min_intensity", 0)
        self.declare_parameter("min_valid_points", 5)

        pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        range_state_topic = str(self.get_parameter("range_state_topic").value)

        self.publisher = self.create_publisher(String, range_state_topic, 10)
        self.nearest_range_publisher = self.create_publisher(Float32, "/blueye/sonar_nearest_range", 10)
        self.create_subscription(PointCloud2, pointcloud_topic, self.pointcloud_callback, 10)

        self.get_logger().info(f"Listening to sonar point cloud on {pointcloud_topic}")
        self.get_logger().info(f"Publishing sonar range state on {range_state_topic}")

    def pointcloud_callback(self, msg: PointCloud2):
        nearest, valid_points = self._nearest_obstacle_range(msg)
        critical_range = float(self.get_parameter("critical_range_m").value)
        clear_range = float(self.get_parameter("clear_range_m").value)
        min_valid_points = int(self.get_parameter("min_valid_points").value)

        if nearest is None or valid_points < min_valid_points or nearest >= clear_range:
            state = "Clear"
        elif nearest < critical_range:
            state = "Critical"
        else:
            state = "Near"

        out = String()
        out.data = state
        self.publisher.publish(out)

        nearest_msg = Float32()
        nearest_msg.data = float("inf") if nearest is None else float(nearest)
        self.nearest_range_publisher.publish(nearest_msg)

        if nearest is None or valid_points < min_valid_points:
            self.get_logger().info(
                f"Sonar {state}: {valid_points} valid points in safety box"
            )
        else:
            self.get_logger().info(
                f"Sonar {state}: nearest obstacle {nearest:.2f} m from {valid_points} valid points"
            )

    def _nearest_obstacle_range(self, msg: PointCloud2) -> tuple[Optional[float], int]:
        field_names = [field.name for field in msg.fields]
        wanted_fields = ("x", "y", "z")
        if "intensity" in field_names:
            wanted_fields = ("x", "y", "z", "intensity")

        min_x = float(self.get_parameter("min_x_m").value)
        max_x = float(self.get_parameter("max_x_m").value)
        max_abs_y = float(self.get_parameter("max_abs_y_m").value)
        max_abs_z = float(self.get_parameter("max_abs_z_m").value)
        min_intensity = float(self.get_parameter("min_intensity").value)

        nearest = None
        valid_points = 0
        for point in point_cloud2.read_points(
            msg,
            field_names=wanted_fields,
            skip_nans=True,
        ):
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            intensity = float(point[3]) if len(point) > 3 else min_intensity

            if intensity < min_intensity:
                continue
            if not (min_x <= x <= max_x):
                continue
            if abs(y) > max_abs_y or abs(z) > max_abs_z:
                continue

            valid_points += 1
            distance = math.sqrt(x * x + y * y + z * z)
            if nearest is None or distance < nearest:
                nearest = distance

        return nearest, valid_points


def main(args=None):
    rclpy.init(args=args)
    node = SonarRangeMonitor()
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
