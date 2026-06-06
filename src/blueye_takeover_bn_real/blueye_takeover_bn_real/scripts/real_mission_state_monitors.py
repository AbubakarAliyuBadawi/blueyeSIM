#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from marine_acoustic_msgs.msg import Dvl
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String


class RealMissionStateMonitors(Node):
    """Publish discrete BN/BT states from real Blueye telemetry topics."""

    def __init__(self):
        super().__init__("real_mission_state_monitors")
        self._declare_parameters()

        self.latest_pose: Optional[Pose] = None
        self.latest_waypoint: Optional[Pose] = None
        self.last_dvl_time = None
        self.last_battery_state: Optional[str] = None
        self.last_battery_percent: Optional[float] = None

        self.depth_state_pub = self.create_publisher(String, "/blueye/depth_state", 10)
        self.battery_level_pub = self.create_publisher(Float32, "/blueye/battery_level", 10)
        self.battery_state_pub = self.create_publisher(String, "/blueye/battery_state", 10)
        self.battery_charge_state_pub = self.create_publisher(String, "/blueye/battery_charge_state", 10)
        self.battery_runtime_pub = self.create_publisher(Float32, "/blueye/battery_runtime_to_empty", 10)
        self.battery_current_pub = self.create_publisher(Float32, "/blueye/battery_current", 10)
        self.altitude_state_pub = self.create_publisher(String, "/blueye/altitude_state", 10)
        self.dvl_status_pub = self.create_publisher(String, "/blueye/dvl_status", 10)
        self.speed_state_pub = self.create_publisher(String, "/blueye/speed_state", 10)
        self.tracking_error_pub = self.create_publisher(String, "/blueye/waypoint_tracking_error", 10)
        self.tracking_error_m_pub = self.create_publisher(Float32, "/blueye/waypoint_tracking_error_m", 10)

        self.create_subscription(
            Float32,
            str(self.get_parameter("depth_topic").value),
            self.depth_callback,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("altitude_topic").value),
            self.altitude_callback,
            10,
        )
        self.create_subscription(
            Pose,
            str(self.get_parameter("battery_topic").value),
            self.battery_callback,
            10,
        )
        self.create_subscription(
            Dvl,
            str(self.get_parameter("dvl_topic").value),
            self.dvl_callback,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("speed_topic").value),
            self.speed_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self.pose_callback,
            10,
        )
        self.create_subscription(
            Pose,
            str(self.get_parameter("waypoint_topic").value),
            self.waypoint_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("desired_state_topic").value),
            self.desired_state_callback,
            10,
        )

        self.create_timer(1.0, self.timeout_callback)
        self.get_logger().info("Real mission state monitors started")

    def _declare_parameters(self):
        self.declare_parameter("depth_topic", "/blueye/depth")
        self.declare_parameter("altitude_topic", "/blueye/altitude")
        self.declare_parameter("battery_topic", "/blueye/battery")
        self.declare_parameter("dvl_topic", "/blueye/sensor/dvl")
        self.declare_parameter("speed_topic", "/blueye/speed")
        self.declare_parameter("pose_topic", "/blueye/pose")
        self.declare_parameter("waypoint_topic", "/blueye/current_waypoint")
        self.declare_parameter("desired_state_topic", "/blueye/guidance/desired_state")

        self.declare_parameter("depth_shallow_m", 0.5)
        self.declare_parameter("depth_deep_m", 5.0)
        self.declare_parameter("altitude_safe_m", 2.0)
        self.declare_parameter("altitude_marginal_m", 1.0)
        self.declare_parameter("battery_high_pct", 75.0)
        self.declare_parameter("battery_medium_pct", 50.0)
        self.declare_parameter("battery_low_pct", 25.0)
        self.declare_parameter("battery_charging_current_threshold_a", 0.1)
        self.declare_parameter("battery_runtime_full_s", 3600.0)
        self.declare_parameter("battery_runtime_high_s", 1800.0)
        self.declare_parameter("battery_runtime_medium_s", 900.0)
        self.declare_parameter("battery_runtime_low_s", 300.0)
        self.declare_parameter("dvl_timeout_s", 2.0)
        self.declare_parameter("dvl_degraded_good_beams", 3)
        self.declare_parameter("speed_safe_mps", 0.3)
        self.declare_parameter("speed_moderate_mps", 0.7)
        self.declare_parameter("tracking_low_m", 0.5)
        self.declare_parameter("tracking_high_m", 1.5)

    def depth_callback(self, msg: Float32):
        depth = float(msg.data)
        shallow = float(self.get_parameter("depth_shallow_m").value)
        deep = float(self.get_parameter("depth_deep_m").value)
        if depth < shallow:
            state = "Shallow"
        elif depth > deep:
            state = "Deep"
        else:
            state = "Nominal"
        self._publish_string(self.depth_state_pub, state)

    def altitude_callback(self, msg: Float32):
        altitude = float(msg.data)
        safe = float(self.get_parameter("altitude_safe_m").value)
        marginal = float(self.get_parameter("altitude_marginal_m").value)
        if altitude > safe:
            state = "Safe"
        elif altitude > marginal:
            state = "Marginal"
        else:
            state = "Unsafe"
        self._publish_string(self.altitude_state_pub, state)

    def battery_callback(self, msg: Pose):
        charging_current = float(msg.position.x)
        battery_current = float(msg.position.z)
        runtime_to_empty = float(msg.orientation.x)

        self._publish_float(self.battery_current_pub, battery_current)
        self._publish_float(self.battery_runtime_pub, runtime_to_empty)

        charging_threshold = float(self.get_parameter("battery_charging_current_threshold_a").value)
        if charging_current > charging_threshold:
            self._publish_string(self.battery_charge_state_pub, "Charging")
        else:
            self._publish_string(self.battery_charge_state_pub, "Discharging")

        soc = self._battery_soc_percent(msg)
        if soc is not None:
            battery_percent = soc
            state = self._battery_state_from_soc(soc)
        elif runtime_to_empty > 0.0:
            battery_percent = self._battery_percent_from_runtime(runtime_to_empty)
            state = self._battery_state_from_soc(battery_percent)
        else:
            battery_percent = self.last_battery_percent
            state = self.last_battery_state

        if battery_percent is None or state is None:
            self.get_logger().warning(
                "Battery message had no usable SOC or runtime; skipping state update"
            )
            return

        self.last_battery_percent = battery_percent
        self.last_battery_state = state
        self._publish_float(self.battery_level_pub, battery_percent)
        self._publish_string(self.battery_state_pub, state)

    def _battery_soc_percent(self, msg: Pose) -> Optional[float]:
        soc = float(msg.position.y)
        if soc <= 0.0:
            return None
        if soc <= 1.5:
            soc *= 100.0
        return max(0.0, min(100.0, soc))

    def _battery_state_from_soc(self, soc: float) -> str:
        high = float(self.get_parameter("battery_high_pct").value)
        medium = float(self.get_parameter("battery_medium_pct").value)
        low = float(self.get_parameter("battery_low_pct").value)
        if soc > high:
            return "High"
        if soc > medium:
            return "Medium"
        if soc > low:
            return "Low"
        return "Critical"

    def _battery_state_from_runtime(self, runtime_to_empty: float) -> str:
        high = float(self.get_parameter("battery_runtime_high_s").value)
        medium = float(self.get_parameter("battery_runtime_medium_s").value)
        low = float(self.get_parameter("battery_runtime_low_s").value)
        if runtime_to_empty > high:
            return "High"
        if runtime_to_empty > medium:
            return "Medium"
        if runtime_to_empty > low:
            return "Low"
        return "Critical"

    def _battery_percent_from_runtime(self, runtime_to_empty: float) -> float:
        full_runtime = max(float(self.get_parameter("battery_runtime_full_s").value), 1.0)
        return max(0.0, min(100.0, 100.0 * runtime_to_empty / full_runtime))

    def dvl_callback(self, msg: Dvl):
        self.last_dvl_time = self.get_clock().now()
        degraded_good_beams = int(self.get_parameter("dvl_degraded_good_beams").value)

        if not self._dvl_velocity_valid(msg):
            state = "Failed"
        elif int(msg.num_good_beams) < degraded_good_beams:
            state = "Degraded"
        else:
            state = "Nominal"
        self._publish_string(self.dvl_status_pub, state)

    def speed_callback(self, msg: Float32):
        speed = abs(float(msg.data))
        safe = float(self.get_parameter("speed_safe_mps").value)
        moderate = float(self.get_parameter("speed_moderate_mps").value)
        if speed < safe:
            state = "Safe"
        elif speed < moderate:
            state = "Moderate"
        else:
            state = "High"
        self._publish_string(self.speed_state_pub, state)

    @staticmethod
    def _dvl_velocity_valid(msg: Dvl) -> bool:
        for attr in ("velocity_valid", "vel_valid"):
            if hasattr(msg, attr):
                return bool(getattr(msg, attr))
        return True

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg.pose
        self._update_tracking_error()

    def waypoint_callback(self, msg: Pose):
        self.latest_waypoint = msg
        self._update_tracking_error()

    def desired_state_callback(self, msg: Odometry):
        self.latest_waypoint = msg.pose.pose
        self._update_tracking_error()

    def _update_tracking_error(self):
        if self.latest_pose is None or self.latest_waypoint is None:
            return

        error = self._distance(self.latest_pose, self.latest_waypoint)
        self._publish_float(self.tracking_error_m_pub, error)

        low = float(self.get_parameter("tracking_low_m").value)
        high = float(self.get_parameter("tracking_high_m").value)
        if error < low:
            state = "Low"
        elif error > high:
            state = "High"
        else:
            state = "Medium"
        self._publish_string(self.tracking_error_pub, state)

    @staticmethod
    def _distance(a: Pose, b: Pose) -> float:
        dx = a.position.x - b.position.x
        dy = a.position.y - b.position.y
        dz = a.position.z - b.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def timeout_callback(self):
        if self.last_dvl_time is None:
            self._publish_string(self.dvl_status_pub, "Failed")
            return

        age = (self.get_clock().now() - self.last_dvl_time).nanoseconds / 1e9
        if age > float(self.get_parameter("dvl_timeout_s").value):
            self._publish_string(self.dvl_status_pub, "Failed")

    @staticmethod
    def _publish_string(pub, value: str):
        msg = String()
        msg.data = value
        pub.publish(msg)

    @staticmethod
    def _publish_float(pub, value: float):
        msg = Float32()
        msg.data = float(value)
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RealMissionStateMonitors()
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
