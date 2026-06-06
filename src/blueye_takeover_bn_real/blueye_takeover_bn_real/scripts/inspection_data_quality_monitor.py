#!/usr/bin/env python3
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class InspectionDataQualityMonitor(Node):
    """Estimate whether the inspection data stream is usable for the mission."""

    def __init__(self):
        super().__init__("inspection_data_quality_monitor")
        self._declare_parameters()

        self.last_seen = {}
        self.camera_quality: Optional[str] = None
        self.sonar_range: Optional[str] = None
        self.dvl_status: Optional[str] = None
        self.usbl_strength: Optional[str] = None
        self.position_valid: Optional[bool] = None
        self.mission_phase: Optional[str] = None

        self.quality_pub = self.create_publisher(String, "/blueye/inspection_data_quality", 10)
        self.score_pub = self.create_publisher(Float32, "/blueye/inspection_data_quality_score", 10)
        self.reason_pub = self.create_publisher(String, "/blueye/inspection_data_quality_reason", 10)

        self.create_subscription(
            String,
            str(self.get_parameter("mission_phase_topic").value),
            self.mission_phase_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("camera_quality_topic").value),
            self.camera_quality_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("sonar_range_topic").value),
            self.sonar_range_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("dvl_status_topic").value),
            self.dvl_status_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("usbl_strength_topic").value),
            self.usbl_strength_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("position_valid_topic").value),
            self.position_valid_callback,
            10,
        )

        rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.create_timer(1.0 / rate_hz, self.publish_quality)
        self.get_logger().info("Inspection data quality monitor started")

    def _declare_parameters(self):
        self.declare_parameter("mission_phase_topic", "/mission/phase")
        self.declare_parameter("camera_quality_topic", "/blueye/camera_quality")
        self.declare_parameter("sonar_range_topic", "/blueye/sonar_range")
        self.declare_parameter("dvl_status_topic", "/blueye/dvl_status")
        self.declare_parameter("usbl_strength_topic", "/blueye/usbl_strength")
        self.declare_parameter("position_valid_topic", "/blueye/position_valid")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("require_inspection_phase", False)
        self.declare_parameter("fresh_timeout_s", 3.0)

    def mission_phase_callback(self, msg: String):
        self.mission_phase = msg.data.strip()
        self._mark_seen("mission_phase")

    def camera_quality_callback(self, msg: String):
        self.camera_quality = msg.data.strip()
        self._mark_seen("camera")

    def sonar_range_callback(self, msg: String):
        self.sonar_range = msg.data.strip()
        self._mark_seen("sonar")

    def dvl_status_callback(self, msg: String):
        self.dvl_status = msg.data.strip()
        self._mark_seen("dvl")

    def usbl_strength_callback(self, msg: String):
        self.usbl_strength = msg.data.strip()
        self._mark_seen("usbl")

    def position_valid_callback(self, msg: Bool):
        self.position_valid = bool(msg.data)
        self._mark_seen("position")

    def publish_quality(self):
        if bool(self.get_parameter("require_inspection_phase").value) and not self._in_inspection_phase():
            self._publish("Inadequate", 0.0, "not in inspection phase")
            return

        score = 0.0
        reasons = []

        score += self._fresh_score("camera", 0.20, reasons)
        score += self._fresh_score("sonar", 0.20, reasons)
        score += self._fresh_score("dvl", 0.15, reasons)
        score += self._fresh_score("usbl", 0.15, reasons)
        score += self._fresh_score("position", 0.15, reasons)
        score += self._fresh_score("mission_phase", 0.05, reasons, optional=True)

        score += self._camera_score(reasons)
        score += self._navigation_score(reasons)

        if score >= 0.75:
            state = "Adequate"
        elif score >= 0.45:
            state = "Marginal"
        else:
            state = "Inadequate"

        self._publish(state, score, "; ".join(reasons) if reasons else "all inspection data healthy")

    def _camera_score(self, reasons) -> float:
        quality = (self.camera_quality or "").lower()
        if quality == "excellent":
            return 0.15
        if quality == "good":
            return 0.12
        if quality == "poor":
            reasons.append("camera quality poor")
            return 0.05
        reasons.append("camera quality failed or unknown")
        return 0.0

    def _navigation_score(self, reasons) -> float:
        dvl = (self.dvl_status or "").lower()
        usbl = (self.usbl_strength or "").lower()
        position_valid = self.position_valid is True

        score = 0.0
        if dvl == "nominal":
            score += 0.07
        elif dvl == "degraded":
            score += 0.03
            reasons.append("dvl degraded")
        else:
            reasons.append("dvl failed or unknown")

        if usbl in ("strong", "moderate"):
            score += 0.05
        elif usbl == "weak":
            score += 0.02
            reasons.append("usbl weak")
        else:
            reasons.append("usbl lost or unknown")

        if position_valid:
            score += 0.03
        else:
            reasons.append("position invalid or unknown")

        return score

    def _fresh_score(self, key: str, weight: float, reasons, optional: bool = False) -> float:
        if self._fresh(key):
            return weight
        if not optional:
            reasons.append(f"{key} stale")
        return 0.0

    def _fresh(self, key: str) -> bool:
        if key not in self.last_seen:
            return False
        age = (self.get_clock().now() - self.last_seen[key]).nanoseconds / 1e9
        return age <= float(self.get_parameter("fresh_timeout_s").value)

    def _mark_seen(self, key: str):
        self.last_seen[key] = self.get_clock().now()

    def _in_inspection_phase(self) -> bool:
        phase = (self.mission_phase or "").lower()
        return "inspect" in phase or "pipeline" in phase

    def _publish(self, state: str, score: float, reason: str):
        state_msg = String()
        state_msg.data = state
        self.quality_pub.publish(state_msg)

        score_msg = Float32()
        score_msg.data = float(max(0.0, min(1.0, score)))
        self.score_pub.publish(score_msg)

        reason_msg = String()
        reason_msg.data = reason
        self.reason_pub.publish(reason_msg)


def main(args=None):
    rclpy.init(args=args)
    node = InspectionDataQualityMonitor()
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
