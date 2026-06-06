#!/usr/bin/env python3
"""Full-mission Bayesian network node for takeover request inference."""

import json
import math
import os
from typing import Dict, Optional

import pysmile
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from mundus_mir_msgs.msg import BatteryStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from stonefish_ros2.msg import DVL


pysmile.License(
    (
        b"SMILE LICENSE b18e1861 e2fb7176 fd1b028d "
        b"THIS IS AN ACADEMIC LICENSE AND CAN BE USED "
        b"SOLELY FOR ACADEMIC RESEARCH AND TEACHING, "
        b"AS DEFINED IN THE BAYESFUSION ACADEMIC "
        b"SOFTWARE LICENSING AGREEMENT. "
        b"Serial #: 4aiii2myovtuprliry4nogll5 "
        b"Issued for: Abubakar Aliyu Badawi (badawi.abubakaraliyu@gmail.com) "
        b"Academic institution: Norwegian University of Science and Technology "
        b"Valid until: 2026-10-29 "
        b"Issued by BayesFusion activation server"
    ),
    [
        0xC1, 0x74, 0x1A, 0x4E, 0x81, 0xDB, 0x00, 0x2E, 0xD8, 0xE3, 0x62, 0x95, 0xA3, 0x1D, 0x14, 0x19,
        0xCA, 0x73, 0xBA, 0x84, 0x77, 0xE3, 0x02, 0x15, 0x99, 0xC7, 0x09, 0x8B, 0xC0, 0x7E, 0x13, 0xF9,
        0x84, 0xD6, 0x15, 0xAE, 0x23, 0x72, 0x16, 0x9F, 0x92, 0x86, 0xD3, 0x03, 0xD1, 0xE7, 0x59, 0xE8,
        0xE5, 0xB5, 0x73, 0xE3, 0xDB, 0x9D, 0xE0, 0xC6, 0x4D, 0x4C, 0x7F, 0x31, 0x9F, 0x47, 0x63, 0xBD,
    ],
)


class TakeoverBayesianNetwork(Node):
    """Passive BN aggregator for full-mission takeover request probability."""

    TARGET_NODE = "TakeoverRequest"

    def __init__(self):
        super().__init__("takeover_bayesian_network")
        self._declare_parameters()
        self.net = pysmile.Network()
        self.network_path = self._resolve_network_path()
        self.net.read_file(self.network_path)
        self.node_ids = set(self.net.get_all_node_ids())
        self.evidence: Dict[str, str] = {}
        self.last_seen: Dict[str, rclpy.time.Time] = {}

        self.current_pose: Optional[Pose] = None
        self.current_waypoint: Optional[Pose] = None
        self.current_reference: Optional[Pose] = None

        self._setup_publishers()
        self._setup_subscribers()

        update_rate = max(float(self.get_parameter("update_rate_hz").value), 0.1)
        self.timer = self.create_timer(1.0 / update_rate, self._periodic_update)

        if bool(self.get_parameter("use_default_mission_phase").value):
            default_phase = str(self.get_parameter("default_mission_phase").value)
            self._set_evidence("MissionPhase", default_phase, "default")

        self.get_logger().info(f"Loaded full mission BN: {self.network_path}")
        self.get_logger().info("Publishing takeover inference on /blueye/takeover_request/*")

    def _declare_parameters(self):
        self.declare_parameter("bn_file", "")
        self.declare_parameter("update_rate_hz", 2.0)
        self.declare_parameter("default_mission_phase", "Transit")
        self.declare_parameter("use_default_mission_phase", True)
        self.declare_parameter("takeover_threshold", 0.65)

        self.declare_parameter("mission_phase_topic", "/mission/phase")
        self.declare_parameter("battery_topic", "/blueye/battery")
        self.declare_parameter("dvl_topic", "/blueye/dvl/sim")
        self.declare_parameter("sonar_image_topic", "/blueye/fls/image")
        self.declare_parameter("pipeline_marker_ids_topic", "/blueye/pipeline_marker_ids")
        self.declare_parameter("aruco_visibility_topic", "/blueye/aruco_visibility")
        self.declare_parameter("current_waypoint_topic", "/blueye/pipeline_current_waypoint")
        self.declare_parameter("current_reference_topic", "/blueye/pipeline_current_reference")
        self.declare_parameter("odometry_topic", "/odometry/filtered")
        self.declare_parameter("current_topic", "/blueye/current")
        self.declare_parameter("speed_topic", "/blueye/speed")
        self.declare_parameter("altitude_topic", "/blueye/altitude")
        self.declare_parameter("camera_quality_topic", "/blueye/camera_quality")
        self.declare_parameter("usbl_strength_topic", "/blueye/usbl_strength")
        self.declare_parameter("fatigue_topic", "/blueye/human/fatigue")
        self.declare_parameter("stress_topic", "/blueye/human/stress")

        self.declare_parameter("dvl_timeout_s", 2.0)
        self.declare_parameter("aruco_timeout_s", 1.5)
        self.declare_parameter("sonar_timeout_s", 2.0)
        self.declare_parameter("target_timeout_s", 2.0)
        self.declare_parameter("dvl_cov_degraded_threshold", 0.4)
        self.declare_parameter("sonar_intensity_threshold", 60)
        self.declare_parameter("sonar_clear_range_m", 2.0)
        self.declare_parameter("sonar_critical_range_m", 0.7)
        self.declare_parameter("range_far_m", 5.0)
        self.declare_parameter("range_close_m", 1.0)
        self.declare_parameter("tracking_low_m", 0.5)
        self.declare_parameter("tracking_high_m", 1.5)
        self.declare_parameter("expected_aruco_markers", 6)

    def _resolve_network_path(self) -> str:
        configured = str(self.get_parameter("bn_file").value)
        if configured:
            return configured
        package_share = get_package_share_directory("blueye_takeover_bn")
        return os.path.join(package_share, "config", "full_mission.xdsl")

    def _setup_publishers(self):
        self.state_pub = self.create_publisher(String, "/blueye/takeover_request/state", 10)
        self.no_takeover_pub = self.create_publisher(Float32, "/blueye/takeover_request/no_takeover_prob", 10)
        self.attention_pub = self.create_publisher(Float32, "/blueye/takeover_request/attention_required_prob", 10)
        self.takeover_pub = self.create_publisher(Float32, "/blueye/takeover_request/takeover_requested_prob", 10)
        self.max_prob_pub = self.create_publisher(Float32, "/blueye/takeover_request/max_probability", 10)
        self.threshold_pub = self.create_publisher(Float32, "/blueye/takeover_request/threshold", 10)
        self.evidence_pub = self.create_publisher(String, "/blueye/takeover_request/evidence", 10)

    def _setup_subscribers(self):
        self.create_subscription(String, self.get_parameter("mission_phase_topic").value, self._mission_phase_cb, 10)
        self.create_subscription(BatteryStatus, self.get_parameter("battery_topic").value, self._battery_cb, 10)
        self.create_subscription(DVL, self.get_parameter("dvl_topic").value, self._dvl_cb, 10)
        self.create_subscription(Image, self.get_parameter("sonar_image_topic").value, self._sonar_cb, 10)
        self.create_subscription(String, self.get_parameter("pipeline_marker_ids_topic").value, self._marker_ids_cb, 10)
        self.create_subscription(String, self.get_parameter("aruco_visibility_topic").value, self._aruco_visibility_cb, 10)
        self.create_subscription(Pose, self.get_parameter("current_waypoint_topic").value, self._waypoint_cb, 10)
        self.create_subscription(Pose, self.get_parameter("current_reference_topic").value, self._reference_cb, 10)
        self.create_subscription(Odometry, self.get_parameter("odometry_topic").value, self._odometry_cb, 10)
        self.create_subscription(Float32, self.get_parameter("current_topic").value, self._current_cb, 10)
        self.create_subscription(Float32, self.get_parameter("speed_topic").value, self._speed_cb, 10)
        self.create_subscription(Float32, self.get_parameter("altitude_topic").value, self._altitude_cb, 10)
        self.create_subscription(String, self.get_parameter("camera_quality_topic").value, self._camera_cb, 10)
        self.create_subscription(String, self.get_parameter("usbl_strength_topic").value, self._usbl_cb, 10)
        self.create_subscription(Float32, self.get_parameter("fatigue_topic").value, self._fatigue_cb, 10)
        self.create_subscription(Float32, self.get_parameter("stress_topic").value, self._stress_cb, 10)

    def _now(self):
        return self.get_clock().now()

    def _mark_seen(self, key: str):
        self.last_seen[key] = self._now()

    def _age(self, key: str) -> Optional[float]:
        if key not in self.last_seen:
            return None
        return (self._now() - self.last_seen[key]).nanoseconds / 1e9

    def _fresh(self, key: str, timeout_s: float) -> bool:
        age = self._age(key)
        return age is not None and age <= timeout_s

    def _set_evidence(self, node_id: str, state_id: str, source: str):
        if node_id not in self.node_ids:
            self.get_logger().warning(f"BN node '{node_id}' is not present; ignored evidence from {source}")
            return
        try:
            outcomes = list(self.net.get_outcome_ids(node_id))
            canonical = self._canonical_state(state_id, outcomes)
            self.net.set_evidence(node_id, outcomes.index(canonical))
            self.evidence[node_id] = canonical
        except Exception as exc:
            self.get_logger().warning(f"Failed to set {node_id}={state_id} from {source}: {exc}")

    def _clear_evidence(self, node_id: str):
        if node_id not in self.evidence:
            return
        try:
            self.net.clear_evidence(node_id)
        except Exception:
            return
        self.evidence.pop(node_id, None)

    @staticmethod
    def _canonical_state(state_id: str, outcomes):
        compact = "".join(ch for ch in str(state_id).lower() if ch.isalnum())
        aliases = {
            "notakeover": "notakeoverrequired",
            "none": "none",
            "no": "none",
            "visible": "some",
            "pipeline_markervisible": "some",
            "pipelinemarkervisible": "some",
            "notdetected": "none",
            "lost": "lost",
        }
        compact = aliases.get(compact, compact)
        for outcome in outcomes:
            if "".join(ch for ch in outcome.lower() if ch.isalnum()) == compact:
                return outcome
        raise ValueError(f"state '{state_id}' not in outcomes {outcomes}")

    def _mission_phase_cb(self, msg: String):
        self._mark_seen("mission_phase")
        self._set_evidence("MissionPhase", msg.data, "mission_phase")

    def _battery_cb(self, msg: BatteryStatus):
        self._mark_seen("battery")
        soc = float(msg.state_of_charge)
        if soc > 1.5:
            soc = soc / 100.0
        if soc > 0.75:
            state = "High"
        elif soc > 0.50:
            state = "Medium"
        elif soc > 0.25:
            state = "Low"
        else:
            state = "Critical"
        self._set_evidence("BatteryLevel", state, "battery")

    def _dvl_cb(self, msg: DVL):
        self._mark_seen("dvl")
        cov = list(msg.velocity_covariance)
        if any(value < 0.0 for value in cov):
            state = "Failed"
        elif cov and max(abs(value) for value in cov) > float(self.get_parameter("dvl_cov_degraded_threshold").value):
            state = "Degraded"
        else:
            state = "Nominal"
        self._set_evidence("DVLStatus", state, "dvl")

    def _sonar_cb(self, msg: Image):
        nearest = self._nearest_sonar_return(msg)
        if nearest is None:
            return
        self._mark_seen("sonar")
        critical = float(self.get_parameter("sonar_critical_range_m").value)
        clear = float(self.get_parameter("sonar_clear_range_m").value)
        if nearest < critical:
            state = "Critical"
        elif nearest < clear:
            state = "Near"
        else:
            state = "Clear"
        self._set_evidence("SonarRange", state, "sonar")

    def _nearest_sonar_return(self, msg: Image) -> Optional[float]:
        if msg.height == 0 or msg.width == 0 or not msg.data:
            return None
        threshold = int(self.get_parameter("sonar_intensity_threshold").value)
        bytes_per_pixel = self._image_bytes_per_pixel(msg.encoding)
        if bytes_per_pixel is None:
            self.get_logger().warning(f"Unsupported sonar image encoding '{msg.encoding}'")
            return None
        max_range = 5.0
        best_row = None
        data = msg.data
        for row in range(msg.height):
            row_start = row * msg.step
            for col in range(msg.width):
                idx = row_start + col * bytes_per_pixel
                if idx >= len(data):
                    break
                if max(data[idx:idx + bytes_per_pixel]) > threshold:
                    best_row = row
                    break
            if best_row is not None:
                break
        if best_row is None:
            return None
        return (best_row / max(float(msg.height - 1), 1.0)) * max_range

    @staticmethod
    def _image_bytes_per_pixel(encoding: str) -> Optional[int]:
        enc = encoding.lower()
        if enc in ("mono8", "8uc1"):
            return 1
        if enc in ("rgb8", "bgr8"):
            return 3
        if enc in ("rgba8", "bgra8"):
            return 4
        return None

    def _marker_ids_cb(self, msg: String):
        self._mark_seen("aruco")
        marker_ids = [item.strip() for item in msg.data.split(",") if item.strip()]
        expected = max(int(self.get_parameter("expected_aruco_markers").value), 1)
        if len(marker_ids) >= expected:
            state = "All"
        elif marker_ids:
            state = "Some"
        else:
            state = "None"
        self._set_evidence("ArUcoVisibility", state, "pipeline_marker_ids")

    def _aruco_visibility_cb(self, msg: String):
        self._mark_seen("aruco")
        value = msg.data.strip().lower()
        if value in ("all", "all_visible"):
            state = "All"
        elif value in ("none", "notdetected", "not_detected", "lost"):
            state = "None"
        else:
            state = "Some"
        self._set_evidence("ArUcoVisibility", state, "aruco_visibility")

    def _waypoint_cb(self, msg: Pose):
        self._mark_seen("target")
        self.current_waypoint = msg
        self._update_target_evidence()

    def _reference_cb(self, msg: Pose):
        self._mark_seen("target")
        self.current_reference = msg
        self._update_target_evidence()

    def _odometry_cb(self, msg: Odometry):
        self._mark_seen("odom")
        self.current_pose = msg.pose.pose
        self._update_target_evidence()

    def _update_target_evidence(self):
        if self.current_pose is None:
            return
        target = self.current_waypoint or self.current_reference
        reference = self.current_reference or self.current_waypoint
        if target is not None:
            distance = self._distance(self.current_pose, target)
            if distance > float(self.get_parameter("range_far_m").value):
                self._set_evidence("RangeToTarget", "Far", "target")
            elif distance < float(self.get_parameter("range_close_m").value):
                self._set_evidence("RangeToTarget", "Close", "target")
            else:
                self._set_evidence("RangeToTarget", "Mid", "target")
        if reference is not None:
            error = self._distance(self.current_pose, reference)
            if error < float(self.get_parameter("tracking_low_m").value):
                self._set_evidence("WaypointTrackingError", "Low", "target")
            elif error > float(self.get_parameter("tracking_high_m").value):
                self._set_evidence("WaypointTrackingError", "High", "target")
            else:
                self._set_evidence("WaypointTrackingError", "Medium", "target")

    @staticmethod
    def _distance(a: Pose, b: Pose) -> float:
        dx = a.position.x - b.position.x
        dy = a.position.y - b.position.y
        dz = a.position.z - b.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _current_cb(self, msg: Float32):
        self._mark_seen("current")
        current = float(msg.data)
        if current < 0.3:
            state = "Low"
        elif current < 0.7:
            state = "Medium"
        else:
            state = "High"
        self._set_evidence("Current", state, "current")

    def _speed_cb(self, msg: Float32):
        self._mark_seen("speed")
        speed = abs(float(msg.data))
        if speed < 0.3:
            state = "Safe"
        elif speed < 0.7:
            state = "Moderate"
        else:
            state = "High"
        self._set_evidence("Speed", state, "speed")

    def _altitude_cb(self, msg: Float32):
        self._mark_seen("altitude")
        altitude = float(msg.data)
        if altitude > 2.0:
            state = "Safe"
        elif altitude > 1.0:
            state = "Marginal"
        else:
            state = "Unsafe"
        self._set_evidence("Altitude", state, "altitude")

    def _camera_cb(self, msg: String):
        self._mark_seen("camera")
        self._set_evidence("CameraQuality", msg.data, "camera")

    def _usbl_cb(self, msg: String):
        self._mark_seen("usbl")
        self._set_evidence("USBLStrength", msg.data, "usbl")

    def _fatigue_cb(self, msg: Float32):
        self._mark_seen("fatigue")
        self._set_evidence("Fatigue", self._low_medium_high(float(msg.data)), "fatigue")

    def _stress_cb(self, msg: Float32):
        self._mark_seen("stress")
        self._set_evidence("Stress", self._low_medium_high(float(msg.data)), "stress")

    @staticmethod
    def _low_medium_high(value: float) -> str:
        if value < 0.3:
            return "Low"
        if value < 0.6:
            return "Medium"
        return "High"

    def _apply_timeouts(self):
        if not self._fresh("dvl", float(self.get_parameter("dvl_timeout_s").value)):
            self._set_evidence("DVLStatus", "Failed", "dvl_timeout")
        if not self._fresh("aruco", float(self.get_parameter("aruco_timeout_s").value)):
            self._set_evidence("ArUcoVisibility", "None", "aruco_timeout")
        if not self._fresh("sonar", float(self.get_parameter("sonar_timeout_s").value)):
            self._clear_evidence("SonarRange")
        if not self._fresh("target", float(self.get_parameter("target_timeout_s").value)):
            self._clear_evidence("RangeToTarget")
            self._clear_evidence("WaypointTrackingError")

    def _periodic_update(self):
        try:
            self._apply_timeouts()
            self.net.update_beliefs()
            outcomes = list(self.net.get_outcome_ids(self.TARGET_NODE))
            values = list(self.net.get_node_value(self.TARGET_NODE))
        except Exception as exc:
            self.get_logger().error(f"BN update failed: {exc}")
            return

        probabilities = dict(zip(outcomes, values))
        state, probability = max(probabilities.items(), key=lambda item: item[1])

        self._publish_string(self.state_pub, state)
        self._publish_float(self.no_takeover_pub, probabilities.get("NoTakeoverRequired", 0.0))
        self._publish_float(self.attention_pub, probabilities.get("IncreasedAttentionRequired", 0.0))
        self._publish_float(self.takeover_pub, probabilities.get("TakeoverRequested", 0.0))
        self._publish_float(self.max_prob_pub, probability)
        self._publish_float(self.threshold_pub, float(self.get_parameter("takeover_threshold").value))
        self._publish_string(
            self.evidence_pub,
            json.dumps(
                {
                    "state": state,
                    "probabilities": probabilities,
                    "evidence": self.evidence,
                },
                sort_keys=True,
            ),
        )

    @staticmethod
    def _publish_float(pub, value: float):
        msg = Float32()
        msg.data = float(value)
        pub.publish(msg)

    @staticmethod
    def _publish_string(pub, value: str):
        msg = String()
        msg.data = value
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TakeoverBayesianNetwork()
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
