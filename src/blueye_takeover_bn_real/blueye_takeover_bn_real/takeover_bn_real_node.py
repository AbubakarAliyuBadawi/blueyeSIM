#!/usr/bin/env python3
"""Real-mission Bayesian network node for takeover request inference."""

import json
import math
import os
from typing import Dict, Optional

import pysmile
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32, Float64, Int32, String


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


class RealTakeoverBayesianNetwork(Node):
    """Passive BN aggregator using real Blueye discrete monitor topics."""

    TARGET_NODE = "TakeoverRequest"

    MISSION_STATE_MAP = {
        1: "Undocking",
        2: "Transit",
        3: "Inspection",
        4: "Docking",
    }

    def __init__(self):
        super().__init__("takeover_bayesian_network_real")
        self._declare_parameters()

        self.net = pysmile.Network()
        self.network_path = self._resolve_network_path()
        self.net.read_file(self.network_path)
        self.node_ids = set(self.net.get_all_node_ids())
        self.evidence: Dict[str, str] = {}
        self.last_seen: Dict[str, rclpy.time.Time] = {}

        self.current_pose: Optional[Pose] = None
        self.current_waypoint: Optional[Pose] = None

        self._setup_publishers()
        self._setup_subscribers()

        default_phase = str(self.get_parameter("default_mission_phase").value)
        self._set_evidence("MissionPhase", default_phase, "default")
        self._set_evidence("Fatigue", "Low", "default")
        self._set_evidence("Stress", "Low", "default")

        update_rate = max(float(self.get_parameter("update_rate_hz").value), 0.1)
        self.timer = self.create_timer(1.0 / update_rate, self._periodic_update)

        self.get_logger().info(f"Loaded real takeover BN: {self.network_path}")
        self.get_logger().info("Publishing takeover inference on /blueye/takeover_request/*")

    def _declare_parameters(self):
        self.declare_parameter("bn_file", "")
        self.declare_parameter("update_rate_hz", 2.0)
        self.declare_parameter("takeover_threshold", 0.65)
        self.declare_parameter("default_mission_phase", "Transit")

        self.declare_parameter("mission_phase_topic", "/mission_phase")
        self.declare_parameter("mission_state_topic", "/mission_state")
        self.declare_parameter("current_topic", "/blueye/current")
        self.declare_parameter("battery_state_topic", "/blueye/battery_state")
        self.declare_parameter("usbl_strength_topic", "/blueye/usbl_strength")
        self.declare_parameter("dvl_status_topic", "/blueye/dvl_status")
        self.declare_parameter("speed_state_topic", "/blueye/speed_state")
        self.declare_parameter("altitude_state_topic", "/blueye/altitude_state")
        self.declare_parameter("camera_quality_topic", "/blueye/camera_quality")
        self.declare_parameter("sonar_range_topic", "/blueye/sonar_range")
        self.declare_parameter("aruco_visibility_topic", "/blueye/aruco_visibility")
        self.declare_parameter("waypoint_tracking_error_topic", "/blueye/waypoint_tracking_error")
        self.declare_parameter("data_quality_topic", "/blueye/inspection_data_quality")
        self.declare_parameter("pose_topic", "/blueye/pose")
        self.declare_parameter("waypoint_topic", "/blueye/current_waypoint")
        self.declare_parameter("fatigue_topic", "/blueye/human/fatigue")
        self.declare_parameter("stress_topic", "/blueye/human/stress")

        self.declare_parameter("range_far_m", 5.0)
        self.declare_parameter("range_close_m", 1.0)
        self.declare_parameter("current_low_mps", 0.3)
        self.declare_parameter("current_medium_mps", 0.7)

        self.declare_parameter("dvl_timeout_s", 2.0)
        self.declare_parameter("aruco_timeout_s", 1.5)
        self.declare_parameter("target_timeout_s", 2.0)
        self.declare_parameter("data_quality_timeout_s", 3.0)

    def _resolve_network_path(self) -> str:
        configured = str(self.get_parameter("bn_file").value)
        if configured:
            return configured
        package_share = get_package_share_directory("blueye_takeover_bn_real")
        return os.path.join(package_share, "config", "full_mission.xdsl")

    def _setup_publishers(self):
        self.state_pub = self.create_publisher(String, "/blueye/takeover_request/state", 10)
        self.no_takeover_pub = self.create_publisher(Float32, "/blueye/takeover_request/no_takeover_prob", 10)
        self.attention_pub = self.create_publisher(Float32, "/blueye/takeover_request/attention_required_prob", 10)
        self.takeover_pub = self.create_publisher(Float32, "/blueye/takeover_request/takeover_requested_prob", 10)
        self.max_prob_pub = self.create_publisher(Float32, "/blueye/takeover_request/max_probability", 10)
        self.threshold_pub = self.create_publisher(Float32, "/blueye/takeover_request/threshold", 10)
        self.urgency_pub = self.create_publisher(Float64, "/blueye/takeover_request/urgency", 10)
        self.evidence_pub = self.create_publisher(String, "/blueye/takeover_request/evidence", 10)

    def _setup_subscribers(self):
        self.create_subscription(Int32, self.get_parameter("mission_phase_topic").value, self._mission_state_cb, 10)
        self.create_subscription(Int32, self.get_parameter("mission_state_topic").value, self._mission_state_cb, 10)
        self.create_subscription(Float32, self.get_parameter("current_topic").value, self._current_cb, 10)
        self.create_subscription(String, self.get_parameter("battery_state_topic").value, self._battery_state_cb, 10)
        self.create_subscription(String, self.get_parameter("usbl_strength_topic").value, self._usbl_cb, 10)
        self.create_subscription(String, self.get_parameter("dvl_status_topic").value, self._dvl_status_cb, 10)
        self.create_subscription(String, self.get_parameter("speed_state_topic").value, self._speed_state_cb, 10)
        self.create_subscription(String, self.get_parameter("altitude_state_topic").value, self._altitude_state_cb, 10)
        self.create_subscription(String, self.get_parameter("camera_quality_topic").value, self._camera_cb, 10)
        self.create_subscription(String, self.get_parameter("sonar_range_topic").value, self._sonar_range_cb, 10)
        self.create_subscription(String, self.get_parameter("aruco_visibility_topic").value, self._aruco_cb, 10)
        self.create_subscription(String, self.get_parameter("waypoint_tracking_error_topic").value, self._tracking_cb, 10)
        self.create_subscription(String, self.get_parameter("data_quality_topic").value, self._data_quality_cb, 10)
        self.create_subscription(PoseStamped, self.get_parameter("pose_topic").value, self._pose_cb, 10)
        self.create_subscription(Pose, self.get_parameter("waypoint_topic").value, self._waypoint_cb, 10)
        self.create_subscription(Float32, self.get_parameter("fatigue_topic").value, self._fatigue_cb, 10)
        self.create_subscription(Float32, self.get_parameter("stress_topic").value, self._stress_cb, 10)

    def _mission_state_cb(self, msg: Int32):
        self._mark_seen("mission_phase")
        state = self.MISSION_STATE_MAP.get(int(msg.data))
        if state is None:
            self.get_logger().warning(f"Unknown mission phase/state {msg.data}; keeping previous MissionPhase")
            return
        self._set_evidence("MissionPhase", state, "mission_phase")

    def _current_cb(self, msg: Float32):
        self._mark_seen("current")
        current = float(msg.data)
        if current < float(self.get_parameter("current_low_mps").value):
            state = "Low"
        elif current < float(self.get_parameter("current_medium_mps").value):
            state = "Medium"
        else:
            state = "High"
        self._set_evidence("Current", state, "current")

    def _battery_state_cb(self, msg: String):
        self._mark_seen("battery")
        self._set_evidence("BatteryLevel", msg.data, "battery_state")

    def _usbl_cb(self, msg: String):
        self._mark_seen("usbl")
        self._set_evidence("USBLStrength", msg.data, "usbl_strength")

    def _dvl_status_cb(self, msg: String):
        self._mark_seen("dvl")
        self._set_evidence("DVLStatus", msg.data, "dvl_status")

    def _speed_state_cb(self, msg: String):
        self._mark_seen("speed")
        self._set_evidence("Speed", msg.data, "speed_state")

    def _altitude_state_cb(self, msg: String):
        self._mark_seen("altitude")
        self._set_evidence("Altitude", msg.data, "altitude_state")

    def _camera_cb(self, msg: String):
        self._mark_seen("camera")
        self._set_evidence("CameraQuality", msg.data, "camera_quality")

    def _sonar_range_cb(self, msg: String):
        self._mark_seen("sonar")
        self._set_evidence("SonarRange", msg.data, "sonar_range")

    def _aruco_cb(self, msg: String):
        self._mark_seen("aruco")
        self._set_evidence("ArUcoVisibility", msg.data, "aruco_visibility")

    def _tracking_cb(self, msg: String):
        self._mark_seen("target")
        self._set_evidence("WaypointTrackingError", msg.data, "waypoint_tracking_error")

    def _data_quality_cb(self, msg: String):
        self._mark_seen("data_quality")
        self._set_evidence("DataQuality", msg.data, "inspection_data_quality")

    def _pose_cb(self, msg: PoseStamped):
        self._mark_seen("pose")
        self.current_pose = msg.pose
        self._update_range_to_target()

    def _waypoint_cb(self, msg: Pose):
        self._mark_seen("target")
        self.current_waypoint = msg
        self._update_range_to_target()

    def _fatigue_cb(self, msg: Float32):
        self._mark_seen("fatigue")
        self._set_evidence("Fatigue", self._low_medium_high(float(msg.data)), "fatigue")

    def _stress_cb(self, msg: Float32):
        self._mark_seen("stress")
        self._set_evidence("Stress", self._low_medium_high(float(msg.data)), "stress")

    def _update_range_to_target(self):
        if self.current_pose is None or self.current_waypoint is None:
            return
        distance = self._distance(self.current_pose, self.current_waypoint)
        if distance > float(self.get_parameter("range_far_m").value):
            state = "Far"
        elif distance < float(self.get_parameter("range_close_m").value):
            state = "Close"
        else:
            state = "Mid"
        self._set_evidence("RangeToTarget", state, "pose_waypoint")

    @staticmethod
    def _distance(a: Pose, b: Pose) -> float:
        dx = a.position.x - b.position.x
        dy = a.position.y - b.position.y
        dz = a.position.z - b.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

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
        if not self._fresh("target", float(self.get_parameter("target_timeout_s").value)):
            self._clear_evidence("RangeToTarget")
            self._clear_evidence("WaypointTrackingError")
        if not self._fresh("data_quality", float(self.get_parameter("data_quality_timeout_s").value)):
            self._clear_evidence("DataQuality")

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
        attention = probabilities.get("IncreasedAttentionRequired", 0.0)
        takeover = probabilities.get("TakeoverRequested", 0.0)
        self._publish_float(self.attention_pub, attention)
        self._publish_float(self.takeover_pub, takeover)
        urgency_msg = Float64()
        urgency_msg.data = 0.5 * attention + 1.0 * takeover
        self.urgency_pub.publish(urgency_msg)
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
            "transit1": "transit",
            "transit2": "transit",
            "pipelineinspection": "inspection",
            "wreckageinspection": "inspection",
            "notdetected": "none",
            "lost": "lost",
        }
        compact = aliases.get(compact, compact)
        for outcome in outcomes:
            if "".join(ch for ch in outcome.lower() if ch.isalnum()) == compact:
                return outcome
        raise ValueError(f"state '{state_id}' not in outcomes {outcomes}")

    def _now(self):
        return self.get_clock().now()

    def _mark_seen(self, key: str):
        self.last_seen[key] = self._now()

    def _fresh(self, key: str, timeout_s: float) -> bool:
        if key not in self.last_seen:
            return False
        age = (self._now() - self.last_seen[key]).nanoseconds / 1e9
        return age <= timeout_s

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
    node = RealTakeoverBayesianNetwork()
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
