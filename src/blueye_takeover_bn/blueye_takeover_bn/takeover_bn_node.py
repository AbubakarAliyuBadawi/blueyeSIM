#!/usr/bin/env python3
"""Full-mission Bayesian network node for takeover request inference.

Works with a static network (v2) and with the dynamic network v4 (temporal plate). With a dynamic network the node
keeps the evidence of the last N updates and enters it slice by slice (sliding window); the published posterior is
that of the last slice. N = parameter dbn_slices (0: the number of slices stored in the file).
"""

import collections
import json
import math
import os
from typing import Dict, Optional

import pysmile
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PointStamped, Pose
from mundus_mir_msgs.msg import BatteryStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float64, Int32, String
from stonefish_ros2.msg import DVL

try:
    import pysmile_license  # noqa: F401  (BayesFusion licence file; keeps the key out of the code)
except ImportError as exc:
    raise SystemExit(
        "pysmile_license.py not found. Put the BayesFusion licence file on the Python path, "
        "e.g. next to pysmile.so in ~/.local/lib/python3.10/site-packages/"
    ) from exc


class TakeoverBayesianNetwork(Node):
    """Passive BN aggregator for full-mission takeover request probability."""

    TARGET_NODE = "TakeoverRequest"
    # /mission_state of the behaviour tree; 6 (takeover) is not a phase, the phase before it still applies
    MISSION_STATES = {1: "Undocking", 2: "Transit", 3: "Inspection", 4: "Homing", 5: "Docking"}

    def __init__(self):
        super().__init__("takeover_bayesian_network")
        self._declare_parameters()
        self.net = pysmile.Network()
        self.network_path = self._resolve_network_path()
        self.net.read_file(self.network_path)
        self.node_ids = set(self.net.get_all_node_ids())
        self.outcomes = {n: list(self.net.get_outcome_ids(n)) for n in self.node_ids}
        # Posteriors of every node are published with the evidence (explanation and bn_state_viewer)
        self.posterior_nodes = sorted(self.node_ids)
        self.net.clear_all_targets()                       # no targets: SMILE updates every node
        self.dynamic = any(self.net.get_node_temporal_type(n) == pysmile.NodeTemporalType.PLATE for n in self.node_ids)
        if self.dynamic:
            n_slices = int(self.get_parameter("dbn_slices").value)
            if n_slices > 0:
                self.net.set_slice_count(n_slices)
            self.n_slices = self.net.get_slice_count()
            self.history = collections.deque(maxlen=self.n_slices)
        self.evidence: Dict[str, str] = {}
        self.last_seen: Dict[str, rclpy.time.Time] = {}

        # Docking controller target (tag frame, compared with odometry_topic)
        self.current_pose: Optional[Pose] = None
        self.current_waypoint: Optional[Pose] = None
        self.current_reference: Optional[Pose] = None
        # Behaviour tree goal (world frame, compared with goal_odometry_topic); the leg starts where the vehicle was
        self.goal_pose: Optional[Pose] = None
        self.goal: Optional[Point] = None
        self.goal_start: Optional[Point] = None

        self._setup_publishers()
        self._setup_subscribers()

        update_rate = max(float(self.get_parameter("update_rate_hz").value), 0.1)
        self.timer = self.create_timer(1.0 / update_rate, self._periodic_update)

        if bool(self.get_parameter("use_default_mission_phase").value):
            default_phase = str(self.get_parameter("default_mission_phase").value)
            self._set_evidence("MissionPhase", default_phase, "default")

        self.get_logger().info(f"Loaded full mission BN: {self.network_path}"
                               + (f" (dynamic, {self.n_slices} slices)" if self.dynamic else " (static)"))
        self.get_logger().info("Publishing takeover inference on /blueye/takeover_request/*")

    def _declare_parameters(self):
        self.declare_parameter("bn_file", "")
        self.declare_parameter("update_rate_hz", 2.0)
        self.declare_parameter("default_mission_phase", "Transit")
        self.declare_parameter("use_default_mission_phase", True)
        self.declare_parameter("takeover_threshold", 0.65)
        self.declare_parameter("dbn_slices", 0)            # dynamic network: window length; 0 = value in the file

        self.declare_parameter("mission_phase_topic", "/mission/phase")
        self.declare_parameter("mission_state_topic", "/mission_state")
        self.declare_parameter("battery_topic", "/blueye/battery")
        self.declare_parameter("dvl_topic", "/blueye/dvl/sim")
        self.declare_parameter("sonar_image_topic", "/blueye/fls/image")
        self.declare_parameter("pipeline_marker_ids_topic", "/blueye/pipeline_marker_ids")
        self.declare_parameter("aruco_visibility_topic", "/blueye/aruco_visibility")
        self.declare_parameter("current_waypoint_topic", "/blueye/current_waypoint")
        self.declare_parameter("current_reference_topic", "/blueye/current_reference")
        self.declare_parameter("odometry_topic", "/odometry/filtered")
        self.declare_parameter("goal_topic", "/blueye/waypoint_goal")
        self.declare_parameter("goal_odometry_topic", "/blueye/odom")
        self.declare_parameter("current_topic", "/blueye/current")
        self.declare_parameter("speed_topic", "/blueye/speed")
        self.declare_parameter("altitude_topic", "/blueye/altitude")
        self.declare_parameter("camera_quality_topic", "/blueye/camera_quality")
        self.declare_parameter("usbl_strength_topic", "/blueye/usbl_strength")
        self.declare_parameter("fatigue_topic", "/blueye/human/fatigue")
        self.declare_parameter("stress_topic", "/blueye/human/stress")

        self.declare_parameter("use_dvl_altitude", True)
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
        return os.path.join(package_share, "config", "UnifiedMissionDBN_v4.xdsl")

    def _setup_publishers(self):
        self.state_pub = self.create_publisher(String, "/blueye/takeover_request/state", 10)
        self.no_takeover_pub = self.create_publisher(Float32, "/blueye/takeover_request/no_takeover_prob", 10)
        self.attention_pub = self.create_publisher(Float32, "/blueye/takeover_request/attention_required_prob", 10)
        self.takeover_pub = self.create_publisher(Float32, "/blueye/takeover_request/takeover_requested_prob", 10)
        self.max_prob_pub = self.create_publisher(Float32, "/blueye/takeover_request/max_probability", 10)
        self.threshold_pub = self.create_publisher(Float32, "/blueye/takeover_request/threshold", 10)
        self.evidence_pub = self.create_publisher(String, "/blueye/takeover_request/evidence", 10)
        self.urgency_pub = self.create_publisher(Float64, "/blueye/takeover_request/urgency", 10)

    def _setup_subscribers(self):
        self.create_subscription(String, self.get_parameter("mission_phase_topic").value, self._mission_phase_cb, 10)
        self.create_subscription(Int32, self.get_parameter("mission_state_topic").value, self._mission_state_cb, 10)
        self.create_subscription(BatteryStatus, self.get_parameter("battery_topic").value, self._battery_cb, 10)
        self.create_subscription(DVL, self.get_parameter("dvl_topic").value, self._dvl_cb, 10)
        self.create_subscription(Image, self.get_parameter("sonar_image_topic").value, self._sonar_cb, 10)
        self.create_subscription(String, self.get_parameter("pipeline_marker_ids_topic").value, self._marker_ids_cb, 10)
        self.create_subscription(String, self.get_parameter("aruco_visibility_topic").value, self._aruco_visibility_cb, 10)
        self.create_subscription(Pose, self.get_parameter("current_waypoint_topic").value, self._waypoint_cb, 10)
        self.create_subscription(Pose, self.get_parameter("current_reference_topic").value, self._reference_cb, 10)
        self.create_subscription(Odometry, self.get_parameter("odometry_topic").value, self._odometry_cb, 10)
        self.create_subscription(PointStamped, self.get_parameter("goal_topic").value, self._goal_cb, 10)
        self.create_subscription(Odometry, self.get_parameter("goal_odometry_topic").value, self._goal_odometry_cb, 10)
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
            outcomes = self.outcomes[node_id]
            canonical = self._canonical_state(state_id, outcomes)
            if not self.dynamic:                           # dynamic network: entered slice by slice at each update
                self.net.set_evidence(node_id, outcomes.index(canonical))
            self.evidence[node_id] = canonical
        except Exception as exc:
            self.get_logger().warning(f"Failed to set {node_id}={state_id} from {source}: {exc}")

    def _clear_evidence(self, node_id: str):
        if node_id not in self.evidence:
            return
        if not self.dynamic:
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

    def _mission_state_cb(self, msg: Int32):
        state = int(msg.data)
        if state in (5, 6):                                # docking controller or operator in control: no BT goal
            self.goal = self.goal_start = None
        phase = self.MISSION_STATES.get(state)
        if phase is None:
            return
        self._mark_seen("mission_phase")
        self._set_evidence("MissionPhase", phase, "mission_state")

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
        if bool(self.get_parameter("use_dvl_altitude").value) and msg.altitude >= 0.0:   # -1: no bottom lock
            self._mark_seen("altitude")
            self._set_evidence("Altitude", self._altitude_state(float(msg.altitude)), "dvl_altitude")

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
        """Range of the nearest echo; math.inf when no echo is above the threshold, None for an unusable image."""
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
            return math.inf                                # sonar working, nothing within range
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
        self._mark_seen("controller")
        self.current_waypoint = msg
        self._update_target_evidence()

    def _reference_cb(self, msg: Pose):
        self._mark_seen("controller")
        self.current_reference = msg
        self._update_target_evidence()

    def _odometry_cb(self, msg: Odometry):
        self._mark_seen("odom")
        self.current_pose = msg.pose.pose
        self._update_target_evidence()

    def _goal_cb(self, msg: PointStamped):
        self.goal = msg.point
        self.goal_start = self.goal_pose.position if self.goal_pose is not None else None
        self._update_target_evidence()

    def _goal_odometry_cb(self, msg: Odometry):
        self.goal_pose = msg.pose.pose
        self._update_target_evidence()

    def _update_target_evidence(self):
        """Docking controller waypoint/reference while it publishes, otherwise the behaviour tree goal."""
        if self._fresh("controller", float(self.get_parameter("target_timeout_s").value)) \
                and self.current_pose is not None:
            target = self.current_waypoint or self.current_reference
            reference = self.current_reference or self.current_waypoint
            distance = self._distance(self.current_pose.position, target.position)
            error = self._distance(self.current_pose.position, reference.position)
        elif self.goal is not None and self.goal_pose is not None:
            position = self.goal_pose.position
            if self.goal_start is None:
                self.goal_start = position
            distance = self._distance(position, self.goal)
            error = self._cross_track(position, self.goal_start, self.goal)
        else:
            return
        self._mark_seen("target")
        if distance > float(self.get_parameter("range_far_m").value):
            self._set_evidence("RangeToTarget", "Far", "target")
        elif distance < float(self.get_parameter("range_close_m").value):
            self._set_evidence("RangeToTarget", "Close", "target")
        else:
            self._set_evidence("RangeToTarget", "Mid", "target")
        if error < float(self.get_parameter("tracking_low_m").value):
            self._set_evidence("WaypointTrackingError", "Low", "target")
        elif error > float(self.get_parameter("tracking_high_m").value):
            self._set_evidence("WaypointTrackingError", "High", "target")
        else:
            self._set_evidence("WaypointTrackingError", "Medium", "target")

    @staticmethod
    def _distance(a: Point, b: Point) -> float:
        return math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))

    @staticmethod
    def _cross_track(p: Point, a: Point, b: Point) -> float:
        """Distance from p to the straight leg a -> b."""
        ab = (b.x - a.x, b.y - a.y, b.z - a.z)
        ap = (p.x - a.x, p.y - a.y, p.z - a.z)
        length2 = sum(c * c for c in ab)
        t = 0.0 if length2 < 1e-9 else max(0.0, min(1.0, sum(u * v for u, v in zip(ap, ab)) / length2))
        return math.dist((p.x, p.y, p.z), (a.x + t * ab[0], a.y + t * ab[1], a.z + t * ab[2]))

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
        self._set_evidence("Altitude", self._altitude_state(float(msg.data)), "altitude")

    @staticmethod
    def _altitude_state(altitude: float) -> str:
        if altitude > 2.0:
            return "Safe"
        if altitude > 1.0:
            return "Marginal"
        return "Unsafe"

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
        if not self._fresh("altitude", float(self.get_parameter("dvl_timeout_s").value)):
            self._clear_evidence("Altitude")               # DVL silent or without bottom lock
        if not self._fresh("aruco", float(self.get_parameter("aruco_timeout_s").value)):
            self._set_evidence("ArUcoVisibility", "None", "aruco_timeout")
        if not self._fresh("sonar", float(self.get_parameter("sonar_timeout_s").value)):
            self._clear_evidence("SonarRange")
        if not self._fresh("target", float(self.get_parameter("target_timeout_s").value)):
            self._clear_evidence("RangeToTarget")
            self._clear_evidence("WaypointTrackingError")

    def _posterior(self, node_id: str) -> Dict[str, float]:
        outcomes = self.outcomes[node_id]
        values = list(self.net.get_node_value(node_id))[-len(outcomes):]   # dynamic network: the last slice
        return dict(zip(outcomes, values))

    def _periodic_update(self):
        try:
            self._apply_timeouts()
            if self.dynamic:
                self._enter_window()
            self.net.update_beliefs()
            probabilities = self._posterior(self.TARGET_NODE)
            posteriors = {node_id: self._posterior(node_id) for node_id in self.posterior_nodes}
        except Exception as exc:
            self.get_logger().error(f"BN update failed: {exc}")
            return

        state, probability = max(probabilities.items(), key=lambda item: item[1])

        self._publish_string(self.state_pub, state)
        self._publish_float(self.no_takeover_pub, probabilities.get("NoTakeoverRequired", 0.0))
        self._publish_float(self.attention_pub, probabilities.get("IncreasedAttentionRequired", 0.0))
        self._publish_float(self.takeover_pub, probabilities.get("TakeoverRequested", 0.0))
        self._publish_float(self.max_prob_pub, probability)
        self._publish_float(self.threshold_pub, float(self.get_parameter("takeover_threshold").value))

        urgency = (0.5 * probabilities.get("IncreasedAttentionRequired", 0.0)
                   + 1.0 * probabilities.get("TakeoverRequested", 0.0))
        urgency_msg = Float64()
        urgency_msg.data = float(urgency)
        self.urgency_pub.publish(urgency_msg)
        self._publish_string(
            self.evidence_pub,
            json.dumps(
                {
                    "state": state,
                    "probabilities": probabilities,
                    "evidence": self.evidence,
                    "posteriors": posteriors,
                },
                sort_keys=True,
            ),
        )

    def _enter_window(self):
        """Evidence of the last N updates, oldest first, in the last slices of the window."""
        self.history.append(dict(self.evidence))
        self.net.clear_all_evidence()
        first = self.n_slices - len(self.history)
        for k, snapshot in enumerate(self.history):
            for node_id, state in snapshot.items():
                self.net.set_temporal_evidence(node_id, first + k, self.outcomes[node_id].index(state))

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
