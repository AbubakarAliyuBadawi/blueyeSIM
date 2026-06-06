import cv2
import math
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String

from blueye_inspection.pipeline_board import id_board, pos_board


class PipelineArucoDetector(Node):
    def __init__(self):
        super().__init__("pipeline_aruco_detector")

        self.declare_parameter("image_topic", "/blueye/cam_down/image_color")
        self.declare_parameter("compressed_image_topic", "/blueye/cam_down/image_color/compressed")
        self.declare_parameter("use_compressed_image", False)
        self.declare_parameter("debug_view", False)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "/blueye/inspection/debug_image")
        self.declare_parameter("min_markers_for_detection", 2)
        self.declare_parameter("board_dictionary_name", "DICT_5X5_50")
        self.declare_parameter("dictionary_names", ["DICT_5X5_50"])
        self.declare_parameter("accepted_marker_ids", [21, 22, 23, 24, 25, 26])
        self.declare_parameter("fx", 1662.8)
        self.declare_parameter("fy", 1662.8)
        self.declare_parameter("cx", 960.0)
        self.declare_parameter("cy", 540.0)
        self.declare_parameter("base_yaw_offset", 0.0)
        self.declare_parameter("debug_axes_length", 0.35)
        self.declare_parameter("pose_z_min", -0.2)
        self.declare_parameter("pose_z_max", 3.0)
        self.declare_parameter("pos_stddev", 0.03)
        self.declare_parameter("yaw_stddev", 0.08)

        self.debug_view = bool(self.get_parameter("debug_view").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.min_markers_for_detection = int(self.get_parameter("min_markers_for_detection").value)
        self.accepted_marker_ids = {int(marker_id) for marker_id in self.get_parameter("accepted_marker_ids").value}
        self.board_dictionary_name = str(self.get_parameter("board_dictionary_name").value)
        self.bridge = CvBridge()
        fx = float(self.get_parameter("fx").value)
        fy = float(self.get_parameter("fy").value)
        cx = float(self.get_parameter("cx").value)
        cy = float(self.get_parameter("cy").value)
        self.camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
        self.distortion = np.zeros(5)
        board_cv_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.board_dictionary_name))
        self.board = cv2.aruco.Board(pos_board, board_cv_dict, id_board)
        self.rot_matrix_180_x = np.diag([1.0, -1.0, -1.0])
        self.status_text = "waiting_for_controller"
        self.progress_text = "WP reached:0 remain:0"
        self.current_waypoint = None
        self.current_pose = None

        self.detectors = []
        for dictionary_name in self.get_parameter("dictionary_names").value:
            if not hasattr(cv2.aruco, dictionary_name):
                self.get_logger().warn(f"Unknown ArUco dictionary '{dictionary_name}', skipping.")
                continue
            dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
            self.detectors.append((dictionary_name, cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())))

        if bool(self.get_parameter("use_compressed_image").value):
            self.image_sub = self.create_subscription(
                CompressedImage,
                self.get_parameter("compressed_image_topic").value,
                self.compressed_image_callback,
                qos_profile_sensor_data,
            )
        else:
            self.image_sub = self.create_subscription(
                Image,
                self.get_parameter("image_topic").value,
                self.image_callback,
                qos_profile_sensor_data,
            )
        self.detected_pub = self.create_publisher(Bool, "/blueye/pipeline_marker_detected", 10)
        self.ids_pub = self.create_publisher(String, "/blueye/pipeline_marker_ids", 10)
        self.visibility_pub = self.create_publisher(String, "/blueye/pipeline_marker_visibility", 10)
        self.pose_pub = self.create_publisher(Pose, "/blueye/pipeline_pose_estimated", 10)
        self.pose_stamped_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/blueye/pipeline_pose_estimated_stamped",
            10,
        )
        self.debug_image_pub = self.create_publisher(
            Image,
            self.get_parameter("debug_image_topic").value,
            10,
        )
        self.status_sub = self.create_subscription(
            String,
            "/blueye/pipeline_inspection_status",
            self.status_callback,
            10,
        )
        self.progress_sub = self.create_subscription(
            String,
            "/blueye/pipeline_inspection_progress",
            self.progress_callback,
            10,
        )
        self.current_waypoint_sub = self.create_subscription(
            Pose,
            "/blueye/pipeline_current_waypoint",
            self.current_waypoint_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "/blueye/odom",
            self.odometry_callback,
            10,
        )

    def status_callback(self, msg):
        self.status_text = msg.data

    def progress_callback(self, msg):
        self.progress_text = msg.data

    def current_waypoint_callback(self, msg):
        self.current_waypoint = msg

    def odometry_callback(self, msg):
        self.current_pose = msg.pose.pose

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def rotation_matrix_to_euler_xyz(rotation):
        sy = math.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(rotation[2, 1], rotation[2, 2])
            y = math.atan2(-rotation[2, 0], sy)
            z = math.atan2(rotation[1, 0], rotation[0, 0])
        else:
            x = math.atan2(-rotation[1, 2], rotation[1, 1])
            y = math.atan2(-rotation[2, 0], sy)
            z = 0.0
        return np.array([x, y, z])

    @staticmethod
    def yaw_to_quaternion(yaw):
        half = yaw / 2.0
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def draw_overlay(self, frame, detected, ids_text, dictionary_hits):
        color = (0, 220, 0) if detected else (0, 0, 255)
        lines = [
            f"Pipeline ArUco: {'VISIBLE' if detected else 'NONE'}",
            f"IDs: {ids_text or 'none'}",
            f"Dictionaries: {dictionary_hits or 'none'}",
            f"Status: {self.status_text}",
            f"Progress: {self.progress_text}",
        ]
        if self.current_waypoint is not None:
            wp = self.current_waypoint
            lines.append(
                "Waypoint: "
                f"x={wp.position.x:.2f} y={wp.position.y:.2f} "
                f"z={wp.position.z:.2f} yaw={wp.orientation.w:.1f}deg"
            )
        if self.current_pose is not None:
            pose = self.current_pose
            yaw = self.yaw_from_quaternion(pose.orientation)
            lines.append(
                "Blueye: "
                f"x={pose.position.x:.2f} y={pose.position.y:.2f} "
                f"z={pose.position.z:.2f} yaw={yaw:.2f}rad"
            )

        x0, y0, dy = 35, 45, 38
        cv2.rectangle(frame, (20, 15), (1120, 20 + dy * len(lines)), (0, 0, 0), -1)
        cv2.rectangle(frame, (20, 15), (1120, 20 + dy * len(lines)), color, 2)
        for i, line in enumerate(lines):
            cv2.putText(
                frame,
                line,
                (x0, y0 + i * dy),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.82,
                color if i == 0 else (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.process_frame(frame, msg.header)

    def compressed_image_callback(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.process_frame(frame, msg.header)

    def process_frame(self, frame, header):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        dictionary_hits = []
        board_corners = None
        board_ids = None
        board_marker_ids = []
        for dictionary_name, detector in self.detectors:
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is None:
                continue
            kept_for_dictionary = []
            kept_corners_for_dictionary = []
            for corner, marker_id in zip(corners, ids.flatten().tolist()):
                marker_id = int(marker_id)
                if not self.accepted_marker_ids or marker_id in self.accepted_marker_ids:
                    kept_for_dictionary.append(marker_id)
                    kept_corners_for_dictionary.append(corner)
            if kept_for_dictionary:
                dictionary_hits.append(f"{dictionary_name}:{','.join(str(i) for i in kept_for_dictionary)}")
            if dictionary_name == self.board_dictionary_name:
                board_corners = kept_corners_for_dictionary
                board_marker_ids = kept_for_dictionary
                if kept_for_dictionary:
                    board_ids = np.array(kept_for_dictionary, dtype=np.int32).reshape((-1, 1))
            if (self.debug_view or self.publish_debug_image) and ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        unique_ids = sorted(set(board_marker_ids))
        detected = len(board_marker_ids) >= self.min_markers_for_detection

        detected_msg = Bool()
        detected_msg.data = detected
        self.detected_pub.publish(detected_msg)

        ids_msg = String()
        ids_msg.data = ",".join(str(marker_id) for marker_id in unique_ids)
        self.ids_pub.publish(ids_msg)

        visibility_msg = String()
        visibility_msg.data = "pipeline_marker_visible" if detected else "none"
        self.visibility_pub.publish(visibility_msg)

        pose_ok = False
        pose_text = "Board pose: waiting"
        if detected:
            obj_points, img_points = self.board.matchImagePoints(board_corners, board_ids)
            if obj_points is None or img_points is None:
                pose_text = f"Board pose: no board match for IDs {ids_msg.data}"
            elif len(obj_points) < 4:
                pose_text = f"Board pose: not enough points ({len(obj_points)})"
            else:
                ok, rvec_board, tvec_board = cv2.solvePnP(
                    obj_points,
                    img_points,
                    self.camera_matrix,
                    self.distortion,
                    None,
                    None,
                    False,
                    cv2.SOLVEPNP_ITERATIVE,
                )
                if ok:
                    if self.debug_view or self.publish_debug_image:
                        cv2.drawFrameAxes(
                            frame,
                            self.camera_matrix,
                            self.distortion,
                            rvec_board,
                            tvec_board,
                            float(self.get_parameter("debug_axes_length").value),
                        )
                    r_camera_board = np.asarray(cv2.Rodrigues(rvec_board)[0])
                    r_board_camera = r_camera_board.T
                    t_board_camera = (-r_board_camera @ np.asarray(tvec_board).reshape(3, 1)).flatten()
                    _, pitch_camera_board, _ = self.rotation_matrix_to_euler_xyz(
                        self.rot_matrix_180_x @ r_board_camera
                    )
                    yaw_blueye_wrt_board = -pitch_camera_board + float(
                        self.get_parameter("base_yaw_offset").value
                    )

                    pose_msg = Pose()
                    pose_msg.position.x = float(t_board_camera[0])
                    pose_msg.position.y = float(t_board_camera[1])
                    pose_msg.position.z = float(t_board_camera[2])
                    pose_msg.orientation.z = float(yaw_blueye_wrt_board)
                    if not self.pose_z_is_valid(pose_msg.position.z):
                        self.get_logger().warn(
                            f"Rejecting pipeline pose with invalid z={pose_msg.position.z:.2f} "
                            "in z-up pipeline_board frame",
                            throttle_duration_sec=1.0,
                        )
                        pose_ok = False
                        pose_text = (
                            f"Board pose: rejected z={pose_msg.position.z:.2f} "
                            f"[{float(self.get_parameter('pose_z_min').value):.1f},"
                            f"{float(self.get_parameter('pose_z_max').value):.1f}]"
                        )
                    else:
                        self.pose_pub.publish(pose_msg)

                        pose_stamped = PoseWithCovarianceStamped()
                        pose_stamped.header.stamp = self.get_clock().now().to_msg()
                        pose_stamped.header.frame_id = "pipeline_board"
                        pose_stamped.pose.pose.position.x = pose_msg.position.x
                        pose_stamped.pose.pose.position.y = pose_msg.position.y
                        pose_stamped.pose.pose.position.z = pose_msg.position.z
                        qx, qy, qz, qw = self.yaw_to_quaternion(yaw_blueye_wrt_board)
                        pose_stamped.pose.pose.orientation.x = qx
                        pose_stamped.pose.pose.orientation.y = qy
                        pose_stamped.pose.pose.orientation.z = qz
                        pose_stamped.pose.pose.orientation.w = qw

                        pos_var = float(self.get_parameter("pos_stddev").value) ** 2
                        yaw_var = float(self.get_parameter("yaw_stddev").value) ** 2
                        pose_stamped.pose.covariance = [
                            pos_var, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, pos_var, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, pos_var, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, yaw_var,
                        ]
                        self.pose_stamped_pub.publish(pose_stamped)
                        pose_ok = True
                        pose_text = (
                            "Board pose: OK "
                            f"x={pose_msg.position.x:.2f} y={pose_msg.position.y:.2f} "
                            f"z={pose_msg.position.z:.2f} yaw={yaw_blueye_wrt_board:.2f}"
                        )
                else:
                    pose_text = "Board pose: solvePnP failed"
        elif board_marker_ids:
            pose_text = (
                f"Board pose: need {self.min_markers_for_detection} marker(s), "
                f"got {len(board_marker_ids)}"
            )

        if self.debug_view or self.publish_debug_image:
            self.draw_overlay(frame, detected, ids_msg.data, "; ".join(dictionary_hits))
            cv2.putText(
                frame,
                pose_text,
                (35, 325),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.82,
                (0, 220, 0) if pose_ok else (0, 165, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                "OpenCV board axes: X=red Y=green Z=blue",
                (35, 365),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        if self.publish_debug_image:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)

        if self.debug_view:
            cv2.imshow("Blueye pipeline ArUco detector", frame)
            cv2.waitKey(1)

    def pose_z_is_valid(self, z):
        z_min = float(self.get_parameter("pose_z_min").value)
        z_max = float(self.get_parameter("pose_z_max").value)
        return z_min <= z <= z_max


def main(args=None):
    rclpy.init(args=args)
    node = PipelineArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
