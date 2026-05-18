import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from mundus_mir_stonefish_docking_controller.aruco_board_stonefish import id_board, pos_board


class StonefishArucoPose(Node):
    def __init__(self):
        super().__init__("stonefish_aruco_pose")

        self.declare_parameter("image_topic", "/blueye/cam")
        self.declare_parameter("camera_frame_id", "blueye_camera_front")
        self.declare_parameter("tag_frame_id", "tag")
        self.declare_parameter("debug_view", False)
        self.declare_parameter("min_markers_for_detection", 1)
        self.declare_parameter("fx", 1662.8)
        self.declare_parameter("fy", 1662.8)
        self.declare_parameter("cx", 960.0)
        self.declare_parameter("cy", 540.0)
        self.declare_parameter("pos_stddev", 0.01)
        self.declare_parameter("yaw_stddev", 0.03)

        self.image_topic = self.get_parameter("image_topic").value
        self.tag_frame_id = self.get_parameter("tag_frame_id").value
        self.debug_view = self.get_parameter("debug_view").value
        self.min_markers_for_detection = self.get_parameter("min_markers_for_detection").value
        fx = self.get_parameter("fx").value
        fy = self.get_parameter("fy").value
        cx = self.get_parameter("cx").value
        cy = self.get_parameter("cy").value
        self.camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
        self.distortion = np.zeros(5)

        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.board = cv2.aruco.Board(pos_board, self.aruco_dict, id_board)
        self.rot_matrix_180_x = np.diag([1.0, -1.0, -1.0])
        self.font = cv2.FONT_HERSHEY_COMPLEX_SMALL

        self.current_waypoint = [0.0, 0.0, 0.0, 0.0]
        self.have_current_waypoint = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.have_filtered_odometry = False
        self.docking_progress = "WP reached:0 remain:0"
        self.have_docking_progress = False

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.filtered_odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.filtered_odometry_callback, 10
        )
        self.current_waypoint_sub = self.create_subscription(
            Pose, "/blueye/current_waypoint", self.current_waypoint_callback, 10
        )
        self.docking_progress_sub = self.create_subscription(
            String, "/blueye/docking_progress", self.docking_progress_callback, 10
        )
        self.pose_pub = self.create_publisher(Pose, "/blueye/pose_estimated_board", 10)
        self.pose_stamped_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/blueye/pose_estimated_board_stamped", 10
        )
        self.detected_pub = self.create_publisher(Bool, "/blueye/docking_station_detected", 10)
        self.visibility_pub = self.create_publisher(String, "/blueye/aruco_visibility", 10)

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

    def current_waypoint_callback(self, msg):
        self.current_waypoint = [
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.orientation.w,
        ]
        self.have_current_waypoint = True

    def docking_progress_callback(self, msg):
        self.docking_progress = msg.data
        self.have_docking_progress = True

    def filtered_odometry_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.have_filtered_odometry = True

    def draw_pose_overlay(self, frame):
        if self.have_filtered_odometry:
            estimated_pose = (
                "Estimated Pose     "
                f"x={self.current_x:4.3f}  "
                f"y={self.current_y:4.3f}  "
                f"z={self.current_z:4.3f}  "
                f"yaw={math.degrees(self.current_yaw):4.3f}"
            )
        else:
            estimated_pose = "Estimated Pose     waiting for /odometry/filtered"

        cv2.putText(
            frame,
            estimated_pose,
            (50, 120),
            self.font,
            2,
            (215, 0, 215),
            2,
            cv2.LINE_AA,
        )

        if self.have_current_waypoint:
            waypoint = (
                "Current Waypoint   "
                f"x={self.current_waypoint[0]:4.3f}  "
                f"y={self.current_waypoint[1]:4.3f}  "
                f"z={self.current_waypoint[2]:4.3f}  "
                f"yaw={self.current_waypoint[3]:4.3f}"
            )
            cv2.putText(
                frame,
                waypoint,
                (50, 190),
                self.font,
                2,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        if self.have_docking_progress:
            cv2.putText(
                frame,
                self.docking_progress,
                (50, 250),
                self.font,
                1.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)
        corners, ids, rejected = detector.detectMarkers(gray)
        marker_count = 0 if ids is None else len(ids)

        detected_msg = Bool()
        detected_msg.data = marker_count >= self.min_markers_for_detection
        self.detected_pub.publish(detected_msg)

        visibility_msg = String()
        visibility_msg.data = "All" if marker_count >= 10 else "Some" if marker_count >= 1 else "None"
        self.visibility_pub.publish(visibility_msg)

        if marker_count == 0:
            if self.debug_view:
                cv2.putText(
                    frame,
                    "No ArUco markers detected",
                    (40, 45),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                self.draw_pose_overlay(frame)
                cv2.imshow("Stonefish ArUco pose", frame)
                cv2.waitKey(1)
            return

        obj_points, img_points = self.board.matchImagePoints(corners, ids)
        if obj_points is None or img_points is None or len(obj_points) < 4:
            if self.debug_view:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.putText(
                    frame,
                    f"Detected {marker_count} marker(s), not enough board points",
                    (40, 45),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    (0, 165, 255),
                    2,
                    cv2.LINE_AA,
                )
                self.draw_pose_overlay(frame)
                cv2.imshow("Stonefish ArUco pose", frame)
                cv2.waitKey(1)
            return

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
        if not ok:
            if self.debug_view:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.putText(
                    frame,
                    "solvePnP failed",
                    (40, 45),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    (0, 165, 255),
                    2,
                    cv2.LINE_AA,
                )
                self.draw_pose_overlay(frame)
                cv2.imshow("Stonefish ArUco pose", frame)
                cv2.waitKey(1)
            return

        r_camera_tag = np.asarray(cv2.Rodrigues(rvec_board)[0])
        r_tag_camera = r_camera_tag.T
        t_tag_camera = (-r_tag_camera @ np.asarray(tvec_board).reshape(3, 1)).flatten()
        _, pitch_camera_board, _ = self.rotation_matrix_to_euler_xyz(self.rot_matrix_180_x @ r_tag_camera)
        yaw_blueye_wrt_tag = -pitch_camera_board

        pose_msg = Pose()
        pose_msg.position.x = float(t_tag_camera[0])
        pose_msg.position.y = float(t_tag_camera[1])
        pose_msg.position.z = float(t_tag_camera[2])
        pose_msg.orientation.z = float(yaw_blueye_wrt_tag)
        self.pose_pub.publish(pose_msg)

        pose_stamped = PoseWithCovarianceStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.tag_frame_id
        pose_stamped.pose.pose.position.x = float(t_tag_camera[2])
        pose_stamped.pose.pose.position.y = float(t_tag_camera[0])
        pose_stamped.pose.pose.position.z = float(t_tag_camera[1])
        qx, qy, qz, qw = self.yaw_to_quaternion(yaw_blueye_wrt_tag - math.pi)
        pose_stamped.pose.pose.orientation.x = qx
        pose_stamped.pose.pose.orientation.y = qy
        pose_stamped.pose.pose.orientation.z = qz
        pose_stamped.pose.pose.orientation.w = qw

        pos_var = self.get_parameter("pos_stddev").value ** 2
        yaw_var = self.get_parameter("yaw_stddev").value ** 2
        pose_stamped.pose.covariance = [
            pos_var, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, pos_var, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, pos_var, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, yaw_var,
        ]
        self.pose_stamped_pub.publish(pose_stamped)

        if self.debug_view:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.drawFrameAxes(frame, self.camera_matrix, self.distortion, rvec_board, tvec_board, 0.25)
            cv2.putText(
                frame,
                f"Detected {marker_count} marker(s)",
                (40, 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            self.draw_pose_overlay(frame)
            cv2.imshow("Stonefish ArUco pose", frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = StonefishArucoPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
