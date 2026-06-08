import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, TwistStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import SetBool


class WaypointHandler:
    def __init__(self):
        self.waypoints_left_loop = [
            [-0.35, -1.5, -0.2, np.deg2rad(98), 0.3],
            [1.5, -1.5, -0.45, np.deg2rad(145), 0.3],
            [1.5, 0.0, -0.45, np.deg2rad(180), 0.13],
            [0.0, 0.0, -0.417, np.deg2rad(180), 0.13],
            # [-0.32, 0.0, -0.29, np.deg2rad(180), 0.08],
            # [-0.854, 0.0, -0.232, np.deg2rad(180), 0.05],
            [-0.198, -0.038, -0.400, np.deg2rad(180), 0.13],
            [-0.777, -0.022, -0.335, np.deg2rad(180), 0.08],

        ]
        self.waypoints_right_loop = [
            [-0.5, 2.5, -0.4, np.deg2rad(-90), 0.3],
            [1.5, 2.5, -0.45, np.deg2rad(-145), 0.3],
            [1.5, 1.25, -0.45, np.deg2rad(-155), 0.3],
            [1.5, 0.0, -0.45, np.deg2rad(-180), 0.13],
            [0.0, 0.0, -0.417, np.deg2rad(-180), 0.13],
            # [-0.32, 0.0, -0.29, np.deg2rad(-180), 0.08],
            # [-0.854, 0.0, -0.232, np.deg2rad(-180), 0.05],
            [-0.198, -0.038, -0.400, np.deg2rad(-180), 0.13],
            [-0.777, -0.022, -0.335, np.deg2rad(-180), 0.08],
        ]
        self.waypoints = self.waypoints_right_loop.copy()

    @staticmethod
    def ssa(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def fetch_waypoint(self, current_pose):
        current_waypoint = self.waypoints[0]
        distance = np.linalg.norm(np.array(current_waypoint[:3]) - np.array(current_pose[:3]))
        heading_error = abs(self.ssa(current_waypoint[3] - current_pose[3]))

        if distance < current_waypoint[4] and heading_error < np.deg2rad(5):
            if len(self.waypoints) > 1:
                self.waypoints.pop(0)
                return True, self.waypoints[0]
        return False, current_waypoint


class Controller:
    def __init__(self, kp, ki, kd, integral_max=0.2):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = integral_max
        self.integral = 0.0
        self.prev_time = time.perf_counter()

    def compute(self, error, velocity, cap=1.0):
        now = time.perf_counter()
        dt = max(now - self.prev_time, 1e-3)
        self.prev_time = now
        self.integral = np.clip(self.integral + error * dt, -self.integral_max, self.integral_max)
        return float(np.clip(self.kp * error + self.ki * self.integral - self.kd * velocity, -cap, cap))

    def reset(self):
        self.integral = 0.0
        self.prev_time = time.perf_counter()


class StonefishDockingController(Node):
    def __init__(self):
        super().__init__("stonefish_docking_controller")

        self.declare_parameter("odometry_topic", "/odometry/filtered")
        self.declare_parameter("visual_pose_topic", "/blueye/pose_estimated_board_stamped")
        self.declare_parameter("velocity_odometry_topic", "/blueye/odom")
        self.declare_parameter("command_topic", "/blueye/ref_vel")
        self.declare_parameter("controller_period", 0.1)
        self.declare_parameter("docking_mission", True)
        self.declare_parameter("require_aruco_detection", True)
        self.declare_parameter("use_visual_pose_fallback", True)
        self.declare_parameter("prefer_visual_pose", True)
        self.declare_parameter("auto_start", False)

        self.command_pub = self.create_publisher(TwistStamped, self.get_parameter("command_topic").value, 10)
        self.status_pub = self.create_publisher(String, "/blueye/docking_controller_status", 10)
        self.progress_pub = self.create_publisher(String, "/blueye/docking_progress", 10)
        self.current_waypoint_pub = self.create_publisher(Pose, "/blueye/current_waypoint", 10)
        self.current_reference_pub = self.create_publisher(Pose, "/blueye/current_reference", 10)
        self.docking_detected_sub = self.create_subscription(
            Bool, "/blueye/docking_station_detected", self.docking_detected_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter("odometry_topic").value, self.odometry_callback, 10
        )
        self.visual_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("visual_pose_topic").value,
            self.visual_pose_callback,
            10,
        )
        self.velocity_odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter("velocity_odometry_topic").value,
            self.velocity_odometry_callback,
            10,
        )
        self.start_service = self.create_service(SetBool, "/blueye/start_docking", self.start_docking_callback)

        self.docking_detected = False
        self.docking_armed = bool(self.get_parameter("auto_start").value)
        self.valid_odometry = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_psi = 0.0
        self.current_u = 0.0
        self.current_v = 0.0
        self.current_w = 0.0
        self.current_r = 0.0
        self.have_ekf_odometry = False
        self.have_visual_pose = False

        self.previous_time = time.perf_counter()
        self.first_update = True
        self.x_d_filtered_previous = 0.0
        self.y_d_filtered_previous = 0.0
        self.z_d_filtered_previous = 0.0

        # self.surge_controller = Controller(0.8, 0.2, 1.8, 0.2)
        # self.sway_controller = Controller(0.8, 0.2, 2.6, 0.2)

        self.surge_controller = Controller(1.3, 0.3, 1.8, 0.2)
        self.sway_controller = Controller(1.3, 0.3, 2.6, 0.2)
        self.heave_controller = Controller(3.5, 0.6, 2.5, 0.15)
        # self.yaw_controller = Controller(2.0, 0.3, 4.0, 0.2)
        self.yaw_controller = Controller(0.8, 0.05, 3.0, 0.05)

        self.waypoint_handler = WaypointHandler()
        self.completed_waypoints = 0
        self.total_waypoints = len(self.waypoint_handler.waypoints)
        self.previous_waypoint = []
        self.timer = self.create_timer(self.get_parameter("controller_period").value, self.command_callback)
        state = "armed" if self.docking_armed else "disarmed; call /blueye/start_docking to begin"
        self.get_logger().info(f"Docking controller started {state}")

    @staticmethod
    def ssa(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def docking_detected_callback(self, msg):
        self.docking_detected = msg.data

    def reset_docking_state(self):
        self.previous_time = time.perf_counter()
        self.first_update = True
        self.x_d_filtered_previous = self.current_x
        self.y_d_filtered_previous = self.current_y
        self.z_d_filtered_previous = self.current_z
        self.waypoint_handler = WaypointHandler()
        self.completed_waypoints = 0
        self.total_waypoints = len(self.waypoint_handler.waypoints)
        self.previous_waypoint = []
        self.surge_controller.reset()
        self.sway_controller.reset()
        self.heave_controller.reset()
        self.yaw_controller.reset()

    def start_docking_callback(self, request, response):
        if request.data:
            self.reset_docking_state()
            self.docking_armed = True
            response.success = True
            response.message = "Docking armed"
            self.publish_status("docking_armed")
            self.get_logger().info("Docking armed")
        else:
            self.docking_armed = False
            self.reset_docking_state()
            self.publish_stop()
            response.success = True
            response.message = "Docking disarmed"
            self.publish_status("docking_disarmed")
            self.get_logger().info("Docking disarmed")
        return response

    @staticmethod
    def closest_waypoint(current_position, waypoints):
        distances = [np.linalg.norm(np.array(current_position[:3]) - np.array(wp[:3])) for wp in waypoints]
        index = int(np.argmin(distances))
        return waypoints[index], index, distances[index]

    def command_callback(self):
        if not self.docking_armed:
            self.publish_status("waiting_for_start")
            return
        if not self.valid_odometry:
            self.publish_status("waiting_for_pose")
            return
        if self.get_parameter("require_aruco_detection").value and not self.docking_detected:
            self.publish_status("waiting_for_aruco_detection")
            self.publish_stop()
            return

        if not self.previous_waypoint:
            self.previous_waypoint = [self.current_x, self.current_y, self.current_z, self.current_psi, 0.3]
            if self.get_parameter("docking_mission").value:
                _, i_left, dist_left = self.closest_waypoint(self.previous_waypoint, self.waypoint_handler.waypoints_left_loop)
                _, i_right, dist_right = self.closest_waypoint(self.previous_waypoint, self.waypoint_handler.waypoints_right_loop)
                if dist_left < dist_right:
                    self.waypoint_handler.waypoints = self.waypoint_handler.waypoints_left_loop[i_left:].copy()
                else:
                    self.waypoint_handler.waypoints = self.waypoint_handler.waypoints_right_loop[i_right:].copy()
                self.completed_waypoints = 0
                self.total_waypoints = len(self.waypoint_handler.waypoints)

        potentially_previous_waypoint = self.waypoint_handler.waypoints[0]
        update, waypoint = self.waypoint_handler.fetch_waypoint(
            [self.current_x, self.current_y, self.current_z, self.current_psi]
        )
        if update:
            self.completed_waypoints += 1
            self.previous_waypoint = potentially_previous_waypoint
            self.get_logger().info("Waypoint reached")

        waypoint_msg = Pose()
        waypoint_msg.position.x = float(waypoint[0])
        waypoint_msg.position.y = float(waypoint[1])
        waypoint_msg.position.z = float(waypoint[2])
        waypoint_msg.orientation.w = float(np.rad2deg(waypoint[3]))
        self.current_waypoint_pub.publish(waypoint_msg)
        self.publish_progress()

        path_vector = np.array(waypoint[0:3]) - np.array(self.previous_waypoint[0:3])
        drone_vector = np.array([self.current_x, self.current_y, self.current_z]) - np.array(self.previous_waypoint[0:3])
        denom = np.dot(path_vector, path_vector)
        t = 0.0 if abs(denom) < 1e-9 else np.dot(drone_vector, path_vector) / denom
        t = max(0.0, min(1.0, t))

        heading0 = self.previous_waypoint[3] % (2.0 * math.pi)
        heading1 = waypoint[3] % (2.0 * math.pi)
        psi_d = self.ssa((1.0 - t) * heading0 + t * heading1)

        now = time.perf_counter()
        dt = now - self.previous_time
        alpha = math.exp(-dt / 7.0)

        if self.first_update:
            self.x_d_filtered_previous = self.current_x
            self.y_d_filtered_previous = self.current_y
            self.z_d_filtered_previous = self.current_z
            self.first_update = False

        x_d = alpha * self.x_d_filtered_previous + (1.0 - alpha) * waypoint[0]
        y_d = alpha * self.y_d_filtered_previous + (1.0 - alpha) * waypoint[1]
        z_d = alpha * self.z_d_filtered_previous + (1.0 - alpha) * waypoint[2]
        self.x_d_filtered_previous = x_d
        self.y_d_filtered_previous = y_d
        self.z_d_filtered_previous = z_d
        self.previous_time = now

        ref_msg = Pose()
        ref_msg.position.x = float(x_d)
        ref_msg.position.y = float(y_d)
        ref_msg.position.z = float(z_d)
        ref_msg.orientation.w = float(np.rad2deg(psi_d))
        self.current_reference_pub.publish(ref_msg)

        delta_x = x_d - self.current_x
        delta_y = y_d - self.current_y
        delta_z = z_d - self.current_z

        error_surge = delta_x * math.cos(self.current_psi) + delta_y * math.sin(self.current_psi)
        error_sway = delta_x * math.sin(self.current_psi) - delta_y * math.cos(self.current_psi)
        error_heave = delta_z
        error_yaw = self.ssa(-(psi_d - self.current_psi))

        limit_heave = 0.3
        limit_yaw = np.deg2rad(4.0)
        delta_r = max(abs(error_heave) - limit_heave, 0.0) + max(abs(error_yaw) - limit_yaw, 0.0)
        lambda_cap = math.exp(delta_r * math.log(0.7))
        final_approach = waypoint[0] <= 0.0
        xy_cap = min(lambda_cap, 0.25) if final_approach else lambda_cap
        yaw_cap = 0.2 if final_approach else 0.4

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "body_frame"
        twist_msg.twist.linear.x = self.surge_controller.compute(error_surge, self.current_u, xy_cap)
        twist_msg.twist.linear.y = -self.sway_controller.compute(error_sway, -self.current_v, xy_cap)
        twist_msg.twist.linear.z = self.heave_controller.compute(error_heave, -self.current_w)
        twist_msg.twist.angular.z = self.yaw_controller.compute(error_yaw, self.current_r, yaw_cap)
        self.command_pub.publish(twist_msg)
        self.publish_status(
            "publishing_ref_vel "
            f"source={'ekf' if self.have_ekf_odometry else 'visual_fallback'} "
            f"x={self.current_x:.3f} y={self.current_y:.3f} z={self.current_z:.3f} "
            f"yaw={self.current_psi:.3f} "
            f"cmd=({twist_msg.twist.linear.x:.3f},"
            f"{twist_msg.twist.linear.y:.3f},"
            f"{twist_msg.twist.linear.z:.3f},"
            f"{twist_msg.twist.angular.z:.3f})"
        )

    def publish_stop(self):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "body_frame"
        self.command_pub.publish(twist_msg)

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_progress(self):
        msg = String()
        msg.data = f"WP reached:{self.completed_waypoints} remain:{len(self.waypoint_handler.waypoints)}"
        self.progress_pub.publish(msg)

    def odometry_callback(self, msg):
        self.have_ekf_odometry = True
        if self.get_parameter("prefer_visual_pose").value and self.have_visual_pose:
            self.current_u = msg.twist.twist.linear.x
            self.current_v = msg.twist.twist.linear.y
            self.current_w = msg.twist.twist.linear.z
            self.current_r = msg.twist.twist.angular.z
            return

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        self.current_psi = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.current_u = msg.twist.twist.linear.x
        self.current_v = msg.twist.twist.linear.y
        self.current_w = msg.twist.twist.linear.z
        self.current_r = msg.twist.twist.angular.z
        self.valid_odometry = True

    def visual_pose_callback(self, msg):
        if not self.get_parameter("prefer_visual_pose").value:
            if self.have_ekf_odometry or not self.get_parameter("use_visual_pose_fallback").value:
                return
        elif not self.get_parameter("use_visual_pose_fallback").value:
            return

        self.have_visual_pose = True
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        self.current_psi = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.valid_odometry = True

    def velocity_odometry_callback(self, msg):
        if not self.get_parameter("prefer_visual_pose").value and self.have_ekf_odometry:
            return
        if not self.get_parameter("use_visual_pose_fallback").value:
            return

        self.current_u = msg.twist.twist.linear.x
        self.current_v = msg.twist.twist.linear.y
        self.current_w = msg.twist.twist.linear.z
        self.current_r = msg.twist.twist.angular.z


def main(args=None):
    rclpy.init(args=args)
    node = StonefishDockingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
