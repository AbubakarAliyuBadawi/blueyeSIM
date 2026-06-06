import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from blueye_inspection.pipeline_board import inspection_waypoints


@dataclass
class Waypoint:
    marker_id: int
    name: str
    x: float
    y: float
    z: float
    yaw: float
    radius: float


class Controller:
    def __init__(self, kp, ki, kd, integral_max=0.3):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = integral_max
        self.integral = 0.0
        self.prev_time = time.perf_counter()

    def compute(self, error, velocity, cap):
        now = time.perf_counter()
        dt = max(now - self.prev_time, 1e-3)
        self.prev_time = now
        self.integral = max(-self.integral_max, min(self.integral + error * dt, self.integral_max))
        command = self.kp * error + self.ki * self.integral - self.kd * velocity
        return max(-cap, min(command, cap))

    def reset(self):
        self.integral = 0.0
        self.prev_time = time.perf_counter()


class PipelineInspectionController(Node):
    def __init__(self):
        super().__init__("pipeline_inspection_controller")

        self.declare_parameter("odometry_topic", "/odometry/filtered")
        self.declare_parameter("visual_pose_topic", "/blueye/pipeline_pose_estimated_stamped")
        self.declare_parameter("velocity_odometry_topic", "/blueye/odom")
        self.declare_parameter("command_topic", "/blueye/ref_vel")
        self.declare_parameter("controller_period", 0.1)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("require_aruco_detection", True)
        self.declare_parameter("require_visual_pose", True)
        self.declare_parameter("marker_timeout", 1.0)
        self.declare_parameter("inspection_z_offset", 0.5)
        self.declare_parameter("desired_speed_cap", 0.35)
        self.declare_parameter("desired_yaw_rate_cap", 0.35)
        self.declare_parameter("heading_tolerance", 0.25)
        self.declare_parameter("require_heading_for_completion", False)
        self.declare_parameter("yaw_control_mode", "face_target")
        self.declare_parameter("face_target_min_distance", 0.15)
        self.declare_parameter("target_selection_mode", "nearest_visible")

        self.command_pub = self.create_publisher(TwistStamped, self.get_parameter("command_topic").value, 10)
        self.status_pub = self.create_publisher(String, "/blueye/pipeline_inspection_status", 10)
        self.progress_pub = self.create_publisher(String, "/blueye/pipeline_inspection_progress", 10)
        self.current_waypoint_pub = self.create_publisher(Pose, "/blueye/pipeline_current_waypoint", 10)
        self.current_reference_pub = self.create_publisher(Pose, "/blueye/pipeline_current_reference", 10)

        self.odom_sub = self.create_subscription(Odometry, self.get_parameter("odometry_topic").value, self.odometry_callback, 10)
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
        self.marker_detected_sub = self.create_subscription(
            Bool,
            "/blueye/pipeline_marker_detected",
            self.marker_detected_callback,
            10,
        )
        self.marker_ids_sub = self.create_subscription(String, "/blueye/pipeline_marker_ids", self.marker_ids_callback, 10)
        self.start_service = self.create_service(SetBool, "/blueye/start_pipeline_inspection", self.start_callback)

        self.armed = bool(self.get_parameter("auto_start").value)
        self.marker_detected = False
        self.last_marker_time = 0.0
        self.last_marker_ids = ""
        self.visible_marker_ids = set()
        self.completed_marker_ids = set()
        self.current_target_id = None
        self.valid_odometry = False
        self.have_visual_pose = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.current_u = 0.0
        self.current_v = 0.0
        self.current_w = 0.0
        self.current_r = 0.0

        self.surge_controller = Controller(0.9, 0.05, 0.6)
        self.sway_controller = Controller(0.9, 0.05, 0.7)
        self.heave_controller = Controller(1.8, 0.15, 0.8)
        self.yaw_controller = Controller(1.0, 0.03, 0.7)
        self.waypoints = self.build_waypoints()
        self.waypoints_by_id = {waypoint.marker_id: waypoint for waypoint in self.waypoints}

        self.timer = self.create_timer(float(self.get_parameter("controller_period").value), self.command_callback)
        state = "armed" if self.armed else "disarmed; call /blueye/start_pipeline_inspection to begin"
        self.get_logger().info(f"Pipeline inspection controller started {state}")

    @staticmethod
    def ssa(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def build_waypoints(self):
        z_offset = float(self.get_parameter("inspection_z_offset").value)
        waypoints = []
        for waypoint in inspection_waypoints:
            waypoints.append(
                Waypoint(
                    int(waypoint["id"]),
                    waypoint["name"],
                    waypoint["x"],
                    waypoint["y"],
                    waypoint["z"] + z_offset,
                    self.ssa(waypoint["yaw"]),
                    waypoint["radius"],
                )
            )
        return waypoints

    def reset(self):
        self.waypoints = self.build_waypoints()
        self.waypoints_by_id = {waypoint.marker_id: waypoint for waypoint in self.waypoints}
        self.completed_marker_ids = set()
        self.current_target_id = None
        self.surge_controller.reset()
        self.sway_controller.reset()
        self.heave_controller.reset()
        self.yaw_controller.reset()

    def start_callback(self, request, response):
        if request.data:
            self.reset()
            self.armed = True
            response.message = "Pipeline inspection armed"
            self.publish_status("pipeline_inspection_armed")
        else:
            self.armed = False
            self.publish_stop()
            response.message = "Pipeline inspection disarmed"
            self.publish_status("pipeline_inspection_disarmed")
        response.success = True
        return response

    def marker_detected_callback(self, msg):
        self.marker_detected = bool(msg.data)
        if self.marker_detected:
            self.last_marker_time = time.perf_counter()

    def marker_ids_callback(self, msg):
        self.last_marker_ids = msg.data
        visible_marker_ids = set()
        for raw_marker_id in msg.data.split(","):
            raw_marker_id = raw_marker_id.strip()
            if not raw_marker_id:
                continue
            try:
                visible_marker_ids.add(int(raw_marker_id))
            except ValueError:
                self.get_logger().warn(f"Ignoring malformed marker id '{raw_marker_id}'")
        self.visible_marker_ids = visible_marker_ids

    def choose_target_waypoint(self):
        unvisited_visible_ids = [
            marker_id
            for marker_id in self.visible_marker_ids
            if marker_id in self.waypoints_by_id and marker_id not in self.completed_marker_ids
        ]
        if not unvisited_visible_ids:
            return None

        mode = str(self.get_parameter("target_selection_mode").value)
        if mode == "ordered_visible":
            marker_id = min(unvisited_visible_ids)
            return self.waypoints_by_id[marker_id]

        def distance_to_marker(marker_id):
            waypoint = self.waypoints_by_id[marker_id]
            dx = waypoint.x - self.current_x
            dy = waypoint.y - self.current_y
            dz = waypoint.z - self.current_z
            return math.sqrt(dx * dx + dy * dy + dz * dz)

        marker_id = min(unvisited_visible_ids, key=distance_to_marker)
        return self.waypoints_by_id[marker_id]

    def odometry_callback(self, msg):
        if self.have_visual_pose and bool(self.get_parameter("require_visual_pose").value):
            self.current_u = msg.twist.twist.linear.x
            self.current_v = msg.twist.twist.linear.y
            self.current_w = msg.twist.twist.linear.z
            self.current_r = msg.twist.twist.angular.z
            return
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        self.current_yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)
        self.current_u = msg.twist.twist.linear.x
        self.current_v = msg.twist.twist.linear.y
        self.current_w = msg.twist.twist.linear.z
        self.current_r = msg.twist.twist.angular.z
        self.valid_odometry = True

    def visual_pose_callback(self, msg):
        self.have_visual_pose = True
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        self.current_yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)
        self.valid_odometry = True

    def velocity_odometry_callback(self, msg):
        self.current_u = msg.twist.twist.linear.x
        self.current_v = msg.twist.twist.linear.y
        self.current_w = msg.twist.twist.linear.z
        self.current_r = msg.twist.twist.angular.z

    def command_callback(self):
        if not self.armed:
            self.publish_status("waiting_for_start")
            self.publish_stop()
            return
        if not self.valid_odometry:
            self.publish_status("waiting_for_odometry")
            return
        if bool(self.get_parameter("require_visual_pose").value) and not self.have_visual_pose:
            self.publish_status("waiting_for_pipeline_visual_pose")
            self.publish_stop()
            return
        if len(self.completed_marker_ids) >= len(self.waypoints):
            self.publish_status("pipeline_inspection_complete")
            self.publish_stop()
            return

        marker_fresh = (time.perf_counter() - self.last_marker_time) <= float(self.get_parameter("marker_timeout").value)
        if bool(self.get_parameter("require_aruco_detection").value) and not marker_fresh:
            self.publish_status("waiting_for_pipeline_aruco")
            self.publish_stop()
            return

        if self.current_target_id not in self.waypoints_by_id or self.current_target_id in self.completed_marker_ids:
            target = self.choose_target_waypoint()
            if target is None:
                self.publish_status(
                    "waiting_for_unvisited_visible_marker "
                    f"visible={sorted(self.visible_marker_ids)} done={sorted(self.completed_marker_ids)}"
                )
                self.publish_stop()
                return
            self.current_target_id = target.marker_id

        waypoint = self.waypoints_by_id[self.current_target_id]
        dx = waypoint.x - self.current_x
        dy = waypoint.y - self.current_y
        dz = waypoint.z - self.current_z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        horizontal_distance = math.sqrt(dx * dx + dy * dy)
        desired_yaw = self.desired_yaw_for_target(waypoint, dx, dy, horizontal_distance)
        yaw_error = self.ssa(desired_yaw - self.current_yaw)
        heading_ok = (
            not bool(self.get_parameter("require_heading_for_completion").value)
            or abs(yaw_error) < float(self.get_parameter("heading_tolerance").value)
        )
        if distance < waypoint.radius and heading_ok:
            self.completed_marker_ids.add(waypoint.marker_id)
            self.current_target_id = None
            self.publish_progress()
            if len(self.completed_marker_ids) >= len(self.waypoints):
                self.publish_status("pipeline_inspection_complete")
                self.publish_stop()
                return
            target = self.choose_target_waypoint()
            if target is None:
                self.publish_status(
                    "reached_marker_waiting_for_next_visible "
                    f"done={sorted(self.completed_marker_ids)} visible={sorted(self.visible_marker_ids)}"
                )
                self.publish_stop()
                return
            self.current_target_id = target.marker_id
            waypoint = target
            dx = waypoint.x - self.current_x
            dy = waypoint.y - self.current_y
            dz = waypoint.z - self.current_z
            horizontal_distance = math.sqrt(dx * dx + dy * dy)
            desired_yaw = self.desired_yaw_for_target(waypoint, dx, dy, horizontal_distance)
            yaw_error = self.ssa(desired_yaw - self.current_yaw)

        surge_error = dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw)
        sway_error = -dx * math.sin(self.current_yaw) + dy * math.cos(self.current_yaw)
        speed_cap = float(self.get_parameter("desired_speed_cap").value)
        yaw_cap = float(self.get_parameter("desired_yaw_rate_cap").value)

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "body_frame"
        twist.twist.linear.x = self.surge_controller.compute(surge_error, self.current_u, speed_cap)
        twist.twist.linear.y = self.sway_controller.compute(sway_error, self.current_v, speed_cap)
        twist.twist.linear.z = self.heave_controller.compute(dz, self.current_w, speed_cap)
        twist.twist.angular.z = self.yaw_controller.compute(yaw_error, self.current_r, yaw_cap)
        self.command_pub.publish(twist)

        self.publish_waypoint(waypoint)
        self.publish_progress()
        self.publish_status(
            f"tracking_{waypoint.name}_id_{waypoint.marker_id} "
            f"visible={sorted(self.visible_marker_ids)} done={sorted(self.completed_marker_ids)} "
            f"cmd=({twist.twist.linear.x:.2f},{twist.twist.linear.y:.2f},{twist.twist.linear.z:.2f},{twist.twist.angular.z:.2f})"
        )

    def desired_yaw_for_target(self, waypoint, dx, dy, horizontal_distance):
        mode = str(self.get_parameter("yaw_control_mode").value)
        if mode == "off":
            return self.current_yaw
        if mode == "fixed":
            return waypoint.yaw
        min_distance = float(self.get_parameter("face_target_min_distance").value)
        if horizontal_distance > min_distance:
            return math.atan2(dy, dx)
        return waypoint.yaw

    def publish_stop(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "body_frame"
        self.command_pub.publish(twist)

    def publish_waypoint(self, waypoint):
        msg = Pose()
        msg.position.x = waypoint.x
        msg.position.y = waypoint.y
        msg.position.z = waypoint.z
        msg.orientation.w = math.degrees(waypoint.yaw)
        self.current_waypoint_pub.publish(msg)
        self.current_reference_pub.publish(msg)

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_progress(self):
        msg = String()
        remaining = max(len(self.waypoints) - len(self.completed_marker_ids), 0)
        target = "none" if self.current_target_id is None else str(self.current_target_id)
        msg.data = (
            f"markers inspected:{len(self.completed_marker_ids)} remain:{remaining} "
            f"target:{target} visible:{sorted(self.visible_marker_ids)}"
        )
        self.progress_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PipelineInspectionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
