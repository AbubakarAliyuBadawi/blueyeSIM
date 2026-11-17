#!/usr/bin/env python3
"""
Autopilot for LAUV Marie in Stonefish: heading, depth and propeller speed.

A torpedo AUV has one propeller and four fins, so it cannot hover or move sideways: it holds a heading with the
vertical fins and a depth by pitching with the horizontal fins, and both need forward speed.

Setpoints (std_msgs/Float64 unless noted):
  /lauv_marie/cmd/heading   heading in degrees, 0 = north, 90 = east
  /lauv_marie/cmd/depth     depth in m, positive down
  /lauv_marie/cmd/rpm       propeller speed in rpm (about 1.15 m/s per 1000 rpm), 0 stops the propeller
  /lauv_marie/cmd/goto      geometry_msgs/Point: north, east [m] and depth [m]. The heading then follows the bearing
                            to the point, and the propeller stops within acceptance_radius_m of it. A new heading
                            setpoint cancels it.

Commands to the simulator (std_msgs/Float64MultiArray):
  /lauv_marie/thrusters     [rpm]
  /lauv_marie/rudders       [top, bottom, port, starboard, 0, 0] fin angles in rad (the last two entries are the
                            fixed hull-lift surfaces of drones/lauv_marie.scn)

State: /lauv_marie/odom, the simulator's ground truth (NED position, body velocities), not a navigation estimate.

Control laws:
  rudder   = kp_heading * (heading error) - kd_heading * (yaw rate)
  pitch_d  = -limit(kp_depth * (depth error) + ki_depth * integral, max_pitch)       nose down to go deeper
  elevator = kp_pitch * (pitch - pitch_d) + kd_pitch * (pitch rate)                  positive = nose down
"""

import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


def wrap(angle):
    """angle in (-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))


def limit(value, bound):
    return max(-bound, min(bound, value))


class Autopilot(Node):
    def __init__(self):
        super().__init__("lauv_marie_autopilot")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("max_fin_angle", 0.436)           # rad (25 deg), as in drones/lauv_marie.scn
        self.declare_parameter("max_rpm", 2000.0)
        self.declare_parameter("kp_heading", 1.0)                # rad of rudder per rad of heading error
        self.declare_parameter("kd_heading", 1.0)                # rad per rad/s
        self.declare_parameter("kp_depth", 0.15)                 # rad of pitch per m of depth error
        self.declare_parameter("ki_depth", 0.01)                 # rad per m s
        self.declare_parameter("max_pitch", 0.35)                # rad (20 deg)
        self.declare_parameter("kp_pitch", 1.5)                  # rad of elevator per rad of pitch error
        self.declare_parameter("kd_pitch", 1.0)                  # rad per rad/s
        self.declare_parameter("acceptance_radius_m", 3.0)
        self.declare_parameter("initial_rpm", 0.0)

        self._state = None               # north, east, depth, pitch, yaw, pitch rate, yaw rate
        self._heading = None             # setpoints; heading and depth are taken from the first odometry message
        self._depth = None
        self._rpm = float(self.get_parameter("initial_rpm").value)
        self._goal = None
        self._depth_integral = 0.0

        self.create_subscription(Odometry, "/lauv_marie/odom", self._odom_cb, 10)
        self.create_subscription(Float64, "/lauv_marie/cmd/heading", self._heading_cb, 10)
        self.create_subscription(Float64, "/lauv_marie/cmd/depth", self._depth_cb, 10)
        self.create_subscription(Float64, "/lauv_marie/cmd/rpm", self._rpm_cb, 10)
        self.create_subscription(Point, "/lauv_marie/cmd/goto", self._goto_cb, 10)
        self._thruster_pub = self.create_publisher(Float64MultiArray, "/lauv_marie/thrusters", 10)
        self._fin_pub = self.create_publisher(Float64MultiArray, "/lauv_marie/rudders", 10)
        # what the autopilot is holding, for displays: [heading deg, depth m, rpm]
        self._setpoint_pub = self.create_publisher(Float64MultiArray, "/lauv_marie/autopilot/setpoints", 10)

        self._dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.create_timer(self._dt, self._tick)
        self.get_logger().info("LAUV Marie autopilot started: holds the heading and depth it starts at; "
                               "set /lauv_marie/cmd/rpm to move")

    def _odom_cb(self, msg: Odometry):
        q, p, w = msg.pose.pose.orientation, msg.pose.pose.position, msg.twist.twist.angular
        pitch = math.asin(limit(2.0 * (q.w * q.y - q.z * q.x), 1.0))
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._state = (p.x, p.y, p.z, pitch, yaw, w.y, w.z)
        if self._heading is None:
            self._heading, self._depth = yaw, p.z

    def _heading_cb(self, msg: Float64):
        self._heading = wrap(math.radians(msg.data))
        self._goal = None

    def _depth_cb(self, msg: Float64):
        self._depth = max(0.0, msg.data)
        self._depth_integral = 0.0

    def _rpm_cb(self, msg: Float64):
        self._rpm = limit(msg.data, float(self.get_parameter("max_rpm").value))

    def _goto_cb(self, msg: Point):
        self._goal = (msg.x, msg.y)
        self._depth = max(0.0, msg.z)
        self._depth_integral = 0.0
        self.get_logger().info(f"Going to north {msg.x:.1f} m, east {msg.y:.1f} m, depth {msg.z:.1f} m")

    def _tick(self):
        if self._state is None:
            return
        north, east, depth, pitch, yaw, pitch_rate, yaw_rate = self._state
        get = lambda name: float(self.get_parameter(name).value)          # noqa: E731
        max_fin = get("max_fin_angle")
        rpm = self._rpm

        if self._goal is not None:
            d_north, d_east = self._goal[0] - north, self._goal[1] - east
            if math.hypot(d_north, d_east) < get("acceptance_radius_m"):
                self.get_logger().info("Reached the point: propeller stopped")
                self._goal, self._rpm, rpm = None, 0.0, 0.0
            else:
                self._heading = math.atan2(d_east, d_north)

        rudder = limit(get("kp_heading") * wrap(self._heading - yaw) - get("kd_heading") * yaw_rate, max_fin)

        depth_error = self._depth - depth
        # integral only close to the depth and under way: no wind-up during a depth change or while stopped
        if abs(rpm) > 1.0 and abs(depth_error) < 1.0:
            self._depth_integral = limit(self._depth_integral + depth_error * self._dt, 10.0)
        max_pitch = get("max_pitch")
        pitch_d = -limit(get("kp_depth") * depth_error + get("ki_depth") * self._depth_integral, max_pitch)
        elevator = limit(get("kp_pitch") * (pitch - pitch_d) + get("kd_pitch") * pitch_rate, max_fin)

        self._thruster_pub.publish(Float64MultiArray(data=[rpm]))
        self._fin_pub.publish(Float64MultiArray(data=[rudder, rudder, elevator, elevator, 0.0, 0.0]))
        self._setpoint_pub.publish(Float64MultiArray(
            data=[math.degrees(self._heading) % 360.0, self._depth, rpm]))


def main():
    rclpy.init()
    node = Autopilot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
