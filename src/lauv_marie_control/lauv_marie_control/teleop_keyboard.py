#!/usr/bin/env python3
"""
Keyboard control of LAUV Marie through its autopilot (lauv_marie_control autopilot). Run it in its own terminal:

  ros2 run lauv_marie_control teleop_keyboard

  w / s     propeller faster / slower (100 rpm steps; about 0.115 m/s each)
  a / d     turn to port / starboard (10 degree steps of the heading setpoint)
  r / f     shallower / deeper (0.5 m steps of the depth setpoint)
  space     stop the propeller
  q         quit (the autopilot keeps the last setpoints)

The line at the bottom shows the setpoints and the vehicle's state from /lauv_marie/odom.
"""

import math
import select
import sys
import termios
import tty

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

RPM_STEP, HEADING_STEP, DEPTH_STEP = 100.0, 10.0, 0.5


class Teleop(Node):
    def __init__(self):
        super().__init__("lauv_marie_teleop")
        self.setpoints = None            # heading deg, depth m, rpm: from the autopilot until a key changes them
        self.state = None                # north, east, depth, heading deg, pitch deg, speed
        self._pubs = {name: self.create_publisher(Float64, f"/lauv_marie/cmd/{name}", 10)
                      for name in ("heading", "depth", "rpm")}
        self.create_subscription(Float64MultiArray, "/lauv_marie/autopilot/setpoints", self._setpoints_cb, 10)
        self.create_subscription(Odometry, "/lauv_marie/odom", self._odom_cb, 10)

    def _setpoints_cb(self, msg: Float64MultiArray):
        self.setpoints = list(msg.data)

    def _odom_cb(self, msg: Odometry):
        q, p = msg.pose.pose.orientation, msg.pose.pose.position
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.state = (p.x, p.y, p.z, math.degrees(yaw) % 360.0, math.degrees(pitch), msg.twist.twist.linear.x)

    def key(self, k):
        """True when the key was a command"""
        if self.setpoints is None:
            return False
        heading, depth, rpm = self.setpoints
        if k == "w":
            self._send("rpm", rpm + RPM_STEP)
        elif k == "s":
            self._send("rpm", rpm - RPM_STEP)
        elif k == " ":
            self._send("rpm", 0.0)
        elif k == "a":
            self._send("heading", (heading - HEADING_STEP) % 360.0)
        elif k == "d":
            self._send("heading", (heading + HEADING_STEP) % 360.0)
        elif k == "r":
            self._send("depth", max(0.0, depth - DEPTH_STEP))
        elif k == "f":
            self._send("depth", depth + DEPTH_STEP)
        else:
            return False
        return True

    def _send(self, name, value):
        self._pubs[name].publish(Float64(data=float(value)))
        self.setpoints[("heading", "depth", "rpm").index(name)] = value     # shown at once; the autopilot confirms

    def status(self):
        if self.setpoints is None:
            return "waiting for the autopilot (/lauv_marie/autopilot/setpoints) ..."
        heading, depth, rpm = self.setpoints
        line = f"set: heading {heading:5.1f} deg  depth {depth:5.1f} m  {rpm:5.0f} rpm"
        if self.state is not None:
            north, east, z, yaw, pitch, speed = self.state
            line += (f"  |  now: heading {yaw:5.1f}  depth {z:5.2f}  pitch {pitch:5.1f}  {speed:4.2f} m/s"
                     f"  N {north:7.1f} E {east:7.1f}")
        return line


def main():
    rclpy.init()
    node = Teleop()
    print(__doc__)
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0.0)[0]:
                k = sys.stdin.read(1)
                if k in ("q", "\x03"):
                    break
                node.key(k.lower())
            sys.stdout.write("\r\033[K" + node.status())
            sys.stdout.flush()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
