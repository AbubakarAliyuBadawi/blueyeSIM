#!/usr/bin/env python3
"""
Bridge between DUNE (the LAUV's onboard software, run on this computer) and Marie in Stonefish.

DUNE runs with its own vehicle simulation switched off (dune/lauv-marie-stonefish.ini). It then needs the vehicle's
motion from outside, and its actuator commands need a vehicle to act on:

  Stonefish  /lauv_marie/odom  ->  IMC SimulatedState (20 Hz)  ->  DUNE: simulated sensors, navigation, control
  DUNE  IMC SetServoPosition (fins), Rpm (propeller)           ->  /lauv_marie/rudders, /lauv_marie/thrusters

So DUNE flies Marie as it flies the real vehicle: plans and manoeuvres from Neptus (Goto, PopUp, YoYo, ...), its own
controllers and safety. Do not run the autopilot node of this package at the same time: both would command the fins.

IMC side (UDP on this computer), a link of its own ([Transports.UDP/Stonefish] in dune/lauv-marie-stonefish.ini), so
Neptus keeps DUNE's usual port 6002:
  to DUNE     dune_host:dune_port (6971). SimulatedState is sent with DUNE's own IMC address as its source, which is
              what DUNE's simulated sensors expect; the address is read from the first message DUNE sends
  from DUNE   listen_port (6970): the link's static destination

Position. Stonefish reports metres north, east and down of its world origin; DUNE wants a WGS-84 reference with
offsets. The reference is origin_latitude, origin_longitude: by default the start position of DUNE's own simulator
in the Trondheim fjord (deep water in DUNE's bathymetry), not the origin of the Stonefish scenario.

Fins. DUNE: servos 0 and 3 are the lower and upper rudder (positive: turn to port), 1 and 2 the port and starboard
elevator (positive: nose down). Stonefish (drones/lauv_marie.scn): [top, bottom, port, starboard, 0, 0], rudders
positive: turn to starboard, elevators positive: nose down.

Propeller. DUNE's speed controller commands a thrust fraction; its simulated motor turns that into the propeller
speed it reports as Rpm. That reported speed is what Stonefish's propeller is set to.

Also published, for other nodes and for checking:
  /lauv_marie/dune/estimated_state   nav_msgs/Odometry: DUNE's own navigation estimate (north, east, depth of the
                                     reference; attitude; body velocities), to compare with /lauv_marie/odom
  /lauv_marie/dune/plan_state        std_msgs/String: "EXECUTING plan_id maneuver_id 42%"
"""

import math
import socket

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from lauv_marie_control import imc

PLAN_STATES = {0: "BLOCKED", 1: "READY", 2: "INITIALIZING", 3: "EXECUTING"}
WANTED = {"SetServoPosition", "Rpm", "EstimatedState", "PlanControl", "PlanControlState", "VehicleState"}


def euler(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def quaternion(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2),
                              math.cos(yaw / 2), math.sin(yaw / 2))
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class DuneBridge(Node):
    def __init__(self):
        super().__init__("lauv_marie_dune_bridge")
        self.declare_parameter("dune_host", "127.0.0.1")
        self.declare_parameter("dune_port", 6971)
        self.declare_parameter("listen_port", 6970)
        self.declare_parameter("state_rate_hz", 20.0)
        self.declare_parameter("origin_latitude", 63.441006)             # degrees
        self.declare_parameter("origin_longitude", 10.351081)
        self.declare_parameter("max_rpm", 2500.0)
        self.declare_parameter("command_timeout_s", 2.0)                 # no command from DUNE: propeller and fins to 0

        self._dune = (self.get_parameter("dune_host").value, int(self.get_parameter("dune_port").value))
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.bind(("0.0.0.0", int(self.get_parameter("listen_port").value)))
        self._rx.setblocking(False)

        self._odom = None
        self._dune_id = None                 # DUNE's IMC address, from its first message
        self._fins = [0.0, 0.0, 0.0, 0.0]    # DUNE's servo commands, id 0-3
        self._rpm = 0.0
        self._last_command = None
        self._plan_state = None
        self._mode = None

        self.create_subscription(Odometry, "/lauv_marie/odom", self._odom_cb, 10)
        self._thruster_pub = self.create_publisher(Float64MultiArray, "/lauv_marie/thrusters", 10)
        self._fin_pub = self.create_publisher(Float64MultiArray, "/lauv_marie/rudders", 10)
        self._estimate_pub = self.create_publisher(Odometry, "/lauv_marie/dune/estimated_state", 10)
        self._plan_pub = self.create_publisher(String, "/lauv_marie/dune/plan_state", 10)

        self.create_timer(0.005, self._receive)
        self.create_timer(1.0 / float(self.get_parameter("state_rate_hz").value), self._send_state)
        self.create_timer(0.05, self._send_commands)
        self.create_timer(5.0, self._watch)
        self.get_logger().info(f"DUNE bridge: vehicle state to {self._dune[0]}:{self._dune[1]}, DUNE's commands "
                               f"on UDP {self.get_parameter('listen_port').value}; waiting for DUNE and Stonefish")

    def _odom_cb(self, msg: Odometry):
        self._odom = msg

    def _receive(self):
        for _ in range(200):                                     # all datagrams that are waiting
            try:
                datagram, _ = self._rx.recvfrom(65535)
            except BlockingIOError:
                return
            for message in imc.decode(datagram, WANTED):
                if self._dune_id is None:
                    self._dune_id = message.source
                    self.get_logger().info(f"DUNE is up (IMC address 0x{message.source:04X})")
                values = message.values
                if message.name == "SetServoPosition" and values["id"] < 4:
                    self._fins[values["id"]] = values["value"]
                    self._last_command = self.get_clock().now()
                elif message.name == "Rpm":
                    self._rpm = float(values["value"])
                    self._last_command = self.get_clock().now()
                elif message.name == "EstimatedState":
                    self._publish_estimate(values)
                elif message.name == "PlanControl" and values["type"] in (1, 2):      # DUNE's reply to a request
                    op = ("start", "stop", "load", "get")[values["op"]] if values["op"] < 4 else str(values["op"])
                    if values["type"] == 2:
                        self.get_logger().error(f"DUNE refused to {op} plan '{values['plan_id']}': {values['info']}")
                    else:
                        self.get_logger().info(f"DUNE: {op} plan '{values['plan_id']}': {values['info']}")
                elif message.name == "VehicleState":
                    mode = ("SERVICE", "CALIBRATION", "ERROR", "MANEUVER", "EXTERNAL", "BOOT")[values["op_mode"]] \
                        if values["op_mode"] < 6 else str(values["op_mode"])
                    if mode != self._mode:
                        self._mode = mode
                        errors = f" ({values['error_ents']})" if values["error_ents"] else ""
                        self.get_logger().info(f"DUNE vehicle mode: {mode}{errors}")
                elif message.name == "PlanControlState":
                    state = PLAN_STATES.get(values["state"], str(values["state"]))
                    text = f"{state} {values['plan_id']} {values['man_id']} {values['plan_progress']:.0f}%"
                    self._plan_pub.publish(String(data=text))
                    if (state, values["plan_id"], values["man_id"]) != self._plan_state:
                        self._plan_state = (state, values["plan_id"], values["man_id"])
                        self.get_logger().info(f"DUNE plan: {state} {values['plan_id']} {values['man_id']}".rstrip())

    def _send_state(self):
        if self._odom is None or self._dune_id is None:
            return
        pose, twist = self._odom.pose.pose, self._odom.twist.twist
        roll, pitch, yaw = euler(pose.orientation)
        state = {"lat": math.radians(float(self.get_parameter("origin_latitude").value)),
                 "lon": math.radians(float(self.get_parameter("origin_longitude").value)), "height": 0.0,
                 "x": pose.position.x, "y": pose.position.y, "z": pose.position.z,
                 "phi": roll, "theta": pitch, "psi": yaw,
                 "u": twist.linear.x, "v": twist.linear.y, "w": twist.linear.z,
                 "p": twist.angular.x, "q": twist.angular.y, "r": twist.angular.z}
        self._tx.sendto(imc.encode("SimulatedState", self._dune_id, state), self._dune)

    def _send_commands(self):
        if self._last_command is None:
            return
        fresh = (self.get_clock().now() - self._last_command).nanoseconds * 1e-9 \
            < float(self.get_parameter("command_timeout_s").value)
        fins, rpm = (self._fins, self._rpm) if fresh else ([0.0] * 4, 0.0)
        limit = float(self.get_parameter("max_rpm").value)
        self._thruster_pub.publish(Float64MultiArray(data=[max(-limit, min(limit, rpm))]))
        # top = servo 3, bottom = servo 0 (rudder sign reversed), port = servo 1, starboard = servo 2
        self._fin_pub.publish(Float64MultiArray(data=[-fins[3], -fins[0], fins[1], fins[2], 0.0, 0.0]))

    def _publish_estimate(self, v):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "dune_reference_ned"
        msg.child_frame_id = "lauv_marie"
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = v["x"], v["y"], v["depth"]
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
         msg.pose.pose.orientation.w) = quaternion(v["phi"], v["theta"], v["psi"])
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = v["u"], v["v"], v["w"]
        msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = v["p"], v["q"], v["r"]
        self._estimate_pub.publish(msg)

    def _watch(self):
        if self._dune_id is None:
            self.get_logger().warn("Nothing from DUNE yet: is it running with lauv-marie-stonefish, and is UDP "
                                   f"{self.get_parameter('listen_port').value} its static destination?")
        if self._odom is None:
            self.get_logger().warn("No /lauv_marie/odom yet: is Stonefish running with the lauv_marie scenario?")


def main():
    rclpy.init()
    node = DuneBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():           # at Ctrl+C the context can be gone before spin returns: not an error then
            raise
    finally:
        try:                     # Ctrl+C reaches this node twice under ros2 launch: the second one lands here
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
