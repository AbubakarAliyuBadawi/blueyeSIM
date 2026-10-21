#!/usr/bin/env python3
"""
Simulated water current: the single owner of the Stonefish ocean current, and its publisher for the BN.

The scenario's uniform current listens on /stonefish_ros2/current_velocity (a <ros_subscriber velocity=.../> tag in
the scenario file), so its speed and direction can change during a run. This node
  - switches the Stonefish currents on once (they are off until /stonefish_ros2/enable_currents is called),
  - holds the current setpoint and sends it to the simulator every tick (so a restarted simulator gets it again),
  - takes new setpoints on /blueye/sim/current_command (geometry_msgs/Vector3, NED in m/s: x north, y east, z down),
    e.g. from the Simulation tab of the operator HUD,
  - publishes the horizontal speed on /blueye/current for the BN, the vector on /blueye/sim/current_state, and every
    change on /scenario/events.

Without a command, the setpoint is calm water, or the scenario file's current when `enabled` is true (from
`enable_after_s` seconds after the start).
  ros2 param set /sim_current enabled true    # scenario current on (until a command arrives)
"""

import json
import math
import os
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3, Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger


class SimCurrent(Node):
    def __init__(self):
        super().__init__("sim_current")

        self.declare_parameter("scenario_file", "")
        self.declare_parameter("current_topic", "/blueye/current")
        self.declare_parameter("command_topic", "/blueye/sim/current_command")
        self.declare_parameter("state_topic", "/blueye/sim/current_state")
        self.declare_parameter("simulator_topic", "/stonefish_ros2/current_velocity")
        self.declare_parameter("events_topic", "/scenario/events")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("enabled", True)
        self.declare_parameter("enable_after_s", 0.0)

        self.scenario = self._scenario_current()
        self.setpoint = (0.0, 0.0, 0.0)
        self.commanded = False                         # a command overrides the scenario default from then on
        self.currents_on = False
        self.pending = None
        self.start = self.get_clock().now()
        self.ticks = 0

        self.enable_client = self.create_client(Trigger, "/stonefish_ros2/enable_currents")
        self.pub = self.create_publisher(Float32, self.get_parameter("current_topic").value, 10)
        self.state_pub = self.create_publisher(Vector3Stamped, self.get_parameter("state_topic").value, 10)
        self.sim_pub = self.create_publisher(Vector3, self.get_parameter("simulator_topic").value, 10)
        self.event_pub = self.create_publisher(String, self.get_parameter("events_topic").value, 10)
        self.create_subscription(Vector3, self.get_parameter("command_topic").value, self._command_cb, 10)

        rate = max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.create_timer(1.0 / rate, self._tick)

    def _scenario_current(self):
        path = str(self.get_parameter("scenario_file").value) or os.path.join(
            get_package_share_directory("stonefish_sim"), "scenarios", "blueye.scn")
        vx = vy = vz = 0.0
        for current in ET.parse(path).getroot().findall("environment/ocean/current"):
            if current.get("type") != "uniform":
                self.get_logger().warning(f"Ignoring '{current.get('type')}' current in {path}: only uniform is read")
                continue
            x, y, z = (float(v) for v in current.find("velocity").get("xyz").split())
            vx, vy, vz = vx + x, vy + y, vz + z
        self.get_logger().info(f"Scenario current ({vx:.2f}, {vy:.2f}, {vz:.2f}) m/s from {path}")
        return (vx, vy, vz)

    def _command_cb(self, msg: Vector3):
        self.commanded = True
        self._set((msg.x, msg.y, msg.z), "command")

    def _set(self, vector, source):
        vector = tuple(float(v) for v in vector)
        if vector == self.setpoint:
            return
        self.setpoint = vector
        speed = math.hypot(vector[0], vector[1])
        direction = math.degrees(math.atan2(vector[1], vector[0])) % 360.0
        self.get_logger().info(f"Current set to {speed:.3f} m/s towards {direction:.0f} deg "
                               f"(NED {vector[0]:+.3f}, {vector[1]:+.3f}, {vector[2]:+.3f}) [{source}]")
        event = String()
        event.data = json.dumps({"event": "current", "speed_ms": round(speed, 3), "direction_deg": round(direction, 1),
                                 "north_ms": round(vector[0], 3), "east_ms": round(vector[1], 3), "source": source})
        self.event_pub.publish(event)
        self._send()

    def _send(self):
        msg = Vector3()
        msg.x, msg.y, msg.z = self.setpoint
        self.sim_pub.publish(msg)

    def _tick(self):
        self.ticks += 1
        if not self.commanded:
            elapsed = (self.get_clock().now() - self.start).nanoseconds / 1e9
            on = bool(self.get_parameter("enabled").value) and \
                elapsed >= float(self.get_parameter("enable_after_s").value)
            self._set(self.scenario if on else (0.0, 0.0, 0.0), "scenario default")
        # currents on (again every 10 s: the simulator may have been restarted), then the setpoint
        if self.pending is None and (not self.currents_on or self.ticks % 20 == 0) and \
                self.enable_client.service_is_ready():
            self.pending = self.enable_client.call_async(Trigger.Request())
            self.pending.add_done_callback(self._enabled)
        self._send()

        speed = Float32()
        speed.data = math.hypot(self.setpoint[0], self.setpoint[1]) if self.currents_on else 0.0
        self.pub.publish(speed)
        state = Vector3Stamped()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = "world_ned"
        if self.currents_on:
            state.vector.x, state.vector.y, state.vector.z = self.setpoint
        self.state_pub.publish(state)

    def _enabled(self, future):
        self.pending = None
        result = future.result()
        if result is not None and result.success:
            if not self.currents_on:
                self.get_logger().info("Stonefish ocean currents on; current set from /blueye/sim/current_command")
            self.currents_on = True
        else:
            self.get_logger().warning("Switching the Stonefish ocean currents on failed; retrying")


def main(args=None):
    rclpy.init(args=args)
    node = SimCurrent()
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
