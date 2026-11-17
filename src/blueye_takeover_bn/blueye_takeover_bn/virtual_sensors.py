#!/usr/bin/env python3
"""
Virtual sensors for the real Blueye: evidence for the sensors it does not carry, so the takeover network sees the same
inputs as in the simulator. Healthy by default; the operator injects faults from the Injected Evidence tab of the
real operator HUD (blueye_real_hud).

  /blueye/usbl_strength       Strong | Moderate | Weak | Lost   (command: /blueye/virtual/usbl_command)
  /blueye/sonar_range_state   Clear | Near | Critical          (command: /blueye/virtual/sonar_command)
  /blueye/aruco_visibility    Real | All | Some | None         (command: /blueye/virtual/aruco_command)
  /blueye/current             water current [m/s], fixed (current_ms): not measured on the vehicle

ArUco visibility comes from the camera: Real relays the ArUco monitor (its output remapped to aruco_real_topic) to
the network unchanged; All, Some or None replace it, e.g. for undocking and docking tests where there is no dock.
The mode (Real or the injected state) is published on /blueye/virtual/aruco_mode for the HUD.

The network reads the sonar state with sonar_source:="state". Each state is published at publish_rate_hz (the
network's timeouts see a silent node as a lost sensor), and every change on /scenario/events. Commands from the shell:
  ros2 topic pub --once /blueye/virtual/sonar_command std_msgs/String "{data: Critical}"
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class VirtualSensors(Node):
    STATES = {"usbl": ("Strong", "Moderate", "Weak", "Lost"), "sonar": ("Clear", "Near", "Critical"),
              "aruco": ("Real", "All", "Some", "None")}
    RELAYED = {"aruco": "Real"}     # sensor: the state in which the real sensor is relayed instead of injected

    def __init__(self):
        super().__init__("virtual_sensors")

        self.declare_parameter("usbl_topic", "/blueye/usbl_strength")
        self.declare_parameter("sonar_topic", "/blueye/sonar_range_state")
        self.declare_parameter("current_topic", "/blueye/current")
        self.declare_parameter("usbl_command_topic", "/blueye/virtual/usbl_command")
        self.declare_parameter("sonar_command_topic", "/blueye/virtual/sonar_command")
        self.declare_parameter("aruco_topic", "/blueye/aruco_visibility")
        self.declare_parameter("aruco_real_topic", "/blueye/aruco_visibility_real")
        self.declare_parameter("aruco_command_topic", "/blueye/virtual/aruco_command")
        self.declare_parameter("aruco_mode_topic", "/blueye/virtual/aruco_mode")
        self.declare_parameter("events_topic", "/scenario/events")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("usbl_state", "Strong")
        self.declare_parameter("sonar_state", "Clear")
        self.declare_parameter("aruco_state", "Real")
        self.declare_parameter("current_ms", 0.0)

        self.state = {}
        for sensor in self.STATES:
            self.state[sensor] = self._canonical(sensor, str(self.get_parameter(f"{sensor}_state").value)) \
                or self.STATES[sensor][0]

        self.pubs = {sensor: self.create_publisher(String, self.get_parameter(f"{sensor}_topic").value, 10)
                     for sensor in self.STATES}
        self.current_pub = self.create_publisher(Float32, self.get_parameter("current_topic").value, 10)
        self.event_pub = self.create_publisher(String, self.get_parameter("events_topic").value, 10)
        for sensor in self.STATES:
            self.create_subscription(String, self.get_parameter(f"{sensor}_command_topic").value,
                                     lambda msg, sensor=sensor: self._set(sensor, msg.data), 10)
        self.aruco_mode_pub = self.create_publisher(String, self.get_parameter("aruco_mode_topic").value, 10)
        self.create_subscription(String, self.get_parameter("aruco_real_topic").value, self._aruco_real_cb, 10)

        rate = max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f"Virtual sensors: USBL {self.state['usbl']}, sonar {self.state['sonar']}, "
                               f"ArUco {self.state['aruco']}, "
                               f"current {float(self.get_parameter('current_ms').value):.2f} m/s")

    def _canonical(self, sensor, value):
        for state in self.STATES[sensor]:
            if state.lower() == value.strip().lower():
                return state
        return None

    def _set(self, sensor, value):
        state = self._canonical(sensor, value)
        if state is None:
            self.get_logger().warning(f"Unknown {sensor} state '{value}', expected one of {self.STATES[sensor]}")
            return
        if state == self.state[sensor]:
            return
        self.get_logger().info(f"{sensor.upper()} {self.state[sensor]} -> {state}")
        self.state[sensor] = state
        event = String()
        event.data = json.dumps({"event": sensor, "state": state, "source": "command"})
        self.event_pub.publish(event)
        self._tick()

    def _aruco_real_cb(self, msg: String):
        if self.state["aruco"] == self.RELAYED["aruco"]:
            self.pubs["aruco"].publish(msg)

    def _tick(self):
        mode = String()
        mode.data = self.state["aruco"]
        self.aruco_mode_pub.publish(mode)
        for sensor, pub in self.pubs.items():
            if self.state[sensor] == self.RELAYED.get(sensor):
                continue                                   # the real sensor's messages are relayed as they come
            msg = String()
            msg.data = self.state[sensor]
            pub.publish(msg)
        current = Float32()
        current.data = float(self.get_parameter("current_ms").value)
        self.current_pub.publish(current)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualSensors()
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
