#!/usr/bin/env python3
"""
Replays the water-current and USBL disturbances of a recorded run, so another run gets exactly the same scenario
(for example an open-loop run matched to a closed-loop one).

The disturbances are read from the recording: /scenario/events, or the sim_current and usbl_strength_monitor logs in
/rosout when that topic was not recorded. Their times are taken relative to the moment the recorded mission entered
`anchor_phase` (default 3, Inspection), and replayed relative to the moment this mission enters the same phase:
  current changes -> /blueye/sim/current_command (the same NED vector as recorded)
  USBL cut/restore -> /blueye/usbl/set_beacon_enabled
Leave the Simulation tab of the operator HUD alone while it runs.

  ros2 run blueye_takeover_bn disturbance_replay --ros-args -p bag:=<recording> [-p dry_run:=true]
"""

import json
import math
import re

import rclpy
import rosbag2_py
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Int32
from std_srvs.srv import SetBool

PHASES = {1: "Undocking", 2: "Transit", 3: "Inspection", 4: "Homing", 5: "Docking", 6: "Takeover"}
CURRENT_LOG = re.compile(r"Current set to ([\d.]+) m/s towards ([\d.]+) deg \(NED ([+-][\d.]+), ([+-][\d.]+), ([+-][\d.]+)\)")


def read_schedule(bag, anchor_phase):
    """[(seconds after the anchor, 'current', (north, east, down)) | (..., 'usbl', enabled)], and the anchor time."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    topics = [t for t in ("/mission_state", "/scenario/events", "/rosout") if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    classes = {t: get_message(types[t]) for t in topics}
    anchor, from_events, from_logs = None, [], []
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        t = stamp / 1e9
        msg = deserialize_message(raw, classes[topic])
        if topic == "/mission_state":
            if anchor is None and msg.data == anchor_phase:
                anchor = t
        elif topic == "/scenario/events":
            e = json.loads(msg.data)
            if e.get("event") == "current":
                from_events.append((t, "current", (e["north_ms"], e["east_ms"], 0.0)))
            elif e.get("event") == "usbl_beacon":
                from_events.append((t, "usbl", bool(e["enabled"])))
        elif msg.name == "sim_current":
            hit = CURRENT_LOG.match(msg.msg)
            if hit:
                from_logs.append((t, "current", (float(hit[3]), float(hit[4]), float(hit[5]))))
        elif msg.name == "usbl_strength_monitor" and msg.msg.startswith("USBL beacon"):
            from_logs.append((t, "usbl", msg.msg.split(":")[0].endswith("ON")))
    if anchor is None:
        raise SystemExit(f"The recording never entered phase {anchor_phase} ({PHASES.get(anchor_phase, '?')})")
    events = from_events or from_logs
    return [(t - anchor, kind, value) for t, kind, value in events if t >= anchor], anchor


def describe(kind, value):
    if kind == "usbl":
        return "USBL beacon " + ("ON" if value else "OFF")
    north, east, _ = value
    speed = math.hypot(north, east)
    if speed < 1e-6:
        return "calm water"
    return f"current {speed:.2f} m/s towards {math.degrees(math.atan2(east, north)) % 360:.0f} deg"


class DisturbanceReplay(Node):
    def __init__(self):
        super().__init__("disturbance_replay")
        self.declare_parameter("bag", "")
        self.declare_parameter("anchor_phase", 3)
        self.declare_parameter("dry_run", False)
        bag = str(self.get_parameter("bag").value)
        if not bag:
            raise SystemExit("Give the recording to replay: --ros-args -p bag:=<path>")
        self.anchor_phase = int(self.get_parameter("anchor_phase").value)
        self.schedule, _ = read_schedule(bag, self.anchor_phase)
        phase = PHASES.get(self.anchor_phase, str(self.anchor_phase))
        self.get_logger().info(f"{len(self.schedule)} disturbances from {bag}, timed from the start of {phase}:")
        for dt, kind, value in self.schedule:
            self.get_logger().info(f"  +{dt:6.1f} s  {describe(kind, value)}")
        self.done = bool(self.get_parameter("dry_run").value) or not self.schedule
        if self.done:
            return

        self.current_pub = self.create_publisher(Vector3, "/blueye/sim/current_command", 10)
        self.usbl_client = self.create_client(SetBool, "/blueye/usbl/set_beacon_enabled")
        self.create_subscription(Int32, "/mission_state", self._on_phase, 10)
        self.start = None
        self.next = 0
        self.create_timer(0.05, self._tick)
        self.get_logger().info(f"Waiting for the mission to enter {phase}; do not use the HUD Simulation tab meanwhile")

    def _on_phase(self, msg):
        if self.start is None and msg.data == self.anchor_phase:
            self.start = self.get_clock().now()
            self.get_logger().info(f"{PHASES.get(self.anchor_phase)} started: replay clock running")

    def _tick(self):
        if self.start is None or self.done:
            return
        elapsed = (self.get_clock().now() - self.start).nanoseconds / 1e9
        while self.next < len(self.schedule) and self.schedule[self.next][0] <= elapsed:
            dt, kind, value = self.schedule[self.next]
            if kind == "current":
                msg = Vector3()
                msg.x, msg.y, msg.z = value
                self.current_pub.publish(msg)
            elif self.usbl_client.service_is_ready():
                self.usbl_client.call_async(SetBool.Request(data=value))
            else:
                self.get_logger().error("USBL service /blueye/usbl/set_beacon_enabled not available: event lost")
            self.get_logger().info(f"+{elapsed:6.1f} s (recorded +{dt:.1f} s): {describe(kind, value)}")
            self.next += 1
        if self.next == len(self.schedule):
            self.get_logger().info("All disturbances replayed")
            self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = DisturbanceReplay()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
