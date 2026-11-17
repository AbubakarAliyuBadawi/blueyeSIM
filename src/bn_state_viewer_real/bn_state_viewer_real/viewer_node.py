#!/usr/bin/env python3
"""
BN state viewer for the real Blueye — the network view of bn_state_viewer, with a panel that shows where each
evidence node gets its input on the vehicle: the topic, its last value, how old it is, and the state the network
entered.

On the vehicle the evidence comes from the handler, the DVL driver, the camera monitors, the behaviour tree, the
operator model and virtual_sensors (sonar and USBL are not fitted; their state is set on the operator HUD). A streamed
input older than STALE_S is drawn red: the network then reads the sensor as lost (DVL: Failed) or leaves the evidence
out.

Usage:
  source ~/Desktop/auto-pilot/install/setup.bash
  ros2 run bn_state_viewer_real bn_state_viewer_real
"""

import signal
import threading
import time
import tkinter as tk

import rclpy
from blueye_interfaces.msg import ArucoDetectionStatus
from bn_state_viewer.viewer_node import App, BnStateViewer, CONTEXT_NODE, PHASE_COLOUR, STALE_S, _spin, \
    severity_colour
from geometry_msgs.msg import PointStamped, Pose, PoseWithCovarianceStamped
from marine_acoustic_msgs.msg import Dvl
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, Int32, String


def _soc_percent(value: float) -> float:
    """The handler packs the state of charge as a fraction (or, on some firmware, in percent)."""
    return value * 100.0 if value <= 1.5 else value


BATTERY_STALE_S = 30.0                   # the drone sends battery telemetry about every 10 s

# topic: (message type, stale after [s], value shown). A streamed topic arrives at a steady rate, so old data is a
# fault; None: published on change.
TOPICS = {
    "/mission_state": (Int32, None, lambda m: f"state {m.data}"),
    "/blueye/battery": (Pose, BATTERY_STALE_S,
                        lambda m: f"SoC {_soc_percent(m.position.y):.0f} %, {m.position.z:+.2f} A"),
    "/blueye/sensor/dvl": (Dvl, STALE_S, lambda m: f"{m.num_good_beams} good beams, altitude {m.altitude:.2f} m"),
    "/blueye/altitude": (Float32, STALE_S, lambda m: f"{m.data:.2f} m"),
    "/blueye/speed": (Float32, STALE_S, lambda m: f"{m.data:.2f} m/s"),
    "/blueye/camera_quality": (String, STALE_S, lambda m: m.data),
    "/blueye/aruco_visibility": (String, STALE_S, lambda m: m.data),
    "/blueye/virtual/aruco_mode": (String, STALE_S, lambda m: "camera" if m.data == "Real" else f"injected {m.data}"),
    "/blueye/aruco/status": (ArucoDetectionStatus, None,
                             lambda m: f"{m.num_tags} markers at {m.estimated_range:.1f} m"),
    "/blueye/sonar_range_state": (String, STALE_S, lambda m: m.data),
    "/blueye/usbl_strength": (String, STALE_S, lambda m: m.data),
    "/blueye/current": (Float32, STALE_S, lambda m: f"{m.data:.2f} m/s"),
    "/blueye/waypoint_goal": (PointStamped, None,
                              lambda m: f"N {m.point.x:.1f}  E {m.point.y:.1f}  depth {m.point.z:.1f}"),
    "/blueye/sensor/ekf/pose": (Odometry, STALE_S,
                                lambda m: f"N {m.pose.pose.position.x:.1f}  E {m.pose.pose.position.y:.1f}  "
                                          f"depth {m.pose.pose.position.z:.1f}"),
    "/blueye/current_waypoint": (Pose, None,
                                 lambda m: f"x {m.position.x:.2f}  y {m.position.y:.2f}  z {m.position.z:.2f}"),
    "/blueye/pose_estimated_board_stamped": (PoseWithCovarianceStamped, None,
                                             lambda m: f"x {m.pose.pose.position.x:.2f}  "
                                                       f"y {m.pose.pose.position.y:.2f}  z {m.pose.pose.position.z:.2f}"),
    "/blueye/human/fatigue": (Float32, STALE_S, lambda m: f"{m.data:.2f}"),
    "/blueye/human/stress": (Float32, STALE_S, lambda m: f"{m.data:.2f}"),
}

# evidence node: (source on the vehicle, input topics, how the network uses them where it differs from the simulator)
ROWS = (
    ("MissionPhase", "behaviour tree, mission.py", ["/mission_state"], "6 (takeover) keeps the phase before it"),
    ("BatteryLevel", "handler", ["/blueye/battery"], ""),
    ("DVLStatus", "Water Linked DVL", ["/blueye/sensor/dvl"],
     "sent only with a valid velocity: silent > 2 s = Failed; < 4 beams = Degraded"),
    ("Altitude", "DVL, handler", ["/blueye/sensor/dvl", "/blueye/altitude"], "left out without bottom lock"),
    ("Speed", "handler", ["/blueye/speed"], ""),
    ("CameraQuality", "camera monitor (RTSP)", ["/blueye/camera_quality"], ""),
    ("ArUcoVisibility", "ArUco detection, or injected on the HUD",
     ["/blueye/aruco_visibility", "/blueye/virtual/aruco_mode", "/blueye/aruco/status"],
     "entered in Undocking and Docking only, once the markers were detected in that phase"),
    ("SonarRange", "virtual, set on the HUD", ["/blueye/sonar_range_state"], "no sonar fitted"),
    ("USBLStrength", "virtual, set on the HUD", ["/blueye/usbl_strength"], "no USBL fitted"),
    ("Current", "virtual", ["/blueye/current"], "not measured: 0"),
    ("RangeToTarget", "goal and position",
     ["/blueye/waypoint_goal", "/blueye/sensor/ekf/pose", "/blueye/current_waypoint",
      "/blueye/pose_estimated_board_stamped"],
     "undocking to homing: goal of mission.py and handler position; docking: controller waypoint and board pose. "
     "Left out in a takeover"),
    ("WaypointTrackingError", "goal and position", [],
     "the inputs of RangeToTarget: distance from the straight leg (docking: from the controller reference)"),
    ("Fatigue", "operator model", ["/blueye/human/fatigue"], ""),
    ("Stress", "operator model", ["/blueye/human/stress"], ""),
)

PANEL_BG = "#f4f6f8"
INK, MUTED, FAINT = "#111111", "#52514e", "#8a8a8a"
STALE_COLOUR, FRESH_COLOUR = "#c0392b", "#1e6b3a"
VIRTUAL_COLOUR = "#6c3fb0"
PANEL_W = 620


# ── ROS node ──────────────────────────────────────────────────────────────────

class RealBnStateViewer(BnStateViewer):
    def __init__(self):
        super().__init__("bn_state_viewer_real")
        self._inputs = {}                                  # topic: (value shown, time received)
        for topic, (msg_type, _, show) in TOPICS.items():
            self.create_subscription(msg_type, topic, lambda msg, t=topic, f=show: self._input_cb(t, f, msg),
                                     qos_profile_sensor_data)

    def _input_cb(self, topic, show, msg):
        try:
            text = show(msg)
        except (AttributeError, TypeError, ValueError):
            text = "?"
        with self._lock:
            self._inputs[topic] = (text, time.monotonic())

    def inputs(self):
        with self._lock:
            return dict(self._inputs)


# ── GUI ───────────────────────────────────────────────────────────────────────

class RealApp(App):
    def __init__(self, ros_node: RealBnStateViewer, network_path: str):
        super().__init__(ros_node, network_path)
        self.title(self.title().replace("BN state viewer", "BN state viewer (real Blueye)"))
        self.geometry("2100x1050")

        panel = tk.Frame(self, bg=PANEL_BG, width=PANEL_W, highlightthickness=1, highlightbackground="#d5d8dc")
        panel.pack(side="right", fill="y", before=self.canvas)
        panel.pack_propagate(False)
        tk.Label(panel, text="Evidence inputs on the vehicle", bg=PANEL_BG, fg=INK, anchor="w",
                 font=("Helvetica", 13, "bold")).pack(fill="x", padx=10, pady=(10, 0))
        tk.Label(panel, text=f"value · age (red: streamed input older than {STALE_S:.0f} s, battery "
                              f"{BATTERY_STALE_S:.0f} s) · state entered",
                 bg=PANEL_BG, fg=MUTED, anchor="w", font=("Helvetica", 9)).pack(fill="x", padx=10, pady=(0, 6))

        self.rows = {}
        for node_id, source, topics, note in ROWS:
            if node_id not in self.nodes:
                continue
            row = tk.Frame(panel, bg="white", highlightthickness=1, highlightbackground="#e1e4e8")
            row.pack(fill="x", padx=8, pady=2)
            row.columnconfigure(0, minsize=215)                 # the longest topic name: rows line up
            row.columnconfigure(1, weight=1)
            tk.Label(row, text=node_id, bg="white", fg=INK, anchor="w",
                     font=("Helvetica", 10, "bold")).grid(row=0, column=0, sticky="w", padx=(6, 4))
            tk.Label(row, text=source, bg="white", anchor="w", font=("Helvetica", 9),
                     fg=VIRTUAL_COLOUR if source.startswith("virtual") else MUTED).grid(row=0, column=1, sticky="w")
            state = tk.Label(row, text="—", bg="#eeeeee", fg=INK, width=16, font=("Helvetica", 9, "bold"))
            state.grid(row=0, column=2, sticky="e", padx=4, pady=2)
            lines = []
            for k, topic in enumerate(topics):
                tk.Label(row, text=topic, bg="white", fg=FAINT, anchor="w",
                         font=("Helvetica", 8)).grid(row=1 + k, column=0, sticky="w", padx=(12, 4))
                value = tk.Label(row, text="", bg="white", fg=INK, anchor="w", font=("Helvetica", 9))
                value.grid(row=1 + k, column=1, sticky="w")
                age = tk.Label(row, text="", bg="white", anchor="e", font=("Helvetica", 9))
                age.grid(row=1 + k, column=2, sticky="e", padx=4)
                lines.append((topic, value, age))
            if note:
                tk.Label(row, text=note, bg="white", fg=FAINT, anchor="w", justify="left", wraplength=PANEL_W - 40,
                         font=("Helvetica", 8, "italic")).grid(row=1 + len(topics), column=0, columnspan=3,
                                                               sticky="w", padx=(12, 4), pady=(0, 2))
            self.rows[node_id] = (state, lines)

    def _tick(self):
        if rclpy.ok():
            self._update_inputs()
        super()._tick()

    def _update_inputs(self):
        inputs = self.ros.inputs()
        data = self.ros.snapshot()[0]
        evidence = data.get("evidence", {}) if data else {}
        now = time.monotonic()
        for node_id, (state_label, lines) in self.rows.items():
            entered = evidence.get(node_id)
            if entered:
                states = self.nodes[node_id]["states"]
                colour = PHASE_COLOUR if node_id == CONTEXT_NODE else \
                    severity_colour(states.index(entered), len(states)) if entered in states else "#cccccc"
                state_label.config(text=entered, bg=colour, fg="white")
            else:
                state_label.config(text="not entered", bg="#eeeeee", fg=FAINT)
            for topic, value_label, age_label in lines:
                stale_after = TOPICS[topic][1]
                streamed = stale_after is not None
                if topic not in inputs:
                    value_label.config(text="no message yet", fg=FAINT)
                    age_label.config(text="never", fg=STALE_COLOUR if streamed else FAINT)
                    continue
                text, stamp = inputs[topic]
                age = now - stamp
                stale = streamed and age > stale_after
                value_label.config(text=text, fg=FAINT if stale else INK)
                age_label.config(text=f"{age:.1f} s" if age < 60 else f"{age / 60:.0f} min",
                                 fg=STALE_COLOUR if stale else FRESH_COLOUR if streamed else MUTED)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = RealBnStateViewer()

    ros_thread = threading.Thread(target=_spin, args=(node,), daemon=True)
    ros_thread.start()

    app = RealApp(node, node.network_path())
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    try:
        app.mainloop()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
