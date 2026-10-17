#!/usr/bin/env python3
"""
BN state viewer — live view of every node of the takeover Bayesian network.

Each node is drawn in its GeNIe position with one box per discrete state. The bar in a box is the posterior
probability of that state. A solid black border marks the observed state of an evidence node; a dashed border marks
the most probable state of a node without evidence. The header shows the active mission phase, the urgency against
the takeover threshold and the recommendation.

Data come from /blueye/takeover_request/evidence, published by the takeover_bn node at every update. The network
structure (nodes, states, arcs, layout, colours) is read from the same .xdsl file the BN node uses.

Usage:
  source ~/Desktop/auto-pilot/install/setup.bash
  ros2 run bn_state_viewer bn_state_viewer
"""

import json
import os
import signal
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, Float64, String


# ── Network roles (as in the takeover_bn node) ────────────────────────────────

EVIDENCE_NODES = (
    "MissionPhase", "Current", "BatteryLevel", "USBLStrength", "DVLStatus", "Speed", "Altitude", "CameraQuality",
    "SonarRange", "RangeToTarget", "Fatigue", "Stress", "ArUcoVisibility", "WaypointTrackingError",
)
CONTEXT_NODE = "MissionPhase"
OUTPUT_NODE = "TakeoverRequest"

# ── Layout and style ──────────────────────────────────────────────────────────

SPREAD_X, SPREAD_Y = 1.6, 1.45        # GeNIe positions spread so that the state boxes fit without overlap
BOX_W, BOX_H, WIDE_W = 190, 64, 270   # WIDE_W: context and output nodes
MARGIN = 30
STALE_S = 2.0

BG = "#ffffff"
HEADER_BG = "#2c3e50"
HEADER_FG = "#ecf0f1"
GOOD, MID, BAD = (46, 158, 91), (224, 160, 48), (214, 69, 65)
PHASE_COLOUR = "#3b7dd8"
STATUS_COLOUR = {"observed": "#1e6b3a", "no evidence": "#999999", "inferred": "#2c3e50", "output": "#8e1b1b"}
SHORT_NAMES = {
    "NoTakeoverRequired": "No takeover", "IncreasedAttentionRequired": "Attention", "TakeoverRequested": "Takeover",
    "Inadequate": "Inadeq.", "Undocking": "Undock", "Inspection": "Inspect",
}


def severity_colour(index: int, count: int) -> str:
    """States are ordered from best to worst: green -> amber -> red."""
    f = index / (count - 1) if count > 1 else 0.0
    a, b, t = (GOOD, MID, 2 * f) if f <= 0.5 else (MID, BAD, 2 * f - 1)
    return "#%02x%02x%02x" % tuple(round(x + (y - x) * t) for x, y in zip(a, b))


def load_network(path: str):
    """Nodes (states, parents), temporal nodes and GeNIe layout (centre, colour) of an .xdsl file."""
    root = ET.parse(path).getroot()
    nodes = {}
    for element in root.find("nodes"):
        nodes[element.get("id")] = {
            "states": [s.get("id") for s in element.findall("state")],
            "parents": (element.findtext("parents") or "").split(),
        }
    temporal = {element.get("id") for element in root.findall("dynamic/cpt")}
    layout = {}
    for element in root.iter("node"):
        position = element.findtext("position")
        if element.get("id") in nodes and position:
            x1, y1, x2, y2 = (float(v) for v in position.split())
            interior = element.find("interior")
            colour = "#" + interior.get("color") if interior is not None else "#eeeeee"
            layout[element.get("id")] = ((x1 + x2) / 2 * SPREAD_X, (y1 + y2) / 2 * SPREAD_Y, colour)
    bottom = max((y for _, y, _ in layout.values()), default=0.0) + 2 * BOX_H
    for i, node_id in enumerate(n for n in nodes if n not in layout):      # no GeNIe position: row at the bottom
        layout[node_id] = (MARGIN + WIDE_W / 2 + i * (WIDE_W + 20), bottom, "#eeeeee")
    return nodes, temporal, layout


# ── ROS node ──────────────────────────────────────────────────────────────────

class BnStateViewer(Node):
    def __init__(self):
        super().__init__("bn_state_viewer")
        self.declare_parameter("bn_file", "")
        self.declare_parameter("evidence_topic", "/blueye/takeover_request/evidence")
        self._lock = threading.Lock()
        self._data, self._stamp, self._version = None, None, 0
        self._urgency, self._threshold = None, None
        self.create_subscription(String, self.get_parameter("evidence_topic").value, self._evidence_cb, 10)
        self.create_subscription(Float64, "/blueye/takeover_request/urgency", self._urgency_cb, 10)
        self.create_subscription(Float32, "/blueye/takeover_request/threshold", self._threshold_cb, 10)

    def network_path(self) -> str:
        configured = str(self.get_parameter("bn_file").value)
        if configured:
            return configured
        share = get_package_share_directory("blueye_takeover_bn")
        return os.path.join(share, "config", "UnifiedMissionDBN_v4.xdsl")

    def _evidence_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self._data, self._stamp, self._version = data, time.monotonic(), self._version + 1

    def _urgency_cb(self, msg: Float64):
        with self._lock:
            self._urgency = float(msg.data)

    def _threshold_cb(self, msg: Float32):
        with self._lock:
            self._threshold = float(msg.data)

    def snapshot(self):
        with self._lock:
            return self._data, self._stamp, self._version, self._urgency, self._threshold


# ── GUI ───────────────────────────────────────────────────────────────────────

class App(tk.Tk):
    def __init__(self, ros_node: BnStateViewer, network_path: str):
        super().__init__()
        self.ros = ros_node
        self.nodes, self.temporal, self.layout = load_network(network_path)
        self.title(f"BN state viewer — {os.path.basename(network_path)}")
        self.geometry("1500x1000")
        self.configure(bg=BG)
        self._seen_version = -1
        self._fonts = {}

        header = tk.Frame(self, bg=HEADER_BG)
        header.pack(fill="x")
        self.phase_lbl = self._header_label(header, 16)
        self.urgency_lbl = self._header_label(header, 13)
        self.urgency_bar = tk.Canvas(header, width=220, height=18, bg=HEADER_BG, highlightthickness=0)
        self.urgency_bar.pack(side="left", padx=(0, 18))
        self.state_lbl = self._header_label(header, 13)
        self.show_arcs = tk.BooleanVar(value=True)
        tk.Checkbutton(header, text="Arcs", variable=self.show_arcs, command=self._redraw, bg=HEADER_BG,
                       fg=HEADER_FG, selectcolor=HEADER_BG, activebackground=HEADER_BG,
                       font=("Helvetica", 11)).pack(side="right", padx=10)
        self.age_lbl = self._header_label(header, 11, side="right")

        self.canvas = tk.Canvas(self, bg=BG, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _event: self._redraw())
        tk.Label(self, bg=BG, fg="#555555", font=("Helvetica", 10),
                 text="bar = posterior probability   ·   solid border = observed evidence   ·   "
                      "dashed border = most probable state (inferred)   ·   grey = no evidence (prior)   ·   "
                      "↻ = temporal node").pack(fill="x", pady=4)
        self.after(200, self._tick)

    def _header_label(self, parent, size, side="left"):
        label = tk.Label(parent, bg=HEADER_BG, fg=HEADER_FG, font=("Helvetica", size, "bold"))
        label.pack(side=side, padx=12, pady=8)
        return label

    # periodic update: redraw when a new message arrived, header every tick (age of the data)
    def _tick(self):
        if not rclpy.ok():                                 # Ctrl+C in the terminal
            self.destroy()
            return
        data, stamp, version, urgency, threshold = self.ros.snapshot()
        if version != self._seen_version:
            self._seen_version = version
            self._redraw()
        self._update_header(data, stamp, urgency, threshold)
        self.after(200, self._tick)

    def _update_header(self, data, stamp, urgency, threshold):
        if data is None:
            self.phase_lbl.config(text="Waiting for /blueye/takeover_request/evidence …")
            self.age_lbl.config(text="")
            return
        phase = data.get("evidence", {}).get(CONTEXT_NODE, "?")
        self.phase_lbl.config(text=f"Phase: {phase}")
        probabilities = data.get("probabilities", {})
        if urgency is None:
            urgency = 0.5 * probabilities.get("IncreasedAttentionRequired", 0.0) + \
                probabilities.get("TakeoverRequested", 0.0)
        over = threshold is not None and urgency >= threshold
        self.urgency_lbl.config(text=f"Urgency {urgency:.2f}" + (f" / {threshold:.2f}" if threshold else ""),
                                fg="#ff7b6b" if over else HEADER_FG)
        bar = self.urgency_bar
        bar.delete("all")
        bar.create_rectangle(0, 3, 220, 15, fill="#45607a", width=0)
        bar.create_rectangle(0, 3, 220 * min(urgency, 1.0), 15, fill="#e74c3c" if over else "#f39c12", width=0)
        if threshold:
            bar.create_line(220 * threshold, 0, 220 * threshold, 18, fill="white", width=2)
        state = data.get("state", "?")
        self.state_lbl.config(text=f"Recommendation: {SHORT_NAMES.get(state, state)} "
                                   f"({100 * probabilities.get(state, 0.0):.0f}%)")
        age = time.monotonic() - stamp
        self.age_lbl.config(text=f"NO DATA for {age:.0f} s" if age > STALE_S else f"updated {age:.1f} s ago",
                            fg="#ff7b6b" if age > STALE_S else "#9fb3c8")

    # ── drawing ──

    def _width(self, node_id):
        return WIDE_W if node_id in (CONTEXT_NODE, OUTPUT_NODE) else BOX_W

    def _redraw(self):
        canvas = self.canvas
        canvas.delete("all")
        cw, ch = canvas.winfo_width(), canvas.winfo_height()
        if cw < 50 or ch < 50:
            return
        x_min = min(x - self._width(n) / 2 for n, (x, _, _) in self.layout.items()) - MARGIN
        x_max = max(x + self._width(n) / 2 for n, (x, _, _) in self.layout.items()) + MARGIN
        y_min = min(y for _, y, _ in self.layout.values()) - BOX_H / 2 - MARGIN
        y_max = max(y for _, y, _ in self.layout.values()) + BOX_H / 2 + MARGIN
        s = min(cw / (x_max - x_min), ch / (y_max - y_min))
        ox = (cw - (x_max - x_min) * s) / 2 - x_min * s
        oy = (ch - (y_max - y_min) * s) / 2 - y_min * s

        def to_px(x, y):
            return ox + x * s, oy + y * s

        data = self.ros.snapshot()[0]
        if self.show_arcs.get():
            for child, info in self.nodes.items():
                for parent in info["parents"]:
                    x0, y0 = to_px(*self._border_point(parent, self.layout[child][:2]))
                    x1, y1 = to_px(*self._border_point(child, self.layout[parent][:2]))
                    context = parent == CONTEXT_NODE
                    canvas.create_line(x0, y0, x1, y1, fill="#d5d5d5" if context else "#9a9a9a",
                                       dash=(3, 3) if context else (), width=1, arrow="last",
                                       arrowshape=(8 * s, 10 * s, 3 * s))
        for node_id in self.nodes:
            self._draw_node(node_id, data, s, to_px)

    def _border_point(self, node_id, toward):
        """Point where the line from the centre of node_id towards `toward` leaves its box."""
        cx, cy, _ = self.layout[node_id]
        dx, dy = toward[0] - cx, toward[1] - cy
        if dx == 0 and dy == 0:
            return cx, cy
        t = min(self._width(node_id) / 2 / abs(dx) if dx else float("inf"), BOX_H / 2 / abs(dy) if dy else float("inf"))
        return cx + dx * t, cy + dy * t

    def _draw_node(self, node_id, data, s, to_px):
        canvas = self.canvas
        states = self.nodes[node_id]["states"]
        cx, cy, fill = self.layout[node_id]
        w = self._width(node_id)
        evidence = data.get("evidence", {}) if data else {}
        posterior = None
        if data:
            posterior = data.get("posteriors", {}).get(node_id)
            if posterior is None and node_id == OUTPUT_NODE:
                posterior = data.get("probabilities")
        observed = evidence.get(node_id)
        if observed:
            status = "observed"
        elif node_id == OUTPUT_NODE:
            status = "output"
        elif node_id in EVIDENCE_NODES:
            status = "no evidence"
        else:
            status = "inferred"
        active = observed or (max(posterior, key=posterior.get) if posterior else None)

        x0, y0 = to_px(cx - w / 2, cy - BOX_H / 2)
        x1, y1 = to_px(cx + w / 2, cy + BOX_H / 2)
        canvas.create_rectangle(x0, y0, x1, y1, fill=fill, outline="#222222" if observed else "#888888",
                                width=max(1.0, (2.5 if observed else 1.0) * s))
        size = max(6, round(9 * s))
        name = node_id + (" ↻" if node_id in self.temporal else "")
        title_font, status_font = self._font(size, "bold"), self._font(max(6, size - 2), "italic")
        canvas.create_text(x0 + 5 * s, y0 + 3 * s, anchor="nw", text=name, font=title_font,
                           fill="#888888" if status == "no evidence" else "#111111")
        if title_font.measure(name) + status_font.measure(status) + 16 * s <= x1 - x0:   # the border shows it too
            canvas.create_text(x1 - 5 * s, y0 + 4 * s, anchor="ne", text=status, font=status_font,
                               fill=STATUS_COLOUR[status])

        pad, gap, top = 6, 3, cy - BOX_H / 2 + 21
        cell_w = (w - 2 * pad - (len(states) - 1) * gap) / len(states)
        label_font, value_font = self._font(max(6, round(8 * s))), self._font(max(6, round(8 * s)), "bold")
        for i, state in enumerate(states):
            vx = cx - w / 2 + pad + i * (cell_w + gap)
            sx0, sy0 = to_px(vx, top)
            sx1, sy1 = to_px(vx + cell_w, cy + BOX_H / 2 - 5)
            canvas.create_rectangle(sx0, sy0, sx1, sy1, fill="white", outline="#bbbbbb")
            p = posterior.get(state) if posterior else (1.0 if state == observed else None)
            if p:
                colour = PHASE_COLOUR if node_id == CONTEXT_NODE else severity_colour(i, len(states))
                canvas.create_rectangle(sx0, sy1 - (sy1 - sy0) * p, sx1, sy1, fill=colour, width=0)
            if state == active:
                canvas.create_rectangle(sx0, sy0, sx1, sy1, outline="#000000", width=max(2.0, 2.5 * s),
                                        dash=() if observed else (4, 2))
            canvas.create_text((sx0 + sx1) / 2, sy0 + 2 * s, anchor="n", font=label_font, fill="#111111",
                               text=self._fit(state, label_font, sx1 - sx0 - 2))
            if p is not None:
                canvas.create_text((sx0 + sx1) / 2, sy1 - 2 * s, anchor="s", text=f"{100 * p:.0f}%",
                                   font=value_font, fill="#111111")

    def _font(self, size, style=""):
        key = (size, style)
        if key not in self._fonts:
            self._fonts[key] = tkfont.Font(family="Helvetica", size=size,
                                           weight="bold" if style == "bold" else "normal",
                                           slant="italic" if style == "italic" else "roman")
        return self._fonts[key]

    @staticmethod
    def _fit(text, font, width):
        """The state name, its short form, or the short form cut to the cell width."""
        short = SHORT_NAMES.get(text, text)
        for candidate in (text, short):
            if font.measure(candidate) <= width:
                return candidate
        while len(short) > 1 and font.measure(short + "…") > width:
            short = short[:-1]
        return short + "…"


# ── Entry point ───────────────────────────────────────────────────────────────

def _spin(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:                      # window closed: main thread shut rclpy down
        pass


def main(args=None):
    rclpy.init(args=args)
    node = BnStateViewer()

    ros_thread = threading.Thread(target=_spin, args=(node,), daemon=True)
    ros_thread.start()

    app = App(node, node.network_path())
    # Ctrl+C: close the window. The default handler would raise inside a Tk callback, which Tk swallows.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    try:
        app.mainloop()
    finally:
        if rclpy.ok():
            rclpy.shutdown()                               # wakes the spin thread; destroy the node after it ends
        ros_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
