#!/usr/bin/env python3
"""
BN Evidence Injector GUI — two panels:

  Panel 1 – Safe topics (no competing publisher):
    /blueye/speed          Float32  0.0–1.5 m/s
    /blueye/current        Float32  0.0–1.5 m/s
    /blueye/usbl_strength  String   Strong/Moderate/Weak/Lost

  Panel 2 – BN direct override via /blueye/bn/evidence_override (String JSON).
    Sends {"node": "<id>", "state": "<state>"} directly into the BN node.
    Does NOT publish on any existing topic → zero competition.
    Nodes: Fatigue, Stress, CameraQuality, DVLStatus, ArUcoVisibility,
           BatteryLevel, MissionPhase

Usage:
  source ~/Desktop/auto-pilot/install/setup.bash
  ros2 run bn_evidence_gui bn_evidence_gui
"""

import json
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


# ── State colour map ──────────────────────────────────────────────────────────

RISK_COLOUR = {
    # low risk
    "Safe": "#27ae60", "Low": "#27ae60", "Strong": "#27ae60",
    "Working": "#27ae60", "Excellent": "#27ae60", "High_bat": "#27ae60",
    "Full": "#27ae60", "Adequate": "#27ae60", "None_vis": "#27ae60",
    # medium risk
    "Moderate": "#f39c12", "Medium": "#f39c12", "Weak": "#f39c12",
    "Degraded": "#f39c12", "Good": "#f39c12", "Low_bat": "#f39c12",
    "Low_vis": "#f39c12", "Poor": "#f39c12",
    # high risk
    "High": "#e74c3c", "Lost": "#e74c3c", "Failed": "#e74c3c",
    "Critical": "#e74c3c", "High_vis": "#e74c3c",
}

def _col(state: str) -> str:
    return RISK_COLOUR.get(state, "#ecf0f1")


def _speed_state(v):
    return "Safe" if v < 0.3 else ("Moderate" if v < 0.7 else "High")

def _current_state(v):
    return "Low" if v < 0.3 else ("Medium" if v < 0.7 else "High")


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class EvidencePublisher(Node):
    def __init__(self):
        super().__init__("bn_evidence_gui")
        self._speed_pub    = self.create_publisher(Float32, "/blueye/speed",                10)
        self._curr_pub     = self.create_publisher(Float32, "/blueye/current",              10)
        self._usbl_pub     = self.create_publisher(String,  "/blueye/usbl_strength",        10)
        self._override_pub = self.create_publisher(String,  "/blueye/bn/evidence_override", 10)

        self.speed_val   = 0.1
        self.current_val = 0.1
        self.usbl_val    = "Strong"

        self.create_timer(0.2, self._publish)   # 5 Hz for safe topics

    def _publish(self):
        f = Float32()
        f.data = float(self.speed_val);   self._speed_pub.publish(f)
        f.data = float(self.current_val); self._curr_pub.publish(f)
        s = String(); s.data = self.usbl_val; self._usbl_pub.publish(s)

    def send_override(self, node_id: str, state: str):
        msg = String()
        msg.data = json.dumps({"node": node_id, "state": state})
        self._override_pub.publish(msg)

    def clear_override(self, node_id: str):
        msg = String()
        msg.data = json.dumps({"node": node_id, "clear": True})
        self._override_pub.publish(msg)


# ── GUI ───────────────────────────────────────────────────────────────────────

BG   = "#2c3e50"
BG2  = "#1a252f"
FG   = "#ecf0f1"
PAD  = dict(padx=12, pady=5)


class App(tk.Tk):
    def __init__(self, node: EvidencePublisher):
        super().__init__()
        self._node = node
        self.title("BN Evidence Injector")
        self.resizable(False, False)
        self.configure(bg=BG)

        # ── Notebook (two tabs) ──
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=8, pady=8)

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TNotebook",        background=BG, borderwidth=0)
        style.configure("TNotebook.Tab",    background=BG2, foreground=FG,
                         padding=[10, 4])
        style.map("TNotebook.Tab",          background=[("selected", "#34495e")])
        style.configure("TSeparator",       background="#4a6278")

        tab1 = tk.Frame(nb, bg=BG)
        tab2 = tk.Frame(nb, bg=BG)
        nb.add(tab1, text="  Safe Topics  ")
        nb.add(tab2, text="  BN Direct Override  ")

        self._build_safe_tab(tab1)
        self._build_override_tab(tab2)

        # Status bar
        self._status = tk.Label(self, text="● Publishing at 5 Hz",
                                 bg=BG, fg="#27ae60", font=("Helvetica", 9))
        self._status.pack(pady=(0, 8))
        self._tick()

    # ── Tab 1: safe topics ────────────────────────────────────────────────────

    def _build_safe_tab(self, parent):
        tk.Label(parent, text="Publishes directly on topics with no competing publisher",
                 bg=BG, fg="#aaaaaa", font=("Helvetica", 8)).grid(
                     row=0, column=0, columnspan=3, pady=(10, 4))

        self._sep(parent, 1)

        # Speed
        self._section_label(parent, "Speed  →  /blueye/speed", 2)
        self._speed_var   = tk.DoubleVar(value=0.1)
        self._speed_lbl   = self._val_label(parent, "0.10 m/s", 3, 1)
        self._speed_state = self._state_label(parent, "Safe", "#27ae60", 3, 2)
        tk.Scale(parent, from_=0.0, to=1.5, resolution=0.01,
                 orient=tk.HORIZONTAL, length=320, variable=self._speed_var,
                 bg=BG, fg=FG, highlightthickness=0, troughcolor=BG2,
                 command=self._on_speed).grid(row=3, column=0, **PAD)

        self._sep(parent, 4)

        # Current
        self._section_label(parent, "Current  →  /blueye/current", 5)
        self._curr_var    = tk.DoubleVar(value=0.1)
        self._curr_lbl    = self._val_label(parent, "0.10 m/s", 6, 1)
        self._curr_state  = self._state_label(parent, "Low", "#27ae60", 6, 2)
        tk.Scale(parent, from_=0.0, to=1.5, resolution=0.01,
                 orient=tk.HORIZONTAL, length=320, variable=self._curr_var,
                 bg=BG, fg=FG, highlightthickness=0, troughcolor=BG2,
                 command=self._on_current).grid(row=6, column=0, **PAD)

        self._sep(parent, 7)

        # USBL
        self._section_label(parent, "USBL Strength  →  /blueye/usbl_strength", 8)
        self._usbl_var = tk.StringVar(value="Strong")
        frame = tk.Frame(parent, bg=BG)
        frame.grid(row=9, column=0, columnspan=3, **PAD)
        for i, (s, c) in enumerate([("Strong","#27ae60"),("Moderate","#f39c12"),
                                     ("Weak","#f39c12"),("Lost","#e74c3c")]):
            tk.Radiobutton(frame, text=s, variable=self._usbl_var, value=s,
                           bg=BG, fg=c, selectcolor=BG2, activebackground=BG,
                           font=("Helvetica", 10),
                           command=self._on_usbl).grid(row=0, column=i, padx=14)

    # ── Tab 2: BN direct override ─────────────────────────────────────────────

    # BN node definitions: (node_id, display_label, [states], risk_order)
    BN_NODES = [
        ("Fatigue",          "Fatigue",          ["Low", "Medium", "High"]),
        ("Stress",           "Stress",            ["Low", "Medium", "High"]),
        ("CameraQuality",    "Camera Quality",    ["Excellent", "Good", "Poor", "Degraded"]),
        ("DVLStatus",        "DVL Status",        ["Working", "Degraded", "Failed"]),
        ("ArUcoVisibility",  "ArUco Visibility",  ["None", "Low", "Medium", "High"]),
        ("BatteryLevel",     "Battery Level",     ["High", "Medium", "Low", "Critical"]),
        ("MissionPhase",     "Mission Phase",     ["Transit", "Inspection", "Docking",
                                                   "Homing", "UnDocking"]),
    ]

    # Colour per state for override buttons
    STATE_BG = {
        "Low": "#27ae60",    "Medium": "#f39c12",    "High": "#e74c3c",
        "Working": "#27ae60","Degraded": "#f39c12",  "Failed": "#e74c3c",
        "Excellent":"#27ae60","Good":"#2980b9",       "Poor":"#f39c12",
        "None": "#27ae60",   "Critical": "#e74c3c",
        "Transit":"#2980b9", "Inspection":"#27ae60", "Docking":"#8e44ad",
        "Homing":"#e74c3c",  "UnDocking":"#1abc9c",
    }

    def _build_override_tab(self, parent):
        tk.Label(parent,
                 text="Sends JSON directly into the BN node — no topic competition",
                 bg=BG, fg="#aaaaaa", font=("Helvetica", 8)).grid(
                     row=0, column=0, columnspan=10, pady=(10, 4), sticky="w", padx=12)

        self._sep(parent, 1)

        self._active_overrides: dict[str, tk.StringVar] = {}

        for row_idx, (node_id, label, states) in enumerate(self.BN_NODES):
            base_row = 2 + row_idx * 2

            tk.Label(parent, text=label, bg=BG, fg=FG,
                     font=("Helvetica", 10, "bold"), width=16,
                     anchor="w").grid(row=base_row, column=0, padx=(12,4), pady=3,
                                      sticky="w")

            var = tk.StringVar(value="— auto —")
            self._active_overrides[node_id] = var

            active_lbl = tk.Label(parent, textvariable=var, bg=BG, fg="#aaaaaa",
                                   font=("Helvetica", 9), width=12)
            active_lbl.grid(row=base_row, column=1, padx=4)

            btn_frame = tk.Frame(parent, bg=BG)
            btn_frame.grid(row=base_row, column=2, padx=4, sticky="w")

            for s in states:
                bg_col = self.STATE_BG.get(s, "#555555")
                tk.Button(btn_frame, text=s, width=9,
                          bg=bg_col, fg="white", relief="flat",
                          activebackground=bg_col, activeforeground="white",
                          font=("Helvetica", 9),
                          command=lambda nid=node_id, st=s, lbl=active_lbl, v=var:
                              self._set_override(nid, st, lbl, v)
                          ).pack(side="left", padx=3)

            tk.Button(btn_frame, text="✕ clear", width=7,
                      bg="#555555", fg="white", relief="flat",
                      activebackground="#444444", activeforeground="white",
                      font=("Helvetica", 9),
                      command=lambda nid=node_id, lbl=active_lbl, v=var:
                          self._clear_override(nid, lbl, v)
                      ).pack(side="left", padx=(8, 3))

            if row_idx < len(self.BN_NODES) - 1:
                self._sep(parent, base_row + 1)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_speed(self, val):
        v = float(val)
        self._node.speed_val = v
        self._speed_lbl.config(text=f"{v:.2f} m/s")
        s = _speed_state(v)
        self._speed_state.config(text=s, fg=_col(s))

    def _on_current(self, val):
        v = float(val)
        self._node.current_val = v
        self._curr_lbl.config(text=f"{v:.2f} m/s")
        s = _current_state(v)
        self._curr_state.config(text=s, fg=_col(s))

    def _on_usbl(self):
        self._node.usbl_val = self._usbl_var.get()

    def _set_override(self, node_id, state, lbl, var):
        self._node.send_override(node_id, state)
        col = self.STATE_BG.get(state, "#ecf0f1")
        lbl.config(fg=col)
        var.set(state)

    def _clear_override(self, node_id, lbl, var):
        self._node.clear_override(node_id)
        lbl.config(fg="#aaaaaa")
        var.set("— auto —")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _sep(self, parent, row):
        ttk.Separator(parent, orient="horizontal").grid(
            row=row, column=0, columnspan=10, sticky="ew", padx=12, pady=2)

    def _section_label(self, parent, text, row):
        tk.Label(parent, text=text, bg=BG, fg=FG,
                 font=("Helvetica", 10, "bold")).grid(
                     row=row, column=0, columnspan=3, sticky="w", **PAD)

    def _val_label(self, parent, text, row, col):
        lbl = tk.Label(parent, text=text, bg=BG, fg=FG,
                       font=("Helvetica", 10), width=10)
        lbl.grid(row=row, column=col, **PAD)
        return lbl

    def _state_label(self, parent, text, colour, row, col):
        lbl = tk.Label(parent, text=text, bg=BG, fg=colour,
                       font=("Helvetica", 10, "bold"), width=10)
        lbl.grid(row=row, column=col, **PAD)
        return lbl

    def _tick(self):
        cur = self._status.cget("fg")
        self._status.config(fg="#27ae60" if cur == BG else BG)
        self.after(500, self._tick)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = EvidencePublisher()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    app = App(node)
    try:
        app.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
