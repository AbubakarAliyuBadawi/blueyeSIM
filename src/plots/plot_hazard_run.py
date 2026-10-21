#!/usr/bin/env python3
"""
Results of one hazard-scenario run (results table and Fig. 9 panel of the paper).

Usage:
  source ~/Desktop/auto-pilot/install/setup.bash
  python3 src/plots/plot_hazard_run.py src/rosbags/paper/s1_obstacle_run01_shadow_20261018 --scenario S1

Events (mission time; t = 0 at the first /mission_state > 0):
  injection  S1: first sonar echo from the transit rock (an echo off the known dock and pipe, within 0.3 m of the rock)
             or --injection <s> to set it by hand
  t_att      first time after the injection that IncreasedAttentionRequired or TakeoverRequested is most probable
  t_req      first time after the injection that the urgency reaches the threshold
  t_haz      S1: contact with the rock (the vehicle stops while it was surging, centre within 0.35 m of the rock)
  lead time  t_haz - t_req
S2 (--scenario S2, current and USBL loss during inspection): injection = first water current switched on (from
/scenario/events, or the sim_current log in /rosout); the USBL beacon cut is the second injection. Reported: t_att and
t_req from the current onset, t_req after the USBL cut, the distance from the planned leg (as the BN computes the
tracking error) and the posteriors that explain the request (DataQuality, Navigation against Collision).
Closed loop (takeover_mode closed_loop) adds: operator decision time (popup to accept/reject), manual control
(accept to hand-back), closest approach to the obstacle, and success = no contact and the mission resumed after the
hand-back.

Output (default src/plots/paper/<bag_name>/):
  hazard_timeline.pdf/.png  evidence that gets worse after the injection, urgency with threshold, events marked;
                            distances are from the vehicle centre to the obstacle outline
  hazard_results.csv/.md    one row of the results table
  hazard_summary.txt
"""

import argparse
import csv
import json
import math
import os
import re

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import ConvexHull

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(HERE)
SIM = os.path.join(SRC, "vortex-stonefish-sim", "stonefish_sim")

PHASES = {1: "Undocking", 2: "Transit", 3: "Inspection", 4: "Homing", 5: "Docking", 6: "Takeover"}
SEVERITY = {
    "SonarRange": ["Clear", "Near", "Critical"], "Altitude": ["Safe", "Marginal", "Unsafe"],
    "DVLStatus": ["Nominal", "Degraded", "Failed"], "USBLStrength": ["Strong", "Moderate", "Weak", "Lost"],
    "CameraQuality": ["Excellent", "Good", "Poor", "Failed"], "ArUcoVisibility": ["All", "Some", "None"],
    "Speed": ["Safe", "Moderate", "High"], "Current": ["Low", "Medium", "High"],
    "WaypointTrackingError": ["Low", "Medium", "High"], "RangeToTarget": ["Far", "Mid", "Close"],
    "BatteryLevel": ["High", "Medium", "Low", "Critical"], "Fatigue": ["Low", "Medium", "High"],
    "Stress": ["Low", "Medium", "High"],
}
INK, MUTED = "#0b0b0b", "#52514e"
EVENT_STYLE = {"injection": ("#52514e", ":"), "t_att": ("#c98500", "--"), "t_req": ("#d62728", "--"), "t_haz": ("#0b0b0b", "-"),
               "accept": ("#1baf7a", "-."), "handback": ("#2a78d6", "-.")}
plt.rcParams.update({"font.size": 8, "axes.labelsize": 8, "legend.fontsize": 7, "xtick.labelsize": 7,
                     "ytick.labelsize": 7, "pdf.fonttype": 42, "ps.fonttype": 42})


def severity_colours(n):
    cmap = plt.get_cmap("RdYlGn_r")
    return [cmap(0.08 + 0.84 * i / max(n - 1, 1)) for i in range(n)]


def load(bag, images=True):
    """Topics of one run; images=False skips the sonar images (only S1 needs them, to find the first rock echo)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    topics = ["/mission_state", "/rosout", "/blueye/odom", "/blueye/ref_vel", "/blueye/takeover_request/urgency",
              "/blueye/waypoint_goal", "/blueye/current", "/scenario/events",
              "/blueye/takeover_request/threshold", "/blueye/takeover_request/evidence", "/blueye/fls/image/compressed",
              "/blueye/takeover_request/pending", "/blueye/takeover_request/human_decision",
              "/blueye/takeover_request/handback"]
    topics = [t for t in topics if t in types and (images or t != "/blueye/fls/image/compressed")]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    classes = {t: get_message(types[t]) for t in topics}
    data = {t: [] for t in topics}
    data.setdefault("/scenario/events", [])
    data.setdefault("/blueye/waypoint_goal", [])
    data.setdefault("/blueye/current", [])
    t0 = None
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        t0 = t0 if t0 is not None else stamp
        ts = (stamp - t0) / 1e9
        msg = deserialize_message(raw, classes[topic])
        if topic == "/blueye/odom":
            p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            data[topic].append((ts, p.x, p.y, yaw, math.hypot(v.x, v.y), p.z))
        elif topic == "/blueye/waypoint_goal":
            data[topic].append((ts, msg.point.x, msg.point.y, msg.point.z))
        elif topic == "/blueye/ref_vel":
            data[topic].append((ts, msg.twist.linear.x))
        elif topic == "/blueye/takeover_request/evidence":
            data[topic].append((ts, json.loads(msg.data)))
        elif topic == "/blueye/fls/image/compressed":
            data[topic].append((ts, bytes(msg.data)))
        elif topic == "/rosout":
            data[topic].append((ts, msg.name, msg.msg))
        else:
            data[topic].append((ts, msg.data))
    return data


def footprints(config_path):
    """Known structures (masked by the BN) and the transit rock, as 2-D polygons in world coordinates."""
    with open(config_path) as f:
        config = yaml.safe_load(f)

    def mesh_xy(mesh, key, scale=1.0):
        x, y, _ = (float(v) for v in str(config[f"{key}_position"]).split())
        yaw = float(str(config[f"{key}_orientation"]).split()[2])
        v = np.array([[float(t) for t in l.split()[1:3]] for l in open(os.path.join(SIM, "data", "object_files", f"{mesh}.obj"))
                      if l.startswith("v ")]) * scale
        c, s = math.cos(yaw), math.sin(yaw)
        return np.stack([x + c * v[:, 0] - s * v[:, 1], y + s * v[:, 0] + c * v[:, 1]], axis=1)

    def faces(mesh, key):
        x, y, _ = (float(v) for v in str(config[f"{key}_position"]).split())
        yaw = float(str(config[f"{key}_orientation"]).split()[2])
        verts, fs = [], []
        for l in open(os.path.join(SIM, "data", "object_files", f"{mesh}.obj")):
            if l.startswith("v "):
                verts.append([float(t) for t in l.split()[1:3]])
            elif l.startswith("f "):
                fs.append([int(t.split("/")[0]) - 1 for t in l.split()[1:]])
        v = np.array(verts)
        c, s = math.cos(yaw), math.sin(yaw)
        w = np.stack([x + c * v[:, 0] - s * v[:, 1], y + s * v[:, 0] + c * v[:, 1]], axis=1)
        return [w[f] for f in fs]

    known = faces("docking_station_rig", "docking_rig") + faces("pipe2", "pipeline")
    rock = None
    if "transit_rock_position" in config:
        pts = mesh_xy("rock", "transit_rock", 0.15)
        rock = pts[ConvexHull(pts).vertices]
    return known, rock


def known_grid(polys, margin=0.5, res=0.1):
    pts = np.concatenate(polys)
    x0, y0 = pts.min(0) - margin - res
    nx, ny = ((pts.max(0) + margin + res - (x0, y0)) / res).astype(int) + 1
    grid = np.zeros((ny, nx), np.uint8)
    cv2.fillPoly(grid, [np.round((p - (x0, y0)) / res).astype(np.int32) for p in polys], 1)
    grid = cv2.dilate(grid, np.ones((2 * int(round(margin / res)) + 1,) * 2, np.uint8))
    return x0, y0, res, grid


def dist_to_polygon(points, poly):
    """Distance of each point to the polygon outline (0 inside is not needed: the vehicle never enters the rock)."""
    best = np.full(len(points), np.inf)
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        t = np.clip(((points - a) @ (b - a)) / ((b - a) @ (b - a)), 0, 1)
        best = np.minimum(best, np.linalg.norm(a + t[:, None] * (b - a) - points, axis=1))
    return best


def first_rock_echo(data, start, grid, rock):
    """Mission time of the first sonar frame with an echo off the known structures and on the rock."""
    odom = data["/blueye/odom"]
    ot = np.array([o[0] for o in odom])
    x0, y0, res, g = grid
    for ts, raw in data["/blueye/fls/image/compressed"]:
        if ts < start:
            continue
        img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        rows, cols = np.nonzero(img > 60)
        if rows.size == 0:
            continue
        o = odom[min(int(np.searchsorted(ot, ts)), len(odom) - 1)]
        rng = 0.1 + (img.shape[0] - 1 - rows) / (img.shape[0] - 1) * 4.9
        brg = o[3] + (cols / (img.shape[1] - 1) - 0.5) * math.radians(90)
        x = o[1] + 0.18 * math.cos(o[3]) + rng * np.cos(brg)
        y = o[2] + 0.18 * math.sin(o[3]) + rng * np.sin(brg)
        i, j = ((x - x0) / res).astype(int), ((y - y0) / res).astype(int)
        inside = (i >= 0) & (i < g.shape[1]) & (j >= 0) & (j < g.shape[0])
        known = np.zeros(x.size, bool)
        known[inside] = g[j[inside], i[inside]] > 0
        pts = np.stack([x[~known], y[~known]], axis=1)
        if len(pts) and (dist_to_polygon(pts, rock) < 0.3).any():
            return ts - start
    return None


# ── S2: current and USBL loss during inspection ────────────────────────────────

S2_POSTERIORS = [("DataQuality", "Inadequate", "#2a78d6"), ("NavigationSystemPerformance", "Inadequate", "#eb6834"),
                 ("CollisionWithObstacle", "Likely", "#1baf7a")]          # validated categorical slots 1-3


def s2_events(data, start):
    """[(mission time, 'current', (speed, direction)) | (mission time, 'usbl', enabled)] from /scenario/events, or from
    the sim_current / usbl_strength_monitor logs in /rosout when the events topic was not recorded."""
    out = []
    if data.get("/scenario/events"):
        for t, raw in data["/scenario/events"]:
            e = json.loads(raw)
            if e.get("event") == "current":
                out.append((t - start, "current", (e["speed_ms"], e["direction_deg"])))
            elif e.get("event") == "usbl_beacon":
                out.append((t - start, "usbl", bool(e["enabled"])))
    else:
        for t, node, m in data["/rosout"]:
            hit = re.match(r"Current set to ([\d.]+) m/s towards ([\d.]+) deg", m) if node == "sim_current" else None
            if hit:
                out.append((t - start, "current", (float(hit[1]), float(hit[2]))))
            elif node == "usbl_strength_monitor" and m.startswith("USBL beacon"):
                out.append((t - start, "usbl", m.split(":")[0].endswith("ON")))
    return sorted(out, key=lambda e: e[0])


def s2_window_end(events, t_inj, t_hb, t_last):
    """End of the S2 analysis window, the same rule for open and closed loop: 40 s after the last disturbance (current
    change or USBL switch), or 30 s after the hand-back when that is later, and not past the end of the recording."""
    end = max((t for t, kind, v in events if t >= t_inj), default=t_inj) + 40.0
    if t_hb is not None:
        end = max(end, t_hb + 30.0)
    return min(end, t_last)


def leg_cross_track(data, start):
    """(mission time, distance from the planned leg) as the BN computes WaypointTrackingError: the leg runs from where
    the vehicle was when the goal arrived to the goal (3-D, clamped to the segment)."""
    odom = data["/blueye/odom"]
    goals = [(t, np.array(g)) for t, *g in data["/blueye/waypoint_goal"]]
    out, gi, a, b = [], -1, None, None
    for o in odom:
        while gi + 1 < len(goals) and goals[gi + 1][0] <= o[0]:
            gi += 1
            b = goals[gi][1]
            a = np.array([o[1], o[2], o[5]])
        if b is None:
            continue
        p = np.array([o[1], o[2], o[5]])
        ab, ap = b - a, p - a
        t = 0.0 if ab @ ab < 1e-9 else float(np.clip((ap @ ab) / (ab @ ab), 0.0, 1.0))
        out.append((o[0] - start, float(np.linalg.norm(p - (a + t * ab)))))
    return out


def s2_results(args, data, name, out, start, end_bag, thr, mode):
    events = s2_events(data, start)
    t_inj = args.injection if args.injection is not None else \
        next((t for t, kind, v in events if kind == "current" and v[0] > 0 and t >= 0), None)
    if t_inj is None:
        raise SystemExit("No current switched on during the mission: give --injection")
    t_usbl = next((t for t, kind, v in events if kind == "usbl" and not v and t >= t_inj), None)
    ev = [(t - start, e) for t, e in data["/blueye/takeover_request/evidence"]]
    urg = [(t - start, v) for t, v in data["/blueye/takeover_request/urgency"]]
    t_att = next((t for t, e in ev if t >= t_inj and max(e["probabilities"], key=e["probabilities"].get)
                  in ("IncreasedAttentionRequired", "TakeoverRequested")), None)
    t_req = next((t for t, v in urg if t >= t_inj and v >= thr), None)
    t_popup = next((t - start for t, v in data.get("/blueye/takeover_request/pending", []) if v and t - start >= t_inj), None)
    decision = next(((t - start, v) for t, v in data.get("/blueye/takeover_request/human_decision", [])
                     if t - start >= t_inj), None)
    t_dec = decision[0] if decision else None
    t_hb = next((t - start for t, v in data.get("/blueye/takeover_request/handback", []) if v and t - start >= t_inj), None)
    resumed = t_hb is not None and any("Waypoint reached" in m and t - start > t_hb for t, n, m in data["/rosout"])
    finished = next((m.split(":")[-1].strip() for t, n, m in data["/rosout"] if "Mission finished" in m), None)
    xt = leg_cross_track(data, start)
    t_end = s2_window_end(events, t_inj, t_hb if mode == "CL" else None, end_bag - start)
    t0 = max(0.0, t_inj - 20.0)
    phase = PHASES.get(next((v for t, v in reversed(data["/mission_state"]) if t - start <= t_inj), 0), "-")
    u_before = max((v for t, v in urg if 0 <= t < t_inj), default=float("nan"))
    u_after = max((v for t, v in urg if t_inj <= t <= t_end), default=float("nan"))
    xt_max = max((d for t, d in xt if t_inj <= t <= t_end), default=float("nan"))           # panel scale
    # off the leg while the vehicle steered itself (closed loop: until the operator took over; the BN has no
    # tracking error under manual control), and under manual control
    t_auto = t_dec if (mode == "CL" and t_dec is not None) else t_end
    xt_auto = max((d for t, d in xt if t_inj <= t <= t_auto), default=float("nan"))
    xt_manual = max((d for t, d in xt if t_dec <= t <= t_hb), default=None) if (t_dec is not None and t_hb is not None) else None
    win = [(t, e) for t, e in ev if t0 <= t <= t_end]
    dt = float(np.median(np.diff([t for t, _ in win]))) if len(win) > 1 else 0.5
    wte_high_s = sum(dt for t, e in win if t >= t_inj and e["evidence"].get("WaypointTrackingError") == "High")
    at_req = min(ev, key=lambda x: abs(x[0] - t_req))[1]["posteriors"] if t_req is not None else None

    def rank(node, state):
        return SEVERITY[node].index(state) if state in SEVERITY[node] else -1
    at_inj = next((e for t, e in reversed(win) if t <= t_inj), win[0][1])["evidence"]
    changed = ["Current", "WaypointTrackingError", "USBLStrength"] + [
        k for k in SEVERITY if k not in ("Fatigue", "Stress", "RangeToTarget", "Current", "WaypointTrackingError",
                                         "USBLStrength")
        and max(rank(k, e["evidence"].get(k)) for t, e in win if t >= t_inj) > max(rank(k, at_inj.get(k)), 0)]

    fig, axes = plt.subplots(4, 1, figsize=(7.2, 0.32 * len(changed) + 5.4), sharex=True,
                             gridspec_kw={"height_ratios": [0.32 * len(changed) + 0.3, 1.2, 1.3, 1.3], "hspace": 0.14})
    a, b, c, d = axes
    for row, node in enumerate(changed):                                   # (a) evidence
        cols = severity_colours(len(SEVERITY[node]))
        y = len(changed) - 1 - row
        for i, (t, e) in enumerate(win):
            t1 = win[i + 1][0] if i + 1 < len(win) else t_end
            st = e["evidence"].get(node)
            a.broken_barh([(t - t_inj, t1 - t)], (y - 0.4, 0.8),
                          facecolors=cols[SEVERITY[node].index(st)] if st in SEVERITY[node] else "#e0e0e0", lw=0)
        starts = [i for i, (t, e) in enumerate(win) if i == 0 or e["evidence"].get(node) != win[i - 1][1]["evidence"].get(node)]
        for k, i in enumerate(starts):
            t_next = win[starts[k + 1]][0] if k + 1 < len(starts) else t_end
            if t_next - win[i][0] >= 0.05 * (t_end - t0):
                a.text(win[i][0] - t_inj + 0.4, y, win[i][1]["evidence"].get(node) or "not entered", va="center",
                       fontsize=6.5, color=INK)
    a.set_yticks(range(len(changed)))
    a.set_yticklabels(list(reversed(changed)))
    a.set_ylim(-0.6, len(changed) - 0.4)

    xs = [(t - t_inj, v) for t, v in xt if t0 <= t <= t_end]                # (b) distance from the leg
    if xs:
        b.plot(*zip(*xs), color=INK, lw=1.2)
    for level, label in ((0.5, "Medium"), (1.5, "High")):
        b.axhline(level, color=MUTED, ls=":", lw=0.9)
        b.text(t_end - t_inj, level, f" {label}", va="center", ha="left", fontsize=6.5, color=MUTED)
    b.set_ylabel("Off the leg [m]")
    b.set_ylim(0, max(2.0, (xt_max if xt_max == xt_max else 0) * 1.15))
    for t, kind, v in events:                                             # current settings as text
        if kind == "current" and t0 <= t <= t_end:
            b.text(t - t_inj + 0.4, b.get_ylim()[1] * 0.93, f"{v[0]:.2f} m/s to {v[1]:.0f}\u00b0" if v[0] > 0 else "calm",
                   fontsize=6.3, color="#2a78d6", va="top", rotation=90)

    times = [t - t_inj for t, _ in win]                                   # (c) why: posteriors
    for node, state, colour in S2_POSTERIORS:
        vals = [e["posteriors"][node][state] for _, e in win]
        c.plot(times, vals, color=colour, lw=1.2, label=f"P({node} = {state})")
    c.set_ylim(0, 1)
    c.set_ylabel("Posterior")
    c.legend(loc="upper left", bbox_to_anchor=(0.035, 1.0), frameon=False, fontsize=6.5)

    ut = [(t - t_inj, v) for t, v in urg if t0 <= t <= t_end]             # (d) urgency
    d.plot(*zip(*ut), color=INK, lw=1.2, label="Urgency $U$")
    d.axhline(thr, color="#d62728", ls="--", lw=1.0, label=f"Threshold {thr:.2f}")
    d.set_ylim(0, 1)
    d.set_ylabel("$U$")
    d.set_xlabel("Time from current onset [s]")
    d.set_xlim(t0 - t_inj, t_end - t_inj)
    d.legend(loc="upper left", bbox_to_anchor=(0.035, 1.0), frameon=False)
    for ax, tag in zip(axes, "abcd"):
        ax.text(0.003, 0.97 if ax is not a else 1.0, f"({tag})", transform=ax.transAxes, va="top", fontweight="bold",
                fontsize=7)

    marks = [("injection", t_inj, "current on"), ("t_att", t_att, "$t_{att}$"), ("usbl", t_usbl, "USBL cut"),
             ("t_req", t_req, "$t_{req}$")]
    marks += [("usbl", t, "USBL back") for t, kind, v in events if kind == "usbl" and v and t > t_inj and t <= t_end]
    marks += [("injection", t, "calm") for t, kind, v in events if kind == "current" and v[0] == 0 and t_inj < t <= t_end]
    if mode == "CL":
        marks += [("accept", t_dec, "accepted" if (decision and decision[1]) else "rejected"), ("handback", t_hb, "hand-back")]
        if t_dec is not None and t_hb is not None:
            for ax in axes:
                ax.axvspan(t_dec - t_inj, t_hb - t_inj, color="#2a78d6", alpha=0.08, lw=0)
    style = dict(EVENT_STYLE, usbl=("#9467bd", "-."))
    last_x, level, top = None, 0, 0
    for key, t, label in sorted((m for m in marks if m[1] is not None), key=lambda m: m[1]):
        colour, ls = style[key]
        x = t - t_inj
        level = level + 1 if (last_x is not None and x - last_x < 0.06 * (t_end - t0)) else 0
        last_x, top = x, max(top, level)
        for ax in axes:
            ax.axvline(x, color=colour, ls=ls, lw=0.9)
        a.text(x, 1.02 + 0.13 * level, label, transform=a.get_xaxis_transform(), ha="center", va="bottom",
               fontsize=6.5, color=colour)
    a.set_title(f"{args.scenario} ({phase}, {mode}): {name}", fontsize=8, loc="left", pad=14 + 11 * (top + 1))
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(out, f"hazard_timeline.{ext}"), dpi=300, bbox_inches="tight")
    plt.close(fig)

    def r(v):
        return "" if v is None or (isinstance(v, float) and math.isnan(v)) else round(v, 2)
    row = {"ID": args.scenario, "phase": phase, "run": name, "mode": mode, "max_U_before": r(u_before),
           "max_U_after": r(u_after), "t_att_s": r(None if t_att is None else t_att - t_inj),
           "t_req_s": r(None if t_req is None else t_req - t_inj),
           "t_req_after_usbl_cut_s": r(None if (t_req is None or t_usbl is None) else t_req - t_usbl),
           "max_off_leg_m": r(xt_auto), "max_off_leg_manual_m": r(xt_manual), "tracking_error_high_s": r(wte_high_s),
           "P_dataquality_inadequate_at_req": r(None if at_req is None else at_req["DataQuality"]["Inadequate"]),
           "P_navigation_inadequate_at_req": r(None if at_req is None else at_req["NavigationSystemPerformance"]["Inadequate"]),
           "P_collision_likely_at_req": r(None if at_req is None else at_req["CollisionWithObstacle"]["Likely"]),
           "decision_s": r(None if (t_dec is None or t_popup is None) else t_dec - t_popup),
           "manual_control_s": r(None if (t_dec is None or t_hb is None) else t_hb - t_dec),
           "success_CL": ("n/a" if mode != "CL" else ("yes" if resumed else "no"))}
    with open(os.path.join(out, "hazard_results.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(row))
        w.writeheader()
        w.writerow(row)
    with open(os.path.join(out, "hazard_results.md"), "w") as f:
        f.write("| " + " | ".join(row) + " |\n|" + "---|" * len(row) + "\n| " + " | ".join(str(v) for v in row.values()) + " |\n")
    first = next(v for t, kind, v in events if kind == "current" and t == t_inj) if args.injection is None else None
    lines = [f"run: {name}", f"scenario: {args.scenario} ({phase}), mode {mode}", f"threshold: {thr:.2f}",
             "disturbances (mission time): " + ("; ".join(
                 (f"{t:.1f} s current {v[0]:.2f} m/s to {v[1]:.0f} deg" if v[0] > 0 else f"{t:.1f} s calm water")
                 if kind == "current" else f"{t:.1f} s USBL beacon {'on' if v else 'cut'}"
                 for t, kind, v in events if t >= 0) or "none found"),
             f"injection (current onset): {t_inj:.1f} s" + (f", {first[0]:.2f} m/s to {first[1]:.0f} deg" if first else ""),
             f"t_att: {r(None if t_att is None else t_att - t_inj)} s after the current onset",
             f"USBL cut: {r(None if t_usbl is None else t_usbl - t_inj)} s after the current onset",
             f"t_req: {r(None if t_req is None else t_req - t_inj)} s after the current onset, "
             f"{r(None if (t_req is None or t_usbl is None) else t_req - t_usbl)} s after the USBL cut",
             f"max distance off the leg: {r(xt_auto)} m" + (" (until the operator took over)" if mode == "CL" else "")
             + f"; tracking error High for {r(wte_high_s)} s",
             "posteriors at t_req: " + ("" if at_req is None else
                 f"DataQuality Inadequate {at_req['DataQuality']['Inadequate']:.2f}, Navigation Inadequate "
                 f"{at_req['NavigationSystemPerformance']['Inadequate']:.2f}, Collision Likely "
                 f"{at_req['CollisionWithObstacle']['Likely']:.2f}"),
             f"max U before the current: {u_before:.3f}; after: {u_after:.3f}",
             f"mission result: {finished or 'not finished in the recording'}"]
    if mode == "CL":
        lines += [f"operator {'accepted' if (decision and decision[1]) else 'rejected' if decision else 'did not decide'}"
                  f" after {r(None if (t_dec is None or t_popup is None) else t_dec - t_popup)} s",
                  f"hand-back: {r(None if t_hb is None else t_hb - t_inj)} s after the current onset (manual control "
                  f"{r(None if (t_dec is None or t_hb is None) else t_hb - t_dec)} s); mission resumed: {'yes' if resumed else 'no'}",
                  f"max distance off the leg under manual control: {r(xt_manual)} m"]
    with open(os.path.join(out, "hazard_summary.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join("  " + l for l in lines))
    print(f"Saved to {out}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag_path")
    parser.add_argument("--scenario", default="S1")
    parser.add_argument("--mode", default=None, help="OL or CL (default: from the takeover mode in the bag)")
    parser.add_argument("--injection", type=float, default=None, help="injection time [s, mission time]")
    parser.add_argument("--config", default=os.path.join(SIM, "config", "blueye_obstacle_config.yaml"))
    parser.add_argument("--out", default=None)
    args = parser.parse_args()
    bag = os.path.abspath(args.bag_path)
    name = os.path.basename(os.path.normpath(bag))
    out = args.out or os.path.join(HERE, "paper", name)
    os.makedirs(out, exist_ok=True)

    print(f"Reading {bag} ...")
    data = load(bag, images=args.scenario != "S2")
    start = next(t for t, v in data["/mission_state"] if v > 0)
    end_bag = data["/blueye/odom"][-1][0]
    thr = float(data["/blueye/takeover_request/threshold"][-1][1]) if data.get("/blueye/takeover_request/threshold") else 0.65
    mode = args.mode or ("OL" if any("shadow" in m.lower() for _, n, m in data["/rosout"] if n == "blueye_stonefish_bt") else "CL")
    if args.scenario.upper().startswith("S2"):
        return s2_results(args, data, name, out, start, end_bag, thr, mode)
    known, rock = footprints(args.config)

    # events
    t_inj = args.injection
    if t_inj is None and args.scenario.upper().startswith("S1"):
        t_inj = first_rock_echo(data, start, known_grid(known), rock)
    if t_inj is None:
        raise SystemExit("No injection time: give --injection")
    ev = [(t - start, e) for t, e in data["/blueye/takeover_request/evidence"]]
    urg = [(t - start, v) for t, v in data["/blueye/takeover_request/urgency"]]
    t_att = next((t for t, e in ev if t >= t_inj and max(e["probabilities"], key=e["probabilities"].get)
                  in ("IncreasedAttentionRequired", "TakeoverRequested")), None)
    t_req = next((t for t, v in urg if t >= t_inj and v >= thr), None)
    t_haz, dist = None, None
    ot = np.array([o[0] - start for o in data["/blueye/odom"]])
    if rock is not None:
        odom = data["/blueye/odom"]
        xy = np.array([[o[1], o[2]] for o in odom])
        dist = dist_to_polygon(xy, rock)                               # vehicle centre to the rock outline
        moving = False
        for k, o in enumerate(odom):
            tm = o[0] - start
            moving = moving or (tm > t_inj and o[4] > 0.1)
            if moving and o[4] < 0.03 and dist[k] < 0.35:
                t_haz = tm
                break
    def dist_at(t):
        return None if dist is None or t is None else float(dist[min(int(np.searchsorted(ot, t)), len(ot) - 1)])

    # closed loop: popup, operator decision, hand-back, resumption
    t_popup = next((t - start for t, v in data.get("/blueye/takeover_request/pending", []) if v and t - start >= t_inj), None)
    decision = next(((t - start, v) for t, v in data.get("/blueye/takeover_request/human_decision", [])
                     if t - start >= t_inj), None)
    t_dec = decision[0] if decision else None
    t_hb = next((t - start for t, v in data.get("/blueye/takeover_request/handback", []) if v and t - start >= t_inj), None)
    resumed = t_hb is not None and any("Waypoint reached" in m and t - start > t_hb for t, n, m in data["/rosout"])
    finished = next((m.split(":")[-1].strip() for t, n, m in data["/rosout"] if "Mission finished" in m), None)
    closest = float(dist[ot >= t_inj].min()) if dist is not None else None
    if mode == "CL":
        t_end = min((t_hb + 20.0) if t_hb is not None else (end_bag - start), end_bag - start)
    else:
        t_end = (t_haz + 3.0) if t_haz is not None else (end_bag - start)
    u_before = max((v for t, v in urg if 0 <= t < t_inj), default=float("nan"))
    u_after = max((v for t, v in urg if t_inj <= t <= t_end), default=float("nan"))
    phase = PHASES.get(next((v for t, v in reversed(data["/mission_state"]) if t - start <= t_inj), 0), "-")
    lead = (t_haz - t_req) if (t_haz is not None and t_req is not None) else None
    shadow_log = next((t - start for t, n, m in data["/rosout"] if "takeover would be requested" in m and t - start >= t_inj), None)

    # figure: evidence that changes after the injection, urgency with threshold, events
    t0 = max(0.0, t_inj - 20.0)
    win = [(t, e) for t, e in ev if t0 <= t <= t_end]
    def rank(node, state):
        return SEVERITY[node].index(state) if state in SEVERITY[node] else -1
    at_inj = next((e for t, e in reversed(win) if t <= t_inj), win[0][1])["evidence"]
    # evidence that gets worse after the injection (a node only entering its nominal state is not shown)
    changed = [k for k in SEVERITY if k not in ("Fatigue", "Stress")
               and max(rank(k, e["evidence"].get(k)) for t, e in win if t >= t_inj) > max(rank(k, at_inj.get(k)), 0)]
    changed = changed or ["SonarRange"]
    fig, (a, b) = plt.subplots(2, 1, figsize=(7.2, 1.2 + 0.32 * len(changed) + 1.6), sharex=True,
                               gridspec_kw={"height_ratios": [0.32 * len(changed) + 0.3, 1.6], "hspace": 0.12})
    for row, node in enumerate(changed):
        cols = severity_colours(len(SEVERITY[node]))
        y = len(changed) - 1 - row
        for i, (t, e) in enumerate(win):
            t1 = win[i + 1][0] if i + 1 < len(win) else t_end
            s = e["evidence"].get(node)
            a.broken_barh([(t - t_inj, t1 - t)], (y - 0.4, 0.8),
                          facecolors=cols[SEVERITY[node].index(s)] if s in SEVERITY[node] else "#e0e0e0", lw=0)
        starts = [i for i, (t, e) in enumerate(win) if i == 0 or e["evidence"].get(node) != win[i - 1][1]["evidence"].get(node)]
        for k, i in enumerate(starts):                                 # state names where they start (not on slivers)
            t_next = win[starts[k + 1]][0] if k + 1 < len(starts) else t_end
            if t_next - win[i][0] >= 0.06 * (t_end - t0):
                a.text(win[i][0] - t_inj + 0.3, y, win[i][1]["evidence"].get(node) or "not entered", va="center",
                       fontsize=6.5, color=INK)
    a.set_yticks(range(len(changed)))
    a.set_yticklabels(list(reversed(changed)))
    a.set_ylim(-0.6, len(changed) - 0.4)
    ut = [(t - t_inj, v) for t, v in urg if t0 <= t <= t_end]
    b.plot(*zip(*ut), color=INK, lw=1.2, label="Urgency $U$")
    b.axhline(thr, color="#d62728", ls="--", lw=1.0, label=f"Threshold {thr:.2f}")
    b.set_ylim(0, 1)
    b.set_ylabel("$U$")
    b.set_xlabel("Time from injection [s]")
    b.set_xlim(t0 - t_inj, t_end - t_inj)
    events = [("injection", 0.0, "injection"), ("t_att", t_att, "$t_{att}$"), ("t_req", t_req, "$t_{req}$"),
              ("t_haz", t_haz, "$t_{haz}$")]
    if mode == "CL":
        events += [("accept", t_dec, "accepted" if (decision and decision[1]) else "rejected"), ("handback", t_hb, "hand-back")]
        if t_dec is not None and t_hb is not None:
            for ax in (a, b):
                ax.axvspan(t_dec - t_inj, t_hb - t_inj, color="#2a78d6", alpha=0.08, lw=0)
            b.text((t_dec + t_hb) / 2 - t_inj, 0.06, "manual control", ha="center", fontsize=6.5, color="#2a78d6")
    last_x, level = None, 0
    for key, t, label in sorted(((k, t, l) for k, t, l in events if t is not None or k == "injection"),
                                key=lambda e: 0.0 if e[0] == "injection" else e[1] - t_inj):
        if t is None:
            continue
        colour, ls = EVENT_STYLE[key]
        x = (t - t_inj) if key != "injection" else 0.0
        level = level + 1 if (last_x is not None and x - last_x < 0.07 * (t_end - t0)) else 0   # stack close labels
        last_x = x
        for ax in (a, b):
            ax.axvline(x, color=colour, ls=ls, lw=1.0)
        d = dist_at(t if key != "injection" else t_inj)
        show_d = d is not None and key in ("injection", "t_att", "t_req", "handback")
        a.text(x, 1.02 + 0.22 * level, label + (f"\n{d:.2f} m" if show_d else ""),
               transform=a.get_xaxis_transform(), ha="center", va="bottom", fontsize=6.5, color=colour)
    b.legend(loc="center left", frameon=False)
    a.set_title(f"{args.scenario} ({phase}, {mode}): {name}", fontsize=8, loc="left", pad=40 if mode == "CL" else 24)
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(out, f"hazard_timeline.{ext}"), dpi=300, bbox_inches="tight")
    plt.close(fig)

    def r(v):
        return "" if v is None or (isinstance(v, float) and math.isnan(v)) else round(v, 2)
    row = {"ID": args.scenario, "phase": phase, "run": name, "mode": mode, "max_U_before": r(u_before),
           "max_U_after": r(u_after), "t_att_s": r(None if t_att is None else t_att - t_inj),
           "t_req_s": r(None if t_req is None else t_req - t_inj), "lead_time_s": r(lead),
           "decision_s": r(None if (t_dec is None or t_popup is None) else t_dec - t_popup),
           "manual_control_s": r(None if (t_dec is None or t_hb is None) else t_hb - t_dec),
           "closest_approach_m": r(closest),
           "success_CL": ("n/a" if mode != "CL" else ("yes" if (t_haz is None and resumed) else "no"))}
    with open(os.path.join(out, "hazard_results.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(row))
        w.writeheader()
        w.writerow(row)
    with open(os.path.join(out, "hazard_results.md"), "w") as f:
        f.write("| " + " | ".join(row) + " |\n|" + "---|" * len(row) + "\n| " + " | ".join(str(v) for v in row.values()) + " |\n")
    lines = [f"run: {name}", f"scenario: {args.scenario} ({phase}), mode {mode}", f"threshold: {thr:.2f}",
             f"injection: {t_inj:.1f} s mission time" + (f" (first sonar echo from the rock, centre {dist_at(t_inj):.2f} m from it)" if dist is not None else ""),
             f"t_att: {r(None if t_att is None else t_att - t_inj)} s after injection" + (f" (centre {dist_at(t_att):.2f} m from the rock)" if t_att is not None and dist is not None else ""),
             f"t_req: {r(None if t_req is None else t_req - t_inj)} s after injection" + (f" (centre {dist_at(t_req):.2f} m from the rock)" if t_req is not None and dist is not None else ""),
             f"behaviour tree shadow log: {r(None if shadow_log is None else shadow_log - t_inj)} s after injection",
             f"t_haz: {r(None if t_haz is None else t_haz - t_inj)} s after injection (contact)" if t_haz is not None else "t_haz: none",
             f"lead time (t_haz - t_req): {r(lead)} s",
             f"max U before injection: {u_before:.3f}; after: {u_after:.3f}",
             f"closest approach (vehicle centre to the obstacle outline): {r(closest)} m"]
    if mode == "CL":
        lines += [f"popup: {r(None if t_popup is None else t_popup - t_inj)} s after injection",
                  f"operator {'accepted' if (decision and decision[1]) else 'rejected' if decision else 'did not decide'}"
                  f" after {r(None if (t_dec is None or t_popup is None) else t_dec - t_popup)} s",
                  f"hand-back: {r(None if t_hb is None else t_hb - t_inj)} s after injection "
                  f"(manual control {r(None if (t_dec is None or t_hb is None) else t_hb - t_dec)} s, "
                  f"{r(dist_at(t_hb))} m from the obstacle)",
                  f"mission resumed after hand-back: {'yes' if resumed else 'no'}; mission result: {finished or 'not finished'}",
                  f"success (no contact, resumed): {'yes' if (t_haz is None and resumed) else 'no'}"]
    with open(os.path.join(out, "hazard_summary.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join("  " + l for l in lines))
    print(f"Saved to {out}")


if __name__ == "__main__":
    main()
