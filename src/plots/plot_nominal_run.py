#!/usr/bin/env python3
"""
Paper figures for a takeover-BN mission run recorded in Stonefish (shadow or closed loop).

Usage:
  source ~/Desktop/auto-pilot/install/setup.bash
  python3 src/plots/plot_nominal_run.py src/rosbags/paper/nominal_run01_shadow_20261018

Output (default src/plots/paper/<bag_name>/), each figure as PDF (paper) and PNG (viewing):
  fig1_mission_map     track coloured by mission phase, waypoints, dock and pipeline footprints
  fig2_timeline        (a) urgency against the threshold, (b) TakeoverRequest posterior,
                       (c) evidence entered in the BN, coloured by severity, (d) operator fatigue and stress
  fig3_hidden_nodes    posteriors of the intermediate BN nodes (the explanation layer)
  phase_summary.csv/.md  per phase: duration, urgency statistics, time above threshold, alarms
  run_summary.txt        mission result, docking result, alarms, peak urgency, battery, distance

The analysis window runs from the first /mission_state > 0 to "Mission finished" (or the end of the bag), so
samples before the behaviour tree starts the mission are left out.
"""

import argparse
import bisect
import csv
import json
import math
import os

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
import yaml
from matplotlib.collections import LineCollection
from matplotlib.patches import Patch
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Phase colours: the validated categorical palette in phase order, the same as plot_mission_plan.py
PHASES = {1: ("Undocking", "#2a78d6"), 2: ("Transit", "#eb6834"), 3: ("Inspection", "#1baf7a"),
          4: ("Homing", "#eda100"), 5: ("Docking", "#e87ba4"), 6: ("Takeover", "#e34948")}

# Observed nodes, states ordered from nominal to worst
EVIDENCE_ROWS = [
    ("SonarRange", ["Clear", "Near", "Critical"]),
    ("Altitude", ["Safe", "Marginal", "Unsafe"]),
    ("DVLStatus", ["Nominal", "Degraded", "Failed"]),
    ("USBLStrength", ["Strong", "Moderate", "Weak", "Lost"]),
    ("CameraQuality", ["Excellent", "Good", "Poor", "Failed"]),
    ("ArUcoVisibility", ["All", "Some", "None"]),
    ("Speed", ["Safe", "Moderate", "High"]),
    ("Current", ["Low", "Medium", "High"]),
    ("WaypointTrackingError", ["Low", "Medium", "High"]),
    ("RangeToTarget", ["Far", "Mid", "Close"]),
    ("BatteryLevel", ["High", "Medium", "Low", "Critical"]),
    ("Fatigue", ["Low", "Medium", "High"]),
    ("Stress", ["Low", "Medium", "High"]),
]

# Intermediate nodes, states ordered from good to bad
HIDDEN_SYSTEM = [
    ("NavigationSystemPerformance", ["Adequate", "Marginal", "Inadequate"]),
    ("PerceptionSystemPerformance", ["Adequate", "Marginal", "Inadequate"]),
    ("DataQuality", ["Adequate", "Marginal", "Inadequate"]),
    ("AutonomousControlPerformance", ["Adequate", "Marginal", "Inadequate"]),
    ("ControlStability", ["High", "Medium", "Low"]),
    ("CollisionWithObstacle", ["Unlikely", "Possible", "Likely"]),
]
HIDDEN_HUMAN = [
    ("Attention", ["High", "Medium", "Low"]),
    ("SituationalAwareness", ["High", "Medium", "Low"]),
    ("OutOfLoopRisk", ["Low", "Medium", "High"]),
    ("HumanDecisionReliability", ["High", "Medium", "Low"]),
    ("PhaseTaskDemand", ["Low", "Medium", "High"]),
]
TAKEOVER_STATES = [("NoTakeoverRequired", "No takeover", "#2ca02c"),
                   ("IncreasedAttentionRequired", "Increased attention", "#ff7f0e"),
                   ("TakeoverRequested", "Takeover requested", "#d62728")]

TOPICS = ["/mission_state", "/rosout", "/blueye/odom", "/blueye/waypoint_goal", "/blueye/battery",
          "/blueye/takeover_request/urgency", "/blueye/takeover_request/threshold",
          "/blueye/takeover_request/evidence", "/blueye/takeover_request/mode",
          "/blueye/human/fatigue", "/blueye/human/stress", "/blueye/docking_controller_status",
          "/blueye/takeover_request/pending", "/blueye/takeover_request/human_decision",
          "/blueye/takeover_request/handback", "/blueye/current", "/scenario/events"]

plt.rcParams.update({"font.size": 8, "axes.labelsize": 8, "axes.titlesize": 8, "legend.fontsize": 7,
                     "xtick.labelsize": 7, "ytick.labelsize": 7, "pdf.fonttype": 42, "ps.fonttype": 42})


def severity_colours(n):
    cmap = plt.get_cmap("RdYlGn_r")
    return [cmap(0.08 + 0.84 * i / max(n - 1, 1)) for i in range(n)]


# ── Data ────────────────────────────────────────────────────────────────────────

def load_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    topics = [t for t in TOPICS if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    classes = {t: get_message(types[t]) for t in topics}
    data = {t: [] for t in TOPICS}
    t0 = None
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        t0 = t0 if t0 is not None else stamp
        ts = (stamp - t0) / 1e9
        msg = deserialize_message(raw, classes[topic])
        if topic == "/blueye/odom":
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            data[topic].append((ts, p.x, p.y, p.z, yaw))
        elif topic == "/blueye/waypoint_goal":
            data[topic].append((ts, msg.point.x, msg.point.y, msg.point.z))
        elif topic == "/blueye/takeover_request/evidence":
            data[topic].append((ts, json.loads(msg.data)))
        elif topic == "/blueye/battery":
            data[topic].append((ts, msg.state_of_charge if hasattr(msg, "state_of_charge") else msg.percentage))
        elif topic == "/rosout":
            data[topic].append((ts, msg.name, msg.msg))
        else:
            data[topic].append((ts, msg.data))
    return data


def mission_window(data):
    starts = [t for t, v in data["/mission_state"] if v > 0]
    start = starts[0] if starts else 0.0
    finished = [(t, m) for t, n, m in data["/rosout"] if "Mission finished" in m]
    end = finished[0][0] if finished else max(t for t, *_ in data["/blueye/odom"])
    result = finished[0][1].split(":")[-1].strip() if finished else "not finished in the recording"
    return start, end, result


def phase_segments(data, start, end):
    segs, current, seg_start = [], None, start
    for t, v in data["/mission_state"]:
        if t < start or v == 0:
            continue
        if t > end:
            break
        if v != current:
            if current is not None:
                segs.append((seg_start, t, current))
            current, seg_start = v, t
    if current is not None:
        segs.append((seg_start, end, current))
    # a mission resumed after a hand-back republishes the earlier phases within a moment: drop segments under 1 s
    kept = [sg for sg in segs if sg[1] - sg[0] >= 1.0] or segs
    merged = []
    for a, b, p in kept:
        if merged and merged[-1][2] == p:
            merged[-1] = (merged[-1][0], b, p)
        else:
            if merged:
                merged[-1] = (merged[-1][0], a, merged[-1][2])
            merged.append((a, b, p))
    return merged


def takeover_events(data, start, end):
    """[(time, label, colour)] of the operator events: takeover request (popup), accepted/rejected, hand-back."""
    events, prev = [], False
    for t, v in data.get("/blueye/takeover_request/pending", []):
        if v and not prev and start <= t <= end:
            events.append((t, "takeover request", "#d62728"))
        prev = v
    for t, v in data.get("/blueye/takeover_request/human_decision", []):
        if start <= t <= end:
            events.append((t, "accepted" if v else "rejected", "#1baf7a" if v else "#d62728"))
    for t, v in data.get("/blueye/takeover_request/handback", []):
        if v and start <= t <= end:
            events.append((t, "hand-back", "#2a78d6"))
    return sorted(events)


def disturbance_events(data, start, end):
    """[(time, label, colour, (speed, direction) or None)] of the injected disturbances: water current changes and USBL
    beacon switches, from /scenario/events or the sim_current / usbl_strength_monitor logs in /rosout."""
    import re
    raw = []
    if data.get("/scenario/events"):
        for t, msg in data["/scenario/events"]:
            e = json.loads(msg)
            if e.get("event") == "current":
                raw.append((t, "current", (e["speed_ms"], e["direction_deg"])))
            elif e.get("event") == "usbl_beacon":
                raw.append((t, "usbl", bool(e["enabled"])))
    else:
        for t, node, m in data["/rosout"]:
            hit = re.match(r"Current set to ([\d.]+) m/s towards ([\d.]+) deg", m) if node == "sim_current" else None
            if hit:
                raw.append((t, "current", (float(hit[1]), float(hit[2]))))
            elif node == "usbl_strength_monitor" and m.startswith("USBL beacon"):
                raw.append((t, "usbl", m.split(":")[0].endswith("ON")))
    out = []
    for t, kind, v in sorted(raw, key=lambda e: e[0]):
        if not start <= t <= end:
            continue
        if kind == "current":
            out.append((t, f"current {v[0]:.2f} m/s to {v[1]:.0f}\u00b0" if v[0] > 0 else "calm water", "#52514e", v))
        else:
            out.append((t, "USBL back" if v else "USBL cut", "#9467bd", None))
    return out


def mark_disturbances(ax, disturbances, start, line=1.13):
    for t, label, colour, _ in disturbances:
        ax.axvline(t - start, color=colour, lw=0.8, ls=(0, (1, 2)))
    if line is not None and disturbances:
        text = "Disturbances: " + ", ".join(f"{label} {t - start:.0f} s" for t, label, _, _ in disturbances)
        ax.text(0.0, line, text, ha="left", va="bottom", fontsize=6.3, color="#52514e", transform=ax.transAxes)


def mark_events(ax, events, start, labels=False):
    """Vertical line per event; with labels, events closer than 10 s share one label."""
    for t, label, colour in events:
        ax.axvline(t - start, color=colour, lw=0.9, ls="-.")
    if not labels or not events:
        return
    # one line above the panel: "Takeover: request 129 s, accepted 133 s, hand-back 159 s"
    text = "Takeover: " + ", ".join(f"{label.replace('takeover ', '')} {t - start:.0f} s" for t, label, _ in events)
    ax.text(events[0][0] - start, 1.13, text, ha="left", va="bottom", fontsize=6.5, color="#d62728",
            transform=ax.get_xaxis_transform())


def phase_at(segs, t):
    for a, b, p in segs:
        if a <= t < b:
            return p
    return segs[-1][2] if segs and t >= segs[-1][1] else None


def count_alarms(samples, threshold):
    """Rising crossings of the threshold, re-armed once urgency falls below 0.8 x threshold (as the BN node)."""
    alarms, armed = [], True
    for t, v in samples:
        if armed and v >= threshold:
            alarms.append(t)
            armed = False
        elif not armed and v < 0.8 * threshold:
            armed = True
    return alarms


def rock_outline(config_path, meshes_dir, scale=0.15):
    """Horizontal outline (N, E) of the transit rock of the obstacle scenario (objects/transit_rock.scn)."""
    from scipy.spatial import ConvexHull
    with open(config_path) as f:
        config = yaml.safe_load(f)
    x, y, _ = (float(v) for v in str(config["transit_rock_position"]).split())
    yaw = float(str(config["transit_rock_orientation"]).split()[2])
    v = np.array([[float(t) for t in l.split()[1:3]] for l in open(os.path.join(meshes_dir, "rock.obj"))
                  if l.startswith("v ")]) * scale
    c, s = math.cos(yaw), math.sin(yaw)
    w = np.stack([x + c * v[:, 0] - s * v[:, 1], y + s * v[:, 0] + c * v[:, 1]], axis=1)
    return w[ConvexHull(w).vertices]


def structure_grid(config_path, meshes_dir, structures, resolution=0.05):
    with open(config_path) as f:
        config = yaml.safe_load(f)
    polygons = []
    for mesh, key in structures:
        x, y, _ = (float(v) for v in str(config[f"{key}_position"]).split())
        yaw = float(str(config[f"{key}_orientation"]).split()[2])
        vertices, faces = [], []
        with open(os.path.join(meshes_dir, f"{mesh}.obj")) as f:
            for line in f:
                if line.startswith("v "):
                    vertices.append([float(v) for v in line.split()[1:3]])
                elif line.startswith("f "):
                    faces.append([int(v.split("/")[0]) - 1 for v in line.split()[1:]])
        v = np.array(vertices)
        c, s = math.cos(yaw), math.sin(yaw)
        world = np.stack([x + c * v[:, 0] - s * v[:, 1], y + s * v[:, 0] + c * v[:, 1]], axis=1)
        polygons.append([world[face] for face in faces])
    grids = []
    for faces in polygons:
        pts = np.concatenate(faces)
        x0, y0 = pts.min(axis=0) - resolution
        nx, ny = ((pts.max(axis=0) + resolution - (x0, y0)) / resolution).astype(int) + 1
        grid = np.zeros((ny, nx), np.uint8)
        cv2.fillPoly(grid, [np.round((p - (x0, y0)) / resolution).astype(np.int32) for p in faces], 1)
        grids.append((x0, y0, resolution, grid))
    return grids


# ── Figures ─────────────────────────────────────────────────────────────────────

def shade_phases(ax, segs, start, labels=False):
    for a, b, p in segs:
        name, colour = PHASES.get(p, (str(p), "grey"))
        ax.axvspan(a - start, b - start, color=colour, alpha=0.10, lw=0)
        total = segs[-1][1] - segs[0][0]
        if labels and (b - a) >= 0.04 * total:          # very short phases keep their shading but no name
            ax.text((a + b) / 2 - start, 1.02, name, transform=ax.get_xaxis_transform(), ha="center",
                    va="bottom", fontsize=7, color=colour)


def fig_mission_map(data, segs, start, end, grids, out, obstacle=None, events=(), disturbances=()):
    odom = [o for o in data["/blueye/odom"] if start <= o[0] <= end]
    fig, ax = plt.subplots(figsize=(3.6, 3.5))
    for (x0, y0, res, grid), (colour, label) in zip(grids, [("#7f7f7f", "Docking station"), ("#8c564b", "Pipeline")]):
        ny, nx = grid.shape
        masked = np.ma.masked_equal(grid.astype(float), 0).T
        # grid rows are y (east), columns x (north): transposed, east is horizontal and north vertical
        ax.imshow(masked, origin="lower", extent=(y0, y0 + ny * res, x0, x0 + nx * res),
                  cmap=matplotlib.colors.ListedColormap([colour]), alpha=0.55, interpolation="nearest")
        ax.plot([], [], "s", color=colour, alpha=0.55, label=label)
    if obstacle is not None:
        ax.fill(obstacle[:, 1], obstacle[:, 0], color="#52514e", alpha=0.9, lw=0, zorder=3, label="Obstacle (rock)")
    pts = np.array([[o[2], o[1]] for o in odom])
    phase = [phase_at(segs, o[0]) for o in odom]
    lines = np.stack([pts[:-1], pts[1:]], axis=1)
    colours = [PHASES.get(p, ("", "grey"))[1] for p in phase[:-1]]
    ax.add_collection(LineCollection(lines, colors=colours, linewidths=1.4))
    goals, last = [], None
    for t, x, y, z in data["/blueye/waypoint_goal"]:
        if (round(x, 2), round(y, 2)) != last:
            goals.append((x, y))
            last = (round(x, 2), round(y, 2))
    # waypoints: the visit order inside a small circle drawn exactly on the waypoint, so labels cannot collide
    for i, (x, y) in enumerate(goals, 1):
        ax.plot(y, x, "o", mfc="white", mec="black", mew=0.8, ms=8.5, zorder=5)
        ax.text(y, x, str(i), ha="center", va="center", fontsize=5.5, fontweight="bold", zorder=6)
    ax.plot([], [], "o", mfc="white", mec="black", mew=0.8, ms=7, label="Waypoint (visit order)")
    ax.plot(pts[0, 0], pts[0, 1], "D", color="black", ms=4.5, zorder=6, label="Start")
    ax.plot(pts[-1, 0], pts[-1, 1], "s", mfc="none", mec="black", ms=6, zorder=6, label="End")
    ot = [o[0] for o in odom]
    currents = [(t, v) for t, label, colour, v in disturbances if v is not None and v[0] > 0]
    if currents:                         # where the water current was first switched on, in its direction
        o = odom[min(bisect.bisect_left(ot, currents[0][0]), len(odom) - 1)]
        top = max(currents, key=lambda c: c[1][0])[1]
        a = math.radians(currents[0][1][1])
        ax.annotate("", xy=(o[2] + 2.2 * math.sin(a), o[1] + 2.2 * math.cos(a)), xytext=(o[2], o[1]),
                    arrowprops=dict(arrowstyle="-|>", color="#52514e", lw=1.8, mutation_scale=11), zorder=8)
        ax.plot([], [], color="#52514e", lw=1.8, label=f"Water current (up to {top[0]:.2f} m/s)")
    for t, label, colour in events:
        if label in ("takeover request", "hand-back"):
            o = odom[min(bisect.bisect_left(ot, t), len(odom) - 1)]
            ax.plot(o[2], o[1], "v" if label == "takeover request" else "^", color=colour, ms=5.5, mec="white",
                    mew=0.5, zorder=7, label=label.capitalize())
    for p in sorted(set(phase) - {None}):
        ax.plot([], [], color=PHASES[p][1], lw=2, label=PHASES[p][0])
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_aspect("equal")
    ax.autoscale_view()
    # view: the track and the docking station (the pipeline continues beyond the view to the east)
    x0, y0, res, grid = grids[0]
    ny, nx = grid.shape
    east = [*pts[:, 0], y0, y0 + ny * res, *(g[1] for g in goals)]
    north = [*pts[:, 1], x0, x0 + nx * res, *(g[0] for g in goals)]
    ax.set_xlim(min(east) - 0.8, max(east) + 1.0)
    ax.set_ylim(min(north) - 0.8, max(north) + 0.8)
    ax.grid(alpha=0.3, lw=0.5)
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), frameon=False)
    save(fig, out, "fig1_mission_map")


def fig_timeline(data, segs, start, end, threshold, alarms, out, events=(), disturbances=()):
    urg = [(t - start, v) for t, v in data["/blueye/takeover_request/urgency"] if start <= t <= end]
    ev = [(t, e) for t, e in data["/blueye/takeover_request/evidence"] if start <= t <= end]
    fig, axes = plt.subplots(4, 1, figsize=(7.2, 6.6), sharex=True,
                             gridspec_kw={"height_ratios": [1.1, 1.0, 2.2, 0.8], "hspace": 0.22})
    a, b, c, d = axes
    for ax in axes:
        shade_phases(ax, segs, start, labels=ax is a)

    t, v = zip(*urg)
    peak = max(urg, key=lambda s: s[1])
    a.plot(t, v, color="black", lw=1.0, label=f"Urgency (max {peak[1]:.2f})")
    a.axhline(threshold, color="#d62728", ls="--", lw=1.0, label=f"Threshold {threshold:.2f}")
    for ta in alarms:
        a.axvline(ta - start, color="#d62728", lw=0.8)
    a.set_ylim(0, 1)
    a.set_ylabel("Urgency")
    a.legend(loc="upper right", ncol=2, frameon=False)
    a.text(0.005, 0.9, "(a)", transform=a.transAxes, fontweight="bold")

    times = [x - start for x, _ in ev]
    for key, label, colour in TAKEOVER_STATES:
        b.plot(times, [e["probabilities"][key] for _, e in ev], color=colour, lw=1.0, label=label)
    b.set_ylim(0, 1)
    b.set_ylabel("P(TakeoverRequest)")
    b.legend(loc="upper right", ncol=3, frameon=False)
    b.text(0.005, 0.9, "(b)", transform=b.transAxes, fontweight="bold")

    for row, (node, states) in enumerate(EVIDENCE_ROWS):
        colours = severity_colours(len(states))
        y = len(EVIDENCE_ROWS) - 1 - row
        for i, (te, e) in enumerate(ev):
            t1 = ev[i + 1][0] if i + 1 < len(ev) else end
            state = e["evidence"].get(node)
            colour = colours[states.index(state)] if state in states else "#e0e0e0"
            c.broken_barh([(te - start, t1 - te)], (y - 0.42, 0.84), facecolors=colour, lw=0)
    c.set_yticks(range(len(EVIDENCE_ROWS)))
    c.set_yticklabels([n for n, _ in reversed(EVIDENCE_ROWS)], fontsize=6.5)
    c.set_ylim(-0.6, len(EVIDENCE_ROWS) - 0.4)
    sev = severity_colours(3)
    c.legend(handles=[Patch(color=sev[0], label="nominal"), Patch(color=sev[1], label="degraded"),
                      Patch(color=sev[2], label="worst state"), Patch(color="#e0e0e0", label="not entered")],
             loc="lower right", bbox_to_anchor=(1.0, 0.99), ncol=4, frameon=False, borderaxespad=0.1)
    c.text(0.005, 1.02, "(c) evidence", transform=c.transAxes, fontweight="bold")

    for topic, label, colour in [("/blueye/human/fatigue", "Fatigue", "#8c564b"),
                                 ("/blueye/human/stress", "Stress", "#e377c2")]:
        s = [(t - start, v) for t, v in data[topic] if start <= t <= end]
        if s:
            d.plot(*zip(*s), color=colour, lw=1.0, label=label)
    d.set_ylim(0, 1)
    d.set_ylabel("Operator")
    d.legend(loc="upper right", ncol=2, frameon=False)
    d.text(0.005, 0.85, "(d)", transform=d.transAxes, fontweight="bold")
    d.set_xlabel("Mission time [s]")
    d.set_xlim(0, end - start)
    for ax in axes:
        mark_events(ax, events, start, labels=ax is a)
        mark_disturbances(ax, disturbances, start, line=(1.24 if events else 1.13) if ax is a else None)
    save(fig, out, "fig2_timeline")


def fig_hidden_nodes(data, segs, start, end, out, events=()):
    ev = [(t - start, e) for t, e in data["/blueye/takeover_request/evidence"] if start <= t <= end]
    times = [t for t, _ in ev]
    fig, axes = plt.subplots(6, 2, figsize=(7.2, 7.0), sharex=True, gridspec_kw={"hspace": 0.35, "wspace": 0.12})
    for col, nodes in enumerate([HIDDEN_SYSTEM, HIDDEN_HUMAN]):
        for row in range(6):
            ax = axes[row, col]
            if row >= len(nodes):
                ax.axis("off")
                continue
            node, states = nodes[row]
            probs = np.array([[e["posteriors"][node][s] for s in states] for _, e in ev]).T
            ax.stackplot(times, probs, colors=severity_colours(len(states)), alpha=0.85, lw=0)
            for a, _, _ in segs[1:]:
                ax.axvline(a - start, color="black", lw=0.6, ls=":")
            mark_events(ax, events, start)
            ax.set_ylim(0, 1)
            ax.set_yticks([0, 0.5, 1])
            ax.set_title(f"{node} ({states[0]} / {states[-1]})", loc="left", fontsize=7, pad=2)
            if col == 1:
                ax.set_yticklabels([])
    axes[5, 0].set_xlabel("Mission time [s]")
    axes[4, 1].set_xlabel("Mission time [s]")
    axes[4, 1].xaxis.set_tick_params(labelbottom=True)
    axes[0, 0].set_xlim(0, end - start)
    sev = severity_colours(3)
    states_legend = axes[5, 1].legend(
        handles=[Patch(color=sev[0], label="good"), Patch(color=sev[1], label="intermediate"),
                 Patch(color=sev[2], label="bad")],
        loc="center", bbox_to_anchor=(0.5, 0.42), frameon=False, ncol=3,
        title="State (posterior probability, stacked)")
    axes[5, 1].text(0.5, 0.02, "Dotted lines: phase changes\n" + " \u2192 ".join(PHASES[p][0] for _, _, p in segs)
                    + ("\nDash-dot lines: takeover request, accepted, hand-back" if events else ""),
                    transform=axes[5, 1].transAxes, ha="center", va="top", fontsize=7)
    save(fig, out, "fig3_hidden_nodes")


def save(fig, out, name):
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(out, f"{name}.{ext}"), dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"  {name}.pdf/.png")


# ── Tables ──────────────────────────────────────────────────────────────────────

def summaries(data, bag, segs, start, end, result, threshold, alarms, out, events=()):
    urg = [(t, v) for t, v in data["/blueye/takeover_request/urgency"] if start <= t <= end]
    ev = [(t, e) for t, e in data["/blueye/takeover_request/evidence"] if start <= t <= end]
    dt = np.median(np.diff([t for t, _ in urg])) if len(urg) > 1 else 0.5
    rows = []
    for a, b, p in segs:
        vals = [v for t, v in urg if a <= t < b]
        ptk = [e["probabilities"]["TakeoverRequested"] for t, e in ev if a <= t < b]
        rows.append({"phase": PHASES.get(p, (str(p),))[0], "start_s": round(a - start, 1), "end_s": round(b - start, 1),
                     "duration_s": round(b - a, 1),
                     "urgency_mean": round(float(np.mean(vals)), 3) if vals else "",
                     "urgency_max": round(float(np.max(vals)), 3) if vals else "",
                     "time_above_threshold_s": round(sum(dt for v in vals if v >= threshold), 1),
                     "alarms": sum(1 for t in alarms if a <= t < b),
                     "P_takeover_max": round(float(np.max(ptk)), 3) if ptk else ""})
    with open(os.path.join(out, "phase_summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(out, "phase_summary.md"), "w") as f:
        f.write("| Phase | Start [s] | Duration [s] | Mean urgency | Max urgency | Time ≥ threshold [s] | Alarms |\n")
        f.write("|---|---|---|---|---|---|---|\n")
        for r in rows:
            f.write(f"| {r['phase']} | {r['start_s']} | {r['duration_s']} | {r['urgency_mean']} | {r['urgency_max']} "
                    f"| {r['time_above_threshold_s']} | {r['alarms']} |\n")

    odom = np.array([o[1:4] for o in data["/blueye/odom"] if start <= o[0] <= end])
    distance = float(np.sum(np.linalg.norm(np.diff(odom[:, :2], axis=0), axis=1)))
    batt = [v for t, v in data["/blueye/battery"] if start <= t <= end]
    peak_t, peak_v = max(urg, key=lambda s: s[1])
    peak_ev = min(ev, key=lambda s: abs(s[0] - peak_t))[1]["evidence"]
    dock = [m for t, n, m in data["/rosout"] if "Docking complete" in m]
    pre = [v for t, v in data["/blueye/takeover_request/urgency"] if t < start]
    mode = next((v for t, v in data["/blueye/takeover_request/mode"]), "unknown")
    lines = [
        f"bag: {os.path.basename(os.path.normpath(bag))}",
        f"takeover mode: {mode}",
        f"mission result: {result}",
        f"docking: {dock[0] if dock else 'no docking_complete reported'}",
        f"analysis window: {start:.1f} s to {end:.1f} s of the bag ({end - start:.1f} s)",
        f"distance travelled: {distance:.1f} m",
        f"battery: {batt[0] * 100:.1f} % -> {batt[-1] * 100:.1f} %" if batt else "battery: not recorded",
        f"threshold: {threshold:.2f}",
        f"alarms (rising crossings, re-armed below 0.8 x threshold): {len(alarms)}",
        f"peak urgency: {peak_v:.3f} at {peak_t - start:.1f} s ({PHASES.get(phase_at(segs, peak_t), ('?',))[0]})",
        f"evidence at peak: {json.dumps(peak_ev)}",
        f"samples before mission start (excluded): {len(pre)}, max urgency {max(pre):.3f}" if pre else
        "samples before mission start (excluded): 0",
    ]
    if events:
        lines.append("operator events: " + "; ".join(f"{label} at {t - start:.1f} s" for t, label, _ in events))
        req = next((t for t, l, _ in events if l == "takeover request"), None)
        dec = next((t for t, l, _ in events if l in ("accepted", "rejected")), None)
        hb = next((t for t, l, _ in events if l == "hand-back"), None)
        if req is not None and dec is not None:
            lines.append(f"operator decision time: {dec - req:.1f} s")
        if dec is not None and hb is not None:
            lines.append(f"manual control (accepted to hand-back): {hb - dec:.1f} s")
    with open(os.path.join(out, "run_summary.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join("  " + l for l in lines))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag_path", help="rosbag2 directory")
    parser.add_argument("--out", default=None, help="output directory (default: src/plots/paper/<bag_name>/)")
    parser.add_argument("--config", default=None, help="scenario config with the structure poses "
                        "(default: stonefish_sim/config/blueye_config.yaml)")
    parser.add_argument("--obstacle-config", default=None, help="scenario config with transit_rock_position/"
                        "orientation (e.g. stonefish_sim/config/blueye_obstacle_config.yaml) to draw the rock")
    args = parser.parse_args()

    bag = os.path.abspath(args.bag_path)
    out = args.out or os.path.join(here, "paper", os.path.basename(os.path.normpath(bag)))
    os.makedirs(out, exist_ok=True)

    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory("stonefish_sim")
    config = args.config or os.path.join(share, "config", "blueye_config.yaml")
    grids = structure_grid(config, os.path.join(share, "data", "object_files"),
                           [("docking_station_rig", "docking_rig"), ("pipe2", "pipeline")])

    print(f"Reading {bag} ...")
    data = load_bag(bag)
    start, end, result = mission_window(data)
    segs = phase_segments(data, start, end)
    thr = data["/blueye/takeover_request/threshold"]
    threshold = float(thr[-1][1]) if thr else 0.65
    alarms = count_alarms([(t, v) for t, v in data["/blueye/takeover_request/urgency"] if start <= t <= end],
                          threshold)
    events = takeover_events(data, start, end)
    disturbances = disturbance_events(data, start, end)
    obstacle = rock_outline(args.obstacle_config, os.path.join(share, "data", "object_files")) \
        if args.obstacle_config else None
    print(f"Writing to {out}")
    fig_mission_map(data, segs, start, end, grids, out, obstacle, events, disturbances)
    fig_timeline(data, segs, start, end, threshold, alarms, out, events, disturbances)
    fig_hidden_nodes(data, segs, start, end, out, events)
    summaries(data, bag, segs, start, end, result, threshold, alarms, out, events)


if __name__ == "__main__":
    main()
