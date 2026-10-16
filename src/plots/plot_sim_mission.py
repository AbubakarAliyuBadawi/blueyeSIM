#!/usr/bin/env python3
"""
Plot simulation mission bag for journal paper figures.

Produces 5 figures:
  1. 3D trajectory (x/y/z in local ENU frame), colour-coded by mission state
  2. Top-down 2D trajectory (x/y)
  3. Depth (z) profile over time with state shading
  4. Control commands (surge/sway/yaw) with takeover windows
  5. Bayesian network — urgency, probabilities, human model (fatigue/stress)

Usage (source workspace first):
  source ~/Desktop/auto-pilot/install/setup.bash
  python3 src/plots/plot_sim_mission.py ~/rosbags/sim_mission_20260624_xxxxxx

Output: PNG files saved inside the bag directory.
"""

import sys
import os
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

# ── State labels and colours ─────────────────────────────────────────────────
STATE_LABELS = {
    0: ("IDLE",       "#aaaaaa"),
    1: ("UNDOCKING",  "#1abc9c"),  # teal
    2: ("TRANSIT",    "#2980b9"),  # blue
    3: ("INSPECTION", "#27ae60"),  # green
    4: ("HOMING",     "#e74c3c"),  # red
    5: ("DOCKING",    "#8e44ad"),  # purple
    6: ("TAKEOVER",   "#e67e22"),  # orange
}


# ── Bag reader ────────────────────────────────────────────────────────────────

def read_topic(bag_path: str, topic: str, msg_type_str: str):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {info.name: info.type for info in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        print(f"  [warn] topic not in bag: {topic}")
        return []

    msg_type = get_message(msg_type_str)
    records = []
    while reader.has_next():
        t_name, raw, t_ns = reader.read_next()
        if t_name == topic:
            records.append((t_ns * 1e-9, deserialize_message(raw, msg_type)))
    return records


def read_all(bag_path: str):
    print("Reading bag:", bag_path)
    d = {}
    d['odom']        = read_topic(bag_path, '/odometry/filtered',  'nav_msgs/msg/Odometry')
    if not d['odom']:
        d['odom']    = read_topic(bag_path, '/blueye/odom',        'nav_msgs/msg/Odometry')

    d['state']       = read_topic(bag_path, '/mission_state',
                                  'std_msgs/msg/Int32')
    d['commands']    = read_topic(bag_path, '/blueye/cmd_force',
                                  'geometry_msgs/msg/WrenchStamped')
    d['urgency']     = read_topic(bag_path, '/blueye/takeover_request/urgency',
                                  'std_msgs/msg/Float64')
    d['threshold']   = read_topic(bag_path, '/blueye/takeover_request/threshold',
                                  'std_msgs/msg/Float32')
    d['human_dec']   = read_topic(bag_path, '/blueye/takeover_request/human_decision',
                                  'std_msgs/msg/Bool')
    d['handback']    = read_topic(bag_path, '/blueye/takeover_request/handback',
                                  'std_msgs/msg/Bool')
    d['prob_takeover']   = read_topic(bag_path,
                                      '/blueye/takeover_request/takeover_requested_prob',
                                      'std_msgs/msg/Float32')
    d['prob_no_takeover']= read_topic(bag_path,
                                      '/blueye/takeover_request/no_takeover_prob',
                                      'std_msgs/msg/Float32')
    d['prob_attention']  = read_topic(bag_path,
                                      '/blueye/takeover_request/attention_required_prob',
                                      'std_msgs/msg/Float32')
    d['fatigue']     = read_topic(bag_path, '/blueye/human/fatigue',
                                  'std_msgs/msg/Float32')
    d['stress']      = read_topic(bag_path, '/blueye/human/stress',
                                  'std_msgs/msg/Float32')
    d['thrusters']   = read_topic(bag_path, '/blueye/thrusters',
                                  'std_msgs/msg/Float64MultiArray')
    d['ref_vel']       = read_topic(bag_path, '/blueye/ref_vel',
                                    'geometry_msgs/msg/TwistStamped')
    d['waypoint_goal'] = read_topic(bag_path, '/blueye/waypoint_goal',
                                    'geometry_msgs/msg/PointStamped')
    return d


# ── Helpers ───────────────────────────────────────────────────────────────────

def state_at(state_records, t):
    current = 0
    for ts, msg in state_records:
        if ts > t:
            break
        current = msg.data
    return current


def takeover_spans(state_records, t0, t_end):
    spans = []
    in_to = False
    t_start = None
    for t, msg in state_records:
        if msg.data == 6 and not in_to:
            in_to = True
            t_start = t - t0
        elif msg.data != 6 and in_to:
            in_to = False
            spans.append((t_start, t - t0))
    if in_to:
        spans.append((t_start, t_end))
    return spans


def shade_spans(ax, spans, colour='#e67e22', alpha=0.2, label='Takeover'):
    for i, (a, b) in enumerate(spans):
        ax.axvspan(a, b, color=colour, alpha=alpha,
                   label=label if i == 0 else '')


def state_legend_patches():
    return [mpatches.Patch(color=c, label=f"{s}: {lbl}")
            for s, (lbl, c) in STATE_LABELS.items() if s > 0]


# ── Plot 1: 3D trajectory ─────────────────────────────────────────────────────

def plot_3d_trajectory(d, out_dir):
    if not d['odom']:
        print("  [skip] no odometry data")
        return

    ts = np.array([r[0] for r in d['odom']])
    x  = np.array([r[1].pose.pose.position.x for r in d['odom']])
    y  = np.array([r[1].pose.pose.position.y for r in d['odom']])
    z  = np.array([r[1].pose.pose.position.z for r in d['odom']])
    states = [state_at(d['state'], t) for t in ts]

    fig = plt.figure(figsize=(9, 7))
    ax  = fig.add_subplot(111, projection='3d')

    for i in range(len(x) - 1):
        colour = STATE_LABELS.get(states[i], ("?", "#aaaaaa"))[1]
        ax.plot(x[i:i+2], y[i:i+2], z[i:i+2], color=colour, linewidth=1.8)

    ax.scatter(x[0],  y[0],  z[0],  c='green',  s=80, zorder=5, label='Start')
    ax.scatter(x[-1], y[-1], z[-1], c='red',    s=80, marker='^', zorder=5, label='End')

    # Takeover event markers
    for t, msg in d['state']:
        if msg.data == 6:
            idx = np.argmin(np.abs(ts - t))
            ax.scatter(x[idx], y[idx], z[idx], c='black', s=100,
                       marker='v', zorder=6, label='Takeover')
            break
    for t, msg in d['handback']:
        if msg.data:
            idx = np.argmin(np.abs(ts - t))
            ax.scatter(x[idx], y[idx], z[idx], c='black', s=100,
                       marker='^', zorder=6, label='Handback')

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("3D Mission Trajectory")
    ax.legend(handles=state_legend_patches() + [
        mpatches.Patch(color='green', label='Start'),
        mpatches.Patch(color='red',   label='End'),
    ], fontsize=7, loc='upper left')

    path = os.path.join(out_dir, "trajectory_3d.png")
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  Saved: {path}")


# ── Plot 2: Top-down 2D trajectory ───────────────────────────────────────────

def plot_2d_trajectory(d, out_dir):
    if not d['odom']:
        return

    ts = np.array([r[0] for r in d['odom']])
    x  = np.array([r[1].pose.pose.position.x for r in d['odom']])
    y  = np.array([r[1].pose.pose.position.y for r in d['odom']])
    states = [state_at(d['state'], t) for t in ts]

    fig, ax = plt.subplots(figsize=(7, 6))
    for i in range(len(x) - 1):
        colour = STATE_LABELS.get(states[i], ("?", "#aaaaaa"))[1]
        ax.plot(x[i:i+2], y[i:i+2], color=colour, linewidth=2)

    ax.plot(x[0],  y[0],  'go', markersize=10, zorder=5, label='Start')
    ax.plot(x[-1], y[-1], 'r^', markersize=10, zorder=5, label='End')

    for t, msg in d['state']:
        if msg.data == 6:
            idx = np.argmin(np.abs(ts - t))
            ax.plot(x[idx], y[idx], 'kv', markersize=10, zorder=6, label='Takeover')
            break
    for t, msg in d['handback']:
        if msg.data:
            idx = np.argmin(np.abs(ts - t))
            ax.plot(x[idx], y[idx], 'k^', markersize=10, zorder=6, label='Handback')

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Top-Down Mission Trajectory")
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(handles=state_legend_patches(), fontsize=8, loc='best')

    path = os.path.join(out_dir, "trajectory_2d.png")
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  Saved: {path}")


# ── Plot 3: Depth profile + heave velocity ────────────────────────────────────

def _shade_states(ax, state_records, t0, t_end):
    if not state_records:
        return
    prev_t = state_records[0][0]
    prev_s = state_records[0][1].data
    for t, msg in state_records[1:]:
        colour = STATE_LABELS.get(prev_s, ("?", "#aaaaaa"))[1]
        ax.axvspan(prev_t - t0, t - t0, alpha=0.12, color=colour)
        prev_t, prev_s = t, msg.data
    ax.axvspan(prev_t - t0, t_end, alpha=0.12,
               color=STATE_LABELS.get(prev_s, ("?", "#aaaaaa"))[1])


def plot_depth(d, out_dir):
    if not d['odom']:
        return

    ts_o  = np.array([r[0] for r in d['odom']])
    z_act = np.array([r[1].pose.pose.position.z  for r in d['odom']])
    vz_act= np.array([r[1].twist.twist.linear.z  for r in d['odom']])
    t0    = ts_o[0]
    ts_or = ts_o - t0

    fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)

    # ── Panel 1: Z position ──
    ax = axes[0]
    _shade_states(ax, d['state'], t0, ts_or[-1])
    ax.plot(ts_or, z_act, color='#2980b9', linewidth=1.5, label='Depth Z (m)')
    ax.invert_yaxis()
    ax.set_ylabel("Depth (m)")
    ax.set_title("Depth Profile")
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(handles=state_legend_patches(), fontsize=8, loc='lower right')

    # ── Panel 2: Heave velocity actual vs commanded ──
    ax = axes[1]
    _shade_states(ax, d['state'], t0, ts_or[-1])
    win = max(1, len(vz_act) // 80)  # ~1.25 % of signal length
    vz_smooth = np.convolve(vz_act, np.ones(win) / win, mode='same')
    ax.plot(ts_or, vz_smooth, color='#2980b9', linewidth=1.2, label='Actual Vz (m/s, smoothed)')
    if d['ref_vel']:
        ts_r  = np.array([r[0] for r in d['ref_vel']])
        vz_cmd= np.array([r[1].twist.linear.z for r in d['ref_vel']])
        ax.plot(ts_r - t0, vz_cmd, color='k', linewidth=1.2,
                linestyle='--', label='Commanded Vz (m/s)', alpha=0.8)
    ax.axhline(0, color='gray', linewidth=0.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Heave Vz (m/s)")
    ax.set_title("Heave Velocity: Actual vs Commanded")
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(fontsize=8, loc='upper right')

    path = os.path.join(out_dir, "depth_profile.png")
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  Saved: {path}")


# ── Plot 4: Control commands ──────────────────────────────────────────────────

def plot_commands(d, out_dir):
    if not d['commands']:
        print("  [skip] no cmd_force data")
        return

    ts    = np.array([r[0] for r in d['commands']])
    surge = np.array([r[1].wrench.force.x  for r in d['commands']])
    sway  = np.array([r[1].wrench.force.y  for r in d['commands']])
    yaw   = np.array([r[1].wrench.torque.z for r in d['commands']])
    t0    = ts[0]
    ts_r  = ts - t0

    spans = takeover_spans(d['state'], t0, ts_r[-1])

    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for ax, sig, lbl, col in zip(
            axes,
            [surge, sway, yaw],
            ['Surge (force.x)', 'Sway (force.y)', 'Yaw (torque.z)'],
            ['tab:blue', 'tab:orange', 'tab:green']):
        shade_spans(ax, spans)
        ax.plot(ts_r, sig, color=col, linewidth=1)
        ax.axhline(0, color='k', linewidth=0.5)
        ax.set_ylabel(lbl)
        ax.grid(True, linestyle='--', alpha=0.5)
        if spans:
            ax.legend(fontsize=8, loc='upper right')

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Control Commands (/blueye/cmd_force)")
    path = os.path.join(out_dir, "control_commands.png")
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  Saved: {path}")


# ── Plot 5: Bayesian network + human model ────────────────────────────────────

def plot_bayesian(d, out_dir):
    if not d['urgency']:
        print("  [skip] no urgency data")
        return

    ts_u = np.array([r[0] for r in d['urgency']])
    t0   = ts_u[0]

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    # ── Subplot 1: Urgency vs threshold ──
    ax = axes[0]
    ax.plot(ts_u - t0, [r[1].data for r in d['urgency']],
            'r-', linewidth=1.5, label='Urgency')
    if d['threshold']:
        thr = np.mean([r[1].data for r in d['threshold']])
        ax.axhline(thr, color='k', linestyle='--', linewidth=1.5,
                   label=f'Threshold ({thr:.2f})')
    for t, msg in d['state']:
        if msg.data == 6:
            ax.axvline(t - t0, color='#e67e22', linestyle=':', linewidth=2,
                       label='Takeover triggered')
            break
    for t, msg in d['human_dec']:
        lbl = 'Accept' if msg.data else 'Reject'
        ax.axvline(t - t0, color='green' if msg.data else 'red',
                   linestyle='-', linewidth=2, label=lbl)
    for t, msg in d['handback']:
        if msg.data:
            ax.axvline(t - t0, color='blue', linestyle='-.', linewidth=2,
                       label='Handback')
    ax.set_ylabel("Urgency")
    ax.set_ylim(0, 1.05)
    ax.set_title("Takeover Urgency and Decision Events")
    ax.grid(True, linestyle='--', alpha=0.5)
    handles, labels = ax.get_legend_handles_labels()
    seen = {}
    for h, l in zip(handles, labels):
        seen.setdefault(l, h)
    ax.legend(seen.values(), seen.keys(), fontsize=8, loc='upper left')

    # ── Subplot 2: Bayesian probabilities ──
    ax = axes[1]
    for records, lbl, col in [
            (d['prob_takeover'],    'P(takeover)',          '#e74c3c'),
            (d['prob_no_takeover'], 'P(no takeover)',       '#27ae60'),
            (d['prob_attention'],   'P(attention required)','#f39c12'),
    ]:
        if records:
            ts_p = np.array([r[0] for r in records])
            ax.plot(ts_p - t0, [r[1].data for r in records],
                    color=col, linewidth=1.5, label=lbl)
    spans = takeover_spans(d['state'], t0, (ts_u[-1] - t0))
    shade_spans(ax, spans)
    ax.set_ylabel("Probability")
    ax.set_ylim(0, 1.05)
    ax.set_title("Bayesian Network Probabilities")
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(fontsize=8, loc='upper left')

    # ── Subplot 3: Human fatigue and stress ──
    ax = axes[2]
    for records, lbl, col in [
            (d['fatigue'], 'Fatigue', '#8e44ad'),
            (d['stress'],  'Stress',  '#c0392b'),
    ]:
        if records:
            ts_h = np.array([r[0] for r in records])
            ax.plot(ts_h - t0, [r[1].data for r in records],
                    color=col, linewidth=1.5, label=lbl)
    shade_spans(ax, spans)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Level")
    ax.set_ylim(0, 1.05)
    ax.set_title("Human Model (Fatigue / Stress)")
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(fontsize=8, loc='upper left')

    path = os.path.join(out_dir, "bayesian_takeover.png")
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  Saved: {path}")


# ── Plot 6: Thruster allocation ───────────────────────────────────────────────

def plot_thrusters(d, out_dir):
    if not d['thrusters']:
        print("  [skip] no thruster data")
        return

    ts   = np.array([r[0] for r in d['thrusters']])
    data = [list(r[1].data) for r in d['thrusters']]
    n_ch = max(len(row) for row in data)
    ch   = [np.array([row[i] if i < len(row) else 0.0 for row in data])
            for i in range(n_ch)]

    t0   = ts[0]
    ts_r = ts - t0
    spans = takeover_spans(d['state'], t0, ts_r[-1])

    # Yaw coupling: imbalance between the two horizontal thrusters
    T0 = ch[0] if n_ch > 0 else np.zeros_like(ts)
    T1 = ch[1] if n_ch > 1 else np.zeros_like(ts)

    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)

    # Raw thruster setpoints
    ax = axes[0]
    colours = ['#2980b9', '#e74c3c', '#27ae60', '#f39c12']
    labels  = ['T0 (front-left)', 'T1 (front-right)', 'T2 (rear-left)', 'T3 (rear-right)']
    for i in range(min(n_ch, 4)):
        ax.plot(ts_r, ch[i], color=colours[i], linewidth=1.2, label=labels[i])
    shade_spans(ax, spans)
    ax.axhline(0, color='k', linewidth=0.5)
    ax.set_ylabel("Setpoint")
    ax.set_title("Thruster Allocation (/blueye/thrusters)")
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(fontsize=8, loc='upper right')

    # T0 vs T1 (surge thrusters — imbalance = yaw coupling)
    ax = axes[1]
    shade_spans(ax, spans)
    ax.plot(ts_r, T0, color='#2980b9', linewidth=1.2, label='T0')
    ax.plot(ts_r, T1, color='#e74c3c', linewidth=1.2, label='T1')
    ax.axhline(0, color='k', linewidth=0.5)
    ax.set_ylabel("Setpoint")
    ax.set_title("T0 vs T1 (surge thrusters)")
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(fontsize=8, loc='upper right')

    # |T0 - T1| imbalance (yaw coupling indicator)
    ax = axes[2]
    imbalance = np.abs(T0 - T1)
    shade_spans(ax, spans)
    ax.fill_between(ts_r, imbalance, alpha=0.4, color='purple', label='|T0−T1|')
    ax.plot(ts_r, imbalance, color='purple', linewidth=1.0)
    ax.axhline(np.mean(imbalance), color='purple', linestyle='--', linewidth=1,
               label=f'mean={np.mean(imbalance):.2f}')
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("|T0−T1| (N)")
    ax.set_title("Thruster Imbalance (yaw coupling indicator)")
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(fontsize=8, loc='upper right')

    # Shade by mission state in background
    if d['state']:
        prev_t = d['state'][0][0]
        prev_s = d['state'][0][1].data
        for t, msg in d['state'][1:]:
            colour = STATE_LABELS.get(prev_s, ("?", "#aaaaaa"))[1]
            for ax in axes:
                ax.axvspan(prev_t - t0, t - t0, alpha=0.07, color=colour)
            prev_t, prev_s = t, msg.data
        for ax in axes:
            ax.axvspan(prev_t - t0, ts_r[-1], alpha=0.07,
                       color=STATE_LABELS.get(prev_s, ("?", "#aaaaaa"))[1])

    path = os.path.join(out_dir, "thruster_allocation.png")
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  Saved: {path}")


# ── Plot 7: Reference velocity (waypoint controller output) ───────────────────

def plot_ref_vel(d, out_dir):
    if not d['ref_vel']:
        print("  [skip] no ref_vel data")
        return

    ts    = np.array([r[0] for r in d['ref_vel']])
    vx    = np.array([r[1].twist.linear.x  for r in d['ref_vel']])
    vy    = np.array([r[1].twist.linear.y  for r in d['ref_vel']])
    wz    = np.array([r[1].twist.angular.z for r in d['ref_vel']])
    t0    = ts[0]
    ts_r  = ts - t0
    spans = takeover_spans(d['state'], t0, ts_r[-1])

    fig, axes = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
    for ax, sig, lbl, col in zip(
            axes,
            [vx, vy, wz],
            ['Surge ref Vx (m/s)', 'Sway ref Vy (m/s)', 'Yaw rate ref Wz (rad/s)'],
            ['tab:blue', 'tab:orange', 'tab:green']):
        shade_spans(ax, spans)
        ax.plot(ts_r, sig, color=col, linewidth=1.2)
        ax.axhline(0, color='k', linewidth=0.5)
        ax.set_ylabel(lbl)
        ax.grid(True, linestyle='--', alpha=0.5)

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Waypoint Controller Reference Velocity (/blueye/ref_vel)")

    path = os.path.join(out_dir, "reference_velocity.png")
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  Saved: {path}")


# ── Plot 8: Heading (yaw) over time ──────────────────────────────────────────

def plot_heading(d, out_dir):
    if not d['odom']:
        return

    ts_o  = np.array([r[0] for r in d['odom']])
    t0    = ts_o[0]
    ts_r  = ts_o - t0

    def q2yaw_deg(q):
        return np.degrees(np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    yaw   = np.array([q2yaw_deg(r[1].pose.pose.orientation) for r in d['odom']])
    wz_act= np.array([r[1].twist.twist.angular.z for r in d['odom']])

    spans = takeover_spans(d['state'], t0, ts_r[-1])

    fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)

    # ── Panel 1: Heading angle ──
    ax = axes[0]
    _shade_states(ax, d['state'], t0, ts_r[-1])
    shade_spans(ax, spans)
    ax.plot(ts_r, yaw, color='#2c3e50', linewidth=1.2, label='Heading (°)')
    ax.axhline(0, color='gray', linewidth=0.5, linestyle='--')
    ax.set_ylabel("Heading (°)")
    ax.set_title("Heading (Yaw) Over Time")
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(handles=state_legend_patches(), fontsize=8, loc='upper right')

    # ── Panel 2: Yaw rate actual vs commanded ──
    ax = axes[1]
    _shade_states(ax, d['state'], t0, ts_r[-1])
    shade_spans(ax, spans)
    ax.plot(ts_r, wz_act, color='#2c3e50', linewidth=1.0,
            label='Actual Wz (rad/s)', alpha=0.8)
    if d['ref_vel']:
        ts_rv = np.array([r[0] for r in d['ref_vel']])
        wz_cmd = np.array([r[1].twist.angular.z for r in d['ref_vel']])
        win = max(1, len(wz_cmd) // 80)
        wz_cmd = np.convolve(wz_cmd, np.ones(win) / win, mode='same')
        ax.plot(ts_rv - t0, wz_cmd, color='k', linewidth=1.2,
                linestyle='--', label='Commanded Wz (rad/s, smoothed)', alpha=0.8)
    ax.axhline(0, color='gray', linewidth=0.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Yaw rate (rad/s)")
    ax.set_title("Yaw Rate: Actual vs Commanded")
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(fontsize=8, loc='upper right')

    path = os.path.join(out_dir, "heading.png")
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  Saved: {path}")


# ── Plot 9: 5-panel position dashboard (state/North/East/Depth/Heading) ───────

def _q2yaw_rad(q):
    return np.arctan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _unwrap_heading(raw_yaw_rad):
    """Remove ±π wrap-around discontinuities from a yaw array (radians)."""
    yaw = [raw_yaw_rad[0]]
    for i in range(1, len(raw_yaw_rad)):
        diff = raw_yaw_rad[i] - yaw[-1]
        # Detect wrap: large jump that is close to 2π
        if abs(diff) > 3.0 and abs(abs(diff) - 2 * np.pi) < 0.5:
            diff -= np.sign(diff) * 2 * np.pi
        yaw.append(yaw[-1] + diff)
    return np.array(yaw)


def _desired_heading(d, ts_odom):
    """Integrate omega_z from ref_vel to build a desired heading signal
    sampled at the odom timestamps. Returns array in radians."""
    if not d['odom'] or not d['ref_vel']:
        return None

    # Seed from the first actual heading
    q0  = d['odom'][0][1].pose.pose.orientation
    psi = _q2yaw_rad(q0)

    rv_t   = np.array([r[0] for r in d['ref_vel']])
    rv_wz  = np.array([r[1].twist.angular.z for r in d['ref_vel']])

    # Integrate omega_z only when |wz| > 0.05 rad/s (ignore noise)
    psi_rv = [psi]
    for i in range(1, len(rv_t)):
        dt  = min(rv_t[i] - rv_t[i - 1], 0.5)
        wz  = rv_wz[i]
        if abs(wz) > 0.05:
            psi += wz * dt
            # keep in [-π, π]
            psi = ((psi + np.pi) % (2 * np.pi)) - np.pi
        psi_rv.append(psi)

    psi_rv = np.array(psi_rv)

    # Interpolate onto odom timestamps
    psi_interp = np.interp(ts_odom, rv_t, psi_rv)
    return psi_interp


def plot_north_east(d, out_dir):
    """5-panel dashboard: mission state, North, East, Depth, Heading over time."""
    if not d['odom']:
        return

    ts_o  = np.array([r[0] for r in d['odom']])
    t0    = ts_o[0]
    ts_r  = ts_o - t0

    # FIX 1 — North/East straight from odom (x=North, y=East in our ENU sim frame)
    x = np.array([r[1].pose.pose.position.x for r in d['odom']])
    y = np.array([r[1].pose.pose.position.y for r in d['odom']])

    # FIX 2 — Depth: odom Z is ENU (up=positive → underwater=negative).
    # Negate so depth is positive when the ROV is submerged, matching BT waypoint z.
    depth = -np.array([r[1].pose.pose.position.z for r in d['odom']])

    # FIX 3 — Heading: unwrap ±π discontinuities; keep in radians like reference
    raw_yaw = np.array([_q2yaw_rad(r[1].pose.pose.orientation) for r in d['odom']])
    yaw     = _unwrap_heading(raw_yaw)          # radians, continuous

    # FIX 4 — Desired heading by integrating omega_z from ref_vel
    yaw_d = _desired_heading(d, ts_o)            # radians, or None

    # Desired waypoint goals (step function — published once per new BT action)
    has_goal = bool(d.get('waypoint_goal'))
    if has_goal:
        ts_g = np.array([r[0] - t0 for r in d['waypoint_goal']])
        gx   = np.array([r[1].point.x for r in d['waypoint_goal']])
        gy   = np.array([r[1].point.y for r in d['waypoint_goal']])
        gz   = np.array([r[1].point.z for r in d['waypoint_goal']])
        # gz is already positive depth (BT z), consistent with negated odom depth

    # Build state step-series for the top panel
    if d['state']:
        st_t = np.array([r[0] - t0 for r in d['state']])
        st_v = np.array([r[1].data  for r in d['state']])
    else:
        st_t = np.array([0.0, ts_r[-1]])
        st_v = np.array([0, 0])

    fig, axes = plt.subplots(5, 1, figsize=(12, 14), sharex=True)
    fig.suptitle("Position Dashboard — North / East / Depth / Heading", fontsize=13)

    # ── Panel 1: Mission state ──
    ax = axes[0]
    ax.step(st_t, st_v, where='post', color='#2c3e50', linewidth=1.5)
    _shade_states(ax, d['state'], t0, ts_r[-1])
    ax.set_yticks(list(STATE_LABELS.keys()))
    ax.set_yticklabels([STATE_LABELS[s][0] for s in STATE_LABELS.keys()], fontsize=7)
    ax.set_ylabel("State")
    ax.set_title("Mission State")
    ax.grid(True, linestyle='--', alpha=0.3)
    ax.legend(handles=state_legend_patches(), fontsize=7, loc='upper left', ncol=3)

    # ── Panel 2: North (X) ──
    ax = axes[1]
    _shade_states(ax, d['state'], t0, ts_r[-1])
    ax.plot(ts_r, x, color='#2980b9', linewidth=1.4, label=r'$\hat{x}$ (m)')
    if has_goal:
        ax.step(ts_g, gx, where='post', color='#e74c3c', linewidth=1.2,
                linestyle='--', label=r'$x_d$ (m)')
    ax.set_ylabel("North X (m)")
    ax.set_title("Estimated and desired position in north direction over time")
    ax.grid(True, linestyle='--', alpha=0.4)
    ax.legend(fontsize=8, loc='upper left')

    # ── Panel 3: East (Y) ──
    ax = axes[2]
    _shade_states(ax, d['state'], t0, ts_r[-1])
    ax.plot(ts_r, y, color='#2980b9', linewidth=1.4, label=r'$\hat{y}$ (m)')
    if has_goal:
        ax.step(ts_g, gy, where='post', color='#e74c3c', linewidth=1.2,
                linestyle='--', label=r'$y_d$ (m)')
    ax.set_ylabel("East Y (m)")
    ax.set_title("Estimated and desired position in east direction over time")
    ax.grid(True, linestyle='--', alpha=0.4)
    ax.legend(fontsize=8, loc='upper left')

    # ── Panel 4: Depth (positive = submerged) ──
    ax = axes[3]
    _shade_states(ax, d['state'], t0, ts_r[-1])
    ax.plot(ts_r, depth, color='#2980b9', linewidth=1.4, label=r'$\hat{d}$ (m)')
    if has_goal:
        ax.step(ts_g, gz, where='post', color='#e74c3c', linewidth=1.2,
                linestyle='--', label=r'$d_d$ (m)')
    ax.set_ylabel("Depth (m)")
    ax.set_title("Estimated and desired depth over time")
    ax.grid(True, linestyle='--', alpha=0.4)
    ax.legend(fontsize=8, loc='upper left')

    # ── Panel 5: Heading (rad, unwrapped, with desired) ──
    ax = axes[4]
    _shade_states(ax, d['state'], t0, ts_r[-1])
    ax.plot(ts_r, yaw, color='#2980b9', linewidth=1.2, label=r'$\hat{\psi}$ (rad)')
    if yaw_d is not None:
        ax.plot(ts_r, yaw_d, color='#e74c3c', linewidth=1.2,
                linestyle='--', label=r'$\psi_d$ (rad)')
    ax.axhline(0, color='gray', linewidth=0.5, linestyle='--')
    ax.set_ylabel("Heading (rad)")
    ax.set_xlabel("Time (s)")
    ax.set_title("Estimated and desired heading over time")
    ax.grid(True, linestyle='--', alpha=0.4)
    ax.legend(fontsize=8, loc='upper left')

    fig.tight_layout()
    path = os.path.join(out_dir, "north_east.png")
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  Saved: {path}")


# ── Plot 10: Velocity tracking — actual vs commanded ─────────────────────────

def plot_velocity_tracking(d, out_dir):
    if not d['odom'] or not d['ref_vel']:
        print("  [skip] velocity tracking: missing odom or ref_vel")
        return

    ts_o   = np.array([r[0] for r in d['odom']])
    act_vx = np.array([r[1].twist.twist.linear.x  for r in d['odom']])
    act_vy = np.array([r[1].twist.twist.linear.y  for r in d['odom']])
    act_vz = np.array([r[1].twist.twist.linear.z  for r in d['odom']])
    act_wz = np.array([r[1].twist.twist.angular.z for r in d['odom']])

    ts_r   = np.array([r[0] for r in d['ref_vel']])
    cmd_vx = np.array([r[1].twist.linear.x  for r in d['ref_vel']])
    cmd_vy = np.array([r[1].twist.linear.y  for r in d['ref_vel']])
    cmd_vz = np.array([r[1].twist.linear.z  for r in d['ref_vel']])
    cmd_wz = np.array([r[1].twist.angular.z for r in d['ref_vel']])

    t0     = min(ts_o[0], ts_r[0])
    ts_or  = ts_o - t0
    ts_rr  = ts_r - t0
    spans  = takeover_spans(d['state'], t0, max(ts_or[-1], ts_rr[-1]))

    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
    rows = [
        (ts_or, act_vx, ts_rr, cmd_vx, 'Surge Vx (m/s)',     'tab:blue'),
        (ts_or, act_vy, ts_rr, cmd_vy, 'Sway Vy (m/s)',      'tab:orange'),
        (ts_or, act_vz, ts_rr, cmd_vz, 'Heave Vz (m/s)',     'tab:red'),
        (ts_or, act_wz, ts_rr, cmd_wz, 'Yaw rate Wz (rad/s)','tab:green'),
    ]
    for ax, (ts_a, act, ts_c, cmd, lbl, col) in zip(axes, rows):
        _shade_states(ax, d['state'], t0, ts_a[-1])
        shade_spans(ax, spans)
        ax.plot(ts_a, act, color=col, linewidth=1.0, label='Actual', alpha=0.85)
        ax.plot(ts_c, cmd, color='k',  linewidth=1.3, linestyle='--',
                label='Commanded', alpha=0.75)
        ax.axhline(0, color='gray', linewidth=0.5)
        ax.set_ylabel(lbl)
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.legend(fontsize=8, loc='upper right')

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Velocity Tracking: Actual vs Commanded")

    path = os.path.join(out_dir, "velocity_tracking.png")
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  Saved: {path}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Plot simulation mission rosbag for journal paper")
    parser.add_argument("bag_path", help="Path to rosbag2 directory")
    plots_dir = os.path.dirname(os.path.abspath(__file__))
    parser.add_argument("--out", default=None,
                        help="Output directory for PNGs (default: src/plots/<bag_name>/)")
    args = parser.parse_args()

    bag_path  = os.path.abspath(args.bag_path)
    bag_name  = os.path.basename(bag_path.rstrip('/'))
    out_dir   = args.out or os.path.join(plots_dir, bag_name)
    os.makedirs(out_dir, exist_ok=True)
    print(f"Output folder: {out_dir}")

    if not rclpy.ok():
        rclpy.init()

    data = read_all(bag_path)

    print("Generating plots...")
    plot_3d_trajectory(data, out_dir)
    plot_2d_trajectory(data, out_dir)
    plot_depth(data, out_dir)
    plot_commands(data, out_dir)
    plot_bayesian(data, out_dir)
    plot_thrusters(data, out_dir)
    plot_ref_vel(data, out_dir)
    plot_heading(data, out_dir)
    plot_north_east(data, out_dir)
    plot_velocity_tracking(data, out_dir)

    print("Done. Files saved to:", out_dir)


if __name__ == "__main__":
    main()
