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
    0: ("IDLE",              "#aaaaaa"),
    1: ("UNDOCKING",         "#f39c12"),
    2: ("TRANSIT",           "#2980b9"),
    3: ("INSPECTION",        "#27ae60"),
    4: ("DOCKING",           "#8e44ad"),
    5: ("EMERGENCY ABORT",   "#c0392b"),
    6: ("EMERGENCY DOCKING", "#e74c3c"),
    7: ("TAKEOVER",          "#e67e22"),
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
        if msg.data == 7 and not in_to:
            in_to = True
            t_start = t - t0
        elif msg.data != 7 and in_to:
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
        if msg.data == 7:
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
        if msg.data == 7:
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


# ── Plot 3: Depth profile ─────────────────────────────────────────────────────

def plot_depth(d, out_dir):
    if not d['odom']:
        return

    ts    = np.array([r[0] for r in d['odom']])
    z     = np.array([r[1].pose.pose.position.z for r in d['odom']])
    t0    = ts[0]
    ts_r  = ts - t0

    fig, ax = plt.subplots(figsize=(10, 4))

    if d['state']:
        prev_t = d['state'][0][0]
        prev_s = d['state'][0][1].data
        for t, msg in d['state'][1:]:
            colour = STATE_LABELS.get(prev_s, ("?", "#aaaaaa"))[1]
            ax.axvspan(prev_t - t0, t - t0, alpha=0.15, color=colour)
            prev_t, prev_s = t, msg.data
        ax.axvspan(prev_t - t0, ts_r[-1], alpha=0.15,
                   color=STATE_LABELS.get(prev_s, ("?", "#aaaaaa"))[1])

    ax.plot(ts_r, z, 'b-', linewidth=1.5, label='Depth Z (m)')
    ax.invert_yaxis()
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Depth (m)")
    ax.set_title("Depth Profile")
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(handles=state_legend_patches(), fontsize=8, loc='lower right')

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
        if msg.data == 7:
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

    print("Done. Files saved to:", out_dir)


if __name__ == "__main__":
    main()
