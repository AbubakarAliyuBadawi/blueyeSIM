#!/usr/bin/env python3
"""
Plot real-world mission bag for journal paper figures.

Produces 4 figures:
  1. Top-down trajectory (lat/lon → metres), colour-coded by mission state
  2. Depth profile over time with state shading
  3. Control commands (surge/sway/yaw) with takeover windows
  4. Urgency vs threshold with takeover / handback event markers

Usage (source blueye workspace first):
  source ~/blueye_ws/install/setup.bash
  python3 src/plots/plot_real_mission.py ~/rosbags/takeover_real_20260713_143022

Output: PNG files saved to src/plots/<bag_name>/
"""

import os
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

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
    d['lat_long']      = read_topic(bag_path, '/blueye/sensor/ekf/lat_long',          'geometry_msgs/msg/Point')
    d['depth']         = read_topic(bag_path, '/blueye/sensor/depth',                  'blueye_interfaces/msg/FloatStamped')
    d['commands']      = read_topic(bag_path, '/blueye/commands',                      'geometry_msgs/msg/WrenchStamped')
    d['state']         = read_topic(bag_path, '/mission_state',                        'std_msgs/msg/Int32')
    d['urgency']       = read_topic(bag_path, '/blueye/takeover_request/urgency',      'std_msgs/msg/Float64')
    d['threshold']     = read_topic(bag_path, '/blueye/takeover_request/threshold',    'std_msgs/msg/Float32')
    d['human_decision']= read_topic(bag_path, '/blueye/takeover_request/human_decision','std_msgs/msg/Bool')
    d['handback']      = read_topic(bag_path, '/blueye/takeover_request/handback',     'std_msgs/msg/Bool')
    return d


# ── Helpers ───────────────────────────────────────────────────────────────────

def latlon_to_meters(lats, lons, ref_lat, ref_lon):
    R = 6371000.0
    x = np.radians(lons - ref_lon) * R * np.cos(np.radians(ref_lat))
    y = np.radians(lats - ref_lat) * R
    return x, y


def state_at(state_records, t):
    current = 0
    for ts, msg in state_records:
        if ts > t:
            break
        current = msg.data
    return current


# ── Plots ─────────────────────────────────────────────────────────────────────

def plot_trajectory(d, out_dir):
    if not d['lat_long']:
        print("  [skip] no lat_long data")
        return

    ts  = np.array([r[0] for r in d['lat_long']])
    lat = np.array([r[1].x for r in d['lat_long']])
    lon = np.array([r[1].y for r in d['lat_long']])
    x, y = latlon_to_meters(lat, lon, lat[0], lon[0])

    states = [state_at(d['state'], t) for t in ts]
    fig, ax = plt.subplots(figsize=(7, 6))

    for i in range(len(x) - 1):
        colour = STATE_LABELS.get(states[i], ("?", "#aaaaaa"))[1]
        ax.plot(x[i:i+2], y[i:i+2], color=colour, linewidth=2)

    ax.plot(x[0],  y[0],  'go', markersize=10, zorder=5, label='Start (dock)')
    ax.plot(x[-1], y[-1], 'r^', markersize=10, zorder=5, label='End (dock)')

    takeover_times = [t for t, msg in d['state'] if msg.data == 7]
    handback_times = [t for t, msg in d['handback'] if msg.data]
    for t in takeover_times:
        idx = np.argmin(np.abs(ts - t))
        ax.plot(x[idx], y[idx], 'kv', markersize=10, zorder=6, label='Takeover')
    for t in handback_times:
        idx = np.argmin(np.abs(ts - t))
        ax.plot(x[idx], y[idx], 'k^', markersize=10, zorder=6, label='Handback')

    patches = [mpatches.Patch(color=c, label=f"{s}: {lbl}")
               for s, (lbl, c) in STATE_LABELS.items() if s > 0]
    ax.legend(handles=patches, fontsize=8, loc='best')
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title("Mission Trajectory")
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.5)

    path = os.path.join(out_dir, "trajectory.png")
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  Saved: {path}")


def plot_depth(d, out_dir):
    if not d['depth']:
        print("  [skip] no depth data")
        return

    ts     = np.array([r[0] for r in d['depth']])
    depth  = np.array([r[1].data for r in d['depth']])
    t0     = ts[0]
    ts_rel = ts - t0

    fig, ax = plt.subplots(figsize=(10, 4))

    if d['state']:
        prev_t, prev_s = d['state'][0][0], d['state'][0][1].data
        for t, msg in d['state'][1:]:
            colour = STATE_LABELS.get(prev_s, ("?", "#aaaaaa"))[1]
            ax.axvspan(prev_t - t0, t - t0, alpha=0.15, color=colour)
            prev_t, prev_s = t, msg.data
        ax.axvspan(prev_t - t0, ts_rel[-1], alpha=0.15,
                   color=STATE_LABELS.get(prev_s, ("?", "#aaaaaa"))[1])

    ax.plot(ts_rel, depth, 'b-', linewidth=1.5, label='Depth (m)')
    ax.invert_yaxis()
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Depth (m)")
    ax.set_title("Depth Profile")
    ax.grid(True, linestyle='--', alpha=0.5)

    patches = [mpatches.Patch(color=c, alpha=0.4, label=lbl)
               for s, (lbl, c) in STATE_LABELS.items() if s > 0]
    ax.legend(handles=patches, fontsize=8, loc='lower right')

    path = os.path.join(out_dir, "depth_profile.png")
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  Saved: {path}")


def plot_commands(d, out_dir):
    if not d['commands']:
        print("  [skip] no commands data")
        return

    ts     = np.array([r[0] for r in d['commands']])
    surge  = np.array([r[1].wrench.force.x  for r in d['commands']])
    sway   = np.array([r[1].wrench.force.y  for r in d['commands']])
    yaw    = np.array([r[1].wrench.torque.z for r in d['commands']])
    t0     = ts[0]
    ts_rel = ts - t0

    takeover_spans = []
    if d['state']:
        in_to = False
        t_start = None
        for t, msg in d['state']:
            if msg.data == 7 and not in_to:
                in_to = True
                t_start = t - t0
            elif msg.data != 7 and in_to:
                in_to = False
                takeover_spans.append((t_start, t - t0))
        if in_to:
            takeover_spans.append((t_start, ts_rel[-1]))

    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for ax, signal, label, colour in zip(
            axes,
            [surge, sway, yaw],
            ['Surge (force.x)', 'Sway (force.y)', 'Yaw (torque.z)'],
            ['tab:blue', 'tab:orange', 'tab:green']):
        for t_a, t_b in takeover_spans:
            ax.axvspan(t_a, t_b, color='#e67e22', alpha=0.2,
                       label='Takeover' if t_a == takeover_spans[0][0] else '')
        ax.plot(ts_rel, signal, color=colour, linewidth=1)
        ax.axhline(0, color='k', linewidth=0.5)
        ax.set_ylabel(label)
        ax.set_ylim(-1.1, 1.1)
        ax.grid(True, linestyle='--', alpha=0.5)
        if takeover_spans:
            ax.legend(fontsize=8, loc='upper right')

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Control Commands (/blueye/commands)")

    path = os.path.join(out_dir, "control_commands.png")
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  Saved: {path}")


def plot_takeover(d, out_dir):
    if not d['urgency']:
        print("  [skip] no urgency data")
        return

    ts_u    = np.array([r[0] for r in d['urgency']])
    urgency = np.array([r[1].data for r in d['urgency']])
    t0      = ts_u[0]

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts_u - t0, urgency, 'r-', linewidth=1.5, label='Urgency')

    if d['threshold']:
        thr = np.mean([r[1].data for r in d['threshold']])
        ax.axhline(thr, color='k', linestyle='--', linewidth=1.5,
                   label=f'Threshold ({thr:.2f})')

    for t, msg in d['state']:
        if msg.data == 7:
            ax.axvline(t - t0, color='#e67e22', linestyle=':', linewidth=1.5,
                       label='Takeover triggered')
            break

    for t, msg in d['human_decision']:
        label  = 'Accept' if msg.data else 'Reject'
        colour = 'green'  if msg.data else 'red'
        ax.axvline(t - t0, color=colour, linestyle='-', linewidth=2, label=label)

    for t, msg in d['handback']:
        if msg.data:
            ax.axvline(t - t0, color='blue', linestyle='-.', linewidth=2, label='Handback')

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Urgency")
    ax.set_title("Takeover Urgency and Events")
    ax.set_ylim(0, 1.05)
    ax.grid(True, linestyle='--', alpha=0.5)

    handles, labels = ax.get_legend_handles_labels()
    seen = {}
    for h, l in zip(handles, labels):
        seen.setdefault(l, h)
    ax.legend(seen.values(), seen.keys(), fontsize=8, loc='upper left')

    path = os.path.join(out_dir, "takeover_events.png")
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  Saved: {path}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Plot real-world mission rosbag for journal paper")
    parser.add_argument("bag_path", help="Path to rosbag2 directory")
    plots_dir = os.path.dirname(os.path.abspath(__file__))
    parser.add_argument("--out", default=None,
                        help="Output directory for PNGs (default: src/plots/<bag_name>/)")
    args = parser.parse_args()

    bag_path = os.path.abspath(args.bag_path)
    bag_name = os.path.basename(bag_path.rstrip('/'))
    out_dir  = args.out or os.path.join(plots_dir, bag_name)
    os.makedirs(out_dir, exist_ok=True)
    print(f"Output folder: {out_dir}")

    if not rclpy.ok():
        rclpy.init()

    data = read_all(bag_path)

    print("Generating plots...")
    plot_trajectory(data, out_dir)
    plot_depth(data, out_dir)
    plot_commands(data, out_dir)
    plot_takeover(data, out_dir)

    print("Done. Files saved to:", out_dir)


if __name__ == "__main__":
    main()
