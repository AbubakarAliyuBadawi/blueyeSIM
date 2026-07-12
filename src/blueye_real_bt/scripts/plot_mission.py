#!/usr/bin/env python3
"""
Plot full mission bag for journal paper figures.

Produces 4 figures:
  1. Top-down trajectory (lat/lon), colour-coded by mission state
  2. Depth profile over time with state shading
  3. Control commands (surge/sway/yaw) with takeover window highlighted
  4. Urgency vs threshold with takeover / handback event markers

Usage (source your workspace first):
  source ~/blueye_ws/install/setup.bash
  python3 plot_mission.py ~/rosbags/full_mission_20260624_xxxxxx

Output: PNG files saved next to the bag directory.
"""

import sys
import os
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.collections import LineCollection

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

# ── State labels and colours ────────────────────────────────────────────────
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


# ── Bag reader helpers ───────────────────────────────────────────────────────

def open_bag(bag_path: str):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def read_topic(bag_path: str, topic: str, msg_type_str: str):
    """Return list of (timestamp_sec, msg) for one topic."""
    reader = open_bag(bag_path)
    msg_type = get_message(msg_type_str)
    records = []
    topic_types = {info.name: info.type for info in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        print(f"  [warn] topic not found in bag: {topic}")
        return records
    while reader.has_next():
        t_name, raw, t_ns = reader.read_next()
        if t_name == topic:
            msg = deserialize_message(raw, msg_type)
            records.append((t_ns * 1e-9, msg))
    return records


def read_all(bag_path: str):
    """Read every topic we care about. Returns dict of lists."""
    print("Reading bag:", bag_path)
    data = {}

    data['lat_long'] = read_topic(bag_path,
        '/blueye/sensor/ekf/lat_long', 'geometry_msgs/msg/Point')

    data['depth'] = read_topic(bag_path,
        '/blueye/sensor/depth', 'blueye_interfaces/msg/FloatStamped')

    data['commands'] = read_topic(bag_path,
        '/blueye/commands', 'geometry_msgs/msg/WrenchStamped')

    data['state'] = read_topic(bag_path,
        '/mission_state', 'std_msgs/msg/Int32')

    data['urgency'] = read_topic(bag_path,
        '/blueye/takeover_request/urgency', 'std_msgs/msg/Float64')

    data['threshold'] = read_topic(bag_path,
        '/blueye/takeover_request/threshold', 'std_msgs/msg/Float32')

    data['human_decision'] = read_topic(bag_path,
        '/blueye/takeover_request/human_decision', 'std_msgs/msg/Bool')

    data['handback'] = read_topic(bag_path,
        '/blueye/takeover_request/handback', 'std_msgs/msg/Bool')

    return data


# ── Helpers ──────────────────────────────────────────────────────────────────

def latlon_to_meters(lats, lons, ref_lat, ref_lon):
    """Convert lat/lon arrays to local ENU metres using equirectangular approx."""
    R = 6371000.0
    x = np.radians(lons - ref_lon) * R * np.cos(np.radians(ref_lat))
    y = np.radians(lats - ref_lat) * R
    return x, y


def state_at(state_records, t):
    """Return the mission state integer active at time t."""
    current = 0
    for ts, msg in state_records:
        if ts > t:
            break
        current = msg.data
    return current


def state_colour_array(ts, state_records):
    """Return array of hex colour strings matching the state at each timestamp."""
    return [STATE_LABELS.get(state_at(state_records, t), ("?", "#aaaaaa"))[1] for t in ts]


# ── Plots ────────────────────────────────────────────────────────────────────

def plot_trajectory(data, out_dir):
    if not data['lat_long']:
        print("  [skip] no lat_long data")
        return

    ts  = np.array([r[0] for r in data['lat_long']])
    lat = np.array([r[1].x for r in data['lat_long']])
    lon = np.array([r[1].y for r in data['lat_long']])

    ref_lat, ref_lon = lat[0], lon[0]
    x, y = latlon_to_meters(lat, lon, ref_lat, ref_lon)

    # Build coloured line segments by state
    states = [state_at(data['state'], t) for t in ts]
    fig, ax = plt.subplots(figsize=(7, 6))

    for i in range(len(x) - 1):
        colour = STATE_LABELS.get(states[i], ("?", "#aaaaaa"))[1]
        ax.plot(x[i:i+2], y[i:i+2], color=colour, linewidth=2)

    # Start / end markers
    ax.plot(x[0],  y[0],  'go', markersize=10, zorder=5, label='Start (dock)')
    ax.plot(x[-1], y[-1], 'r^', markersize=10, zorder=5, label='End (dock)')

    # Takeover event markers
    takeover_times = [t for t, msg in data['state'] if msg.data == 7]
    handback_times = [t for t, msg in data['handback'] if msg.data]
    for t in takeover_times:
        idx = np.argmin(np.abs(ts - t))
        ax.plot(x[idx], y[idx], 'kv', markersize=10, zorder=6, label='Takeover')
    for t in handback_times:
        idx = np.argmin(np.abs(ts - t))
        ax.plot(x[idx], y[idx], 'k^', markersize=10, zorder=6, label='Handback')

    # Legend for states
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


def plot_depth(data, out_dir):
    if not data['depth']:
        print("  [skip] no depth data")
        return

    ts    = np.array([r[0] for r in data['depth']])
    depth = np.array([r[1].data for r in data['depth']])
    t0    = ts[0]
    ts_rel = ts - t0

    fig, ax = plt.subplots(figsize=(10, 4))

    # State shading
    if data['state']:
        prev_t, prev_s = data['state'][0][0], data['state'][0][1].data
        for t, msg in data['state'][1:]:
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

    # State legend
    patches = [mpatches.Patch(color=c, alpha=0.4, label=lbl)
               for s, (lbl, c) in STATE_LABELS.items() if s > 0]
    ax.legend(handles=patches, fontsize=8, loc='lower right')

    path = os.path.join(out_dir, "depth_profile.png")
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  Saved: {path}")


def plot_commands(data, out_dir):
    if not data['commands']:
        print("  [skip] no commands data")
        return

    ts     = np.array([r[0] for r in data['commands']])
    surge  = np.array([r[1].wrench.force.x  for r in data['commands']])
    sway   = np.array([r[1].wrench.force.y  for r in data['commands']])
    yaw    = np.array([r[1].wrench.torque.z for r in data['commands']])
    t0     = ts[0]
    ts_rel = ts - t0

    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)

    # Takeover windows (state == 7)
    takeover_spans = []
    if data['state']:
        in_takeover = False
        t_start = None
        for t, msg in data['state']:
            if msg.data == 7 and not in_takeover:
                in_takeover = True
                t_start = t - t0
            elif msg.data != 7 and in_takeover:
                in_takeover = False
                takeover_spans.append((t_start, t - t0))
        if in_takeover:
            takeover_spans.append((t_start, ts_rel[-1]))

    for ax, signal, label, colour in zip(
            axes,
            [surge, sway, yaw],
            ['Surge (force.x)', 'Sway (force.y)', 'Yaw (torque.z)'],
            ['tab:blue', 'tab:orange', 'tab:green']):
        for t_a, t_b in takeover_spans:
            ax.axvspan(t_a, t_b, color='#e67e22', alpha=0.2, label='Takeover' if t_a == takeover_spans[0][0] else '')
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


def plot_takeover(data, out_dir):
    if not data['urgency']:
        print("  [skip] no urgency data")
        return

    ts_u   = np.array([r[0] for r in data['urgency']])
    urgency = np.array([r[1].data for r in data['urgency']])
    t0      = ts_u[0]

    fig, ax = plt.subplots(figsize=(10, 4))

    ax.plot(ts_u - t0, urgency, 'r-', linewidth=1.5, label='Urgency')

    # Threshold line
    if data['threshold']:
        thr_vals = [r[1].data for r in data['threshold']]
        ax.axhline(np.mean(thr_vals), color='k', linestyle='--',
                   linewidth=1.5, label=f'Threshold ({np.mean(thr_vals):.2f})')

    # Takeover trigger markers (urgency crosses threshold)
    for t, msg in data['state']:
        if msg.data == 7:
            ax.axvline(t - t0, color='#e67e22', linestyle=':', linewidth=1.5, label='Takeover triggered')
            break

    # Human decision markers
    for t, msg in data['human_decision']:
        label = 'Accept' if msg.data else 'Reject'
        colour = 'green' if msg.data else 'red'
        ax.axvline(t - t0, color=colour, linestyle='-', linewidth=2, label=label)

    # Handback markers
    for t, msg in data['handback']:
        if msg.data:
            ax.axvline(t - t0, color='blue', linestyle='-.', linewidth=2, label='Handback')

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Urgency")
    ax.set_title("Takeover Urgency and Events")
    ax.set_ylim(0, 1.05)
    ax.grid(True, linestyle='--', alpha=0.5)

    # Deduplicate legend
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


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Plot full mission rosbag for journal paper")
    parser.add_argument("bag_path", help="Path to rosbag2 directory")
    parser.add_argument("--out", default=None,
                        help="Output directory for PNGs (default: same as bag)")
    args = parser.parse_args()

    bag_path = os.path.abspath(args.bag_path)
    out_dir  = args.out or bag_path
    os.makedirs(out_dir, exist_ok=True)

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
