#!/usr/bin/env python3
"""
Interactive 3D trajectory viewer.

Usage (source workspace first):
  source ~/Desktop/auto-pilot/install/setup.bash
  python3 src/plots/view_3d_map.py ~/rosbags/<bag_dir>

Click and drag to rotate. Scroll to zoom. Right-drag to pan.
"""

import sys
import numpy as np
import matplotlib
matplotlib.use('TkAgg')          # interactive window; fall back to Qt5Agg if needed
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

# ── State colours ─────────────────────────────────────────────────────────────
STATE_LABELS = {
    0: ("IDLE",       "#aaaaaa"),
    1: ("UNDOCKING",  "#1abc9c"),
    2: ("TRANSIT",    "#2980b9"),
    3: ("INSPECTION", "#27ae60"),
    4: ("HOMING",     "#e74c3c"),
    5: ("DOCKING",    "#8e44ad"),
    6: ("TAKEOVER",   "#e67e22"),
}


def read_topic(bag_path, topic, msg_type_str):
    storage = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    conv    = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    reader  = rosbag2_py.SequentialReader()
    reader.open(storage, conv)

    available = {info.name: info.type for info in reader.get_all_topics_and_types()}
    if topic not in available:
        print(f"  [warn] topic not in bag: {topic}")
        return []

    msg_type = get_message(msg_type_str)
    records  = []
    while reader.has_next():
        t_name, raw, t_ns = reader.read_next()
        if t_name == topic:
            records.append((t_ns * 1e-9, deserialize_message(raw, msg_type)))
    return records


def state_at(state_records, t):
    current = 0
    for ts, msg in state_records:
        if ts > t:
            break
        current = msg.data
    return current


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 view_3d_map.py <bag_directory>")
        sys.exit(1)

    bag_path = sys.argv[1]
    # Accept either the bag directory or the .db3 file inside it
    if bag_path.endswith('.db3'):
        import os
        bag_path = os.path.dirname(bag_path)
    print("Reading bag:", bag_path)

    rclpy.init()

    odom  = read_topic(bag_path, '/blueye/odom',   'nav_msgs/msg/Odometry')
    if not odom:
        odom = read_topic(bag_path, '/odometry/filtered', 'nav_msgs/msg/Odometry')
    state = read_topic(bag_path, '/mission_state', 'std_msgs/msg/Int32')

    if not odom:
        print("No odometry found — cannot plot.")
        rclpy.shutdown()
        sys.exit(1)

    x = np.array([r[1].pose.pose.position.x for r in odom])
    y = np.array([r[1].pose.pose.position.y for r in odom])
    z = np.array([r[1].pose.pose.position.z for r in odom])
    t = np.array([r[0] for r in odom])

    rclpy.shutdown()

    # ── Build figure ──────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(11, 8))
    ax  = fig.add_subplot(111, projection='3d')

    # Draw trajectory segments coloured by mission state
    for i in range(len(x) - 1):
        s      = state_at(state, t[i]) if state else 0
        colour = STATE_LABELS.get(s, ("?", "#aaaaaa"))[1]
        ax.plot(x[i:i+2], y[i:i+2], z[i:i+2], color=colour, linewidth=1.8)

    # Start / end markers
    ax.scatter([x[0]],  [y[0]],  [z[0]],  color='lime',  s=60, zorder=5, label='Start')
    ax.scatter([x[-1]], [y[-1]], [z[-1]], color='red',   s=60, zorder=5, label='End')

    # Legend
    patches = [mpatches.Patch(color=c, label=f"{s}: {lbl}")
               for s, (lbl, c) in STATE_LABELS.items() if s > 0]
    patches += [mpatches.Patch(color='lime', label='Start'),
                mpatches.Patch(color='red',  label='End')]
    ax.legend(handles=patches, fontsize=8, loc='upper left')

    ax.set_xlabel("X — North (m)")
    ax.set_ylabel("Y — East (m)")
    ax.set_zlabel("Z — Depth (m)")
    ax.invert_zaxis()
    ax.set_title("3D Mission Trajectory\n(drag to rotate · scroll to zoom · right-drag to pan)")

    ax.grid(True, linestyle='--', alpha=0.4)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
