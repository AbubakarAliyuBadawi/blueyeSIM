#!/usr/bin/env python3
"""Check yaw sign consistency in a docking debug rosbag.

Usage:
  source /opt/ros/humble/setup.bash
  source ~/Desktop/auto-pilot/install/setup.bash
  python3 analyze_docking_yaw_sign.py /path/to/docking_debug_bag
"""

import math
import sys
from bisect import bisect_left
from collections import defaultdict

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def sign(value, threshold=1e-6):
    if value > threshold:
        return 1
    if value < -threshold:
        return -1
    return 0


def unwrap(samples):
    if not samples:
        return []
    output = []
    previous_raw = samples[0][1]
    offset = 0.0
    output.append((samples[0][0], previous_raw))
    for timestamp, raw in samples[1:]:
        delta = raw - previous_raw
        if delta > math.pi:
            offset -= 2.0 * math.pi
        elif delta < -math.pi:
            offset += 2.0 * math.pi
        output.append((timestamp, raw + offset))
        previous_raw = raw
    return output


def nearest(samples, timestamp):
    if not samples:
        return None
    times = [row[0] for row in samples]
    idx = bisect_left(times, timestamp)
    candidates = []
    for i in (idx - 1, idx, idx + 1):
        if 0 <= i < len(samples):
            candidates.append(samples[i])
    return min(candidates, key=lambda row: abs(row[0] - timestamp))


def rates_from_unwrapped(samples):
    rates = []
    for first, second in zip(samples, samples[1:]):
        dt = second[0] - first[0]
        if dt <= 1e-6:
            continue
        midpoint = 0.5 * (first[0] + second[0])
        rates.append((midpoint, (second[1] - first[1]) / dt))
    return rates


def read_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    message_types = {name: get_message(type_name) for name, type_name in topic_types.items()}

    data = defaultdict(list)
    start_time = None
    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()
        if start_time is None:
            start_time = timestamp
        t = (timestamp - start_time) * 1e-9
        msg = deserialize_message(serialized, message_types[topic])

        if topic == "/blueye/ref_vel":
            data["ref_yaw"].append((t, msg.twist.angular.z))
        elif topic == "/blueye/cmd_force":
            data["torque_z"].append((t, msg.wrench.torque.z))
        elif topic == "/blueye/odom":
            data["odom_yaw_rate"].append((t, msg.twist.twist.angular.z))
        elif topic == "/odometry/filtered":
            data["filtered_yaw_rate"].append((t, msg.twist.twist.angular.z))
        elif topic == "/blueye/pose_estimated_board_stamped":
            data["aruco_yaw"].append((t, yaw_from_quat(msg.pose.pose.orientation)))
        elif topic == "/blueye/current_reference":
            # This controller publishes yaw degrees in orientation.w, not a quaternion.
            data["reference_yaw"].append((t, math.radians(msg.orientation.w)))
        elif topic == "/blueye/docking_controller_status":
            data["status"].append((t, msg.data))

    for rows in data.values():
        rows.sort(key=lambda row: row[0])
    return data


def compare_signs(rows, left_name, right_name, threshold_left, threshold_right):
    considered = 0
    same = 0
    opposite = 0
    examples = []
    for t, left in rows[left_name]:
        if abs(left) < threshold_left:
            continue
        right_row = nearest(rows[right_name], t)
        if right_row is None:
            continue
        right = right_row[1]
        if abs(right) < threshold_right:
            continue
        considered += 1
        left_sign = sign(left)
        right_sign = sign(right)
        if left_sign == right_sign:
            same += 1
        else:
            opposite += 1
            if len(examples) < 5:
                examples.append((t, left, right))
    return considered, same, opposite, examples


def main():
    if len(sys.argv) != 2:
        print("Usage: analyze_docking_yaw_sign.py /path/to/docking_debug_bag")
        return 2

    bag_path = sys.argv[1]
    rows = read_bag(bag_path)
    rows["aruco_yaw_unwrapped"] = unwrap(rows["aruco_yaw"])
    rows["reference_yaw_unwrapped"] = unwrap(rows["reference_yaw"])
    rows["aruco_yaw_rate"] = rates_from_unwrapped(rows["aruco_yaw_unwrapped"])
    rows["reference_yaw_rate"] = rates_from_unwrapped(rows["reference_yaw_unwrapped"])

    print(f"Bag: {bag_path}")
    print("Samples:")
    for key in (
        "ref_yaw",
        "torque_z",
        "odom_yaw_rate",
        "filtered_yaw_rate",
        "aruco_yaw",
        "aruco_yaw_rate",
        "reference_yaw",
    ):
        print(f"  {key}: {len(rows[key])}")

    print("\nYaw sign checks:")
    checks = [
        ("ref_yaw", "torque_z", 0.03, 0.05),
        ("ref_yaw", "odom_yaw_rate", 0.03, 0.002),
        ("ref_yaw", "filtered_yaw_rate", 0.03, 0.002),
        ("ref_yaw", "aruco_yaw_rate", 0.03, 0.002),
        ("odom_yaw_rate", "aruco_yaw_rate", 0.002, 0.002),
    ]

    for left, right, left_threshold, right_threshold in checks:
        considered, same, opposite, examples = compare_signs(
            rows, left, right, left_threshold, right_threshold
        )
        if considered == 0:
            print(f"  {left} vs {right}: no useful samples")
            continue
        same_pct = 100.0 * same / considered
        opposite_pct = 100.0 * opposite / considered
        print(
            f"  {left} vs {right}: same={same}/{considered} ({same_pct:.1f}%), "
            f"opposite={opposite}/{considered} ({opposite_pct:.1f}%)"
        )
        for t, left_value, right_value in examples:
            print(f"    opposite example at {t:.2f}s: {left}={left_value:.4f}, {right}={right_value:.4f}")

    if rows["aruco_yaw"]:
        first = rows["aruco_yaw_unwrapped"][0]
        last = rows["aruco_yaw_unwrapped"][-1]
        print(
            "\nArUco yaw changed from "
            f"{math.degrees(first[1]):.1f} deg to {math.degrees(last[1]):.1f} deg"
        )
    if rows["reference_yaw"]:
        first = rows["reference_yaw_unwrapped"][0]
        last = rows["reference_yaw_unwrapped"][-1]
        print(
            "Reference yaw changed from "
            f"{math.degrees(first[1]):.1f} deg to {math.degrees(last[1]):.1f} deg"
        )

    print("\nInterpretation:")
    print("  ref_yaw vs torque_z should mostly be same sign.")
    print("  ref_yaw vs odom_yaw_rate should mostly be same sign.")
    print("  odom_yaw_rate vs aruco_yaw_rate should mostly be same sign.")
    print("  If odom_yaw_rate and aruco_yaw_rate are mostly opposite, the yaw frames disagree.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
