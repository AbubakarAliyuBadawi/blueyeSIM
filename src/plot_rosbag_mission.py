#!/usr/bin/env python3
"""Create mission overview and trajectory plots from a ROS 2 bag.

The default topics match the Blueye/Stonefish simulation bags in this
workspace.  ``/blueye/odom`` is preferred over ``/odometry/filtered`` because
the former and ``/blueye/waypoint_goal`` are both expressed in ``world_ned``.

Examples
--------
    python3 plot_rosbag_mission.py \
        rosbags/takeover_test_6_20260829_012037

    python3 plot_rosbag_mission.py BAG.db3 --output-dir /tmp/mission_plots

Outputs
-------
    mission_overview.png
    trajectory_2d_state.png
    trajectory_3d_state.png
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

# Avoid warnings/failures when the normal Matplotlib cache is not writable.
os.environ.setdefault("MPLCONFIGDIR", f"/tmp/autopilot-matplotlib-{os.getuid()}")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:  # pragma: no cover - depends on the ROS environment
    raise SystemExit(
        "ROS 2 Python modules are unavailable. Source your ROS 2/workspace "
        "setup before running this script.\n"
        "Example: source /opt/ros/humble/setup.bash"
    ) from exc


STATE_STYLE = {
    0: ("Idle", "#9e9e9e"),
    1: ("Undocking", "#17a589"),
    2: ("Transit", "#2e86c1"),
    3: ("Pipeline inspection", "#28b463"),
    4: ("Homing", "#d35400"),
    5: ("Docking", "#8e44ad"),
    6: ("Takeover", "#c0392b"),
}

DEFAULT_ODOM_TOPICS = ("/blueye/odom", "/odometry/filtered")
DEFAULT_STATE_TOPIC = "/mission_state"
DEFAULT_GOAL_TOPIC = "/blueye/waypoint_goal"
DOCKING_ODOM_TOPIC = "/odometry/filtered"
DOCKING_REFERENCE_TOPIC = "/blueye/current_reference"

Record = Tuple[int, object]


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plot mission state, actual/desired north-east-depth-heading, "
            "and state-coloured 2D/3D ROV trajectories from a ROS 2 bag."
        )
    )
    parser.add_argument(
        "bag",
        type=Path,
        help="ROS 2 bag directory, metadata.yaml, or .db3 file",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Output directory (default: ./plots/<bag-name>)",
    )
    parser.add_argument(
        "--odom-topic",
        help="Odometry topic override (default: prefer /blueye/odom)",
    )
    parser.add_argument(
        "--state-topic",
        default=DEFAULT_STATE_TOPIC,
        help=f"Mission-state topic (default: {DEFAULT_STATE_TOPIC})",
    )
    parser.add_argument(
        "--goal-topic",
        default=DEFAULT_GOAL_TOPIC,
        help=f"Desired waypoint topic (default: {DEFAULT_GOAL_TOPIC})",
    )
    parser.add_argument(
        "--coordinates",
        choices=("auto", "ned", "enu"),
        default="auto",
        help=(
            "Coordinate convention. NED maps x/y/z to north/east/depth; "
            "ENU maps y/x/-z (default: infer from odometry frame)."
        ),
    )
    parser.add_argument(
        "--initial-goal",
        nargs=3,
        type=float,
        metavar=("NORTH", "EAST", "DEPTH"),
        help=(
            "Desired N/E/depth before the first recorded waypoint. If omitted, "
            "it is inferred from the ROV position when the first goal arrives."
        ),
    )
    parser.add_argument(
        "--initial-heading",
        type=float,
        help=(
            "Desired heading in radians before the first recorded waypoint. "
            "If omitted, the initial measured heading is used."
        ),
    )
    parser.add_argument("--dpi", type=int, default=180, help="PNG resolution (default: 180)")
    parser.add_argument(
        "--show",
        action="store_true",
        help="Also open the figures interactively after saving",
    )
    return parser.parse_args(argv)


def resolve_bag_directory(path: Path) -> Path:
    path = path.expanduser().resolve()
    if path.is_file() and path.name == "metadata.yaml":
        path = path.parent
    elif path.is_file() and path.suffix == ".db3":
        path = path.parent

    if not path.is_dir():
        raise FileNotFoundError(f"Bag directory does not exist: {path}")
    if not (path / "metadata.yaml").is_file():
        raise FileNotFoundError(f"metadata.yaml not found in bag directory: {path}")
    return path


def choose_topic(
    available: Dict[str, str], requested: Optional[str], candidates: Iterable[str], label: str
) -> str:
    if requested:
        if requested not in available:
            raise ValueError(
                f"Requested {label} topic {requested!r} is not in the bag. "
                f"Available topics:\n  " + "\n  ".join(sorted(available))
            )
        return requested
    for topic in candidates:
        if topic in available:
            return topic
    raise ValueError(
        f"No {label} topic found. Tried: {', '.join(candidates)}. "
        f"Use --{label}-topic to select one."
    )


def read_bag(
    bag_dir: Path, odom_override: Optional[str], state_topic: str, goal_topic: str
) -> Tuple[Dict[str, List[Record]], Dict[str, str], str]:
    """Read required topics in a single sequential pass through the bag."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    available = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    odom_topic = choose_topic(available, odom_override, DEFAULT_ODOM_TOPICS, "odom")

    if state_topic not in available:
        raise ValueError(f"Mission-state topic {state_topic!r} is not in the bag")
    if goal_topic not in available:
        print(
            f"Warning: desired-goal topic {goal_topic!r} is absent; "
            "desired position/heading curves will be omitted.",
            file=sys.stderr,
        )

    selected = [odom_topic, state_topic]
    if goal_topic in available:
        selected.append(goal_topic)
    # These two topics let us transform the docking controller's tag-frame
    # reference back into the mission's world NED frame.
    for optional_topic in (DOCKING_ODOM_TOPIC, DOCKING_REFERENCE_TOPIC):
        if optional_topic in available and optional_topic not in selected:
            selected.append(optional_topic)
    message_classes = {topic: get_message(available[topic]) for topic in selected}
    records: Dict[str, List[Record]] = {topic: [] for topic in selected}

    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        if topic in records:
            records[topic].append(
                (timestamp_ns, deserialize_message(raw, message_classes[topic]))
            )

    if not records[odom_topic]:
        raise ValueError(f"Odometry topic {odom_topic!r} contains no messages")
    if not records[state_topic]:
        raise ValueError(f"Mission-state topic {state_topic!r} contains no messages")
    return records, available, odom_topic


def infer_coordinates(requested: str, odom_topic: str, first_odom: object) -> str:
    if requested != "auto":
        return requested
    frame = str(getattr(first_odom.header, "frame_id", "")).lower()
    if "ned" in frame:
        return "ned"
    if "enu" in frame:
        return "enu"
    if odom_topic == "/blueye/odom":
        return "ned"
    print(
        f"Warning: could not infer axes from frame {frame!r}; assuming ENU. "
        "Use --coordinates ned or --coordinates enu if needed.",
        file=sys.stderr,
    )
    return "enu"


def quaternion_yaw(q: object) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def odom_to_ned(msg: object, convention: str) -> Tuple[float, float, float, float]:
    position = msg.pose.pose.position
    yaw = quaternion_yaw(msg.pose.pose.orientation)
    if convention == "ned":
        return position.x, position.y, position.z, yaw
    # ENU yaw is counter-clockwise from East. Heading is clockwise from North.
    heading = math.atan2(math.cos(yaw), math.sin(yaw))
    return position.y, position.x, -position.z, heading


def goal_to_ned(msg: object, convention: str) -> Tuple[float, float, float]:
    point = msg.point
    if convention == "ned":
        return point.x, point.y, point.z
    return point.y, point.x, -point.z


def compress_states(records: Sequence[Record]) -> Tuple[np.ndarray, np.ndarray]:
    times: List[int] = []
    values: List[int] = []
    for timestamp_ns, msg in records:
        state = int(msg.data)
        if not values or state != values[-1]:
            times.append(timestamp_ns)
            values.append(state)
    return np.asarray(times, dtype=np.int64), np.asarray(values, dtype=int)


def active_values(
    query_ns: np.ndarray, event_ns: np.ndarray, values: np.ndarray, initial: np.ndarray
) -> np.ndarray:
    """Return the latest event value at every query time (zero-order hold)."""
    indices = np.searchsorted(event_ns, query_ns, side="right") - 1
    result = np.empty((len(query_ns), values.shape[1]), dtype=float)
    before_first = indices < 0
    result[before_first] = initial
    result[~before_first] = values[indices[~before_first]]
    return result


def state_at_times(query_ns: np.ndarray, state_ns: np.ndarray, states: np.ndarray) -> np.ndarray:
    indices = np.searchsorted(state_ns, query_ns, side="right") - 1
    result = np.zeros(len(query_ns), dtype=int)
    valid = indices >= 0
    result[valid] = states[indices[valid]]
    return result


def line_colours(states: np.ndarray) -> List[str]:
    return [STATE_STYLE.get(int(state), (f"State {state}", "#555555"))[1] for state in states]


def state_label(state: int) -> str:
    return STATE_STYLE.get(state, (f"State {state}", "#555555"))[0]


def state_legend(observed_states: Iterable[int]) -> List[Line2D]:
    return [
        Line2D(
            [0],
            [0],
            color=STATE_STYLE.get(int(state), ("", "#555555"))[1],
            lw=3,
            label=state_label(int(state)),
        )
        for state in sorted(set(int(value) for value in observed_states))
    ]


def infer_initial_goal(
    odom_ns: np.ndarray,
    actual_nedh: np.ndarray,
    goal_ns: np.ndarray,
    explicit_initial: Optional[Sequence[float]],
) -> Optional[np.ndarray]:
    if explicit_initial is not None:
        return np.asarray(explicit_initial, dtype=float)
    if len(goal_ns) == 0:
        return None
    # A goal published after the bag starts usually means the previous goal was
    # reached. Its terminal actual position is the best bag-only estimate.
    index = int(np.searchsorted(odom_ns, goal_ns[0], side="right") - 1)
    index = max(0, min(index, len(actual_nedh) - 1))
    inferred = actual_nedh[index, :3].copy()
    print(
        "Note: no --initial-goal supplied; inferred the initial desired "
        f"N/E/depth as ({inferred[0]:.3f}, {inferred[1]:.3f}, {inferred[2]:.3f}) m."
    )
    return inferred


def derived_desired_heading(actual_nedh: np.ndarray, desired_ned: np.ndarray) -> np.ndarray:
    delta_north = desired_ned[:, 0] - actual_nedh[:, 0]
    delta_east = desired_ned[:, 1] - actual_nedh[:, 1]
    heading = np.arctan2(delta_east, delta_north)

    # Close to a waypoint the line-of-sight angle becomes noise-sensitive.
    # Hold its last useful value inside the controller's 0.5 m acceptance area.
    horizontal_distance = np.hypot(delta_north, delta_east)
    last = np.nan
    for index in range(len(heading)):
        if horizontal_distance[index] >= 0.5:
            last = heading[index]
        elif np.isfinite(last):
            heading[index] = last
        else:
            heading[index] = np.nan
    return heading


def infer_stationkeeping_headings(
    odom_ns: np.ndarray,
    actual_heading: np.ndarray,
    goal_ns: np.ndarray,
    goals_ned: np.ndarray,
    desired_heading: np.ndarray,
    final_time_ns: int,
) -> None:
    """Infer fixed-heading holds signalled by a repeated position goal.

    PointStamped does not carry heading.  In this mission a repeated position
    starts StationKeeping.  If that hold lasts at least two seconds, use the
    settled measured heading near its end as the best bag-only estimate.
    """
    for index in range(1, len(goal_ns)):
        if np.linalg.norm(goals_ned[index] - goals_ned[index - 1]) > 1e-3:
            continue
        end_ns = int(goal_ns[index + 1]) if index + 1 < len(goal_ns) else final_time_ns
        if end_ns - int(goal_ns[index]) < 2_000_000_000:
            continue
        settled = (odom_ns >= end_ns - 2_000_000_000) & (odom_ns < end_ns)
        if not np.any(settled):
            continue
        # Circular mean handles samples close to the +/-pi wrap.
        heading = math.atan2(
            float(np.mean(np.sin(actual_heading[settled]))),
            float(np.mean(np.cos(actual_heading[settled]))),
        )
        hold = (odom_ns >= goal_ns[index]) & (odom_ns < end_ns)
        desired_heading[hold] = heading


def docking_reference_in_world(
    world_odom_records: Sequence[Record],
    world_actual_nedh: np.ndarray,
    local_odom_records: Sequence[Record],
    reference_records: Sequence[Record],
    docking_start_ns: int,
) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, float]]:
    """Calibrate tag XY/height against world NED and transform docking refs.

    The docking controller publishes tag-frame x/y/z in Pose.position and yaw
    *degrees* in Pose.orientation.w.  A planar rigid transform is estimated
    from simultaneous /odometry/filtered and /blueye/odom positions.  Tag Z is
    positive-up, hence world depth is ``-tag_z + offset``.
    """
    if not local_odom_records or not reference_records:
        return None

    world_ns = np.asarray([timestamp for timestamp, _ in world_odom_records], dtype=np.int64)
    local_ns_all = np.asarray([timestamp for timestamp, _ in local_odom_records], dtype=np.int64)
    local_xyz_all = np.asarray(
        [
            (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            )
            for _, msg in local_odom_records
        ],
        dtype=float,
    )
    calibration_mask = local_ns_all >= docking_start_ns
    if np.count_nonzero(calibration_mask) < 10:
        return None
    local_ns = local_ns_all[calibration_mask]
    local_xyz = local_xyz_all[calibration_mask]

    # Subtract a local epoch before interpolation to preserve timestamp
    # precision (bag timestamps are around 1e18 ns).
    epoch_ns = min(int(world_ns[0]), int(local_ns[0]))
    world_t = (world_ns - epoch_ns) * 1e-9
    local_t = (local_ns - epoch_ns) * 1e-9
    world_at_local = np.column_stack(
        [
            np.interp(local_t, world_t, world_actual_nedh[:, axis])
            for axis in range(3)
        ]
    )

    # Orthogonal Procrustes/Kabsch fit in the horizontal plane:
    # [north, east] = rotation @ [tag_x, tag_y] + translation.
    local_xy_center = local_xyz[:, :2].mean(axis=0)
    world_xy_center = world_at_local[:, :2].mean(axis=0)
    local_centered = local_xyz[:, :2] - local_xy_center
    world_centered = world_at_local[:, :2] - world_xy_center
    u, _singular_values, vt = np.linalg.svd(local_centered.T @ world_centered)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    translation = world_xy_center - rotation @ local_xy_center
    depth_offset = float(np.median(world_at_local[:, 2] + local_xyz[:, 2]))

    predicted = np.column_stack(
        (
            local_xyz[:, :2] @ rotation.T + translation,
            -local_xyz[:, 2] + depth_offset,
        )
    )
    rms_error = float(np.sqrt(np.mean(np.sum((predicted - world_at_local) ** 2, axis=1))))

    reference_ns = np.asarray([timestamp for timestamp, _ in reference_records], dtype=np.int64)
    reference_xyz = np.asarray(
        [(msg.position.x, msg.position.y, msg.position.z) for _, msg in reference_records],
        dtype=float,
    )
    reference_xy_world = reference_xyz[:, :2] @ rotation.T + translation
    reference_ned = np.column_stack(
        (reference_xy_world, -reference_xyz[:, 2] + depth_offset)
    )

    local_heading = np.radians(
        np.asarray([msg.orientation.w for _, msg in reference_records], dtype=float)
    )
    local_forward = np.column_stack((np.cos(local_heading), np.sin(local_heading)))
    world_forward = local_forward @ rotation.T
    reference_heading = np.arctan2(world_forward[:, 1], world_forward[:, 0])
    return reference_ns, reference_ned, reference_heading, rms_error


def plot_overview(
    output: Path,
    time_s: np.ndarray,
    actual_nedh: np.ndarray,
    state_time_s: np.ndarray,
    states: np.ndarray,
    desired_ned: Optional[np.ndarray],
    desired_heading: Optional[np.ndarray],
    dpi: int,
) -> plt.Figure:
    fig, axes = plt.subplots(
        5,
        1,
        figsize=(13, 13),
        sharex=True,
        gridspec_kw={"height_ratios": [0.8, 1, 1, 1, 1]},
    )

    axes[0].step(state_time_s, states, where="post", color="purple", lw=1.8, label="State")
    observed = sorted(set(int(value) for value in states))
    axes[0].set_yticks(observed)
    axes[0].set_yticklabels([state_label(value) for value in observed])
    axes[0].set_ylabel("State")
    axes[0].set_title("Mission state over time")
    axes[0].legend(loc="upper right")

    actual_labels = (
        r"Estimated north $\hat{n}$",
        r"Estimated east $\hat{e}$",
        r"Estimated depth $\hat{d}$",
    )
    desired_labels = (r"Desired north $n_d$", r"Desired east $e_d$", r"Desired depth $d_d$")
    titles = (
        "Estimated and desired north over time",
        "Estimated and desired east over time",
        "Estimated and desired depth over time",
    )
    ylabels = ("North [m]", "East [m]", "Depth [m]")

    for index, ax in enumerate(axes[1:4]):
        ax.plot(time_s, actual_nedh[:, index], color="blue", lw=1.4, label=actual_labels[index])
        if desired_ned is not None:
            ax.plot(
                time_s,
                desired_ned[:, index],
                color="red",
                lw=1.3,
                drawstyle="steps-post",
                label=desired_labels[index],
            )
        ax.set_ylabel(ylabels[index])
        ax.set_title(titles[index])
        ax.legend(loc="upper right")

    axes[4].plot(time_s, actual_nedh[:, 3], color="blue", lw=1.2, label=r"Estimated $\hat{\psi}$")
    if desired_heading is not None:
        axes[4].plot(
            time_s,
            desired_heading,
            color="red",
            lw=1.2,
            label=r"Desired $\psi_d$ (recorded/inferred)",
        )
    axes[4].set_ylabel("Heading [rad]")
    axes[4].set_xlabel("Time [s]")
    axes[4].set_title("Estimated and desired heading over time")
    axes[4].set_ylim(-math.pi - 0.2, math.pi + 0.2)
    axes[4].set_yticks([-math.pi, -math.pi / 2, 0, math.pi / 2, math.pi])
    axes[4].set_yticklabels([r"$-\pi$", r"$-\pi/2$", "0", r"$\pi/2$", r"$\pi$"])
    axes[4].legend(loc="upper right")

    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.set_xlim(time_s[0], time_s[-1])

    fig.tight_layout()
    fig.savefig(output, dpi=dpi, bbox_inches="tight")
    return fig


def plot_2d_trajectory(
    output: Path,
    actual_nedh: np.ndarray,
    odom_states: np.ndarray,
    goals_ned: Optional[np.ndarray],
    dpi: int,
) -> plt.Figure:
    north, east = actual_nedh[:, 0], actual_nedh[:, 1]
    points = np.column_stack((east, north))
    segments = np.stack((points[:-1], points[1:]), axis=1)

    fig, ax = plt.subplots(figsize=(8, 7))
    ax.add_collection(
        LineCollection(segments, colors=line_colours(odom_states[:-1]), linewidths=1.8)
    )
    ax.autoscale()
    ax.scatter(east[0], north[0], color="green", s=70, zorder=4, label="Start")
    ax.scatter(east[-1], north[-1], color="red", marker="^", s=70, zorder=4, label="End")

    if goals_ned is not None and len(goals_ned):
        ax.plot(
            goals_ned[:, 1],
            goals_ned[:, 0],
            "--o",
            color="black",
            lw=1.0,
            ms=4,
            alpha=0.7,
            label="Recorded goals",
        )

    handles = state_legend(odom_states)
    handles.extend(
        [
            Line2D([0], [0], marker="o", color="none", markerfacecolor="green", label="Start"),
            Line2D([0], [0], marker="^", color="none", markerfacecolor="red", label="End"),
        ]
    )
    if goals_ned is not None and len(goals_ned):
        handles.append(Line2D([0], [0], ls="--", marker="o", color="black", label="Recorded goals"))
    ax.legend(handles=handles, fontsize=8, loc="best")
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_title("ROV mission trajectory — top view")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, ls="--", alpha=0.35)
    fig.tight_layout()
    fig.savefig(output, dpi=dpi, bbox_inches="tight")
    return fig


def plot_3d_trajectory(
    output: Path,
    actual_nedh: np.ndarray,
    odom_states: np.ndarray,
    goals_ned: Optional[np.ndarray],
    dpi: int,
) -> plt.Figure:
    north, east, depth = actual_nedh[:, 0], actual_nedh[:, 1], actual_nedh[:, 2]
    points = np.column_stack((east, north, depth))
    segments = np.stack((points[:-1], points[1:]), axis=1)

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.add_collection3d(
        Line3DCollection(segments, colors=line_colours(odom_states[:-1]), linewidths=1.5)
    )
    ax.auto_scale_xyz(east, north, depth)
    ax.scatter(east[0], north[0], depth[0], color="green", s=70)
    ax.scatter(east[-1], north[-1], depth[-1], color="red", marker="^", s=70)

    if goals_ned is not None and len(goals_ned):
        ax.plot(
            goals_ned[:, 1],
            goals_ned[:, 0],
            goals_ned[:, 2],
            "--o",
            color="black",
            lw=1.0,
            ms=4,
            alpha=0.7,
        )

    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_zlabel("Depth [m]")
    ax.set_title("ROV mission trajectory — 3D")
    ax.invert_zaxis()  # positive depth is visually downward
    ax.legend(handles=state_legend(odom_states), fontsize=8, loc="upper left")
    fig.tight_layout()
    fig.savefig(output, dpi=dpi, bbox_inches="tight")
    return fig


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    try:
        bag_dir = resolve_bag_directory(args.bag)
        records, _available, odom_topic = read_bag(
            bag_dir, args.odom_topic, args.state_topic, args.goal_topic
        )
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    odom_records = records[odom_topic]
    state_records = records[args.state_topic]
    goal_records = records.get(args.goal_topic, [])
    convention = infer_coordinates(args.coordinates, odom_topic, odom_records[0][1])

    odom_ns = np.asarray([timestamp for timestamp, _ in odom_records], dtype=np.int64)
    actual_nedh = np.asarray(
        [odom_to_ned(msg, convention) for _, msg in odom_records], dtype=float
    )
    state_ns, states = compress_states(state_records)
    t0_ns = min(int(odom_ns[0]), int(state_ns[0]))
    time_s = (odom_ns - t0_ns) * 1e-9
    state_time_s = (state_ns - t0_ns) * 1e-9
    mission_end_s = float(time_s[-1])
    if state_time_s[-1] < mission_end_s:
        state_time_s = np.append(state_time_s, mission_end_s)
        states = np.append(states, states[-1])

    odom_states = state_at_times(odom_ns, state_ns, states[: len(state_ns)])
    goals_ned: Optional[np.ndarray] = None
    desired_ned: Optional[np.ndarray] = None
    desired_heading: Optional[np.ndarray] = None
    docking_transform_error: Optional[float] = None
    if goal_records:
        goal_ns = np.asarray([timestamp for timestamp, _ in goal_records], dtype=np.int64)
        goals_ned = np.asarray(
            [goal_to_ned(msg, convention) for _, msg in goal_records], dtype=float
        )
        initial_goal = infer_initial_goal(odom_ns, actual_nedh, goal_ns, args.initial_goal)
        assert initial_goal is not None
        desired_ned = active_values(odom_ns, goal_ns, goals_ned, initial_goal)
        desired_heading = derived_desired_heading(actual_nedh, desired_ned)
        initial_heading = (
            float(args.initial_heading)
            if args.initial_heading is not None
            else float(actual_nedh[0, 3])
        )
        desired_heading[odom_ns < goal_ns[0]] = initial_heading

        # A repeated position goal denotes station keeping in this mission.
        docking_state_times = state_ns[states[: len(state_ns)] == 5]
        heading_inference_end = (
            int(docking_state_times[0]) if len(docking_state_times) else int(odom_ns[-1])
        )
        infer_stationkeeping_headings(
            odom_ns,
            actual_nedh[:, 3],
            goal_ns,
            goals_ned,
            desired_heading,
            heading_inference_end,
        )

        # During docking the controller changes to the local ArUco/tag frame.
        # Convert its filtered reference back into world NED before replacing
        # the now-stale last world waypoint in the overview curves.
        local_odom_records = records.get(DOCKING_ODOM_TOPIC, [])
        docking_reference_records = records.get(DOCKING_REFERENCE_TOPIC, [])
        if (
            odom_topic != DOCKING_ODOM_TOPIC
            and len(docking_state_times)
            and local_odom_records
            and docking_reference_records
        ):
            docking_result = docking_reference_in_world(
                odom_records,
                actual_nedh,
                local_odom_records,
                docking_reference_records,
                int(docking_state_times[0]),
            )
            if docking_result is not None:
                docking_ns, docking_ned, docking_heading, docking_transform_error = docking_result
                active_docking_ned = active_values(
                    odom_ns, docking_ns, docking_ned, docking_ned[0]
                )
                active_docking_heading = active_values(
                    odom_ns,
                    docking_ns,
                    docking_heading[:, np.newaxis],
                    np.asarray([docking_heading[0]]),
                )[:, 0]
                docking_mask = (odom_ns >= docking_ns[0]) & (odom_states == 5)
                desired_ned[docking_mask] = active_docking_ned[docking_mask]
                desired_heading[docking_mask] = active_docking_heading[docking_mask]

    output_dir = (
        args.output_dir.expanduser().resolve()
        if args.output_dir
        else (Path(__file__).resolve().parent / "plots" / bag_dir.name)
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    outputs = {
        "overview": output_dir / "mission_overview.png",
        "2D trajectory": output_dir / "trajectory_2d_state.png",
        "3D trajectory": output_dir / "trajectory_3d_state.png",
    }

    print(f"Bag: {bag_dir}")
    print(f"Odometry: {odom_topic} ({len(odom_records)} messages)")
    print(f"Coordinate convention: {convention.upper()}")
    print(
        "State transitions: "
        + " -> ".join(
            f"{state_label(int(state))} @ {(timestamp - t0_ns) * 1e-9:.1f}s"
            for timestamp, state in zip(state_ns, states[: len(state_ns)])
        )
    )
    if goal_records:
        print(f"Desired goals: {args.goal_topic} ({len(goal_records)} messages)")
    if docking_transform_error is not None:
        print(
            "Docking reference: transformed tag -> world NED "
            f"(calibration RMS error {docking_transform_error:.3f} m)"
        )

    figures = [
        plot_overview(
            outputs["overview"],
            time_s,
            actual_nedh,
            state_time_s,
            states,
            desired_ned,
            desired_heading,
            args.dpi,
        ),
        plot_2d_trajectory(
            outputs["2D trajectory"], actual_nedh, odom_states, goals_ned, args.dpi
        ),
        plot_3d_trajectory(
            outputs["3D trajectory"], actual_nedh, odom_states, goals_ned, args.dpi
        ),
    ]

    for label, path in outputs.items():
        print(f"Saved {label}: {path}")

    if args.show:
        # Pyplot may still use Agg in headless sessions, but this is harmless.
        plt.show()
    for figure in figures:
        plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
