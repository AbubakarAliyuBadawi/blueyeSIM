#!/usr/bin/env python3
"""
Figures and tables for a field run of the real Blueye (pier), laid out like the simulation runs: the timeline and
hidden-node figures are drawn by plot_nominal_run.py's functions, so both sets look the same.

Usage:
  source ~/Desktop/auto-pilot/install/setup.bash
  python3 src/plots/plot_field_run.py src/rosbags/field/pier_nominal_run01_shadow_20261025

Output (default src/plots/field/<bag_name>/), each figure as PDF (paper) and PNG (viewing):
  fig1_mission_map       track coloured by mission phase in the drone's own position estimate (DVL dead reckoning
                         from the position reset; north/east from the reset point, the dock reference), the planned
                         waypoints as the mission sent them, start and end; takeover request / hand-back markers
  fig2_timeline          as for the simulation runs; injected evidence changes and the takeovers (TO1, TO2, ...) marked
  fig3_hidden_nodes      as for the simulation runs
  phase_summary.csv/.md  per phase: duration, urgency statistics, time above threshold, alarms
  run_summary.txt        mission result, alarms, peak urgency, battery, distance, start position
  takeover_events.csv/.md  closed-loop runs: one row per takeover with the metrics of the simulation runs (t_att and
                         t_req from the injection, max U before and after, decision time, manual control)

field_run.yaml in the output folder holds what is particular to the run (as scenario_config.yaml does for the
simulation runs):
  site: pier
  exclude: [["2026-10-25 16:50:56", "2026-10-25 17:04:22"]]   left out of all figures and tables; the samples after
                                                               a window move up, the cut is marked on the time axes
  takeovers: [TO1, TO2, TO3]                                     labels of the takeovers, in order
  replay:                                                     for --replay (below)
    aruco_in_phases: {Undocking: All}                         ArUcoVisibility entered as this state in these phases
    restart_operator_model: true                              fatigue and stress from the operator model started at
                                                              the mission start (time on task as drawn)
    note: "..."                                               printed above the figure

--replay draws fig2_timeline_replay and fig3_hidden_nodes_replay instead of the run's figures: the recorded evidence of every network update, with
the corrections of field_run.yaml, fed again through the same network (UnifiedMissionDBN_v4.xdsl, same window and
urgency as takeover_bn_node). replay_summary.txt reports how closely the uncorrected replay reproduces the recorded
urgency, and the urgency per phase of the corrected replay. A left-out window is not marked on the time axes; the note
says so.

The analysis window runs from the start of the behaviour tree to its stop (or the end of the bag). The phase comes
from /mission_state; the undocking, published before the recorder subscribed, is taken from the tree's start.

The simulator twin of a field run (scenario SC, blueye_stonefish_bt in the bag) is drawn the same way, for a direct
comparison: fig1_mission_map is the simulation map (plot_nominal_run.py, rocks from scenario_config.yaml in the output
folder), the window runs from the first mission state to "Mission finished", and the line above the timeline lists only
the injected USBL changes (the rocks give the sonar readings). Output by default in src/plots/paper/<bag_name>/:
  python3 src/plots/plot_field_run.py src/rosbags/paper/sf_field_twin_run01_closedloop_20261027
"""

import argparse
import bisect
import csv
import json
import math
import os
import sys
from datetime import datetime

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
import yaml
from matplotlib.collections import LineCollection
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "blueye_real_bt", "scripts", "mission_planner_scripts"))
import plot_nominal_run as pnr  # noqa: E402
import mission_sites  # noqa: E402

TOPICS = ["/mission_state", "/rosout", "/blueye/sensor/ekf/pose", "/blueye/odom", "/blueye/waypoint_goal",
          "/blueye/battery",
          "/blueye/takeover_request/urgency", "/blueye/takeover_request/threshold",
          "/blueye/takeover_request/evidence", "/blueye/takeover_request/mode",
          "/blueye/human/fatigue", "/blueye/human/stress", "/blueye/takeover_request/human_decision",
          "/blueye/takeover_request/handback", "/scenario/events"]
INJECTION_COLOURS = {"sonar": "#52514e", "usbl": "#9467bd"}
TREE_NODES = ("blueye_real_bt", "blueye_stonefish_bt")      # field, simulator


# ── Data ────────────────────────────────────────────────────────────────────────

def load_bag(path):
    """The topics in plot_nominal_run's format (time in s from the first message): the position estimate (field) or
    the simulator's odometry as /blueye/odom (t, north, east, depth, yaw), the battery as a fraction."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    topics = [t for t in TOPICS if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    classes = {t: get_message(types[t]) for t in topics}
    data = {t: [] for t in TOPICS + ["/blueye/odom"]}
    t0 = None
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        t0 = t0 if t0 is not None else stamp
        ts = (stamp - t0) / 1e9
        msg = deserialize_message(raw, classes[topic])
        if topic in ("/blueye/sensor/ekf/pose", "/blueye/odom"):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            data["/blueye/odom"].append((ts, p.x, p.y, p.z, yaw))
        elif topic == "/blueye/waypoint_goal":
            data[topic].append((ts, msg.point.x, msg.point.y, msg.point.z))
        elif topic == "/blueye/takeover_request/evidence":
            data[topic].append((ts, json.loads(msg.data)))
        elif topic == "/blueye/battery":
            # the handler packs the state of charge in position.y; the simulator sends a battery message
            data[topic].append((ts, msg.position.y if hasattr(msg, "position") else
                                msg.state_of_charge if hasattr(msg, "state_of_charge") else msg.percentage))
        elif topic == "/scenario/events":
            data[topic].append((ts, json.loads(msg.data)))
        elif topic == "/rosout":
            data[topic].append((ts, msg.name, msg.msg))
        else:
            data[topic].append((ts, msg.data))
    return data, t0 / 1e9


def exclude_windows(data, t0, windows):
    """Leave out the wall-clock windows: drop their samples and move the later ones up. Returns the cut positions
    (bag time after the cut) with the removed durations."""
    spans = []
    for a, b in windows:
        ta = datetime.strptime(a, "%Y-%m-%d %H:%M:%S").timestamp() - t0
        tb = datetime.strptime(b, "%Y-%m-%d %H:%M:%S").timestamp() - t0
        spans.append((ta, tb))
    spans.sort()
    for topic, series in data.items():
        kept = []
        for sample in series:
            t, shift, inside = sample[0], 0.0, False
            for ta, tb in spans:
                if ta <= t <= tb:
                    inside = True
                elif t > tb:
                    shift += tb - ta
            if not inside:
                kept.append((t - shift,) + tuple(sample[1:]))
        data[topic] = kept
    cuts, removed = [], 0.0
    for ta, tb in spans:
        cuts.append((ta - removed, tb - ta))
        removed += tb - ta
    return cuts


def replay_network(samples, corrections=None, start=None, seed=1):
    """[(t, urgency, message)] for the recorded network updates [(t, message)], fed again through the network as
    takeover_bn_node does (evidence of the last N updates in the last N slices, urgency 0.5 P(IA) + P(TR)). With
    corrections: ArUcoVisibility per phase, and Fatigue and Stress from the operator model started at `start`."""
    import collections
    import random
    import pysmile
    import pysmile_license  # noqa: F401  (the BayesFusion licence, as in takeover_bn_node)
    from ament_index_python.packages import get_package_share_directory
    net = pysmile.Network()
    net.read_file(os.path.join(get_package_share_directory("blueye_takeover_bn"), "config", "UnifiedMissionDBN_v4.xdsl"))
    ids = sorted(net.get_all_node_ids())
    outcomes = {n: list(net.get_outcome_ids(n)) for n in ids}
    net.clear_all_targets()
    history = collections.deque(maxlen=net.get_slice_count())
    rng = random.Random(seed)
    out, operator = [], []
    for t, message in samples:
        evidence = dict(message["evidence"])
        if corrections:
            state = corrections.get("aruco_in_phases", {}).get(evidence.get("MissionPhase"))
            if state:
                evidence["ArUcoVisibility"] = state
            if corrections.get("restart_operator_model"):
                progress = min(max(t - start, 0.0) / 900.0, 1.0)      # human_model: ramp_duration_s 900
                values = [min(max(a + (b - a) * progress + rng.gauss(0.0, 0.02), 0.0), 1.0)
                          for a, b in ((0.15, 0.55), (0.15, 0.50))]
                for node, value in zip(("Fatigue", "Stress"), values):
                    evidence[node] = "Low" if value < 0.3 else ("Medium" if value < 0.6 else "High")
                operator.append((t, *values))
        history.append(evidence)
        net.clear_all_evidence()
        first = net.get_slice_count() - len(history)
        for k, snapshot in enumerate(history):
            for node, state in snapshot.items():
                net.set_temporal_evidence(node, first + k, outcomes[node].index(state))
        net.update_beliefs()
        posteriors = {n: dict(zip(outcomes[n], list(net.get_node_value(n))[-len(outcomes[n]):])) for n in ids}
        probabilities = posteriors["TakeoverRequest"]
        urgency = 0.5 * probabilities.get("IncreasedAttentionRequired", 0.0) + probabilities.get("TakeoverRequested", 0.0)
        out.append((t, urgency, {"state": max(probabilities, key=probabilities.get), "probabilities": probabilities,
                                 "evidence": evidence, "posteriors": posteriors}))
    return out, operator


def is_sim(data):
    """A simulator bag (the twin of a field run): its behaviour tree is blueye_stonefish_bt."""
    return any(n == "blueye_stonefish_bt" for _, n, _ in data["/rosout"])


def mission_window(data):
    """From the start of the tree to its stop (or the end of the bag); in a simulator bag as plot_nominal_run.py."""
    if is_sim(data):
        start, end, result = pnr.mission_window(data)
        return start, end, f"mission finished: {result}"
    tree = [t for t, n, m in data["/rosout"] if n == "blueye_real_bt" and m.startswith("Takeover mode")]
    start = tree[0] if tree else min(t for t, v in data["/mission_state"])
    stop = [t for t, n, m in data["/rosout"] if n == "blueye_real_bt" and "Interrupt signal" in m and t > start]
    end = stop[0] if stop else max(t for t, *_ in data["/blueye/odom"])
    # the undocking state was published before the recorder subscribed to /mission_state
    states = data["/mission_state"]
    if not states or states[0][1] != 1:
        data["/mission_state"] = [(start, 1)] + states
    completed = any("mission completed successfully" in m for t, n, m in data["/rosout"] if start <= t <= end)
    docking = [t for t, v in data["/mission_state"] if v == 5 and start <= t <= end]
    if completed and docking:
        result = f"survey completed; docking step from {docking[0] - start:.0f} s to the end of the run"
    elif completed:
        result = "survey completed"
    else:
        last = max((v for t, v in data["/mission_state"] if v in pnr.PHASES and v != 6 and t <= end), default=None)
        result = f"survey not completed in the recording (last phase: {pnr.PHASES[last][0] if last else '?'})"
    return start, end, result


def takeover_events(data, start, end):
    """[(time, label, colour)]: takeover request (the tree's request; the HUD opens the popup with it), accepted /
    rejected, hand-back."""
    events = [(t, "takeover request", "#d62728") for t, n, m in data["/rosout"]
              if n in TREE_NODES and m.startswith("Takeover requested") and start <= t <= end]
    events += [(t, "accepted" if v else "rejected", "#1baf7a" if v else "#d62728")
               for t, v in data["/blueye/takeover_request/human_decision"] if start <= t <= end]
    events += [(t, "hand-back", "#2a78d6") for t, v in data["/blueye/takeover_request/handback"]
               if v and start <= t <= end]
    return sorted(events)


def injections(data, start, end):
    """[(time, label, colour, None)] of the evidence injected from the HUD (virtual sonar and USBL), or in the
    simulator twin the USBL changes of field_twin_events and the sonar readings the rocks gave. Only changes: a state
    sent again (the twin holds USBL Strong from the start) is not one."""
    out, state = [], {"sonar": "Clear", "usbl": "Strong"}
    for t, e in data["/scenario/events"]:
        kind = e.get("event")
        if kind in INJECTION_COLOURS and e["state"] != state[kind]:
            state[kind] = e["state"]
            if start <= t <= end:
                name = "USBL" if kind == "usbl" else "sonar"
                out.append((t, f"{name} {e['state']}", INJECTION_COLOURS[kind], None))
    return out


def group_takeovers(events, labels):
    """[(label, request, decision, accepted, hand-back)] per takeover, in order."""
    groups = []
    for t, label, _ in events:
        if label == "takeover request":
            groups.append({"request": t})
        elif groups and label in ("accepted", "rejected") and "decision" not in groups[-1]:
            groups[-1]["decision"], groups[-1]["accepted"] = t, label == "accepted"
        elif groups and label == "hand-back" and "hand-back" not in groups[-1]:
            groups[-1]["hand-back"] = t
    for i, g in enumerate(groups):
        g["label"] = labels[i] if i < len(labels) else f"TO{i + 1}"
    return groups


def label_takeovers(ax, groups, start, y):
    """The takeover's name above its block, centred between the request and the hand-back."""
    for g in groups:
        t = (g["request"] + g.get("hand-back", g["request"])) / 2 - start
        ax.text(t, y, g["label"], ha="center", va="top", fontsize=7, fontweight="bold", color="#d62728", zorder=8,
                bbox=dict(fc="white", ec="none", alpha=0.8, pad=0.8))


# ── Figures ─────────────────────────────────────────────────────────────────────

def fig_mission_map(data, segs, start, end, waypoint_names, out, events=()):
    odom = [o for o in data["/blueye/odom"] if start <= o[0] <= end]
    fig, ax = plt.subplots(figsize=(3.6, 3.5))
    pts = np.array([[o[2], o[1]] for o in odom])
    phase = [pnr.phase_at(segs, o[0]) for o in odom]
    # no line across a left-out window: break the track where consecutive samples are far apart in time
    lines, colours = [], []
    for i in range(len(odom) - 1):
        if odom[i + 1][0] - odom[i][0] < 2.0:
            lines.append([pts[i], pts[i + 1]])
            colours.append(pnr.PHASES.get(phase[i], ("", "grey"))[1])
    ax.add_collection(LineCollection(lines, colors=colours, linewidths=1.4))
    goals, last = [], None
    for t, x, y, z in data["/blueye/waypoint_goal"]:
        key = (round(x, 1), round(y, 1))
        if key != last and key not in [(round(gx, 1), round(gy, 1)) for gx, gy in goals]:
            goals.append((x, y))
        last = key
    # the mission sends the goals in the order of the site layout; a run that ended early sent fewer
    names = waypoint_names[:len(goals)] if len(goals) <= len(waypoint_names) else \
        [str(i) for i in range(1, len(goals) + 1)]
    if len(goals) >= 4:                                  # the assumed pipeline under the inspection line
        insp = [g for g, n in zip(goals, names) if n.startswith("I")] or goals[:4]
        ax.plot([g[1] for g in insp], [g[0] for g in insp], color="#ffd84d", lw=6, alpha=0.5, solid_capstyle="butt",
                zorder=1, label="Pipeline (assumed)")
    for (x, y), name in zip(goals, names):
        ax.plot(y, x, "o", mfc="white", mec="black", mew=0.8, ms=9.5, zorder=5)
        ax.text(y, x, name, ha="center", va="center", fontsize=4.8, fontweight="bold", zorder=6)
    ax.plot([], [], "o", mfc="white", mec="black", mew=0.8, ms=7, label="Waypoint")
    ax.plot(0, 0, "s", mfc="#ffd84d", mec="black", ms=6, zorder=6, label="Dock reference")
    ax.plot(pts[0, 0], pts[0, 1], "D", color="black", ms=4.5, zorder=6, label="Start")
    ax.plot(pts[-1, 0], pts[-1, 1], "s", mfc="none", mec="black", ms=6, zorder=6, label="End")
    ot = [o[0] for o in odom]
    shown = set()
    for t, label, colour in events:
        if label in ("takeover request", "hand-back"):
            o = odom[min(bisect.bisect_left(ot, t), len(odom) - 1)]
            ax.plot(o[2], o[1], "v" if label == "takeover request" else "^", color=colour, ms=5.5, mec="white",
                    mew=0.5, zorder=7, label=None if label in shown else label.capitalize())
            shown.add(label)
    for p in sorted(set(phase) - {None}):
        ax.plot([], [], color=pnr.PHASES[p][1], lw=2, label=pnr.PHASES[p][0])
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_aspect("equal")
    east = [*pts[:, 0], 0.0, *(g[1] for g in goals)]
    north = [*pts[:, 1], 0.0, *(g[0] for g in goals)]
    ax.set_xlim(min(east) - 1.5, max(east) + 1.5)
    ax.set_ylim(min(north) - 1.5, max(north) + 1.5)
    ax.grid(alpha=0.3, lw=0.5)
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), frameon=False)
    pnr.save(fig, out, "fig1_mission_map")


# ── Tables ──────────────────────────────────────────────────────────────────────

def takeover_table(data, groups, inj, start, end, threshold, out, changes="Injected"):
    """The simulation's hazard metrics per takeover, times from the injection that started it."""
    urg = [(t, v) for t, v in data["/blueye/takeover_request/urgency"] if start <= t <= end]
    ev = [(t, e) for t, e in data["/blueye/takeover_request/evidence"] if start <= t <= end]
    rows, prev_end = [], start
    for g in groups:
        req = g["request"]
        # the injections since the previous takeover's hand-back, up to the request
        own = [(t, label) for t, label, _, _ in inj if prev_end < t <= req]
        trigger = next(((t, l) for t, l in reversed(own) if not l.endswith(("Clear", "Strong"))), None)
        first = next(((t, l) for t, l in own if not l.endswith(("Clear", "Strong"))), None)
        t_inj = first[0] if first else None
        t_att = next((t for t, e in ev if t_inj is not None and t >= t_inj and max(
            e["probabilities"], key=e["probabilities"].get) in ("IncreasedAttentionRequired", "TakeoverRequested")),
            None)
        t_req = next((t for t, v in urg if t_inj is not None and t >= t_inj and v >= threshold), None)
        hb = g.get("hand-back")
        # cleared: the last change back to Clear / Strong before the hand-back (the twin's sonar can flicker first)
        cleared = next((t for t, label, _, _ in reversed(inj)
                        if req < t <= (hb or end) and label.endswith(("Clear", "Strong"))), None)
        u_before = max((v for t, v in urg if prev_end <= t < (t_inj or req)), default=float("nan"))
        u_after = max((v for t, v in urg if (t_inj or req) <= t <= (hb or end)), default=float("nan"))
        phase = next((e["evidence"].get("MissionPhase") for t, e in ev if t >= req), "?")

        def r(x):
            return "" if x is None or (isinstance(x, float) and math.isnan(x)) else round(x, 2)
        rows.append({
            "ID": g["label"], "phase": phase,
            "injected": ", ".join(f"{l} ({t - start:.0f} s)" for t, l in own),
            "t_injection_s": r(None if t_inj is None else t_inj - start),
            "max_U_before": r(u_before), "max_U_after": r(u_after),
            "t_att_s": r(None if t_att is None else t_att - t_inj),
            "t_req_s": r(None if t_req is None else t_req - t_inj),
            # the twin logs a sonar change from the network's evidence, a few ms after the urgency of the same update
            "t_req_after_trigger_s": r(None if (t_req is None or trigger is None) else max(t_req - trigger[0], 0.0)),
            "decision": "accepted" if g.get("accepted") else ("rejected" if "decision" in g else ""),
            "decision_s": r(g["decision"] - req if "decision" in g else None),
            "manual_control_s": r(hb - g["decision"] if (hb is not None and "decision" in g) else None),
            "handback_after_cleared_s": r(None if (hb is None or cleared is None) else hb - cleared),
        })
        prev_end = hb or req
    with open(os.path.join(out, "takeover_events.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(out, "takeover_events.md"), "w") as f:
        f.write("Times in s. t_att and t_req from the first injected change of the takeover; t_req after trigger from "
                "the change that crossed the threshold; decision time from the request (popup) to the operator's "
                "answer; manual control from the answer to the hand-back.")
        if changes != "Injected":
            f.write(" The sonar changes are logged from the network's own evidence, so a request on a sonar change "
                    "reads 0 s after it (the same network update).")
        f.write("\n\n")
        f.write(f"| ID | Phase | {changes} (mission time) | max U before | max U after | t_att | t_req | t_req after "
                "trigger | Decision | Decision time | Manual control | Hand-back after cleared |\n")
        f.write("|---|---|---|---|---|---|---|---|---|---|---|---|\n")
        for x in rows:
            f.write(f"| {x['ID']} | {x['phase']} | {x['injected']} | {x['max_U_before']} | {x['max_U_after']} | "
                    f"{x['t_att_s']} | {x['t_req_s']} | {x['t_req_after_trigger_s']} | {x['decision']} | "
                    f"{x['decision_s']} | {x['manual_control_s']} | {x['handback_after_cleared_s']} |\n")
    print("  takeover_events.csv/.md")
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag_path", help="rosbag2 directory")
    parser.add_argument("--out", default=None, help="output directory (default: src/plots/field/<bag_name>/, for a "
                        "simulator bag src/plots/paper/<bag_name>/)")
    parser.add_argument("--replay", action="store_true", help="fig2_timeline_replay from the corrected replay "
                        "(field_run.yaml: replay) instead of the run's figures")
    args = parser.parse_args()

    bag = os.path.abspath(args.bag_path)
    name = os.path.basename(os.path.normpath(bag))
    print(f"Reading {bag} ...")
    data, t0 = load_bag(bag)
    sim = is_sim(data)
    out = args.out or os.path.join(HERE, "paper" if sim else "field", name)
    os.makedirs(out, exist_ok=True)
    run_file = os.path.join(out, "field_run.yaml")
    run = yaml.safe_load(open(run_file)) if os.path.exists(run_file) else {}
    site = run.get("site", "pier")
    waypoint_names = [w[0] for w in mission_sites.SITES[site]["waypoints"]]

    if args.replay:
        # check first: the recorded evidence, uncorrected, must give the recorded urgency again
        check, _ = replay_network(data["/blueye/takeover_request/evidence"])
        recorded = data["/blueye/takeover_request/urgency"]
        rt = [t for t, _ in recorded]
        warm = check[0][0] + 10.0                 # the node's window was already full when the recording began
        # the node publishes the urgency just before the evidence of the same update
        errors = [abs(u - recorded[max(bisect.bisect_right(rt, t) - 1, 0)][1]) for t, u, _ in check if t >= warm]
    cuts = exclude_windows(data, t0, run.get("exclude", []))
    start, end, result = mission_window(data)
    if args.replay:
        corrected, operator = replay_network(data["/blueye/takeover_request/evidence"], run.get("replay", {}), start)
        data["/blueye/takeover_request/evidence"] = [(t, m) for t, _, m in corrected]
        data["/blueye/takeover_request/urgency"] = [(t, u) for t, u, _ in corrected]
        if operator:
            data["/blueye/human/fatigue"] = [(t, f) for t, f, _ in operator]
            data["/blueye/human/stress"] = [(t, s_) for t, _, s_ in operator]
    segs = pnr.phase_segments(data, start, end)
    thr = data["/blueye/takeover_request/threshold"]
    threshold = float(thr[-1][1]) if thr else 0.65
    alarms = pnr.count_alarms([(t, v) for t, v in data["/blueye/takeover_request/urgency"] if start <= t <= end],
                              threshold)
    events = takeover_events(data, start, end)
    inj = injections(data, start, end)
    groups = group_takeovers(events, run.get("takeovers", []))

    # evidence rows: only the nodes the field network entered (the simulation set has more)
    entered = {k for t, e in data["/blueye/takeover_request/evidence"] for k in e["evidence"]}
    pnr.EVIDENCE_ROWS = [row for row in pnr.EVIDENCE_ROWS if row[0] in entered]

    # vertical lines for the takeover requests, the operator's answers and the hand-backs, as in the simulation runs
    # (the injections are listed in the line of text above the figure)
    def mark_events(ax, evs, s, labels=False):
        for t, label, colour in evs:
            ax.axvline(t - s, color=colour, lw=0.9, ls="-.")
        if labels and groups:
            text = "Takeover: " + "; ".join(
                f"{g['label']} request {g['request'] - s:.0f} s"
                + (f", {'accepted' if g.get('accepted') else 'rejected'} {g['decision'] - s:.0f} s" if "decision" in g else "")
                + (f", hand-back {g['hand-back'] - s:.0f} s" if "hand-back" in g else "") for g in groups)
            ax.text(0.0, 1.13, text, ha="left", va="bottom", fontsize=6.3, color="#d62728", transform=ax.transAxes)
            label_takeovers(ax, groups, s, 0.97)

    def mark_injections(ax, dist, s, line=1.13):
        if sim:                        # the twin injects only the USBL; the sonar readings came from the rocks
            dist = [d for d in dist if d[1].startswith("USBL")]
        if line is not None and dist:
            text = "Injected: " + ", ".join(f"{label} {t - s:.0f} s" for t, label, _, _ in dist)
            if sim:
                text += " (sonar: from the rocks)"
            ax.text(0.0, line, text, ha="left", va="bottom", fontsize=6.3, color="#52514e", transform=ax.transAxes)
    pnr.mark_events, pnr.mark_disturbances = mark_events, mark_injections

    # phase names above the timeline: one per phase, centred on the whole phase (a takeover inside it does not split
    # it), for phases of at least 7 % of the run, and for the undocking and the docking, short but always first / last
    def shade_phases(ax, sg, s, labels=False):
        total = sg[-1][1] - sg[0][0]
        for a, b, p in sg:
            ax.axvspan(a - s, b - s, color=pnr.PHASES.get(p, (str(p), "grey"))[1], alpha=0.10, lw=0)
        if not labels:
            return
        spans = []
        for a, b, p in sg:
            if p == 6:                     # takeover
                continue
            if spans and spans[-1][2] == p:
                spans[-1][1] = b
            else:
                spans.append([a, b, p])
        char = 0.0115 * total               # width of one character at 7 pt across the 7.2 in timeline, in s
        right = -math.inf                   # where the previous name ends
        for i, (a, b, p) in enumerate(spans):
            phase_name, colour = pnr.PHASES.get(p, (str(p), "grey"))
            if (b - a) >= 0.07 * total:
                # centred on the phase, moved right if the name before it (a short undocking) would touch it
                x = max((a + b) / 2 - s, right + char + len(phase_name) * char / 2)
                ax.text(x, 1.02, phase_name, transform=ax.get_xaxis_transform(), ha="center",
                        va="bottom", fontsize=7, color=colour)
                right = x + len(phase_name) * char / 2
            elif i == 0 and p == 1:
                ax.text(a - s, 1.02, phase_name, transform=ax.get_xaxis_transform(), ha="left", va="bottom",
                        fontsize=7, color=colour)
                right = a - s + len(phase_name) * char
            elif i == len(spans) - 1 and p == 5:      # the docking, short but always last: up to the end of the axis
                ax.text(b - s, 1.02, phase_name, transform=ax.get_xaxis_transform(), ha="right", va="bottom",
                        fontsize=7, color=colour)
    pnr.shade_phases = shade_phases

    # the cut of a left-out window: a double line on the time axes of the timeline and hidden-node figures
    save = pnr.save

    def save_with_cuts(fig, o, figname):
        # the phase sequence under the hidden-node panels: three phases per line (takeovers make it long)
        for ax in fig.axes:
            for text in ax.texts:
                lines = text.get_text().split("\n")
                if lines[0] == "Dotted lines: phase changes" and len(lines) > 1:
                    seq = lines[1].split(" \u2192 ")
                    wrapped = [" \u2192 ".join(seq[i:i + 3]) + (" \u2192" if i + 3 < len(seq) else "")
                               for i in range(0, len(seq), 3)]
                    text.set_text("\n".join([lines[0]] + wrapped + lines[2:]).replace(
                        "takeover request, accepted, hand-back",
                        "takeover request (red), accepted (green), hand-back (blue)"))
        if figname == "fig2_timeline":
            legends_timeline(fig)
            if args.replay:
                fig.axes[0].text(0.0, 1.17, run.get("replay", {}).get("note", "Offline replay"), ha="left",
                                 va="bottom", fontsize=6.3, color="#52514e", transform=fig.axes[0].transAxes,
                                 wrap=True)
                figname = "fig2_timeline_replay"
        elif figname == "fig3_hidden_nodes" and args.replay:
            fig.axes[0].text(0.0, 1.32, run.get("replay", {}).get("note", "Offline replay"), ha="left",
                             va="bottom", fontsize=6.3, color="#52514e", transform=fig.axes[0].transAxes)
            figname = "fig3_hidden_nodes_replay"
        if cuts and figname not in ("fig1_mission_map", "fig2_timeline_replay", "fig3_hidden_nodes_replay"):
            for ax in fig.axes:
                if ax.get_ylim() != (0.0, 1.0) and not ax.get_yticks().size:
                    continue
                for t, removed in cuts:
                    if start < t < end:
                        for dx in (-0.8, 0.8):
                            ax.axvline(t - start + dx, color="#52514e", lw=0.6, zorder=10)
        save(fig, o, figname)
    pnr.save = save_with_cuts

    def legends_timeline(fig):
        """Panel (a): the threshold and the takeover lines (request, accepted, hand-back); the urgency curve needs no
        entry. Panel (d): every severity colour, also the two middle ones of the nodes with four states (USBL
        Moderate/Weak, CameraQuality Good/Poor, BatteryLevel Medium/Low)."""
        a = fig.axes[0]
        d = next(ax for ax in fig.axes if any(t.get_text() == "(d) evidence" for t in ax.texts))
        handles, labels = zip(*[(h, l) for h, l in zip(*a.get_legend_handles_labels()) if not l.startswith("Urgency")])
        handles, labels = list(handles), list(labels)
        if groups:
            handles += [plt.Line2D([], [], color="#d62728", lw=0.9, ls="-."),
                        plt.Line2D([], [], color="#1baf7a", lw=0.9, ls="-."),
                        plt.Line2D([], [], color="#2a78d6", lw=0.9, ls="-.")]
            labels += ["Takeover request", "Takeover accepted", "Hand-back"]
        a.legend(handles, labels, loc="lower right", ncol=len(labels), frameon=False)   # clear of the urgency curve
        three, four = pnr.severity_colours(3), pnr.severity_colours(4)
        entries = [(three[0], "nominal"), (four[1], "slightly degraded"), (three[1], "degraded"),
                   (four[2], "severely degraded (USBL Weak)"), (three[2], "worst state"), ("#e0e0e0", "not entered")]
        d.legend(handles=[pnr.Patch(color=colour, label=label) for colour, label in entries], loc="lower right",
                 bbox_to_anchor=(1.0, 0.99), ncol=3, frameon=False, borderaxespad=0.1, fontsize=6.5,
                 handlelength=1.2, columnspacing=0.9, handletextpad=0.4)
        d.get_subplotspec().get_gridspec().update(hspace=0.36)

    print(f"Writing to {out}")
    if args.replay:
        pnr.fig_timeline(data, segs, start, end, threshold, alarms, out, events, inj)
        pnr.fig_hidden_nodes(data, segs, start, end, out, events)
        urg = [(t, u) for t, u in data["/blueye/takeover_request/urgency"] if start <= t <= end]
        lines = ["Offline replay of the recorded network evidence (plot_field_run.py --replay)",
                 f"corrections: {json.dumps({k: v for k, v in run.get('replay', {}).items() if k != 'note'})}",
                 f"check, uncorrected replay against the recorded urgency: max difference {max(errors):.4f}, "
                 f"mean {np.mean(errors):.5f} ({len(errors)} updates)",
                 f"alarms in the corrected replay (rising crossings): {len(alarms)}"]
        for a, b, p in segs:
            vals = [u for t, u in urg if a <= t < b]
            if vals:
                lines.append(f"  {pnr.PHASES.get(p, (str(p),))[0]:10s} mean U {np.mean(vals):.3f}, max U {max(vals):.3f}")
        with open(os.path.join(out, "replay_summary.txt"), "w") as f:
            f.write("\n".join(lines) + "\n")
        print("\n".join("  " + l for l in lines))
        return
    if sim:                                # the simulation map: dock, pipeline and the rocks of the run's config
        from ament_index_python.packages import get_package_share_directory
        meshes = os.path.join(get_package_share_directory("stonefish_sim"), "data", "object_files")
        config = os.path.join(out, "scenario_config.yaml")
        with open(config) as fh:
            pipe_mesh = str(yaml.safe_load(fh).get("pipeline_mesh", "pipe2"))
        grids = pnr.structure_grid(config, meshes, [("docking_station_rig", "docking_rig"), (pipe_mesh, "pipeline")])
        pnr.fig_mission_map(data, segs, start, end, grids, out, pnr.rocks(config, meshes), events)
    else:
        fig_mission_map(data, segs, start, end, waypoint_names, out, events)
    pnr.fig_timeline(data, segs, start, end, threshold, alarms, out, events, inj)
    pnr.fig_hidden_nodes(data, segs, start, end, out, events)
    pnr.summaries(data, bag, segs, start, end, result, threshold, alarms, out, events)
    first = next(o for o in data["/blueye/odom"] if o[0] >= start)
    if len(groups) > 1:                    # those two lines cover only the first takeover: see takeover_events
        path = os.path.join(out, "run_summary.txt")
        lines = [l for l in open(path) if not l.startswith(("operator decision time", "manual control"))]
        open(path, "w").writelines(lines)
    with open(os.path.join(out, "run_summary.txt"), "a") as f:
        if not sim:
            f.write(f"site: {site}\n")
            f.write(f"start: {first[1]:.1f} m north, {first[2]:.1f} m east of the dock reference, "
                    f"depth {first[3]:.2f} m\n")
            f.write("positions: the drone's own estimate (DVL dead reckoning from the position reset at the dock)\n")
        for t, removed in cuts:
            f.write(f"left out: {removed:.0f} s at {t - start:.0f} s of mission time\n")
        if groups:
            f.write("takeovers: " + "; ".join(f"{g['label']} request at {g['request'] - start:.1f} s" for g in groups)
                    + "\n")
    if groups:
        takeover_table(data, groups, inj, start, end, threshold, out,
                       "Evidence changes: USBL injected, sonar from the rocks" if sim else "Injected")


if __name__ == "__main__":
    main()
