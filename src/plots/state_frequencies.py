#!/usr/bin/env python3
"""
How often each state of the evidence nodes occurred in normal operation, next to the prior of the node in
UnifiedMissionDBN_v4.xdsl. Segments with injected (pre-programmed) disturbances are left out, so that degraded states do
not look more frequent than they are. Analysis only: the bags, the .xdsl file and the CPTs are not changed.

Runs (identities from the filing in src/rosbags/README.md):
  SN  paper/nominal_run01_shadow_20261027           nominal simulation, shadow mode: whole mission
  SC  paper/sf_field_twin_run01_closedloop_20261027 closed-loop simulation with disturbances: until 10 s before the first
  FN  field/pier_nominal_run01_shadow_20261025      nominal field run, shadow mode: whole mission, without the left-out
                                                    window of plots/field/<run>/field_run.yaml (13-min hold at H1)
  FC  field/pier_faults_run01_closedloop_20261025   closed-loop field run with injected disturbances: until 10 s before
                                                    the first

The first disturbance of a run is the earliest of
  - an injected disturbance: a non-nominal state on /blueye/virtual/sonar_command or /blueye/virtual/usbl_command, or a
    /scenario/events entry with "source": "command" and a non-nominal state (HUD in the field, field_twin_events in SC);
  - a simulated hazard: a /scenario/events entry with "source": "simulated" and a non-nominal state (field_twin_events
    logs the sonar readings the rocks of scenario SC cause; the rocks themselves have no injection time).
A nominal state on these topics (USBL Strong, sonar Clear) is not a disturbance.

The mission runs from the start of the behaviour tree (its "Takeover mode" log, or the first /mission_state > 0) to
"Mission finished" or "Interrupt signal" (or the end of the bag). Updates are the messages on
/blueye/takeover_request/evidence (about 2 Hz); "evidence" holds the states the network entered, a node that is absent
was not entered (no evidence, e.g. after a timeout or outside its gating).

Unmeasured nodes: some nodes were fixed by design in some runs (UNMEASURED below). They are reported per run but left
out of the pooled frequencies and the comparison with the priors (--pool-unmeasured keeps them).

Usage:
  source ~/Desktop/auto-pilot/install/setup.bash        # or: pip install rosbags (used when ROS 2 is not sourced)
  python3 src/plots/state_frequencies.py [--keep-excluded] [--pool-unmeasured] [--reader rosbags]

Output (src/plots/paper/state_frequencies/):
  nominal_state_frequencies.csv   source, run, phase, node, state, n_observed, fraction, n_episodes, prior. Rows with
                                  state "(not entered)" give the updates without evidence for the node: n_observed is
                                  their number, fraction their share of all updates. run "pooled" sums the runs of a
                                  source. Phases: "all", and each mission phase for ArUcoVisibility, RangeToTarget and
                                  SonarRange.
  bag_inventory.csv               every bag in src/rosbags: run, start, duration, topics with message counts
  state_frequencies_summary.md    runs and segments, injection times, the comparison with the priors, observations
"""

import argparse
import csv
import json
import math
import os
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(HERE)
BAGS = os.path.join(SRC, "rosbags")
XDSL = os.path.join(SRC, "blueye_takeover_bn", "config", "UnifiedMissionDBN_v4.xdsl")
OUT = os.path.join(HERE, "paper", "state_frequencies")

RUNS = {
    "SN": {"bag": "paper/nominal_run01_shadow_20261027", "source": "sim", "what": "nominal simulation, shadow mode"},
    "SC": {"bag": "paper/sf_field_twin_run01_closedloop_20261027", "source": "sim",
           "what": "closed-loop simulation with disturbances (field twin)"},
    "FN": {"bag": "field/pier_nominal_run01_shadow_20261025", "source": "field", "what": "nominal field run, shadow mode",
           "run_file": "field/pier_nominal_run01_shadow_20261025/field_run.yaml"},
    "FC": {"bag": "field/pier_faults_run01_closedloop_20261025", "source": "field",
           "what": "closed-loop field run with injected disturbances"},
}
EVIDENCE_TOPIC = "/blueye/takeover_request/evidence"
TOPICS = [EVIDENCE_TOPIC, "/mission_state", "/rosout", "/scenario/events", "/blueye/virtual/sonar_command",
          "/blueye/virtual/usbl_command"]
TREE_NODES = ("blueye_stonefish_bt", "blueye_real_bt")
NODES = ["Current", "Speed", "BatteryLevel", "USBLStrength", "DVLStatus", "Altitude", "CameraQuality", "SonarRange",
         "RangeToTarget", "ArUcoVisibility", "WaypointTrackingError", "Fatigue", "Stress", "MissionPhase"]
BY_PHASE = ["ArUcoVisibility", "RangeToTarget", "SonarRange"]
PHASES = ["Undocking", "Transit", "Inspection", "Homing", "Docking"]
NOMINAL = {"sonar": "Clear", "usbl": "Strong"}
SKIP_BEFORE_S = 10.0           # data end this long before the first disturbance
GAP_S = 1.5                    # updates further apart than this are not consecutive (an episode ends)
FLAG_DIFF = 0.15
# (run or source, node): why the node was not measured there
UNMEASURED = {
    ("field", "USBLStrength"): "no USBL fitted: virtual_sensors holds Strong unless a state is injected on the HUD",
    ("field", "SonarRange"): "no sonar fitted: virtual_sensors holds Clear unless a state is injected on the HUD",
    ("field", "Current"): "not measured: virtual_sensors publishes 0 m/s",
    ("SC", "USBLStrength"): "held Strong by field_twin_events from the start (the field run's virtual USBL)",
    ("FN", "ArUcoVisibility"): "entered only from marker detections kept from an earlier attempt ('acquired' gating)",
}


# ── Bags ────────────────────────────────────────────────────────────────────────

def read_topics(bag, topics, reader="auto"):
    """{topic: [(time s, msg)]}, receive times since the epoch; rosbag2_py if ROS 2 is sourced, else rosbags."""
    if reader != "rosbags":
        try:
            import rosbag2_py
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.utilities import get_message
        except ImportError:
            if reader == "rosbag2_py":
                raise
        else:
            r = rosbag2_py.SequentialReader()
            r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
            types = {t.name: t.type for t in r.get_all_topics_and_types()}
            wanted = [t for t in topics if t in types]
            r.set_filter(rosbag2_py.StorageFilter(topics=wanted))
            out = {t: [] for t in topics}
            classes = {t: get_message(types[t]) for t in wanted}
            while r.has_next():
                topic, raw, stamp = r.read_next()
                out[topic].append((stamp / 1e9, deserialize_message(raw, classes[topic])))
            return out
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    out = {t: [] for t in topics}
    with AnyReader([Path(bag)], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as r:
        connections = [c for c in r.connections if c.topic in topics]
        for conn, stamp, raw in r.messages(connections=connections):
            out[conn.topic].append((stamp / 1e9, r.deserialize(raw, conn.msgtype)))
    return out


def inventory():
    """Every bag under src/rosbags, from its metadata.yaml."""
    known = {cfg["bag"]: run for run, cfg in RUNS.items()}
    rows = []
    for meta in sorted(Path(BAGS).rglob("metadata.yaml")):
        rel = str(meta.parent.relative_to(BAGS))
        try:
            info = yaml.safe_load(open(meta))["rosbag2_bagfile_information"]
            start = datetime.fromtimestamp(info["starting_time"]["nanoseconds_since_epoch"] / 1e9)
            topics = sorted((t["topic_metadata"]["name"], t["message_count"]) for t in info["topics_with_message_count"])
            duration = round(info["duration"]["nanoseconds"] / 1e9, 1)
        except (KeyError, TypeError, yaml.YAMLError) as e:
            start, topics, duration = None, [], f"unreadable metadata ({e})"
        folder = rel.split("/")[0]
        rows.append({
            "bag": rel,
            "run": known.get(rel, "not used"),
            "folder": folder if "/" in rel else "(top level)",
            "start": start.strftime("%Y-%m-%d %H:%M:%S") if start else "",
            "duration_s": duration,
            "n_topics": len(topics),
            "evidence_updates": dict(topics).get(EVIDENCE_TOPIC, 0),
            "topics": "; ".join(f"{n} ({c})" for n, c in topics),
        })
    return rows


# ── Runs ────────────────────────────────────────────────────────────────────────

def mission_window(data):
    states = [t for t, m in data["/mission_state"] if m.data > 0]
    tree = [t for t, m in data["/rosout"] if m.name in TREE_NODES and m.msg.startswith("Takeover mode")]
    start = min(([states[0]] if states else []) + tree[:1])
    stops = [t for t, m in data["/rosout"] if m.name in TREE_NODES and t > start
             and (m.msg.startswith("Mission finished") or "Interrupt signal" in m.msg)]
    end = stops[0] if stops else data[EVIDENCE_TOPIC][-1][0]
    return start, end


def disturbances(data):
    """[(time, description)] of injected disturbances and simulated hazards (see the module docstring)."""
    out = []
    for t, m in data["/scenario/events"]:
        e = json.loads(m.data)
        if e.get("event") in NOMINAL and e.get("state") != NOMINAL[e["event"]]:
            kind = "injected" if e.get("source") == "command" else "simulated hazard"
            out.append((t, f"{e['event']} {e['state']} ({kind}, /scenario/events)"))
    for topic, kind in (("/blueye/virtual/sonar_command", "sonar"), ("/blueye/virtual/usbl_command", "usbl")):
        for t, m in data[topic]:
            if m.data and m.data != NOMINAL[kind] and m.data != "Real":
                out.append((t, f"{kind} {m.data} (injected, {topic})"))
    merged = []                        # the same change on /blueye/virtual/* and /scenario/events: one entry
    for t, d in sorted(out):
        what = d.split(" (")[0]
        if merged and merged[-1][1].split(" (")[0] == what and t - merged[-1][0] < 0.5:
            merged[-1] = (merged[-1][0], merged[-1][1][:-1] + " and " + d.split(", ", 1)[1])
        else:
            merged.append((t, d))
    return merged


def excluded_windows(cfg, keep):
    if keep or "run_file" not in cfg:
        return []
    run = yaml.safe_load(open(os.path.join(HERE, cfg["run_file"])))
    return [tuple(datetime.strptime(x, "%Y-%m-%d %H:%M:%S").timestamp() for x in w) for w in run.get("exclude", [])]


def segment(run, cfg, args):
    data = read_topics(os.path.join(BAGS, cfg["bag"]), TOPICS, args.reader)
    t0 = data[EVIDENCE_TOPIC][0][0]
    start, end = mission_window(data)
    dist = disturbances(data)
    first = next(((t, d) for t, d in dist if t >= start), None)
    stop = min(end, first[0] - SKIP_BEFORE_S) if first else end
    windows = excluded_windows(cfg, args.keep_excluded)
    updates = [(t, json.loads(m.data)["evidence"]) for t, m in data[EVIDENCE_TOPIC]
               if start <= t < stop and not any(a <= t <= b for a, b in windows)]
    bag_start = min(series[0][0] for series in data.values() if series)
    return {
        "run": run, "source": cfg["source"], "bag": cfg["bag"], "what": cfg["what"], "updates": updates,
        "bag_start": bag_start, "start": start, "end": end, "stop": stop, "first": first, "disturbances": dist,
        "windows": windows, "all_updates": len(data[EVIDENCE_TOPIC]),
        "before_start": sum(1 for t, _ in data[EVIDENCE_TOPIC] if t < start),
        "excluded": sum(1 for t, _ in data[EVIDENCE_TOPIC] if start <= t < stop and any(a <= t <= b for a, b in windows)),
        "scenario_nominal": [(t, json.loads(m.data)) for t, m in data["/scenario/events"]
                             if json.loads(m.data).get("state") == NOMINAL.get(json.loads(m.data).get("event"))],
        "t0": t0,
    }


def unmeasured(run, source, node):
    return UNMEASURED.get((run, node)) or UNMEASURED.get((source, node))


# ── Frequencies ─────────────────────────────────────────────────────────────────

def count(updates, node):
    """{state: [n, episodes]} with "(not entered)" for updates without evidence for the node; an episode is a stretch of
    consecutive updates (no more than GAP_S apart) in the same state."""
    out, prev_state, prev_t = {}, None, None
    for t, ev in updates:
        state = ev.get(node, "(not entered)")
        new = state != prev_state or prev_t is None or t - prev_t > GAP_S
        entry = out.setdefault(state, [0, 0])
        entry[0] += 1
        entry[1] += 1 if new else 0
        prev_state, prev_t = state, t
    return out


def priors():
    """{node: {state: prior}} of the evidence nodes; Speed (parent Current) gets the marginal implied by Current."""
    root = ET.parse(XDSL).getroot()
    cpts = {c.get("id"): c for c in root.find("nodes").iter("cpt")}
    out, parents = {}, {}
    for node in NODES:
        c = cpts[node]
        states = [s.get("id") for s in c.findall("state")]
        p = [float(v) for v in c.find("probabilities").text.split()]
        par = c.find("parents")
        parents[node] = par.text.split() if par is not None else []
        if not parents[node]:
            out[node] = dict(zip(states, p))
    for node, pars in parents.items():
        if len(pars) == 1 and pars[0] in out:              # marginal over the single (root) parent
            states = [s.get("id") for s in cpts[node].findall("state")]
            p = [float(v) for v in cpts[node].find("probabilities").text.split()]
            k = len(states)
            out[node] = {s: sum(prior * p[i * k + j] for i, prior in enumerate(out[pars[0]].values()))
                         for j, s in enumerate(states)}
    temporal = sorted({c.get("id") for c in root.find("dynamic").iter("cpt") if c.get("id") in NODES}) \
        if root.find("dynamic") is not None else []
    conditional = {}                   # {node: {parent state: {state: p}}} for single-parent nodes
    for node, pars in parents.items():
        if len(pars) == 1:
            states = [s.get("id") for s in cpts[node].findall("state")]
            p = [float(v) for v in cpts[node].find("probabilities").text.split()]
            conditional[node] = {ps: dict(zip(states, p[i * len(states):(i + 1) * len(states)]))
                                 for i, ps in enumerate(s.get("id") for s in cpts[pars[0]].findall("state"))}
    return out, parents, temporal, conditional


def table(segments, prior, pool_unmeasured):
    rows = []

    def add(source, run, phase, node, counts, n_updates):
        n_obs = sum(v[0] for s, v in counts.items() if s != "(not entered)")
        states = list(prior.get(node, {})) + sorted(s for s in counts if s not in prior.get(node, {}) and s != "(not entered)")
        for s in states:
            n, ep = counts.get(s, [0, 0])
            rows.append({"source": source, "run": run, "phase": phase, "node": node, "state": s, "n_observed": n,
                         "fraction": round(n / n_obs, 4) if n_obs else "", "n_episodes": ep,
                         "prior": round(prior[node][s], 4) if s in prior.get(node, {}) else ""})
        n, ep = counts.get("(not entered)", [0, 0])
        rows.append({"source": source, "run": run, "phase": phase, "node": node, "state": "(not entered)",
                     "n_observed": n, "fraction": round(n / n_updates, 4) if n_updates else "", "n_episodes": ep,
                     "prior": ""})

    for source in ("sim", "field"):
        runs = [s for s in segments if s["source"] == source]
        for node in NODES:
            for phase in ["all"] + (PHASES if node in BY_PHASE else []):
                pooled, pooled_n = {}, 0
                for seg in runs:
                    ups = seg["updates"] if phase == "all" else [(t, e) for t, e in seg["updates"]
                                                                 if e.get("MissionPhase") == phase]
                    counts = count(ups, node)
                    add(source, seg["run"], phase, node, counts, len(ups))
                    if pool_unmeasured or not unmeasured(seg["run"], source, node):
                        pooled_n += len(ups)
                        for s, (n, ep) in counts.items():
                            pooled.setdefault(s, [0, 0])
                            pooled[s][0] += n
                            pooled[s][1] += ep
                if pooled_n:
                    add(source, "pooled", phase, node, pooled, pooled_n)
    return rows


# ── Summary ─────────────────────────────────────────────────────────────────────

def fmt_t(seg, t):
    return f"{t - seg['start']:.1f} s"


def summary(segments, rows, inv, prior, parents, temporal, conditional, args):
    get = {(r["source"], r["run"], r["phase"], r["node"], r["state"]): r for r in rows}
    L = ["# Observed state frequencies of the evidence nodes against their priors", "",
         f"Generated by `src/plots/state_frequencies.py` from `{os.path.relpath(XDSL, SRC)}` and the bags below. "
         f"Analysis only; nothing was changed.", "",
         "## Runs and segments used", "",
         "| Run | Bag | What | Mission (bag time) | First disturbance (mission time) | Data used (mission time) | "
         "Updates used |", "|---|---|---|---|---|---|---|"]
    for seg in segments:
        first = f"{fmt_t(seg, seg['first'][0])}: {seg['first'][1]}" if seg["first"] else "none"
        used = f"0 to {seg['stop'] - seg['start']:.1f} s"
        if seg["windows"]:
            used += "; without " + ", ".join(f"{a - seg['start']:.0f}–{b - seg['start']:.0f} s"
                                              for a, b in seg["windows"])
        L.append(f"| {seg['run']} | `{seg['bag']}` | {seg['what']} | {seg['start'] - seg['bag_start']:.1f} to "
                 f"{seg['end'] - seg['bag_start']:.1f} s | {first} | {used} | {len(seg['updates'])} of "
                 f"{seg['all_updates']} |")
    L += ["",
          "Mission time starts at the start of the behaviour tree. Updates before it (vehicle not yet in the mission) "
          "are not used: " + ", ".join(f"{s['run']} {s['before_start']}" for s in segments) + ". "
          + ("FN without its left-out window (`plots/field/pier_nominal_run01_shadow_20261025/field_run.yaml`: the "
             f"13-min hold at the last homing waypoint, {next(s['excluded'] for s in segments if s['run'] == 'FN')} "
             "updates), as in the paper figures." if not args.keep_excluded else "FN with its 13-min hold."), "",
          "## How the injection times were found", ""]
    for seg in segments:
        if not seg["disturbances"]:
            L.append(f"- **{seg['run']}**: no disturbance on `/scenario/events` or `/blueye/virtual/*_command`; whole "
                     "mission used.")
            continue
        listed = "; ".join(f"{fmt_t(seg, t)} {d}" for t, d in seg["disturbances"][:4])
        L.append(f"- **{seg['run']}**: first disturbance at {fmt_t(seg, seg['first'][0])} ({seg['first'][1]}), so data "
                 f"end at {seg['stop'] - seg['start']:.1f} s. Disturbances in order: {listed}"
                 + (" …" if len(seg["disturbances"]) > 4 else "") + ".")
        for t, e in seg["scenario_nominal"]:
            if t >= seg["first"][0]:           # only the commands before the cut matter here
                continue
            L.append(f"  - {fmt_t(seg, t)}: {e['event']} {e['state']} ({e.get('source')}) is a nominal state, not a "
                     "disturbance.")
    L += ["- In SC the rocks are static objects with no injection time. `field_twin_events` logs the sonar readings "
          "they cause on `/scenario/events` (`source: simulated`), and the first of them (sonar Near at rock A) comes "
          "before the first scripted USBL change. In FC the HUD's injection commands on `/blueye/virtual/*_command` "
          "and `/scenario/events` give the same times.", "",
          "## Not measured in some runs", "",
          "Reported per run in the CSV, left out of the pooled frequencies and the comparison" +
          (" (kept here: --pool-unmeasured)" if args.pool_unmeasured else "") + ":", ""]
    for (who, node), why in UNMEASURED.items():
        L.append(f"- {node} in {who}: {why}.")
    L += ["- Fatigue and Stress come from the synthetic operator model (linear rise over 15 min) in every run, so "
          "their frequencies describe that model, not an operator.", "",
          "## Comparison with the priors (all phases, pooled per source)", "",
          f"Fraction of the updates with evidence for the node (n). Flags: **never** = state never observed; "
          f"**Δ** = |prior − observed| > {FLAG_DIFF} (sim, field). '–' = not measured in that source.",
          ""]
    if "Speed" in parents and parents["Speed"]:
        low = conditional["Speed"].get("Low", {})
        L.append("Speed has the parent Current; its prior is the marginal implied by the prior of Current. Current was "
                 "Low in every run, where the CPT gives P(Speed | Current = Low) = "
                 + ", ".join(f"{s} {v:.2f}" for s, v in low.items()) + ". "
                 + (f"{', '.join(temporal)} also carry over from the previous time slice (temporal CPT); their prior "
                    "is that of the first slice." if temporal else ""))
        L.append("")
    L += ["| Node | State | Prior | Sim | Field | Flags |", "|---|---|---|---|---|---|"]
    flags_all = []
    for node in NODES:
        n_tot = {}
        for src in ("sim", "field"):
            n_tot[src] = sum(r["n_observed"] for k, r in get.items()
                             if k[:4] == (src, "pooled", "all", node) and k[4] != "(not entered)")
        for s, p in prior[node].items():
            cells, flags = [], []
            for src in ("sim", "field"):
                r = get.get((src, "pooled", "all", node, s))
                if r is None or not n_tot[src]:            # not measured, or never entered: no data, no flags
                    cells.append("–")
                    continue
                cells.append(f"{r['fraction']:.2f} ({r['n_observed']})" if r["fraction"] != "" else "0 (0)")
                if r["n_observed"] == 0:
                    flags.append(f"never ({src})")
                elif abs(p - r["fraction"]) > FLAG_DIFF:
                    flags.append(f"Δ {r['fraction'] - p:+.2f} ({src})")
            if any(f.startswith("Δ") for f in flags):
                flags_all.append((node, s, p, flags))
            L.append(f"| {node} | {s} | {p:.2f} | {cells[0]} | {cells[1]} | {'; '.join(flags)} |")
    L += ["", "Updates with no evidence for the node (share of the updates used, pooled):", "",
          "| Node | Sim | Field |", "|---|---|---|"]
    for node in NODES:
        cells = []
        for src in ("sim", "field"):
            r = get.get((src, "pooled", "all", node, "(not entered)"))
            cells.append(f"{r['fraction']:.2f}" if r else "–")
        L.append(f"| {node} | {cells[0]} | {cells[1]} |")
    L += ["", "## By mission phase (pooled per source)", "",
          "| Node | Phase | Sim: observed states (share) | Sim: not entered (of n updates) | "
          "Field: observed states (share) | Field: not entered (of n updates) |", "|---|---|---|---|---|---|"]
    for node in BY_PHASE:
        for phase in PHASES:
            cells = []
            for src in ("sim", "field"):
                obs = [get[k] for k in get if k[:4] == (src, "pooled", phase, node) and k[4] != "(not entered)"
                       and get[k]["n_observed"]]
                ne = get.get((src, "pooled", phase, node, "(not entered)"))
                cells.append(", ".join(f"{r['state']} {r['fraction']:.2f}" for r in obs) or "–")
                cells.append(f"{ne['fraction']:.2f} of {sum(r['n_observed'] for r in obs) + ne['n_observed']}"
                             if ne else "–")
            L.append(f"| {node} | {phase} | {cells[0]} | {cells[1]} | {cells[2]} | {cells[3]} |")
    L += ["", "## Observations", ""] + observations(segments, get, prior, flags_all) + [
          "", "## Bag inventory", "",
          f"{len(inv)} bags under `src/rosbags` (full list with topics in `bag_inventory.csv`). Used: "
          + ", ".join(f"{r['run']} `{r['bag']}` ({r['duration_s']} s, {r['n_topics']} topics)"
                      for r in inv if r["run"] != "not used") + ". Not used:", ""]
    folders = {}
    for r in inv:
        if r["run"] == "not used":
            folders.setdefault(r["folder"], []).append(r)
    for f, rs in sorted(folders.items()):
        L.append(f"- {f}: {len(rs)} bags ({FOLDER_NOTES.get(f, '')})")
    return "\n".join(L) + "\n"


FOLDER_NOTES = {
    "archive_2026-06_to_08": "older BN and BT versions, June to August 2026",
    "archive_v3_layout_2026-10-20": "simulation runs in the earlier layout v3",
    "development_2026-10_bn_v4": "bring-up and failed attempts, October 2026",
    "field": "field attempts that were not used (field/excluded_20261025, reasons in its README)",
    "(top level)": "unfiled real-drone test recordings of 22 and 23 October (pool)",
    "paper": "",
}


def observations(segments, get, prior, flags_all):
    """Data-driven remarks: the largest differences and where the observed mode differs from the prior's."""
    out = []
    for src in ("sim", "field"):
        n = sum(len(s["updates"]) for s in segments if s["source"] == src)
        out.append(f"- {src}: {n} updates ({n / 2 / 60:.0f} min at 2 Hz) from "
                   + ", ".join(f"{s['run']} ({len(s['updates'])})" for s in segments if s["source"] == src) + ".")
    for node in NODES:
        for src in ("sim", "field"):
            obs = {k[4]: r for k, r in get.items() if k[:4] == (src, "pooled", "all", node) and k[4] != "(not entered)"}
            if not obs or not sum(r["n_observed"] for r in obs.values()):
                continue
            mode = max(obs.values(), key=lambda r: r["n_observed"])
            pmode = max(prior[node], key=prior[node].get)
            if mode["state"] != pmode:
                out.append(f"- {node} ({src}): most frequent observed state {mode['state']} "
                           f"({mode['fraction']:.2f}, {mode['n_episodes']} episode"
                           f"{'' if mode['n_episodes'] == 1 else 's'}), prior mode {pmode} "
                           f"({prior[node][pmode]:.2f}).")
    for src in ("sim", "field"):
        single = []
        for node in NODES:
            obs = [r for k, r in get.items() if k[:4] == (src, "pooled", "all", node) and k[4] != "(not entered)"
                   and r["n_observed"]]
            if len(obs) == 1:
                single.append(f"{node} {obs[0]['state']} ({obs[0]['n_observed']} updates, {obs[0]['n_episodes']} "
                              f"episode{'' if obs[0]['n_episodes'] == 1 else 's'})")
        if single:
            out.append(f"- Never left one state ({src}): " + "; ".join(single) + ". Their other states were not "
                       "exercised in normal operation, so the data cannot check the prior mass on them.")
    for src in ("sim", "field"):
        missing = [(node, r["fraction"]) for node in NODES
                   for r in [get.get((src, "pooled", "all", node, "(not entered)"))]
                   if r and r["fraction"] != "" and r["fraction"] >= 0.05]
        if missing:
            out.append(f"- Evidence missing in part of the updates ({src}), where the network relied on the prior or "
                       "the previous slices: " + ", ".join(f"{n} {f:.2f}" for n, f in missing) + ".")
    for src in ("sim", "field"):
        obs = [r for k, r in get.items() if k[0] == src and k[1] == "pooled" and k[2] == "all"
               and k[4] != "(not entered)" and r["n_observed"]]
        if obs:
            out.append(f"- Successive updates are strongly correlated ({src}): {sum(r['n_observed'] for r in obs)} "
                       f"node-state observations fall in {sum(r['n_episodes'] for r in obs)} episodes, so there are "
                       "far fewer independent observations than updates.")
    big = sorted(flags_all, key=lambda f: -max(abs(float(x.split()[1])) for x in f[3] if x.startswith("Δ")))[:6]
    if big:
        out.append("- Largest differences from the prior: " + "; ".join(
            f"{node} {s} (prior {p:.2f}: " + ", ".join(x for x in flags if x.startswith("Δ")) + ")"
            for node, s, p, flags in big) + ".")
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--keep-excluded", action="store_true",
                        help="keep FN's left-out window (13-min hold) instead of leaving it out")
    parser.add_argument("--pool-unmeasured", action="store_true",
                        help="pool and compare the nodes in UNMEASURED like measured ones")
    parser.add_argument("--reader", choices=["auto", "rosbag2_py", "rosbags"], default="auto")
    parser.add_argument("--out", default=OUT)
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    inv = inventory()
    with open(os.path.join(args.out, "bag_inventory.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(inv[0]))
        w.writeheader()
        w.writerows(inv)
    print(f"bag_inventory.csv: {len(inv)} bags")

    prior, parents, temporal, conditional = priors()
    segments = []
    for run, cfg in RUNS.items():
        print(f"reading {run} ({cfg['bag']}) ...")
        segments.append(segment(run, cfg, args))
    rows = table(segments, prior, args.pool_unmeasured)
    with open(os.path.join(args.out, "nominal_state_frequencies.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["source", "run", "phase", "node", "state", "n_observed", "fraction",
                                          "n_episodes", "prior"])
        w.writeheader()
        w.writerows(rows)
    print(f"nominal_state_frequencies.csv: {len(rows)} rows")
    with open(os.path.join(args.out, "state_frequencies_summary.md"), "w") as f:
        f.write(summary(segments, rows, inv, prior, parents, temporal, conditional, args))
    print(f"state_frequencies_summary.md written to {args.out}")


if __name__ == "__main__":
    main()
