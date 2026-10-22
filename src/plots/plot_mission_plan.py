#!/usr/bin/env python3
"""
Plan view of the simulated inspection mission, drawn from the behaviour tree itself so the picture always matches
what the vehicle does: waypoints from the mission sequence of the BT (NavigateToWaypoint / StationKeeping between
PublishState nodes), ArUco markers from the pipeline scenario, docking station and pipeline footprints from their
meshes, and the start (docked) pose from the scenario config.

Usage:
  source ~/Desktop/auto-pilot/install/setup.bash
  python3 src/plots/plot_mission_plan.py            # default: TestMission.xml -> src/plots/paper/mission_plan/
  python3 src/plots/plot_mission_plan.py --bt <tree.xml> --out <dir>
"""

import argparse
import math
import os
import re
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Polygon

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(HERE)

# Phase colours: first five slots of the validated categorical palette, in phase order (adjacent phases separate
# for colour-blind readers); every phase is also labelled in text on the route.
PHASES = {1: ("Undocking", "#2a78d6"), 2: ("Transit", "#eb6834"), 3: ("Inspection", "#1baf7a"),
          4: ("Homing", "#eda100"), 5: ("Docking", "#e87ba4")}
INK, INK_MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
DOCK_FILL, PIPE_FILL = "#a8a7a1", "#c9b49b"

NAMES = {"exit_dock": "Dock exit", "homing_midpoint": "Homing midpoint", "dock_approach": "Dock approach"}
# label offsets (points) for the default mission; anything else is labelled up-right
OFFSETS = {"exit_dock": (-44, -30), "nav_aruco_05": (-62, -24), "nav_aruco_06": (8, -8), "nav_aruco_04": (-10, -26),
           "nav_aruco_03": (-80, 6), "nav_aruco_02": (-28, 12), "homing_midpoint": (-26, 10),
           "dock_approach": (-22, 10)}

plt.rcParams.update({"font.size": 8, "axes.labelsize": 8, "legend.fontsize": 7, "xtick.labelsize": 7,
                     "ytick.labelsize": 7, "pdf.fonttype": 42, "ps.fonttype": 42,
                     "axes.edgecolor": INK_MUTED, "xtick.color": INK_MUTED, "ytick.color": INK_MUTED})


def mission_steps(bt_path):
    """[(phase, kind, name, north, east, depth)] from the mission sequence of the behaviour tree."""
    root = ET.parse(bt_path).getroot()
    seq = root.find(".//Sequence[@name='mission']")
    steps, phase = [], None
    for e in seq:
        if e.tag == "PublishState":
            phase = int(e.get("state"))
        elif e.tag in ("NavigateToWaypoint", "StationKeeping"):
            steps.append((phase, e.tag, e.get("name"), float(e.get("x")), float(e.get("y")), float(e.get("z"))))
    return steps


def scenario():
    share = os.path.join(SRC, "vortex-stonefish-sim", "stonefish_sim")
    with open(os.path.join(share, "config", "blueye_config.yaml")) as f:
        config = yaml.safe_load(f)
    return share, config


def pose(config, key):
    x, y, _ = (float(v) for v in str(config[f"{key}_position"]).split())
    return x, y, float(str(config[f"{key}_orientation"]).split()[2])


def footprint(share, config, mesh, key):
    x, y, yaw = pose(config, key)
    vertices, faces = [], []
    with open(os.path.join(share, "data", "object_files", f"{mesh}.obj")) as f:
        for line in f:
            if line.startswith("v "):
                vertices.append([float(v) for v in line.split()[1:3]])
            elif line.startswith("f "):
                faces.append([int(v.split("/")[0]) - 1 for v in line.split()[1:]])
    v = np.array(vertices)
    c, s = math.cos(yaw), math.sin(yaw)
    world = np.stack([x + c * v[:, 0] - s * v[:, 1], y + s * v[:, 0] + c * v[:, 1]], axis=1)
    return [world[face] for face in faces]


def markers(share, config):
    """{name: (north, east, yaw)} of the ArUco markers on the pipeline, in world coordinates."""
    px, py, pyaw = pose(config, "pipeline")
    text = open(os.path.join(share, "objects", "pipe.scn")).read()
    out = {}
    for name, xyz, rpy in re.findall(r'<link name="(aruco_\d+)".*?<origin xyz="([^"]+)" rpy="([^"]+)"', text, re.S):
        x, y, _ = (float(v) for v in xyz.split())
        c, s = math.cos(pyaw), math.sin(pyaw)
        out[name] = (px + c * x - s * y, py + s * x + c * y, pyaw + float(rpy.split()[2]))
    return out


def arrow(ax, a, b, colour, dashed=False):
    ax.annotate("", xy=(b[1], b[0]), xytext=(a[1], a[0]),
                arrowprops=dict(arrowstyle="-|>", color=colour, lw=2.0, shrinkA=6, shrinkB=7,
                                linestyle=(0, (4, 2)) if dashed else "-", mutation_scale=10))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bt", default=os.path.join(SRC, "blueye_stonefish_bt", "behavior_trees", "TestMission.xml"))
    parser.add_argument("--out", default=os.path.join(HERE, "paper", "mission_plan"))
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    share, config = scenario()
    steps = mission_steps(args.bt)
    start_n, start_e, _ = pose(config, "blueye")
    tags = markers(share, config)

    fig, ax = plt.subplots(figsize=(7.2, 3.4))
    structures = []
    for faces, colour in [(footprint(share, config, "docking_station_rig", "docking_rig"), DOCK_FILL),
                          (footprint(share, config, str(config.get("pipeline_mesh", "pipe2")), "pipeline"), PIPE_FILL)]:
        for f in faces:
            ax.add_patch(Polygon(f[:, ::-1], closed=True, color=colour, lw=0, zorder=1))
            structures.append(f)

    # waypoints in order; consecutive steps at the same position (transit end = first marker, heading hold) merge
    points = []
    for phase, kind, name, n, e, z in steps:
        if points and math.hypot(points[-1]["n"] - n, points[-1]["e"] - e) < 0.05:
            if kind == "StationKeeping":
                points[-1]["hold"] = True
            else:
                points[-1]["names"].append(name)
            continue
        points.append({"phase": phase, "names": [name], "n": n, "e": e, "z": z, "hold": False})

    # route: undocking from the docked pose, the legs between waypoints, docking back into the dock (dashed,
    # offset so it does not hide the undocking leg)
    prev = (start_n - 0.25, start_e)
    for i, p in enumerate(points):
        a = (prev[0] - 0.25, prev[1]) if i == 0 else prev
        b = (p["n"] - 0.25, p["e"]) if i == 0 else (p["n"], p["e"])
        arrow(ax, a, b, PHASES[p["phase"]][1])
        prev = (p["n"], p["e"])
    arrow(ax, (prev[0] + 0.3, prev[1]), (start_n + 0.3, start_e), PHASES[5][1], dashed=True)

    visited = set()
    for p in points:
        for name, (n, e, yaw) in tags.items():
            if math.hypot(n - p["n"], e - p["e"]) < 0.1:
                visited.add(name)
    for name, (n, e, yaw) in tags.items():
        h, c, s = 0.09, math.cos(yaw), math.sin(yaw)
        sq = np.array([[-h, -h], [h, -h], [h, h], [-h, h]]) @ np.array([[c, s], [-s, c]]) + [n, e]
        ax.add_patch(Polygon(sq[:, ::-1], closed=True, color=INK if name in visited else "#9b9a95", zorder=3))

    # waypoint circles; inspection waypoints get the marker number below the pipe, the others name and position
    for i, p in enumerate(points, 1):
        ax.plot(p["e"], p["n"], "o", mfc="white", mec=INK, mew=1.0, ms=10, zorder=5)
        ax.text(p["e"], p["n"], str(i), ha="center", va="center", fontsize=6.5, fontweight="bold", color=INK, zorder=6)
        aruco = next((nm for nm in p["names"] if nm.startswith("nav_aruco")), None)
        if aruco:
            text = f"ArUco {aruco[-2:]}"
            if aruco.endswith("01"):
                text += f"\n({p['n']:.1f}, {p['e']:.1f})\nz {p['z']:.2f} m"
            ax.annotate(text, (p["e"], p["n"]), xytext=(0, -11), textcoords="offset points", ha="center", va="top",
                        fontsize=6.3, color=INK)
            continue
        label = next((NAMES[nm] for nm in p["names"] if nm in NAMES), p["names"][0])
        text = f"{label}\n({p['n']:.1f}, {p['e']:.1f}), z {p['z']:.2f} m" + ("\nface the dock, 30 s" if p["hold"] else "")
        below = p["names"][0] == "exit_dock"
        ax.annotate(text, (p["e"], p["n"]), xytext=(4, -11) if below else (4, 9), textcoords="offset points",
                    ha="left", va="top" if below else "bottom", fontsize=6.3, color=INK)
    ax.plot(start_e, start_n, "D", color=INK, ms=5, zorder=6)

    # leg lengths and phase names along the route (text in ink; the colour is only on the arrows)
    def along(a, b, label, dn=0.0, rotate=True):
        angle = math.degrees(math.atan2(b["n"] - a["n"], b["e"] - a["e"])) if rotate else 0.0
        if angle > 90 or angle < -90:
            angle -= 180 * (1 if angle > 0 else -1)
        ax.text((a["e"] + b["e"]) / 2, (a["n"] + b["n"]) / 2 + dn, label, fontsize=6.8, color=INK, ha="center",
                va="bottom", rotation=angle, rotation_mode="anchor", fontstyle="italic")
    first = {}
    for i, p in enumerate(points):
        first.setdefault(p["phase"], i)
    last = {ph: max(i for i, p in enumerate(points) if p["phase"] == ph) for ph in first}
    if 1 in first and 3 in first:                                  # transit: dock exit -> first inspection point
        a, b = points[last[1]], points[first[3] if first.get(2) is None else first[2]]
        along(a, b, f"Transit {math.hypot(b['n'] - a['n'], b['e'] - a['e']):.0f} m", dn=0.6)
    if 3 in first:                                                 # from the transit end, which is the first marker
        a, b = points[last[2]] if 2 in last else points[first[3]], points[last[3]]
        along(a, b, f"Inspection {math.hypot(b['n'] - a['n'], b['e'] - a['e']):.0f} m", dn=1.4)
    if 4 in first:
        prev_p = points[last[3]] if 3 in last else points[first[4]]
        for i in range(first[4], last[4] + 1):
            a, b = (prev_p if i == first[4] else points[i - 1]), points[i]
            along(a, b, f"{'Homing ' if i == first[4] else ''}{math.hypot(b['n'] - a['n'], b['e'] - a['e']):.0f} m",
                  dn=0.5)

    handles = [Line2D([], [], color=PHASES[p][1], lw=2, ls="--" if p == 5 else "-", label=PHASES[p][0])
               for p in sorted(PHASES)]
    handles += [Patch(color=DOCK_FILL, label="Docking station"), Patch(color=PIPE_FILL, label="Pipeline"),
                Line2D([], [], marker="s", color=INK, ls="", ms=5, label="ArUco marker (inspected)"),
                Line2D([], [], marker="D", color=INK, ls="", ms=5, label="Start / end (docked)"),
                Line2D([], [], marker="o", mfc="white", mec=INK, ls="", ms=8, label="Waypoint (order)")]
    if len(visited) < len(tags):
        handles.insert(-2, Line2D([], [], marker="s", color="#9b9a95", ls="", ms=5, label="ArUco marker (not visited)"))
    ax.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=5, frameon=False)
    ax.set_aspect("equal")
    ax.set_xlabel("East [m]", color=INK)
    ax.set_ylabel("North [m]", color=INK)
    ax.grid(color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    allpts = np.concatenate([np.array([[p["n"], p["e"]] for p in points] + [[start_n, start_e]])] + structures)
    ax.set_xlim(allpts[:, 1].min() - 2.5, allpts[:, 1].max() + 4.5)
    ax.set_ylim(allpts[:, 0].min() - 7.0, allpts[:, 0].max() + 5.5)
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(args.out, f"mission_plan.{ext}"), dpi=300, bbox_inches="tight")
    print(f"Saved {os.path.join(args.out, 'mission_plan')}.pdf/.png")
    for i, p in enumerate(points, 1):
        print(f"  {i}. {PHASES[p['phase']][0]:10s} {', '.join(p['names']):34s} ({p['n']:.2f}, {p['e']:.2f}, {p['z']:.2f})"
              + ("  + heading hold" if p["hold"] else ""))


if __name__ == "__main__":
    main()
