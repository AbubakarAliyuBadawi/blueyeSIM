#!/usr/bin/env python3
"""
Mission map of the simulation study for the paper: the nominal run (SN) and the closed-loop twin of the field run (SC)
in the same layout, with close-ups of the two rocks where the supervisor steered round them.

Usage:
  source ~/Desktop/auto-pilot/install/setup.bash
  python3 src/plots/plot_sim_maps.py      # default runs below; --s0 / --sf <bag> for others

Output (src/plots/paper/sim_results/), PDF (paper) and PNG (viewing):
  sim_mission_maps   (a) SN track coloured by phase, (b) SC track with rocks A and B and the takeovers TO1-TO3,
                     (c) and (d) close-ups of rocks A and B: planned transit leg, request and hand-back positions
"""

import argparse
import bisect
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.collections import LineCollection

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plot_field_run as pfr  # noqa: E402
import plot_nominal_run as pnr  # noqa: E402

BAGS = os.path.join(HERE, "..", "rosbags", "paper")
SN = "nominal_run01_shadow_20261027"
SC = "sf_field_twin_run01_closedloop_20261027"
ROCK_NAMES = {"transit_rock_a": "A", "transit_rock": "B"}


def load(bag, labels=()):
    data, _ = pfr.load_bag(bag)
    start, end, _ = pfr.mission_window(data)
    segs = pnr.phase_segments(data, start, end)
    odom = [o for o in data["/blueye/odom"] if start <= o[0] <= end]
    groups = pfr.group_takeovers(pfr.takeover_events(data, start, end), list(labels))
    goals, last = [], None
    for t, x, y, z in data["/blueye/waypoint_goal"]:
        if (round(x, 2), round(y, 2)) != last:
            goals.append((x, y))
            last = (round(x, 2), round(y, 2))
    return odom, segs, groups, goals


def track(ax, odom, segs, lw=1.4):
    pts = np.array([[o[2], o[1]] for o in odom])
    colours = [pnr.PHASES.get(pnr.phase_at(segs, o[0]), ("", "grey"))[1] for o in odom[:-1]]
    ax.add_collection(LineCollection(np.stack([pts[:-1], pts[1:]], axis=1), colors=colours, linewidths=lw))
    return pts


def where(odom, t):
    o = odom[min(bisect.bisect_left([s[0] for s in odom], t), len(odom) - 1)]
    return o[2], o[1]                                  # east, north


def structures(ax, grids, rocks):
    for (x0, y0, res, grid), colour in zip(grids, ["#7f7f7f", "#8c564b"]):
        ny, nx = grid.shape
        ax.imshow(np.ma.masked_equal(grid.astype(float), 0).T, origin="lower",
                  extent=(y0, y0 + ny * res, x0, x0 + nx * res),
                  cmap=matplotlib.colors.ListedColormap([colour]), alpha=0.55, interpolation="nearest")
    for rock in rocks:
        ax.fill(rock[:, 1], rock[:, 0], color="#52514e", alpha=0.9, lw=0, zorder=3)


def takeovers(ax, odom, groups, label=True, ms=6):
    for g in groups:
        e, n = where(odom, g["request"])
        ax.plot(e, n, "v", color="#d62728", ms=ms, mec="white", mew=0.5, zorder=7)
        if label:
            ax.text(e, n + 0.9, g["label"], ha="center", va="bottom", fontsize=7, fontweight="bold",
                    color="#d62728", zorder=8)
        if "hand-back" in g:
            e, n = where(odom, g["hand-back"])
            ax.plot(e, n, "^", color="#2a78d6", ms=ms, mec="white", mew=0.5, zorder=7)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--s0", default=os.path.join(BAGS, SN))
    parser.add_argument("--sf", default=os.path.join(BAGS, SC))
    parser.add_argument("--out", default=os.path.join(HERE, "paper", "sim_results"))
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    from ament_index_python.packages import get_package_share_directory
    meshes = os.path.join(get_package_share_directory("stonefish_sim"), "data", "object_files")
    config = os.path.join(HERE, "paper", os.path.basename(os.path.normpath(args.sf)), "scenario_config.yaml")
    with open(config) as fh:
        cfg = yaml.safe_load(fh)
    grids = pnr.structure_grid(config, meshes, [("docking_station_rig", "docking_rig"),
                                                (str(cfg.get("pipeline_mesh", "pipe2")), "pipeline")])
    keys = [k for k in pnr.ROCK_KEYS if f"{k}_position" in cfg]
    rocks = [pnr.rock_outline(config, meshes, key=k) for k in keys]
    centres = [tuple(float(v) for v in str(cfg[f"{k}_position"]).split()[:2]) for k in keys]

    print("Reading the runs ...")
    s0 = load(args.s0)
    sf = load(args.sf, ["TO1", "TO2", "TO3"])

    fig = plt.figure(figsize=(7.2, 5.6))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.0, 1.0, 1.35], hspace=0.42, wspace=0.18)
    a = fig.add_subplot(gs[0, :])
    b = fig.add_subplot(gs[1, :], sharex=a, sharey=a)
    zooms = [fig.add_subplot(gs[2, 0]), fig.add_subplot(gs[2, 1])]

    for ax, (odom, segs, groups, goals), title in (
            (a, s0, "(a) SN, nominal mission, shadow mode"),
            (b, sf, "(b) SC, closed-loop twin of the field run: rocks A and B on the transit, takeovers TO1–TO3")):
        structures(ax, grids, rocks if ax is b else [])
        pts = track(ax, odom, segs)
        ax.plot([g[1] for g in goals], [g[0] for g in goals], "o", mfc="white", mec="black", mew=0.7, ms=4, zorder=5)
        ax.plot(pts[0, 0], pts[0, 1], "D", color="black", ms=4, zorder=6)
        takeovers(ax, odom, groups)
        ax.set_title(title, loc="left", fontsize=7.5, fontweight="bold", pad=3)
        ax.set_aspect("equal")
        ax.set_ylabel("North [m]")
        ax.grid(alpha=0.3, lw=0.5)
    for (n, e), key in zip(centres, keys):
        b.text(e, n - 1.0, ROCK_NAMES.get(key, key), ha="center", va="top", fontsize=7, fontweight="bold",
               color="#52514e")
    a.set_xlim(-25.5, 47)
    a.set_ylim(-5.5, 9)
    plt.setp(a.get_xticklabels(), visible=False)
    b.set_xlabel("East [m]")

    # close-ups: the planned transit leg (exit of the dock to the start of the pipeline), the track and the takeover
    odom, segs, groups, goals = sf
    leg = np.array([[goals[0][1], goals[0][0]], [goals[1][1], goals[1][0]]])
    for ax, rock, (n, e), key, g in zip(zooms, rocks, centres, keys, groups):
        structures(ax, grids, [rock])
        ax.plot(leg[:, 0], leg[:, 1], color="#52514e", lw=0.8, ls="--", zorder=2)
        track(ax, odom, segs, lw=1.8)
        takeovers(ax, odom, [g], label=False, ms=7)
        ax.set_xlim(e - 3.2, e + 3.2)
        ax.set_ylim(n - 2.0, n + 2.0)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3, lw=0.5)
        ax.set_xlabel("East [m]")
        ax.set_title(f"({'c' if ax is zooms[0] else 'd'}) rock {ROCK_NAMES.get(key, key)}, {g['label']}",
                     loc="left", fontsize=7.5, fontweight="bold", pad=3)
    zooms[0].set_ylabel("North [m]")

    handles = [plt.Line2D([], [], color=pnr.PHASES[p][1], lw=2, label=pnr.PHASES[p][0]) for p in (1, 2, 3, 4, 5, 6)]
    handles += [plt.Line2D([], [], color="#52514e", lw=0.8, ls="--", label="Planned transit leg"),
                matplotlib.patches.Patch(color="#7f7f7f", alpha=0.55, label="Docking station"),
                matplotlib.patches.Patch(color="#8c564b", alpha=0.55, label="Pipeline"),
                matplotlib.patches.Patch(color="#52514e", alpha=0.9, label="Rock"),
                plt.Line2D([], [], ls="", marker="o", mfc="white", mec="black", ms=4, label="Waypoint"),
                plt.Line2D([], [], ls="", marker="D", color="black", ms=4, label="Start (docked)"),
                plt.Line2D([], [], ls="", marker="v", color="#d62728", ms=6, label="Takeover request"),
                plt.Line2D([], [], ls="", marker="^", color="#2a78d6", ms=6, label="Hand-back")]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.045), ncol=7, frameon=False, fontsize=6.5,
               handlelength=1.6, columnspacing=1.0)
    pnr.save(fig, args.out, "sim_mission_maps")


if __name__ == "__main__":
    main()
