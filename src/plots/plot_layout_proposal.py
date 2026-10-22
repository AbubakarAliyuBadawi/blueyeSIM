#!/usr/bin/env python3
"""
Proposed longer mission (v4 layout) for discussion, drawn over the seabed of the Stonefish world.

  - straight 30 m pipeline with six markers every 6 m, in the deep corridor east of the dock
  - transit 37 m from the dock exit to the start of the pipeline, S1 rock at two thirds of it
  - inspection eastwards along the pipe, S2 disturbance on the leg after marker 3
  - homing back to the dock centre line and along it to the approach point (dock unchanged)
The current mission (zig-zag pipe, short legs) is drawn faintly for comparison. The timeline is an estimate from the
speed of the vehicle in the current runs.

Usage:
  source ~/Desktop/auto-pilot/install/setup.bash
  python3 src/plots/plot_layout_proposal.py
Output: src/plots/paper/mission_plan/layout_proposal.pdf/.png
"""

import math
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap, BoundaryNorm
from scipy.interpolate import griddata

import plot_hazard_run as H

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "paper", "mission_plan")
INK, MUTED = "#0b0b0b", "#52514e"
PHASE = {"Undocking": "#2a78d6", "Transit": "#eb6834", "Inspection": "#1baf7a", "Homing": "#eda100", "Docking": "#e87ba4"}
DEPTH_LEVELS = [3.0, 3.25, 3.5, 3.75, 4.0, 4.25]
DEPTH_RAMP = ["#eef4fb", "#d9e7f6", "#c2d8f0", "#a6c5e8", "#88b0de"]       # one hue, light (shallow) to dark (deep)
SHALLOW = "#d6d4ce"
plt.rcParams.update({"font.size": 8, "axes.labelsize": 8, "legend.fontsize": 7, "xtick.labelsize": 7,
                     "ytick.labelsize": 7, "pdf.fonttype": 42, "ps.fonttype": 42})

# proposal (north, east) in the Stonefish NED frame; depths as in the current mission
EXIT = (6.5, -20.67)
PIPE_START, PIPE_END, N_MARKERS = (-3.0, 15.0), (-3.0, 45.0), 6
HOMING_MID, DOCK_APPROACH = (6.5, 20.0), (6.5, -19.0)
ROCK_FRACTION, S2_AFTER_MARKER = 2 / 3, 3
CRUISE, PER_WAYPOINT, UNDOCK, DOCK, HEADING_HOLD = 0.12, 30.0, 40.0, 60.0, 120.0   # s, from the current runs


def seabed_grid():
    v = np.array([list(map(float, l.split()[1:4])) for l in
                  open(os.path.join(H.SIM, "data", "object_files", "seafloor.obj")) if l.startswith("v ")])
    r = 1.57                                                          # world_transform rpy="1.57 0 0", z +3.0
    north, east = v[:, 0], v[:, 1] * math.cos(r) - v[:, 2] * math.sin(r)
    depth = v[:, 1] * math.sin(r) + v[:, 2] * math.cos(r) + 3.0
    n, e = np.meshgrid(np.arange(-22, 22.01, 0.5), np.arange(-32, 52.01, 0.5), indexing="ij")
    return n, e, griddata(np.stack([north, east], 1), depth, (n, e), method="linear")


def current_route():
    xml = open(os.path.join(H.SRC, "blueye_stonefish_bt", "behavior_trees", "TestMission.xml")).read()
    pts = []
    for m in re.finditer(r'<NavigateToWaypoint([^>]*)/?>', xml):
        a = dict(re.findall(r'(\w+)="([^"]*)"', m.group(1)))
        if "x" in a and not a.get("name", "").startswith("emergency"):
            pts.append((float(a["x"]), float(a["y"])))
    return pts


def main():
    os.makedirs(OUT, exist_ok=True)
    markers = [(PIPE_START[0], PIPE_START[1] + k * (PIPE_END[1] - PIPE_START[1]) / (N_MARKERS - 1)) for k in range(N_MARKERS)]
    rock = (EXIT[0] + ROCK_FRACTION * (PIPE_START[0] - EXIT[0]), EXIT[1] + ROCK_FRACTION * (PIPE_START[1] - EXIT[1]))
    legs = {"transit": math.dist(EXIT, PIPE_START), "inspection": math.dist(PIPE_START, PIPE_END),
            "homing1": math.dist(PIPE_END, HOMING_MID), "homing2": math.dist(HOMING_MID, DOCK_APPROACH)}

    # timeline estimate (minutes)
    t_transit = UNDOCK
    t_insp = t_transit + PER_WAYPOINT + legs["transit"] / CRUISE
    t_rock = t_transit + PER_WAYPOINT + ROCK_FRACTION * legs["transit"] / CRUISE
    leg_t = PER_WAYPOINT + (legs["inspection"] / (N_MARKERS - 1)) / CRUISE
    t_marker = [t_insp + k * leg_t for k in range(N_MARKERS)]
    t_s2 = t_marker[S2_AFTER_MARKER - 1]
    t_homing = t_marker[-1]
    t_docking = t_homing + 2 * PER_WAYPOINT + (legs["homing1"] + legs["homing2"]) / CRUISE + HEADING_HOLD
    t_end = t_docking + DOCK
    phases = [("Undocking", 0, t_transit), ("Transit", t_transit, t_insp), ("Inspection", t_insp, t_homing),
              ("Homing", t_homing, t_docking), ("Docking", t_docking, t_end)]

    fig = plt.figure(figsize=(7.2, 4.9))
    gs = fig.add_gridspec(2, 1, height_ratios=[4.2, 0.75], hspace=0.12)
    ax = fig.add_subplot(gs[0])
    n, e, d = seabed_grid()
    ax.contourf(e, n, d, levels=[0, DEPTH_LEVELS[0]], colors=[SHALLOW], zorder=0)
    cf = ax.contourf(e, n, d, levels=DEPTH_LEVELS, cmap=ListedColormap(DEPTH_RAMP),
                     norm=BoundaryNorm(DEPTH_LEVELS, len(DEPTH_RAMP)), extend="max", zorder=0)
    ax.text(-30.5, -20.5, "too shallow for the vehicle (seabed < 3 m)", fontsize=6.5, color=INK, va="bottom")

    # current mission, faint
    known, _ = H.footprints(os.path.join(H.SIM, "config", "blueye_config.yaml"))
    for poly in known:
        ax.fill(poly[:, 1], poly[:, 0], color="#b9b7b0", lw=0, zorder=1)
    old = np.array(current_route())
    ax.plot(old[:, 1], old[:, 0], color=MUTED, lw=0.8, ls=(0, (2, 2)), zorder=2)
    ax.text(-8.5, -1.0, "current mission\n(zig-zag pipe, ≈ 45 m)", fontsize=6.3, color=MUTED, ha="center", va="top")

    # proposed pipeline and markers
    ax.plot([PIPE_START[1], PIPE_END[1]], [PIPE_START[0], PIPE_END[0]], color="#8a7766", lw=5, solid_capstyle="round",
            zorder=3)
    for k, (mn, me) in enumerate(markers, 1):
        ax.plot(me, mn, "o", mfc="white", mec=INK, mew=0.8, ms=8, zorder=6)
        ax.text(me, mn, str(k), ha="center", va="center", fontsize=5.5, fontweight="bold", zorder=7)

    # proposed route by phase
    ax.plot([EXIT[1], PIPE_START[1]], [EXIT[0], PIPE_START[0]], color=PHASE["Transit"], lw=1.8, zorder=4)
    ax.plot([PIPE_START[1], PIPE_END[1]], [PIPE_START[0] + 0.9, PIPE_END[0] + 0.9], color=PHASE["Inspection"], lw=1.8,
            zorder=4)
    ax.plot([PIPE_END[1], HOMING_MID[1], DOCK_APPROACH[1]], [PIPE_END[0], HOMING_MID[0], DOCK_APPROACH[0]],
            color=PHASE["Homing"], lw=1.8, zorder=4)
    for a, b in ((PIPE_END, HOMING_MID), (HOMING_MID, DOCK_APPROACH), (EXIT, PIPE_START)):
        mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        ax.annotate("", xy=(mid[1] + 0.01 * (b[1] - a[1]), mid[0] + 0.01 * (b[0] - a[0])), xytext=(mid[1], mid[0]),
                    arrowprops=dict(arrowstyle="-|>", color=INK, lw=0.8, mutation_scale=8), zorder=5)

    # S1 rock and S2 disturbance
    ax.add_patch(plt.Circle((rock[1], rock[0]), 0.45, color=INK, zorder=6))
    ax.annotate(f"S1: rock at 2/3 of the transit\n(≈ {t_rock / 60:.1f} min into the mission)", (rock[1], rock[0]),
                xytext=(rock[1] - 3, rock[0] - 9.5), fontsize=6.5, color=INK, ha="center",
                arrowprops=dict(arrowstyle="-", color=INK, lw=0.6))
    a, b = markers[S2_AFTER_MARKER - 1], markers[S2_AFTER_MARKER]
    ax.plot([a[1], b[1]], [a[0] - 1.6, b[0] - 1.6], color=INK, lw=0.8)
    for x in (a[1], b[1]):
        ax.plot([x, x], [a[0] - 2.0, a[0] - 1.2], color=INK, lw=0.8)
    ax.text((a[1] + b[1]) / 2, a[0] - 2.3, f"S2: cross-current + USBL loss\nafter marker {S2_AFTER_MARKER} "
            f"(≈ {t_s2 / 60:.1f} min)", fontsize=6.5, color=INK, ha="center", va="top")
    ax.annotate("", xy=(a[1] + 1.5, a[0] + 5.2), xytext=(a[1] + 1.5, a[0] + 1.6),
                arrowprops=dict(arrowstyle="-|>", color=MUTED, lw=1.4, mutation_scale=9), zorder=5)
    ax.text(a[1] + 0.9, a[0] + 3.6, "S2 current", fontsize=6.3, color=INK, va="center", ha="right")

    # leg labels
    ax.text(EXIT[1] + 0.62 * (PIPE_START[1] - EXIT[1]), EXIT[0] + 0.62 * (PIPE_START[0] - EXIT[0]) + 1.3,
            f"transit {legs['transit']:.0f} m",
            fontsize=6.8, color=INK, rotation=-math.degrees(math.atan2(PIPE_START[0] - EXIT[0], PIPE_START[1] - EXIT[1])) * -1,
            rotation_mode="anchor", ha="center")
    ax.text((PIPE_START[1] + PIPE_END[1]) / 2, PIPE_START[0] - 7.6,
            f"straight pipeline {legs['inspection']:.0f} m, markers every {legs['inspection'] / (N_MARKERS - 1):.0f} m",
            fontsize=6.8, color=INK, ha="center", va="top")
    ax.text((HOMING_MID[1] + DOCK_APPROACH[1]) / 2, HOMING_MID[0] + 0.8,
            f"homing along the dock centre line {legs['homing2']:.0f} m", fontsize=6.8, color=INK, ha="center")
    ax.text((PIPE_END[1] + HOMING_MID[1]) / 2 + 1.5, (PIPE_END[0] + HOMING_MID[0]) / 2 + 1.0,
            f"homing {legs['homing1']:.0f} m", fontsize=6.8, color=INK, ha="left")
    ax.text(-23.5, 9.7, "dock", fontsize=6.8, color=INK, ha="center")

    # scale bar, axes
    ax.plot([38, 48], [-19, -19], color=INK, lw=1.5)
    ax.text(43, -18.4, "10 m", fontsize=6.5, ha="center", va="bottom")
    ax.set_aspect("equal")
    ax.set_xlim(-32, 52)
    ax.set_ylim(-21, 21)
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    cb = fig.colorbar(cf, ax=ax, fraction=0.025, pad=0.01)
    cb.set_label("Seabed depth [m]")
    total = sum(legs.values()) + 2 * 2.2
    ax.set_title(f"Proposed mission: ≈ {total:.0f} m of path, ≈ {t_end / 60:.0f} min (current: ≈ 45 m, 13 min)",
                 fontsize=8, loc="left")

    # estimated timeline
    tl = fig.add_subplot(gs[1])
    for name, a_, b_ in phases:
        tl.barh(0, (b_ - a_) / 60, left=a_ / 60, height=0.55, color=PHASE[name], lw=0)
        if (b_ - a_) / t_end > 0.08:
            tl.text((a_ + b_) / 120, 0, name, ha="center", va="center", fontsize=6.5, color=INK)
    for t, label, ha in ((t_rock, "S1 rock", "right"), (t_s2, "S2 injection", "left")):
        tl.plot(t / 60, 0.48, "v", color=INK, ms=6)
        tl.text(t / 60 + (0.15 if ha == "left" else -0.15), 0.62, f"{label} ≈ {t / 60:.1f} min", ha=ha,
                va="bottom", fontsize=6.5, color=INK)
    tl.set_xlim(0, t_end / 60)
    tl.set_ylim(-0.5, 1.2)
    tl.set_yticks([])
    tl.set_xlabel("Estimated mission time [min] (from the vehicle speed in the current runs)")
    for side in ("top", "right", "left"):
        tl.spines[side].set_visible(False)

    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(OUT, f"layout_proposal.{ext}"), dpi=300, bbox_inches="tight")
    print(f"legs: {', '.join(f'{k} {v:.1f} m' for k, v in legs.items())}; total ≈ {total:.0f} m")
    print(f"estimate: rock {t_rock / 60:.1f} min, inspection {t_insp / 60:.1f} min, S2 {t_s2 / 60:.1f} min, end {t_end / 60:.1f} min")
    print(f"rock (north, east) = ({rock[0]:.2f}, {rock[1]:.2f}); markers: {[(round(a, 1), round(b, 1)) for a, b in markers]}")
    print(f"saved {os.path.join(OUT, 'layout_proposal')}.pdf/.png")


if __name__ == "__main__":
    main()
