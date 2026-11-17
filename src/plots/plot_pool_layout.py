"""
Pool mission of the real Blueye in the dock frame (up: out of the dock opening; right: to the right looking out),
drawn from the site layout the mission uses (blueye_real_bt/scripts/mission_planner_scripts/mission_sites.py).
Pool, dock and obstacle outlines are measured on the map (2026-10-23) and assume the dock centred on the south wall.

  python3 plot_pool_layout.py   ->  plots/real_drone/pool_layout_proposal.png
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blueye_real_bt" / "scripts" / "mission_planner_scripts"))
import mission_sites  # noqa: E402
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch

INK, MUTED, GRID = "#0b0b0b", "#52514e", "#d9d8d4"
TRANSIT, INSPECT, HOMING = "#2a78d6", "#eb6834", "#1baf7a"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": MUTED,
                     "xtick.color": MUTED, "ytick.color": MUTED})

# assumptions (to confirm): pool 13.8 x 8.7 m, dock centred on the south wall, opening 1.0 m from it,
# obstacle 1.55 m square centred on the dock axis 9.2 m from the south wall
W, SOUTH, NORTH = 4.35, -1.0, 12.8
OBS_X, OBS = 8.2, 1.55
UNDOCK_END = (1.5, 0.0)     # 0.4 power for 6 s (FullMission.xml), about 1.2-1.5 m
SITE = mission_sites.SITES["pool"]
WAYPOINTS = [(name, (f, r)) for name, f, r, _, _, _ in SITE["waypoints"]]
PHASE = {name: phase for name, _, _, _, phase, _ in SITE["waypoints"]}
first_homing = next(i for i, (name, _) in enumerate(WAYPOINTS) if PHASE[name] == "homing")
route = {
    "transit": [UNDOCK_END, WAYPOINTS[0][1]],
    "inspection": [p for _, p in WAYPOINTS[:first_homing]],
    "homing": [WAYPOINTS[first_homing - 1][1]] + [p for _, p in WAYPOINTS[first_homing:]],
}
fig, ax = plt.subplots(figsize=(6.2, 8.4))
ax.add_patch(Rectangle((-W, SOUTH), 2 * W, NORTH - SOUTH, fill=False, lw=1.5, ec=MUTED))
ax.add_patch(Rectangle((-OBS / 2, OBS_X - OBS / 2), OBS, OBS, fc="#e9e8e4", ec=MUTED, lw=1.0, hatch="///"))
ax.text(0, OBS_X + OBS / 2 + 0.2, "obstacle 1.55 m", ha="center", va="bottom", color=INK)
ax.add_patch(Rectangle((-0.6, SOUTH), 1.2, 1.0, fc=MUTED, ec=MUTED))
ax.text(0.75, SOUTH + 0.5, "dock", ha="left", va="center", color=INK)

def path(points, colour, label):
    xs, ys = zip(*points)
    ax.plot(ys, xs, color=colour, lw=2, solid_capstyle="round", label=label, zorder=3)
    for (x0, y0), (x1, y1) in zip(points[:-1], points[1:]):
        ax.add_patch(FancyArrowPatch((y0, x0), ((y0 + y1) / 2, (x0 + x1) / 2), arrowstyle="-|>", mutation_scale=12,
                                     color=colour, lw=0, zorder=4))
path(route["transit"], TRANSIT, "Transit")
path(route["inspection"], INSPECT, "Inspection (1.2 m depth)")
path(route["homing"], HOMING, "Homing (to 1.7 m, the dock depth)")
ax.plot([0, 0], [0, 1.5], color=INK, lw=1.5, ls=(0, (4, 3)), label="Undocking: reverse ~1.5 m")
ax.plot([0, 0], [0, 2.5], color=INK, lw=1.0, ls=(0, (1, 2.5)), label="Docking controller (from the stop point)")

NOTES = {"S": "stop point, facing the dock"}
points = [("U", *UNDOCK_END, "right", "end of undocking")] + \
    [(name, f, r, "right" if r >= 0 and name != "S" else "left", NOTES.get(name, "")) for name, (f, r) in WAYPOINTS]
for name, x, y, side, note in points:
    ax.plot(y, x, "o", ms=7, mfc="white", mec=INK, mew=1.4, zorder=5)
    dx = 0.25 if side == "right" else -0.25
    ax.text(y + dx, x, name + (f"  {note}" if note and side == "right" else ""), ha="left" if side == "right" else "right",
            va="center", color=INK, fontsize=8.5, zorder=6)
    if note and side == "left":
        ax.text(y + dx, x - 0.45, note, ha="right", va="center", color=MUTED, fontsize=8)

def clearance(y0, x0, y1, x1, text):
    ax.annotate("", xy=(y1, x1), xytext=(y0, x0), arrowprops=dict(arrowstyle="<->", color=MUTED, lw=0.8))
    ax.text((y0 + y1) / 2, (x0 + x1) / 2 + 0.18, text, ha="center", va="bottom", color=MUTED, fontsize=8)
clearance(OBS / 2, OBS_X, 2.5, OBS_X, "1.7 m")
clearance(2.5, 6.0, W, 6.0, "1.85 m")
clearance(-2.5, 6.0, -W, 6.0, "1.85 m")
ax.annotate("", xy=(3.1, 11.0), xytext=(3.1, NORTH), arrowprops=dict(arrowstyle="<->", color=MUTED, lw=0.8))
ax.text(3.25, 11.9, "1.8 m", ha="left", va="center", color=MUTED, fontsize=8)

ax.set_xlim(-W - 0.8, W + 0.8); ax.set_ylim(SOUTH - 0.6, NORTH + 0.9); ax.set_aspect("equal")
ax.set_xlabel("metres to the right, looking out of the dock")
ax.set_ylabel("metres out of the dock opening")
ax.grid(color=GRID, lw=0.5); ax.set_axisbelow(True)
for s in ("top", "right"): ax.spines[s].set_visible(False)
ax.text(-W, NORTH + 0.25, "pool 8.7 x 13.8 m", color=MUTED, fontsize=8)
length = sum(((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
             for leg in route.values() for a, b in zip(leg[:-1], leg[1:])) + UNDOCK_END[0]
fig.suptitle(f"Pool mission, about {length:.0f} m", x=0.06, ha="left", fontsize=12, color=INK, fontweight="bold")
fig.text(0.06, 0.935, "Assumed: dock centred on the south wall; obstacle on the dock axis, 9.2 m from the south wall",
         fontsize=8.5, color=MUTED)
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=2, frameon=False, fontsize=8.5, labelcolor=INK)
fig.subplots_adjust(top=0.91, bottom=0.17, left=0.12, right=0.97)
out = Path(__file__).resolve().parent / "real_drone" / "pool_layout_proposal.png"
fig.savefig(out, dpi=150, facecolor="white"); print(out)
