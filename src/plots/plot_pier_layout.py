"""
Pier mission of the real Blueye on the satellite image (north up, metres from the dock), drawn from the site layout
the mission uses (blueye_real_bt/scripts/mission_planner_scripts/mission_sites.py, site "pier"), with the dock's
axis at the site's nominal bearing; the mission itself takes the axis from the undocking.

Background: Google Maps satellite image (2026-10-24), with the measuring overlay of the screenshot painted out.
Its corners were placed from the map centre in the screenshot's URL and its scale bar (61 px = 2 m).

  python3 plot_pier_layout.py   ->  plots/field/mission_plan/pier_layout.png and .pdf
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blueye_real_bt" / "scripts" / "mission_planner_scripts"))
import mission_sites  # noqa: E402
import matplotlib
matplotlib.use("Agg")
import matplotlib.patheffects as pe
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

HERE = Path(__file__).resolve().parent
BACKGROUND = HERE / "field" / "mission_plan" / "pier_background.png"
BACKGROUND_NW = (63.44085405674485, 10.348943590810668)     # latitude, longitude of the image corners
BACKGROUND_SE = (63.440706988950396, 10.34971223111989)
OUT = HERE / "field" / "mission_plan" / "pier_layout"

SITE = mission_sites.SITES["pier"]
AXIS = SITE["nominal_out_bearing_deg"]
UNDOCK_END = 1.5    # m reversed out of the dock (0.4 power for 6 s, FullMission.xml)

TRANSIT, INSPECT, HOMING, UNDOCK, DOCK = "#6cb4ff", "#ff8f52", "#4ee0a6", "#ffffff", "#ffd84d"
PHASE_COLOUR = {"transit": TRANSIT, "inspection": INSPECT, "homing": HOMING}
halo = [pe.withStroke(linewidth=2.5, foreground="#000000")]
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 8})


def from_dock(lat, lon):
    """(north, east) in metres of a coordinate from the dock."""
    m_lat, m_lon = mission_sites.metres_per_degree(SITE["dock"][0])
    return (lat - SITE["dock"][0]) * m_lat, (lon - SITE["dock"][1]) * m_lon


fig, ax = plt.subplots(figsize=(7.0, 3.55))
north_top, east_left = from_dock(*BACKGROUND_NW)
north_bottom, east_right = from_dock(*BACKGROUND_SE)
ax.imshow(plt.imread(BACKGROUND), extent=[east_left, east_right, north_bottom, north_top], zorder=0)

route = [("U", mission_sites.dock_frame_to_north_east(UNDOCK_END, 0.0, AXIS), None)] + \
    [(name, mission_sites.dock_frame_to_north_east(f, r, AXIS), phase) for name, f, r, _, phase, _ in SITE["waypoints"]]
inspection = [ne for name, ne, phase in route if name.startswith("I")]

# the assumed pipeline under the inspection line
(n1, e1), (n4, e4) = inspection[0], inspection[-1]
ax.plot([e1 - 1.0, e4 + 1.0], [n1, n4], color=DOCK, lw=7, alpha=0.35, solid_capstyle="butt", zorder=2)

for (_, (na, ea), _), (_, (nb, eb), phase) in zip(route[:-1], route[1:]):
    colour = PHASE_COLOUR[phase]
    ax.plot([ea, eb], [na, nb], color=colour, lw=2, solid_capstyle="round", zorder=3)
    ax.add_patch(FancyArrowPatch((ea, na), ((ea + eb) / 2, (na + nb) / 2), arrowstyle="-|>", mutation_scale=11,
                                 color=colour, lw=0, zorder=4))
(nu, eu) = route[0][1]
ax.plot([0, eu], [0, nu], color=UNDOCK, lw=1.6, ls=(0, (2, 1.5)), zorder=3)
ax.plot(eu, nu, "o", ms=4.5, mfc="#111111", mec="#ffffff", mew=1.2, zorder=7)

for name, (n, e), _ in route[1:]:
    ax.plot(e, n, "o", ms=6.5, mfc="#111111", mec="#ffffff", mew=1.4, zorder=7)
    above = name.startswith("I")
    ax.annotate(name, (e, n), xytext=(0, 7 if above else -8), textcoords="offset points", ha="center",
                va="bottom" if above else "top", color="#ffffff", fontsize=8, path_effects=halo, zorder=8)
ax.plot(0, 0, "s", ms=8.5, mfc=DOCK, mec="#000000", mew=1.0, zorder=7)
ax.annotate("Dock", (0, 0), xytext=(0, -8), textcoords="offset points", ha="center", va="top", color="#ffffff",
            fontsize=8, path_effects=halo, zorder=8)
ax.text(0.01, 0.015, "Imagery © 2026 Google", transform=ax.transAxes, color="#d0d0d0", fontsize=6, ha="left",
        va="bottom", zorder=8)

handles = [
    ax.plot([], [], "s", ms=7, mfc=DOCK, mec="#000000", ls="none", label="Docking station")[0],
    ax.plot([], [], color=UNDOCK, lw=1.6, ls=(0, (2, 1.5)), label="Undocking")[0],
    ax.plot([], [], color=TRANSIT, lw=2, label="Transit")[0],
    ax.plot([], [], color=INSPECT, lw=2, label=f"Inspection ({SITE['wait_s']:.0f} s at each point)")[0],
    ax.plot([], [], color=HOMING, lw=2, label="Homing")[0],
    ax.plot([], [], color=DOCK, lw=6, alpha=0.35, label="Pipeline (assumed)")[0],
]
ax.legend(handles=handles, loc="lower right", fontsize=7, facecolor="#1a1a1a", edgecolor="#555555",
          labelcolor="#ffffff", framealpha=0.9)
ax.set_xlabel("East of the dock [m]")
ax.set_ylabel("North of the dock [m]")
ax.set_xlim(east_left, east_right)
ax.set_ylim(north_bottom, north_top)
ax.set_aspect("equal")
fig.tight_layout(pad=0.3)
for ext in ("png", "pdf"):
    fig.savefig(OUT.with_suffix(f".{ext}"), dpi=300, bbox_inches="tight", pad_inches=0.03)
print(f"{OUT}.png / .pdf")
