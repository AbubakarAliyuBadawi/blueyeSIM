
#!/usr/bin/env python3
"""Phase-gating heatmap for the unified mission Bayesian network.

Reads a GeNIe/SMILE XDSL file without PySMILE.

Outputs:
    gating_heatmap.png
    gating_heatmap.pdf
    gating_heatmap.csv

Abbreviations are shown on the heatmap.
Their full meanings are provided in the LaTeX caption,
not inside the figure.
"""

from pathlib import Path
import csv
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ============================================================
# 1. FILE PATHS
# ============================================================

XDSL = Path(
    "/home/badawi/Desktop/auto-pilot/src/"
    "blueye_takeover_bn/config/UnifiedMissionDBN_v4.xdsl"
)

OUTDIR = Path(
    "/home/badawi/Desktop/auto-pilot/src/plots"
)

OUTDIR.mkdir(parents=True, exist_ok=True)


# ============================================================
# 2. SETTINGS
# ============================================================

PHASE = "MissionPhase"

PHASE_LABEL = {}

SHORT = {
    "NavigationSystemPerformance": "Navigation system performance",
    "PerceptionSystemPerformance": "Perception system performance",
    "CollisionWithObstacle": "Collision with obstacle",
    "DataQuality": "Data quality",
    "OutOfLoopRisk": "Out-of-loop risk",
    "TakeoverRequest": "Takeover request",
    "AutonomousControlPerformance": "Autonomous control performance",
    "HumanDecisionReliability": "Human decision reliability",
    "USBLStrength": "USBL strength",
    "DVLStatus": "DVL status",
    "WaypointTrackingError": "Waypoint tracking error",
    "CameraQuality": "Camera quality",
    "ArUcoVisibility": "ArUco visibility",
    "SonarRange": "Sonar range",
    "RangeToTarget": "Range to target",
}

ABBREV = {
    "NavigationSystemPerformance": "NSP",
    "PerceptionSystemPerformance": "PSP",
    "CollisionWithObstacle": "COL",
    "DataQuality": "DQ",
    "OutOfLoopRisk": "OOL",
    "TakeoverRequest": "TR",
    "AutonomousControlPerformance": "ACP",
    "HumanDecisionReliability": "HDR",
    "USBLStrength": "USBL",
    "DVLStatus": "DVL",
    "WaypointTrackingError": "WTE",
    "CameraQuality": "CQ",
    "ArUcoVisibility": "AV",
    "SonarRange": "SR",
    "RangeToTarget": "RTT",
}

ORDER = [
    "NavigationSystemPerformance",
    "PerceptionSystemPerformance",
    "CollisionWithObstacle",
    "DataQuality",
    "OutOfLoopRisk",
    "TakeoverRequest",
]


# ============================================================
# 3. READ XDSL NETWORK
# ============================================================

if not XDSL.exists():
    raise FileNotFoundError(
        f"XDSL file not found:\n{XDSL}"
    )

print(f"Reading network:\n{XDSL}")

root = ET.parse(XDSL).getroot()

nodes = {}

for element in root.findall("./nodes/cpt"):

    node_id = element.get("id")

    states = [
        state.get("id")
        for state in element.findall("state")
    ]

    parent_element = element.find("parents")

    parents = (
        parent_element.text.split()
        if parent_element is not None
        and parent_element.text
        else []
    )

    probability_element = element.find("probabilities")

    if (
        probability_element is None
        or probability_element.text is None
    ):
        continue

    probabilities = np.fromstring(
        probability_element.text,
        sep=" "
    )

    nodes[node_id] = {
        "states": states,
        "parents": parents,
        "probs": probabilities,
    }

if PHASE not in nodes:
    raise KeyError(
        f"Node '{PHASE}' not found in the network."
    )

phases = nodes[PHASE]["states"]

print("Mission phases:", phases)


# ============================================================
# 4. COMPUTE PARENT INFLUENCE
# ============================================================

rows = []
labels = []
full_labels = []
groups = []
row_children = []
used_parents = []

for child in ORDER:

    if child not in nodes:
        print(f"Skipping missing node: {child}")
        continue

    nd = nodes[child]

    if PHASE not in nd["parents"]:
        print(f"Skipping {child}: no MissionPhase parent")
        continue

    shape = [
        len(nodes[parent]["states"])
        for parent in nd["parents"]
    ] + [len(nd["states"])]

    expected_size = int(np.prod(shape))

    if len(nd["probs"]) != expected_size:
        raise ValueError(
            f"CPT size mismatch for {child}: "
            f"expected {expected_size}, "
            f"found {len(nd['probs'])}"
        )

    cpt = nd["probs"].reshape(shape)

    if not np.allclose(
        cpt.sum(axis=-1),
        1.0,
        atol=1e-4
    ):
        raise ValueError(
            f"CPT rows for {child} do not sum to one."
        )

    # Severity: first child state = 0,
    # last child state = 1.
    severity_scores = np.linspace(
        0.0,
        1.0,
        len(nd["states"])
    )

    expected_severity = cpt @ severity_scores

    phase_axis = nd["parents"].index(PHASE)

    start = len(rows)

    for parent_axis, parent in enumerate(nd["parents"]):

        if parent == PHASE:
            continue

        selected = np.moveaxis(
            expected_severity,
            [parent_axis, phase_axis],
            [0, 1]
        )

        # Last minus first parent state.
        difference = selected[-1] - selected[0]

        # Uniform average over other parent configurations.
        influence = difference.reshape(
            len(phases), -1
        ).mean(axis=1)

        rows.append(influence)

        labels.append(
            ABBREV.get(parent, parent)
        )

        full_labels.append(
            SHORT.get(parent, parent)
        )

        row_children.append(child)

        if parent not in used_parents:
            used_parents.append(parent)

    if len(rows) > start:
        groups.append(
            (child, start, len(rows))
        )

if not rows:
    raise RuntimeError(
        "No phase-gated relationships were found."
    )

M = np.asarray(rows)


# ============================================================
# 5. SAVE CSV WITH FULL NODE NAMES
# ============================================================

csv_path = OUTDIR / "gating_heatmap.csv"

with open(
    csv_path,
    "w",
    newline="",
    encoding="utf-8"
) as file:

    writer = csv.writer(file)

    writer.writerow(
        ["child", "parent"] + phases
    )

    for i, values in enumerate(M):

        writer.writerow(
            [
                SHORT.get(
                    row_children[i],
                    row_children[i]
                ),
                full_labels[i],
            ] + [
                f"{value:.4f}"
                for value in values
            ]
        )


# ============================================================
# 6. FIGURE STYLE
# ============================================================

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": [
        "Times New Roman",
        "Times",
        "DejaVu Serif",
    ],
    "font.size": 8,
    "axes.linewidth": 0.6,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

# Original heatmap proportions.
fig, ax = plt.subplots(
    figsize=(5.5, 0.27 * len(M) + 1.3)
)


# ============================================================
# 7. DRAW HEATMAP
# ============================================================

vmax = float(np.max(np.abs(M)))

if vmax == 0:
    vmax = 1.0

im = ax.imshow(
    M,
    cmap="Blues",
    vmin=0,
    vmax=vmax,
    aspect="auto"
)

# Mission phases at the top.
ax.set_xticks(range(len(phases)))

ax.set_xticklabels(
    [PHASE_LABEL.get(p, p) for p in phases],
    fontsize=8
)

ax.xaxis.tick_top()
ax.tick_params(length=0)

# Abbreviated parent labels on the left.
ax.set_yticks(range(len(M)))

ax.set_yticklabels(
    labels,
    fontsize=8
)


# ============================================================
# 8. NUMERICAL VALUES INSIDE CELLS
# ============================================================

for i in range(len(M)):

    for j in range(len(phases)):

        value = M[i, j]

        value_text = (
            "0"
            if abs(value) < 5e-4
            else f"{value:+.2f}"
        )

        text_color = (
            "white"
            if value > 0.6 * vmax
            else "black"
        )

        ax.text(
            j,
            i,
            value_text,
            ha="center",
            va="center",
            fontsize=7,
            color=text_color
        )


# ============================================================
# 9. GRID AND GROUP SEPARATORS
# ============================================================

ax.set_xticks(
    np.arange(-0.5, len(phases)),
    minor=True
)

ax.set_yticks(
    np.arange(-0.5, len(M)),
    minor=True
)

ax.grid(
    which="minor",
    color="white",
    linewidth=1.0
)

ax.tick_params(
    which="minor",
    length=0
)

ax.set_xlim(
    -0.5,
    len(phases) - 0.5
)

ax.set_ylim(
    len(M) - 0.5,
    -0.5
)

# Keep group separators.
# No child-node labels on the right.
for child, start, end in groups:

    if start > 0:
        ax.axhline(
            start - 0.5,
            color="black",
            linewidth=0.8
        )


# ============================================================
# 10. SHORT COLOR BAR
# ============================================================

cb = fig.colorbar(
    im,
    ax=ax,
    orientation="horizontal",
    fraction=0.05,
    pad=0.03,
    aspect=40
)

cb.set_label(
    "Parent influence on child severity",
    fontsize=8
)

cb.outline.set_linewidth(0.5)

cb.ax.tick_params(
    length=2,
    labelsize=7
)


# ============================================================
# 11. SAVE FIGURE
# No abbreviation legend inside the figure.
# ============================================================

pdf_path = OUTDIR / "gating_heatmap.pdf"
png_path = OUTDIR / "gating_heatmap.png"

fig.savefig(
    pdf_path,
    bbox_inches="tight",
    pad_inches=0.08
)

fig.savefig(
    png_path,
    dpi=300,
    bbox_inches="tight",
    pad_inches=0.08
)

plt.close(fig)


# ============================================================
# 12. PRINT ABBREVIATIONS FOR LATEX CAPTION
# ============================================================

print("\nAbbreviations used in the heatmap:")

for parent in used_parents:

    print(
        f"{ABBREV.get(parent, parent)} = "
        f"{SHORT.get(parent, parent)}"
    )

print("\nFinished.")
print(f"PNG: {png_path}")
print(f"PDF: {pdf_path}")
print(f"CSV: {csv_path}")
