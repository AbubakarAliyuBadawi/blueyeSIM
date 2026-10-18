#!/usr/bin/env python3
"""
Open loop against closed loop for the obstacle scenario (S1): the same rock, with and without the takeover.

Usage:
  source ~/Desktop/auto-pilot/install/setup.bash
  python3 src/plots/plot_ol_cl_comparison.py \
      --ol src/rosbags/paper/s1_obstacle_run01_shadow_20261018 \
      --cl src/rosbags/paper/s1_obstacle_run01_closedloop_20261018

Output (default src/plots/paper/s1_obstacle_comparison/):
  s1_ol_vs_cl.pdf/.png      (a) plan view around the rock, (b) distance to the rock, (c) urgency, against the time
                            from the injection (first sonar echo from the rock)
  s1_ol_vs_cl.csv/.md       t_att, t_req, outcome, closest approach, lead time / operator times
"""

import argparse
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

import plot_hazard_run as H

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = {"OL": ("Open loop", "#2a78d6"), "CL": ("Closed loop", "#eb6834")}    # validated categorical slots 1 and 2
INK, MUTED = "#0b0b0b", "#52514e"
plt.rcParams.update({"font.size": 8, "axes.labelsize": 8, "legend.fontsize": 7, "xtick.labelsize": 7,
                     "ytick.labelsize": 7, "pdf.fonttype": 42, "ps.fonttype": 42})


def analyse(bag, rock, grid):
    data = H.load(bag)
    start = next(t for t, v in data["/mission_state"] if v > 0)
    thr = float(data["/blueye/takeover_request/threshold"][-1][1]) if data["/blueye/takeover_request/threshold"] else 0.65
    t_inj = H.first_rock_echo(data, start, grid, rock)
    odom = data["/blueye/odom"]
    t = np.array([o[0] - start for o in odom])
    xy = np.array([[o[1], o[2]] for o in odom])
    speed = np.array([o[4] for o in odom])
    dist = H.dist_to_polygon(xy, rock)
    ev = [(ts - start, e) for ts, e in data["/blueye/takeover_request/evidence"]]
    urg = [(ts - start, v) for ts, v in data["/blueye/takeover_request/urgency"]]
    r = {"name": os.path.basename(os.path.normpath(bag)), "t": t, "xy": xy, "dist": dist, "urg": urg, "t_inj": t_inj, "thr": thr}
    r["t_att"] = next((ts for ts, e in ev if ts >= t_inj and max(e["probabilities"], key=e["probabilities"].get)
                       in ("IncreasedAttentionRequired", "TakeoverRequested")), None)
    r["t_req"] = next((ts for ts, v in urg if ts >= t_inj and v >= thr), None)
    moving, r["t_haz"] = False, None
    for k in range(len(t)):
        moving = moving or (t[k] > t_inj and speed[k] > 0.1)
        if moving and speed[k] < 0.03 and dist[k] < 0.35:
            r["t_haz"] = t[k]
            break
    r["t_dec"] = next((ts - start for ts, v in data.get("/blueye/takeover_request/human_decision", []) if ts - start >= t_inj), None)
    r["t_popup"] = next((ts - start for ts, v in data.get("/blueye/takeover_request/pending", []) if v and ts - start >= t_inj), None)
    r["t_hb"] = next((ts - start for ts, v in data.get("/blueye/takeover_request/handback", []) if v and ts - start >= t_inj), None)
    r["finished"] = next((m.split(":")[-1].strip() for ts, n, m in data["/rosout"] if "Mission finished" in m), None)
    r["closest"] = float(dist[t >= t_inj].min())
    return r


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ol", required=True)
    parser.add_argument("--cl", required=True)
    parser.add_argument("--config", default=os.path.join(H.SIM, "config", "blueye_obstacle_config.yaml"))
    parser.add_argument("--out", default=os.path.join(HERE, "paper", "s1_obstacle_comparison"))
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    known, rock = H.footprints(args.config)
    grid = H.known_grid(known)
    runs = {"OL": analyse(os.path.abspath(args.ol), rock, grid), "CL": analyse(os.path.abspath(args.cl), rock, grid)}

    fig = plt.figure(figsize=(7.2, 3.6))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.0, 1.25], height_ratios=[1, 1], wspace=0.28, hspace=0.12)
    ax_map = fig.add_subplot(gs[:, 0])
    ax_d = fig.add_subplot(gs[0, 1])
    ax_u = fig.add_subplot(gs[1, 1], sharex=ax_d)

    # (a) plan view around the rock (east horizontal, north vertical)
    ax_map.fill(rock[:, 1], rock[:, 0], color=MUTED, lw=0, zorder=2)
    ax_map.text(rock[:, 1].mean() + 0.35, rock[:, 0].mean() - 0.35, "rock", fontsize=7, color=MUTED)
    for key, r in runs.items():
        label, colour = RUNS[key]
        m = (r["t"] >= r["t_inj"] - 20) & (r["t"] <= (r["t_haz"] if r["t_haz"] is not None else (r["t_hb"] or r["t"][-1]) + 45))
        ax_map.plot(r["xy"][m, 1], r["xy"][m, 0], color=colour, lw=1.6, label=label, zorder=3)
        k_req = int(np.searchsorted(r["t"], r["t_req"]))
        ax_map.plot(r["xy"][k_req, 1], r["xy"][k_req, 0], "v", color=colour, mec=INK, mew=0.6, ms=6, zorder=5)
        if r["t_haz"] is not None:
            k = int(np.searchsorted(r["t"], r["t_haz"]))
            ax_map.plot(r["xy"][k, 1], r["xy"][k, 0], "X", color=INK, ms=7, zorder=6)
        if r["t_hb"] is not None:
            k = int(np.searchsorted(r["t"], r["t_hb"]))
            ax_map.plot(r["xy"][k, 1], r["xy"][k, 0], "^", color=colour, mec=INK, mew=0.6, ms=6, zorder=5)
    handles = [Line2D([], [], color=RUNS[k][1], lw=1.6, label=RUNS[k][0]) for k in RUNS] + [
        Line2D([], [], marker="v", color="white", mec=INK, ls="", ms=6, label="Takeover request"),
        Line2D([], [], marker="X", color=INK, ls="", ms=7, label="Contact (open loop)"),
        Line2D([], [], marker="^", color="white", mec=INK, ls="", ms=6, label="Hand-back (closed loop)")]
    ax_map.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2, frameon=False, fontsize=6.5)
    ax_map.set_aspect("equal")
    ax_map.set_xlim(-18.6, -13.4)
    ax_map.set_ylim(2.2, 6.4)
    ax_map.set_xlabel("East [m]")
    ax_map.set_ylabel("North [m]")
    ax_map.grid(color="#e4e3df", lw=0.6)
    ax_map.text(0.02, 0.92, "(a)", transform=ax_map.transAxes, fontweight="bold")

    # (b) distance to the rock, (c) urgency, against the time from the injection
    t_max = max(((r["t_haz"] if r["t_haz"] is not None else (r["t_hb"] or 0) + 20) - r["t_inj"]) for r in runs.values())
    for key, r in runs.items():
        label, colour = RUNS[key]
        m = (r["t"] >= r["t_inj"] - 5) & (r["t"] - r["t_inj"] <= t_max)
        ax_d.plot(r["t"][m] - r["t_inj"], r["dist"][m], color=colour, lw=1.4, label=label)
        u = [(ts - r["t_inj"], v) for ts, v in r["urg"] if -5 <= ts - r["t_inj"] <= t_max]
        ax_u.plot(*zip(*u), color=colour, lw=1.4, label=label)
        for ax in (ax_d, ax_u):
            ax.axvline(r["t_req"] - r["t_inj"], color=colour, ls="--", lw=0.9)
        if r["t_haz"] is not None:
            for ax in (ax_d, ax_u):
                ax.axvline(r["t_haz"] - r["t_inj"], color=INK, ls="-", lw=0.9)
        if r["t_dec"] is not None and r["t_hb"] is not None:
            for ax in (ax_d, ax_u):
                ax.axvspan(r["t_dec"] - r["t_inj"], r["t_hb"] - r["t_inj"], color=colour, alpha=0.10, lw=0)
    ol, cl = runs["OL"], runs["CL"]
    ax_d.text(ol["t_req"] - ol["t_inj"] - 0.5, 4.6, "$t_{req}$", ha="right", fontsize=7, color=INK)
    if ol["t_haz"] is not None:
        ax_d.text(ol["t_haz"] - ol["t_inj"] + 0.6, 0.12, "contact", ha="left", va="bottom", fontsize=7, color=INK)
    if cl["t_dec"] is not None and cl["t_hb"] is not None:
        ax_d.text((cl["t_dec"] + cl["t_hb"]) / 2 - cl["t_inj"], 4.6, "manual control", ha="center", fontsize=7, color=INK)
    ax_d.set_ylim(0, 5.4)
    ax_d.set_ylabel("Distance to rock [m]")
    ax_d.legend(loc="lower left", frameon=False)
    ax_d.grid(color="#e4e3df", lw=0.6)
    ax_d.tick_params(labelbottom=False)
    ax_d.text(0.01, 0.88, "(b)", transform=ax_d.transAxes, fontweight="bold")
    ax_u.axhline(ol["thr"], color="#d62728", ls=":", lw=1.0)
    ax_u.text(t_max, ol["thr"] + 0.02, f"threshold {ol['thr']:.2f}", ha="right", va="bottom", fontsize=6.5, color="#d62728")
    ax_u.set_ylim(0, 1)
    ax_u.set_ylabel("Urgency $U$")
    ax_u.set_xlabel("Time from first sonar echo of the rock [s]")
    ax_u.set_xlim(-5, t_max)
    ax_u.grid(color="#e4e3df", lw=0.6)
    ax_u.text(0.01, 0.88, "(c)", transform=ax_u.transAxes, fontweight="bold")
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(args.out, f"s1_ol_vs_cl.{ext}"), dpi=300, bbox_inches="tight")
    plt.close(fig)

    def f(v):
        return "" if v is None else f"{v:.1f}"
    rows = []
    for key, r in runs.items():
        rows.append({"run": r["name"], "mode": key, "t_att_s": f(r["t_att"] - r["t_inj"]), "t_req_s": f(r["t_req"] - r["t_inj"]),
                     "distance_at_request_m": f"{float(r['dist'][int(np.searchsorted(r['t'], r['t_req']))]):.2f}",
                     "outcome": "contact" if r["t_haz"] is not None else ("held, resumed" if r["t_hb"] is not None else "held"),
                     "lead_time_s": f(None if r["t_haz"] is None else r["t_haz"] - r["t_req"]),
                     "closest_approach_m": f"{r['closest']:.2f}",
                     "operator_decision_s": f(None if (r["t_dec"] is None or r["t_popup"] is None) else r["t_dec"] - r["t_popup"]),
                     "manual_control_s": f(None if (r["t_dec"] is None or r["t_hb"] is None) else r["t_hb"] - r["t_dec"]),
                     "mission": r["finished"] or "stopped at contact"})
    with open(os.path.join(args.out, "s1_ol_vs_cl.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(args.out, "s1_ol_vs_cl.md"), "w") as fh:
        fh.write("| " + " | ".join(rows[0]) + " |\n|" + "---|" * len(rows[0]) + "\n")
        for row in rows:
            fh.write("| " + " | ".join(row.values()) + " |\n")
    print(open(os.path.join(args.out, "s1_ol_vs_cl.md")).read())
    print(f"Saved to {args.out}")


if __name__ == "__main__":
    main()
