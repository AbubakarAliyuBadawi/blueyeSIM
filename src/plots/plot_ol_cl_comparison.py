#!/usr/bin/env python3
"""
Open loop against closed loop for one hazard scenario: the same hazard, with and without the takeover.

Usage:
  source ~/Desktop/auto-pilot/install/setup.bash
  python3 src/plots/plot_ol_cl_comparison.py \
      --ol src/rosbags/paper/s1_obstacle_run01_shadow_20261018 \
      --cl src/rosbags/paper/s1_obstacle_run01_closedloop_20261018
  python3 src/plots/plot_ol_cl_comparison.py --scenario S2 \
      --ol src/rosbags/paper/s2_current_usbl_run01_shadow_20261019 \
      --cl src/rosbags/paper/s2_current_usbl_run01_closedloop_20261019

Output, S1 (default src/plots/paper/s1_obstacle_comparison/):
  s1_ol_vs_cl.pdf/.png      (a) plan view around the rock, (b) distance to the rock, (c) urgency, against the time
                            from the injection (first sonar echo from the rock)
  s1_ol_vs_cl.csv/.md       t_att, t_req, outcome, closest approach, lead time / operator times
Output, S2 (default src/plots/paper/s2_current_usbl_comparison/):
  s2_ol_vs_cl.pdf/.png      (a) plan view of the inspection leg, (b) water current and USBL loss as injected,
                            (c) distance off the leg, (d) urgency, against the time from the current onset
  s2_ol_vs_cl.csv/.md       disturbances, t_att, t_req (also after the USBL cut), distance off the leg, operator
                            times, mission result
"""

import argparse
import csv
import math
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


def compare_s1(args):
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
    shown = [rock]
    for key, r in runs.items():
        label, colour = RUNS[key]
        m = (r["t"] >= r["t_inj"] - 20) & (r["t"] <= (r["t_haz"] if r["t_haz"] is not None else (r["t_hb"] or r["t"][-1]) + 45))
        ax_map.plot(r["xy"][m, 1], r["xy"][m, 0], color=colour, lw=1.6, label=label, zorder=3)
        shown.append(r["xy"][m & (np.linalg.norm(r["xy"] - rock.mean(0), axis=1) < 3.5)])
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
    e0, e1, n0, n1 = H.view_box(shown, margin=0.5, min_span=4.0)
    ax_map.set_xlim(e0, e1)
    ax_map.set_ylim(n0, n1)
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


# ── S2: water current and USBL loss during inspection ─────────────────────────



def analyse_s2(bag):
    data = H.load(bag, images=False)
    start = next(t for t, v in data["/mission_state"] if v > 0)
    thr = float(data["/blueye/takeover_request/threshold"][-1][1]) if data["/blueye/takeover_request/threshold"] else 0.65
    events = H.s2_events(data, start)
    t_inj = next(t for t, kind, v in events if kind == "current" and v[0] > 0 and t >= 0)
    t_cut = next((t for t, kind, v in events if kind == "usbl" and not v and t >= t_inj), None)
    t_back = next((t for t, kind, v in events if kind == "usbl" and v and t_cut is not None and t > t_cut), None)
    odom = data["/blueye/odom"]
    t = np.array([o[0] - start for o in odom])
    ev = [(ts - start, e) for ts, e in data["/blueye/takeover_request/evidence"]]
    urg = [(ts - start, v) for ts, v in data["/blueye/takeover_request/urgency"]]
    r = {"name": os.path.basename(os.path.normpath(bag)), "data": data, "start": start,
         "t": t, "xy": np.array([[o[1], o[2]] for o in odom]),
         "urg": urg, "thr": thr, "t_inj": t_inj, "t_cut": t_cut, "t_back": t_back, "end": float(t[-1]),
         "xt": H.leg_cross_track(data, start), "goals": [(g[1], g[2]) for g in data["/blueye/waypoint_goal"]],
         "currents": [(ts, v[0], v[1]) for ts, kind, v in events if kind == "current"]}
    r["t_att"] = next((ts for ts, e in ev if ts >= t_inj and max(e["probabilities"], key=e["probabilities"].get)
                       in ("IncreasedAttentionRequired", "TakeoverRequested")), None)
    r["t_req"] = next((ts for ts, v in urg if ts >= t_inj and v >= thr), None)
    r["t_popup"] = next((ts - start for ts, v in data.get("/blueye/takeover_request/pending", []) if v and ts - start >= t_inj), None)
    r["t_dec"] = next((ts - start for ts, v in data.get("/blueye/takeover_request/human_decision", []) if ts - start >= t_inj), None)
    r["t_hb"] = next((ts - start for ts, v in data.get("/blueye/takeover_request/handback", []) if v and ts - start >= t_inj), None)
    r["finished"] = next((m.split(":")[-1].strip() for ts, n, m in data["/rosout"] if "Mission finished" in m), None)
    # same window as plot_hazard_run.py: 40 s after the last disturbance, or 30 s after the hand-back when later
    t_end = r["t_end"] = H.s2_window_end(events, t_inj, r["t_hb"], r["end"])
    # off the leg while the vehicle steered itself (closed loop: until the operator took over), and under manual control
    t_auto = r["t_dec"] if r["t_dec"] is not None else t_end
    r["xt_max"] = max(d for ts, d in r["xt"] if t_inj <= ts <= t_auto)
    r["xt_manual"] = max((d for ts, d in r["xt"] if r["t_dec"] <= ts <= r["t_hb"]), default=None) \
        if (r["t_dec"] is not None and r["t_hb"] is not None) else None
    win = [(ts, e) for ts, e in ev if t_inj <= ts <= t_end]
    dt = float(np.median(np.diff([ts for ts, _ in win]))) if len(win) > 1 else 0.5
    r["wte_high_s"] = sum(dt for ts, e in win if e["evidence"].get("WaypointTrackingError") == "High")
    r["u_peak"] = max(v for ts, v in urg if t_inj <= ts <= t_end)
    return r


def current_at(r, ts):
    speed = 0.0
    for tc, v, _ in r["currents"]:
        if tc <= ts:
            speed = v
    return speed


def same_disturbances(a, b, tol=1.0):
    """True when both runs got the same current steps and USBL outage (within tol seconds), e.g. a replayed run."""
    ca = [(t - a["t_inj"], v) for t, v, _ in a["currents"] if t >= a["t_inj"]]
    cb = [(t - b["t_inj"], v) for t, v, _ in b["currents"] if t >= b["t_inj"]]
    if len(ca) != len(cb) or any(abs(x[0] - y[0]) > tol or abs(x[1] - y[1]) > 1e-3 for x, y in zip(ca, cb)):
        return False
    for key in ("t_cut", "t_back"):
        if (a[key] is None) != (b[key] is None) or \
                (a[key] is not None and abs((a[key] - a["t_inj"]) - (b[key] - b["t_inj"])) > tol):
            return False
    return True


def compare_s2(args):
    known, _ = H.footprints(args.config or H.run_config(os.path.join(HERE, "paper", os.path.basename(os.path.normpath(args.ol)))))
    runs = {"OL": analyse_s2(os.path.abspath(args.ol)), "CL": analyse_s2(os.path.abspath(args.cl))}
    ol, cl = runs["OL"], runs["CL"]
    t_max = max(r["t_end"] - r["t_inj"] for r in runs.values())

    fig = plt.figure(figsize=(7.2, 4.3))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.0, 1.3], height_ratios=[0.75, 1, 1], wspace=0.3, hspace=0.14)
    ax_map = fig.add_subplot(gs[:, 0])
    ax_c = fig.add_subplot(gs[0, 1])
    ax_d = fig.add_subplot(gs[1, 1], sharex=ax_c)
    ax_u = fig.add_subplot(gs[2, 1], sharex=ax_c)

    # (a) plan view of the inspection leg (east horizontal, north vertical)
    for poly in known:
        ax_map.fill(poly[:, 1], poly[:, 0], color="#d9d4cc", lw=0, zorder=1)
    leg = [p for p in H.leg_at(ol["data"], ol["start"], ol["t_inj"] + 0.5) if p is not None]
    if len(leg) == 2:
        ax_map.plot([leg[0][1], leg[1][1]], [leg[0][0], leg[1][0]], color=INK, lw=0.8, ls=(0, (4, 2)), zorder=2)
    for n, e in leg:
        ax_map.plot(e, n, "o", mfc="white", mec=INK, mew=0.8, ms=7, zorder=6)
        ax_map.text(e + 0.45, n - 0.05, H.waypoint_label((n, e)), fontsize=6.5, color=INK, va="center", zorder=6)
    shown = list(leg) + [(n, e + 2.8) for n, e in leg]          # room for the marker labels on the right
    for key, r in runs.items():
        label, colour = RUNS[key]
        stop = r["t_end"] + 30
        m = (r["t"] >= r["t_inj"] - 10) & (r["t"] <= stop)
        ax_map.plot(r["xy"][m, 1], r["xy"][m, 0], color=colour, lw=1.5, label=label, zorder=3)
        shown.append(r["xy"][m])
        for ts, marker in ((r["t_cut"], "X"), (r["t_req"], "v"), (r["t_hb"], "^")):
            if ts is not None:
                k = min(int(np.searchsorted(r["t"], ts)), len(r["t"]) - 1)
                ax_map.plot(r["xy"][k, 1], r["xy"][k, 0], marker, color=colour, mec=INK, mew=0.6, ms=6, zorder=5)
    e0, e1, n0, n1 = H.view_box(shown, margin=0.8, min_span=6.0)
    size, direction = 0.15 * min(e1 - e0, n1 - n0), math.radians(ol["currents"][0][2])
    tail = (e1 - 0.14 * (e1 - e0), n1 - 0.3 * (n1 - n0))
    ax_map.annotate("", xy=(tail[0] + size * math.sin(direction), tail[1] + size * math.cos(direction)), xytext=tail,
                    arrowprops=dict(arrowstyle="-|>", color=MUTED, lw=1.6, mutation_scale=10))
    ax_map.text(tail[0], tail[1] - 0.04 * (n1 - n0), "water\ncurrent", fontsize=6.5, color=MUTED, ha="center", va="top")
    handles = [Line2D([], [], color=RUNS[k][1], lw=1.5, label=RUNS[k][0]) for k in RUNS] + [
        Line2D([], [], color=INK, lw=0.8, ls=(0, (4, 2)), label="Planned leg"),
        Line2D([], [], marker="X", color="white", mec=INK, ls="", ms=6, label="USBL cut"),
        Line2D([], [], marker="v", color="white", mec=INK, ls="", ms=6, label="Takeover request"),
        Line2D([], [], marker="^", color="white", mec=INK, ls="", ms=6, label="Hand-back (closed loop)")]
    ax_map.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=2, frameon=False, fontsize=6.5)
    ax_map.set_aspect("equal")
    ax_map.set_xlim(e0, e1)
    ax_map.set_ylim(n0, n1)
    ax_map.set_xlabel("East [m]")
    ax_map.set_ylabel("North [m]")
    ax_map.grid(color="#e4e3df", lw=0.6)
    ax_map.text(0.03, 0.03, "(a)", transform=ax_map.transAxes, fontweight="bold")

    # (b) the disturbances as injected, (c) distance off the leg, (d) urgency, against the time from the current onset
    matched = same_disturbances(ol, cl)          # replayed: draw the disturbances once, for both runs
    for i, (key, r) in enumerate(runs.items()):
        label, colour = RUNS[key]
        if not (matched and key == "CL"):
            shade = INK if matched else colour
            ts = np.linspace(-10, t_max, 2000)
            ax_c.plot(ts, [current_at(r, x + r["t_inj"]) for x in ts], color=shade, lw=1.4, drawstyle="steps-post")
            if matched:
                tc, v, _ = max(r["currents"], key=lambda c: c[1])
                ax_c.text(tc - r["t_inj"] + 1.0, v + 0.01, "current, both runs", fontsize=6.3, color=INK, va="bottom")
        if r["t_cut"] is not None and not (matched and key == "CL"):
            y = 0.40 - 0.045 * i
            back = (r["t_back"] if r["t_back"] is not None else r["end"]) - r["t_inj"]
            text = "USBL lost, both runs" if matched else "USBL lost"
            ax_c.plot([r["t_cut"] - r["t_inj"], min(back, t_max)], [y, y], color=shade, lw=3.0, solid_capstyle="butt")
            ax_c.text(min(back, t_max) + 1.0 if back < t_max - 15 else r["t_cut"] - r["t_inj"] - 1.0, y,
                      text, fontsize=6.3, color=shade, va="center", ha="left" if back < t_max - 15 else "right")
        xt = [(x - r["t_inj"], d) for x, d in r["xt"] if -10 <= x - r["t_inj"] <= t_max]
        ax_d.plot(*zip(*xt), color=colour, lw=1.4, label=label)
        u = [(x - r["t_inj"], v) for x, v in r["urg"] if -10 <= x - r["t_inj"] <= t_max]
        ax_u.plot(*zip(*u), color=colour, lw=1.4, label=label)
        for ax in (ax_d, ax_u):
            if r["t_req"] is not None:
                ax.axvline(r["t_req"] - r["t_inj"], color=colour, ls="--", lw=0.9)
            if r["t_dec"] is not None and r["t_hb"] is not None:
                ax.axvspan(r["t_dec"] - r["t_inj"], r["t_hb"] - r["t_inj"], color=colour, alpha=0.10, lw=0)
    ax_d.legend(loc="upper right", frameon=False)
    if cl["t_dec"] is not None and cl["t_hb"] is not None:
        ax_u.text((cl["t_dec"] + cl["t_hb"]) / 2 - cl["t_inj"], 0.9, "manual control", ha="center", fontsize=6.5, color=INK)
    ax_c.set_ylim(0, 0.44)
    ax_c.set_ylabel("Current [m/s]")
    ax_c.grid(color="#e4e3df", lw=0.6)
    ax_c.tick_params(labelbottom=False)
    ax_c.text(0.01, 0.80, "(b)", transform=ax_c.transAxes, fontweight="bold")
    for level, name in ((0.5, "Medium"), (1.5, "High")):
        ax_d.axhline(level, color=MUTED, ls=":", lw=0.8)
        ax_d.text(t_max, level + 0.04, name, ha="right", va="bottom", fontsize=6.3, color=MUTED)
    ax_d.set_ylim(0, 1.2 * max(d for r in runs.values() for x, d in r["xt"] if -10 <= x - r["t_inj"] <= t_max))
    ax_d.set_ylabel("Off the leg [m]")
    ax_d.grid(color="#e4e3df", lw=0.6)
    ax_d.tick_params(labelbottom=False)
    ax_d.text(0.01, 0.86, "(c)", transform=ax_d.transAxes, fontweight="bold")
    ax_u.axhline(ol["thr"], color="#d62728", ls=":", lw=1.0)
    ax_u.text(-9, ol["thr"] - 0.03, f"threshold {ol['thr']:.2f}", ha="left", va="top", fontsize=6.5, color="#d62728")
    ax_u.set_ylim(0, 1)
    ax_u.set_ylabel("Urgency $U$")
    ax_u.set_xlabel("Time from the current onset [s]")
    ax_u.set_xlim(-10, t_max)
    ax_u.grid(color="#e4e3df", lw=0.6)
    ax_u.text(0.01, 0.86, "(d)", transform=ax_u.transAxes, fontweight="bold")
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(args.out, f"s2_ol_vs_cl.{ext}"), dpi=300, bbox_inches="tight")
    plt.close(fig)

    def f(v):
        return "" if v is None else f"{v:.1f}"
    rows = []
    for key, r in runs.items():
        seq = [v for tc, v, _ in r["currents"] if r["t_inj"] <= tc <= (r["t_req"] or r["end"]) + 15]
        rows.append({"run": r["name"], "mode": key,
                     "current_m_s": " → ".join(f"{v:.2f}" for v in seq) + f" (to {r['currents'][0][2]:.0f}°)",
                     "usbl_cut_s": f(None if r["t_cut"] is None else r["t_cut"] - r["t_inj"]),
                     "t_att_s": f(None if r["t_att"] is None else r["t_att"] - r["t_inj"]),
                     "t_req_s": f(None if r["t_req"] is None else r["t_req"] - r["t_inj"]),
                     "t_req_after_usbl_cut_s": f(None if (r["t_req"] is None or r["t_cut"] is None) else r["t_req"] - r["t_cut"]),
                     "max_off_leg_m": f"{r['xt_max']:.2f}",
                     "max_off_leg_manual_m": "" if r["xt_manual"] is None else f"{r['xt_manual']:.2f}",
                     "tracking_error_high_s": f(r["wte_high_s"]),
                     "usbl_lost_s": f(None if r["t_cut"] is None else (r["t_back"] if r["t_back"] is not None else r["end"]) - r["t_cut"])
                     + ("" if r["t_back"] is not None else " (not restored)"),
                     "peak_U": f"{r['u_peak']:.2f}",
                     "operator_decision_s": f(None if (r["t_dec"] is None or r["t_popup"] is None) else r["t_dec"] - r["t_popup"]),
                     "manual_control_s": f(None if (r["t_dec"] is None or r["t_hb"] is None) else r["t_hb"] - r["t_dec"]),
                     "mission": r["finished"] or "not finished (recording ended)"})
    with open(os.path.join(args.out, "s2_ol_vs_cl.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(args.out, "s2_ol_vs_cl.md"), "w") as fh:
        fh.write("| " + " | ".join(rows[0]) + " |\n|" + "---|" * len(rows[0]) + "\n")
        for row in rows:
            fh.write("| " + " | ".join(row.values()) + " |\n")
    print(open(os.path.join(args.out, "s2_ol_vs_cl.md")).read())
    print(f"Saved to {args.out}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario", choices=["S1", "S2"], default="S1")
    parser.add_argument("--ol", required=True)
    parser.add_argument("--cl", required=True)
    parser.add_argument("--config", default=None,
                        help="simulator config with the structure poses (default: blueye_obstacle_config.yaml for S1, "
                             "blueye_config.yaml for S2)")
    parser.add_argument("--out", default=None)
    args = parser.parse_args()
    args.out = args.out or os.path.join(HERE, "paper", {"S1": "s1_obstacle_comparison", "S2": "s2_current_usbl_comparison"}[args.scenario])
    os.makedirs(args.out, exist_ok=True)
    if args.scenario == "S1":                       # the rock of the open-loop run, as it was recorded
        args.config = args.config or H.run_config(os.path.join(HERE, "paper", os.path.basename(os.path.normpath(args.ol))))
        compare_s1(args)
    else:
        compare_s2(args)


if __name__ == "__main__":
    main()
