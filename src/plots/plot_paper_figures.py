#!/usr/bin/env python3
"""
Simulation figures of the paper (Results, Simulation study).

  sim_nominal_mission.pdf/.png  Fig. sim-nominal: one nominal mission. Top: active phase; middle: posterior of the three
                                recommendations; bottom: urgency U with the threshold.
  sim_closed_loop.pdf/.png      Fig. sim-closed-loop: open loop against closed loop for (a) S1 and (b) S2, the matched
                                run01 pairs: plan view, distance to the hazard and U, takeover and hand-back marked.
  explanation_example.pdf/.png  Fig. explanation: the S1 and S2 takeover requests traced through the network, arcs
                                labelled with rule, row of the arc trace (Table S2) and source in the task analysis.
  sim_traces.pdf/.png           Fig. sim-traces: one open-loop run of each hazard scenario, (a) S1 and (b) S2. Each panel:
                                the evidence that changes after the injection (state names in the bars) and U with the
                                threshold; injection, t_att, t_req, the other disturbances and t_haz marked.

Usage:
  source ~/Desktop/auto-pilot/install/setup.bash
  python3 src/plots/plot_paper_figures.py [--nominal <bag>] [--s1 <bag>] [--s2 <bag>]

Output: src/plots/paper/sim_results/ (copy the PDFs to figures/results/ of the paper). The S1 and S2 event times are
taken from each run's hazard_results.csv, so run plot_hazard_run.py on those runs first.
"""

import argparse
import csv
import math
import os

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

import plot_hazard_run as H

HERE = os.path.dirname(os.path.abspath(__file__))
PAPER = os.path.join(HERE, "paper")
BAGS = os.path.join(H.SRC, "rosbags", "paper")
OUT = os.path.join(PAPER, "sim_results")
INK, MUTED, GRID, EMPTY = "#0b0b0b", "#52514e", "#e4e3df", "#e4e3df"
# status steps (validated: good / amber / critical); amber needs direct labels or a second cue, given everywhere
GOOD, AMBER, CRITICAL = "#0ca30c", "#eda100", "#d03b3b"
RECOMMENDATIONS = [("NoTakeoverRequired", "No takeover", GOOD, "-"),
                   ("IncreasedAttentionRequired", "Increased attention", AMBER, (0, (5, 1.5))),
                   ("TakeoverRequested", "Takeover requested", CRITICAL, (0, (1.5, 1)))]
PHASE_TONES = ["#efeeea", "#dcdad3"]                          # neutral: phases are named, not colour-coded
plt.rcParams.update({"font.size": 8, "axes.labelsize": 8, "legend.fontsize": 7, "xtick.labelsize": 7,
                     "ytick.labelsize": 7, "axes.edgecolor": MUTED, "axes.linewidth": 0.6,
                     "xtick.color": MUTED, "ytick.color": MUTED, "pdf.fonttype": 42, "ps.fonttype": 42})


def mission_window(data):
    start = next(t for t, v in data["/mission_state"] if v > 0)
    end = next((t for t, n, m in data["/rosout"] if "Mission finished" in m), data["/mission_state"][-1][0])
    return start, end


def phase_segments(data, start, end):
    segs, cur, t0 = [], None, None
    for t, v in data["/mission_state"]:
        if not start <= t <= end or v == cur:
            continue
        if cur in H.PHASES and cur != 6:
            segs.append((t0 - start, t - start, H.PHASES[cur]))
        cur, t0 = v, t
    if cur in H.PHASES and cur != 6:
        segs.append((t0 - start, end - start, H.PHASES[cur]))
    return segs


def style(ax):
    ax.grid(axis="y", color=GRID, lw=0.5)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)


def fig_nominal(bag):
    data = H.load(bag, images=False)
    start, end = mission_window(data)
    thr = float(data["/blueye/takeover_request/threshold"][-1][1])
    segs = phase_segments(data, start, end)
    ev = [(t - start, e["probabilities"]) for t, e in data["/blueye/takeover_request/evidence"] if start <= t <= end]
    urg = [(t - start, v) for t, v in data["/blueye/takeover_request/urgency"] if start <= t <= end]
    span = end - start

    fig, (ax_p, ax_r, ax_u) = plt.subplots(3, 1, figsize=(7.2, 3.9), sharex=True,
                                           gridspec_kw={"height_ratios": [0.28, 1.0, 0.9], "hspace": 0.12})
    for i, (a, b, name) in enumerate(segs):
        ax_p.axvspan(a, b, color=PHASE_TONES[i % 2], lw=0)
        ax_p.text((a + b) / 2, 0.5, name, ha="center", va="center", fontsize=7, color=INK)
    ax_p.set_yticks([])
    ax_p.set_ylim(0, 1)
    for side in ("top", "right", "left", "bottom"):
        ax_p.spines[side].set_visible(False)

    times = [t for t, _ in ev]
    ends = []
    for key, label, colour, dash in RECOMMENDATIONS:
        values = [p[key] for _, p in ev]
        ax_r.plot(times, values, color=colour, lw=1.3, ls=dash, label=label, solid_capstyle="round")
        ends.append([values[-1], label])
    ends.sort()
    for k in range(1, len(ends)):                          # keep the end labels at least 0.07 apart
        ends[k][0] = max(ends[k][0], ends[k - 1][0] + 0.07)
    for y, label in ends:
        ax_r.annotate(label, (times[-1], y), xytext=(4, 0), textcoords="offset points", va="center",
                      fontsize=6.5, color=INK, annotation_clip=False)
    ax_r.set_ylim(0, 1)
    ax_r.set_ylabel("Posterior")
    ax_r.legend(loc="upper center", ncol=3, frameon=False, handlelength=2.6, borderaxespad=0.3)
    ax_u.plot([t for t, _ in urg], [v for _, v in urg], color=INK, lw=1.1)
    ax_u.axhline(thr, color=CRITICAL, ls=(0, (4, 2)), lw=0.9)
    ax_u.text(span, thr + 0.03, f"threshold {thr:.2f}", ha="right", va="bottom", fontsize=6.5, color=INK)
    ax_u.set_ylim(0, 1)
    ax_u.set_ylabel("Urgency $U$")
    ax_u.set_xlabel("Mission time [s]")
    ax_u.set_xlim(0, span)
    for ax in (ax_r, ax_u):
        style(ax)
        for a, _, _ in segs[1:]:
            ax.axvline(a, color="#c9c7c0", lw=0.6, zorder=0)
    return fig


def results_row(run):
    with open(os.path.join(PAPER, run, "hazard_results.csv")) as fh:
        return next(csv.DictReader(fh))


def evidence_strip(ax, ev, nodes, t0, t1):
    """One bar per node, coloured by severity (best: good, worst: critical, between: amber), state names in the bars."""
    for row, node in enumerate(nodes):
        states = H.SEVERITY[node]
        y = len(nodes) - 1 - row
        series = [(t, e["evidence"].get(node)) for t, e in ev if t0 - 1 <= t <= t1 + 1]
        k = 0
        while k < len(series):
            j = k
            while j + 1 < len(series) and series[j + 1][1] == series[k][1]:
                j += 1
            a = max(series[k][0], t0)
            b = min(series[j + 1][0] if j + 1 < len(series) else t1, t1)
            state = series[k][1]
            if b > a:
                if state is None:
                    colour = EMPTY
                else:
                    level = states.index(state)
                    colour = GOOD if level == 0 else CRITICAL if level == len(states) - 1 else AMBER
                ax.barh(y, b - a, left=a, height=0.78, color=colour, lw=0)
                if (b - a) > 0.07 * (t1 - t0):
                    ax.text(a + 0.006 * (t1 - t0), y, state or "not entered", va="center", ha="left", fontsize=6.3,
                            color=INK)
            k = j + 1
    ax.set_yticks(range(len(nodes)))
    ax.set_yticklabels(list(reversed(nodes)))
    ax.set_ylim(-0.6, len(nodes) - 0.4)
    ax.tick_params(axis="y", length=0)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)


def mark_events(ax_top, axes, events):
    """Vertical lines through all axes; labels above the top axes, stacked when they are closer than 6 % of the span."""
    lo, hi = ax_top.get_xlim()
    placed = []
    for t, label, colour, dash in sorted(events):
        for ax in axes:
            ax.axvline(t, color=colour, ls=dash, lw=0.9, zorder=3)
        level = 0
        while any(abs(t - p) < 0.06 * (hi - lo) and lv == level for p, lv in placed):
            level += 1
        placed.append((t, level))
        ax_top.annotate(label, (t, 1.0), xycoords=("data", "axes fraction"), xytext=(0, 3 + 9 * level),
                        textcoords="offset points", ha="center", va="bottom", fontsize=6.5, color=INK)
    return max((lv for _, lv in placed), default=0)


def fig_traces(s1_run, s2_run):
    fig = plt.figure(figsize=(7.2, 5.4))
    outer = fig.add_gridspec(2, 1, height_ratios=[1.0, 1.35], hspace=0.62)
    panels = [("S1", s1_run, ["SonarRange"], "(a) S1: obstacle on the transit leg"),
              ("S2", s2_run, ["Current", "WaypointTrackingError", "USBLStrength"],
               "(b) S2: cross-current and loss of the USBL signal during inspection")]
    for i, (sid, run, nodes, title) in enumerate(panels):
        data = H.load(os.path.join(BAGS, run), images=False)
        start, _ = mission_window(data)
        row = results_row(run)
        urg = [(t - start, v) for t, v in data["/blueye/takeover_request/urgency"]]
        thr = float(data["/blueye/takeover_request/threshold"][-1][1])
        t_req_m = next(t for t, v in urg if v >= thr)
        t_inj = t_req_m - float(row["t_req_s"])                     # injection, mission time
        t_att, t_req = float(row["t_att_s"]), float(row["t_req_s"])
        events = [(t_att, "$t_{\\mathrm{att}}$", INK, (0, (4, 2))), (t_req, "$t_{\\mathrm{req}}$", CRITICAL, (0, (4, 2)))]
        if sid == "S1":
            t_haz = t_req + float(row["lead_time_s"])
            events += [(0.0, "first echo", MUTED, ":"), (t_haz, "contact ($t_{\\mathrm{haz}}$)", INK, "-")]
            t1 = t_haz + 6
        else:
            for t, kind, v in H.s2_events(data, start):
                if t < t_inj - 1:
                    continue
                if kind == "current":
                    label = "current on" if abs(t - t_inj) < 0.5 else (f"{v[0]:.2f} m/s" if v[0] > 0 else "calm")
                    events.append((t - t_inj, label, MUTED, ":"))
                else:
                    events.append((t - t_inj, "USBL back" if v else "USBL cut", MUTED, (0, (6, 1.5, 1, 1.5))))
            t1 = max(t for t, *_ in events) + 30
        t0 = -10.0
        ev = [(t - start - t_inj, e) for t, e in data["/blueye/takeover_request/evidence"]]
        inner = outer[i].subgridspec(2, 1, height_ratios=[0.32 * len(nodes) + 0.15, 1.0], hspace=0.08)
        ax_e = fig.add_subplot(inner[0])
        ax_u = fig.add_subplot(inner[1], sharex=ax_e)
        evidence_strip(ax_e, ev, nodes, t0, t1)
        ax_e.tick_params(labelbottom=False, axis="x", length=0)
        u = [(t - t_inj, v) for t, v in urg if t0 <= t - t_inj <= t1]
        ax_u.plot([t for t, _ in u], [v for _, v in u], color=INK, lw=1.1)
        ax_u.axhline(thr, color=CRITICAL, ls=(0, (4, 2)), lw=0.9)
        ax_u.text(t0 + 0.01 * (t1 - t0), thr + 0.03, f"threshold {thr:.2f}", ha="left", va="bottom", fontsize=6.5,
                  color=INK)
        ax_u.set_ylim(0, 1)
        ax_u.set_ylabel("Urgency $U$")
        ax_u.set_xlim(t0, t1)
        ax_u.set_xlabel("Time from the first sonar echo of the rock [s]" if sid == "S1"
                        else "Time from the onset of the current [s]")
        style(ax_u)
        levels = mark_events(ax_e, [ax_e, ax_u], events)
        ax_e.set_title(title, loc="left", fontsize=8, fontweight="bold", pad=8 + 9 * (levels + 1))
    handles = [Patch(color=GOOD, label="best state"), Patch(color=AMBER, label="degraded"),
               Patch(color=CRITICAL, label="worst state"), Patch(color=EMPTY, label="not entered")]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.035), ncol=4, frameon=False,
               title="Evidence state", title_fontsize=7)
    return fig


RUNS = {"OL": ("Open loop", "#2a78d6"), "CL": ("Closed loop", "#eb6834")}       # validated categorical slots 1 and 2


def run_series(run):
    """Track, urgency and operator events of one hazard run, with time from the injection (from hazard_results.csv)."""
    data = H.load(os.path.join(BAGS, run), images=False)
    start, _ = mission_window(data)
    row = results_row(run)
    thr = float(data["/blueye/takeover_request/threshold"][-1][1])
    urg = [(t - start, v) for t, v in data["/blueye/takeover_request/urgency"]]
    t_inj = next(t for t, v in urg if v >= thr) - float(row["t_req_s"])
    odom = data["/blueye/odom"]

    def first(topic, value=None):
        return next((t - start - t_inj for t, v in data.get(topic, []) if t - start >= t_inj and (value is None or v == value)), None)
    return {"data": data, "start": start, "t_inj": t_inj, "row": row, "thr": thr,
            "t": np.array([o[0] - start - t_inj for o in odom]), "xy": np.array([[o[1], o[2]] for o in odom]),
            "u": [(t - t_inj, v) for t, v in urg], "t_req": float(row["t_req_s"]),
            "t_dec": first("/blueye/takeover_request/human_decision"),
            "t_hb": first("/blueye/takeover_request/handback", True)}


def at(r, ts):
    k = min(int(np.searchsorted(r["t"], ts)), len(r["t"]) - 1)
    return r["xy"][k]


def time_panels(ax_d, ax_u, runs, t0, t1):
    for key, r in runs.items():
        colour = RUNS[key][1]
        u = [(t, v) for t, v in r["u"] if t0 <= t <= t1]
        ax_u.plot([t for t, _ in u], [v for _, v in u], color=colour, lw=1.2)
        for ax in (ax_d, ax_u):
            ax.axvline(r["t_req"], color=colour, ls=(0, (4, 2)), lw=0.9)
            if r["t_dec"] is not None and r["t_hb"] is not None:
                ax.axvspan(r["t_dec"], r["t_hb"], color=colour, alpha=0.12, lw=0)
    thr = runs["OL"]["thr"]
    ax_u.axhline(thr, color=MUTED, ls=(0, (4, 2)), lw=0.8)
    ax_u.text(t0 + 0.01 * (t1 - t0), thr + 0.03, f"threshold {thr:.2f}", ha="left", va="bottom", fontsize=6.5, color=INK)
    ax_u.set_ylim(0, 1)
    ax_u.set_ylabel("Urgency $U$")
    ax_u.set_xlim(t0, t1)
    ax_d.tick_params(labelbottom=False)
    for ax in (ax_d, ax_u):
        style(ax)


def fig_closed_loop(s1_ol, s1_cl, s2_ol, s2_cl):
    fig = plt.figure(figsize=(7.2, 6.2))
    outer = fig.add_gridspec(2, 1, hspace=0.42)
    rows = []
    for i in range(2):
        inner = outer[i].subgridspec(2, 2, width_ratios=[0.95, 1.5], wspace=0.3, hspace=0.12)
        ax_m = fig.add_subplot(inner[:, 0])
        ax_d = fig.add_subplot(inner[0, 1])
        ax_u = fig.add_subplot(inner[1, 1], sharex=ax_d)
        rows.append((ax_m, ax_d, ax_u))

    # (a) S1: the same rock, open loop runs into it, closed loop holds and goes round it
    runs = {"OL": run_series(s1_ol), "CL": run_series(s1_cl)}
    _, rock = H.footprints(os.path.join(PAPER, s1_ol, "obstacle_config.yaml"))
    ax_m, ax_d, ax_u = rows[0]
    ax_m.fill(rock[:, 1], rock[:, 0], color=MUTED, lw=0, zorder=2)
    ax_m.text(rock[:, 1].mean() + 0.35, rock[:, 0].mean() - 0.35, "rock", fontsize=6.5, color=INK)
    ol, cl = runs["OL"], runs["CL"]
    ol["t_haz"] = ol["t_req"] + float(ol["row"]["lead_time_s"])
    t1 = cl["t_hb"] + 15
    shown = [rock]
    for key, r in runs.items():
        colour = RUNS[key][1]
        dist = H.dist_to_polygon(r["xy"], rock)
        stop = r.get("t_haz", r["t_hb"] + 40 if r["t_hb"] is not None else r["t"][-1])
        m = (r["t"] >= -20) & (r["t"] <= stop)
        ax_m.plot(r["xy"][m, 1], r["xy"][m, 0], color=colour, lw=1.5, zorder=3)
        shown.append(r["xy"][m & (np.linalg.norm(r["xy"] - rock.mean(0), axis=1) < 3.5)])
        m = (r["t"] >= -5) & (r["t"] <= t1)
        ax_d.plot(r["t"][m], dist[m], color=colour, lw=1.2)
        x, y = at(r, r["t_req"])
        ax_m.plot(y, x, "v", color=colour, mec=INK, mew=0.6, ms=6, zorder=5)
        if r["t_hb"] is not None:
            x, y = at(r, r["t_hb"])
            ax_m.plot(y, x, "^", color=colour, mec=INK, mew=0.6, ms=6, zorder=5)
    x, y = at(ol, ol["t_haz"])
    ax_m.plot(y, x, "X", color=INK, ms=7, zorder=6)
    time_panels(ax_d, ax_u, runs, -5, t1)
    for ax in (ax_d, ax_u):
        ax.axvline(ol["t_haz"], color=INK, lw=0.9)
    ax_d.text(ol["t_haz"] + 0.8, 4.6, "contact (OL)", fontsize=6.5, color=INK, va="center")
    ax_d.text(ol["t_haz"] + 0.8, 3.4, "manual control (CL)", fontsize=6.5, color=INK, va="center")
    ax_d.set_ylim(0, 5.4)
    ax_d.set_ylabel("Distance to\nrock [m]")
    ax_u.set_xlabel("Time from the first sonar echo of the rock [s]")
    e0, e1, n0, n1 = H.view_box(shown, margin=0.5, min_span=4.0)
    ax_m.set_xlim(e0, e1)
    ax_m.set_ylim(n0, n1)

    # (b) S2: the same disturbances, open loop drifts on, closed loop hands control to the supervisor
    runs = {"OL": run_series(s2_ol), "CL": run_series(s2_cl)}
    known, _ = H.footprints(H.run_config(os.path.join(PAPER, s2_ol)))
    ax_m, ax_d, ax_u = rows[1]
    for poly in known:
        ax_m.fill(poly[:, 1], poly[:, 0], color="#d9d4cc", lw=0, zorder=1)
    ol, cl = runs["OL"], runs["CL"]
    leg = [p for p in H.leg_at(ol["data"], ol["start"], ol["t_inj"] + 0.5) if p is not None]
    if len(leg) == 2:
        ax_m.plot([leg[0][1], leg[1][1]], [leg[0][0], leg[1][0]], color=INK, lw=0.8, ls=(0, (4, 2)), zorder=2)
    for n, e in leg:
        ax_m.plot(e, n, "o", mfc="white", mec=INK, mew=0.8, ms=6, zorder=6)
        ax_m.text(e + 0.45, n - 0.05, H.waypoint_label((n, e)), fontsize=6.5, color=INK, va="center", zorder=6)
    shown = list(leg) + [(n, e + 2.8) for n, e in leg]          # room for the marker labels on the right
    events = [(t - ol["t_inj"], kind, v) for t, kind, v in H.s2_events(ol["data"], ol["start"]) if t >= ol["t_inj"] - 1]
    t1 = max(t for t, *_ in events) + 40
    for key, r in runs.items():
        colour = RUNS[key][1]
        xt = [(t - r["t_inj"], d) for t, d in H.leg_cross_track(r["data"], r["start"]) if -5 <= t - r["t_inj"] <= t1]
        ax_d.plot([t for t, _ in xt], [d for _, d in xt], color=colour, lw=1.2)
        m = (r["t"] >= -10) & (r["t"] <= t1 + 30)
        ax_m.plot(r["xy"][m, 1], r["xy"][m, 0], color=colour, lw=1.5, zorder=3)
        shown.append(r["xy"][m])
        x, y = at(r, r["t_req"])
        ax_m.plot(y, x, "v", color=colour, mec=INK, mew=0.6, ms=6, zorder=5)
        if r["t_hb"] is not None:
            x, y = at(r, r["t_hb"])
            ax_m.plot(y, x, "^", color=colour, mec=INK, mew=0.6, ms=6, zorder=5)
    time_panels(ax_d, ax_u, runs, -5, t1)
    top = 1.2 * max(d for r in runs.values() for t, d in H.leg_cross_track(r["data"], r["start"])
                    if -5 <= t - r["t_inj"] <= t1)
    for t, kind, v in events:
        if kind == "usbl":
            for ax in (ax_d, ax_u):
                ax.axvline(t, color=MUTED, ls=(0, (6, 1.5, 1, 1.5)), lw=0.9)
            ax_d.text(t + 0.8 if v else t - 0.8, 0.93 * top, "USBL back" if v else "USBL cut", fontsize=6.5, color=INK,
                      va="top", ha="left" if v else "right")
    for level, name in ((0.5, "Medium"), (1.5, "High")):
        ax_d.axhline(level, color=MUTED, ls=":", lw=0.7)
        ax_d.text(t1, level + 0.05, name, ha="right", va="bottom", fontsize=6.3, color=INK)
    ax_d.text((cl["t_dec"] + cl["t_hb"]) / 2, 1.0, "manual control (CL)", fontsize=6.5, color=INK, ha="center",
              va="center", zorder=5, bbox=dict(facecolor="white", edgecolor="none", pad=0.8))
    ax_d.set_ylim(0, top)
    ax_d.set_ylabel("Off the\nleg [m]")
    ax_u.set_xlabel("Time from the onset of the current [s]")
    e0, e1, n0, n1 = H.view_box(shown, margin=0.8, min_span=6.0)
    ax_m.set_xlim(e0, e1)
    ax_m.set_ylim(n0, n1)
    size = 0.15 * min(e1 - e0, n1 - n0)
    direction = math.radians(next(v[1] for t, kind, v in events if kind == "current" and v[0] > 0))
    tail = (e1 - 0.14 * (e1 - e0), n1 - 0.3 * (n1 - n0))
    ax_m.annotate("", xy=(tail[0] + size * math.sin(direction), tail[1] + size * math.cos(direction)), xytext=tail,
                  arrowprops=dict(arrowstyle="-|>", color=MUTED, lw=1.4, mutation_scale=9))
    ax_m.text(tail[0], tail[1] - 0.04 * (n1 - n0), "current", fontsize=6.5, color=INK, ha="center", va="top")

    for (ax_m, ax_d, _), title in zip(rows, ("(a) S1: obstacle on the transit leg",
                                            "(b) S2: cross-current and loss of the USBL signal during inspection")):
        ax_m.set_aspect("equal")
        ax_m.set_xlabel("East [m]")
        ax_m.set_ylabel("North [m]")
        ax_m.grid(color=GRID, lw=0.5)
        x0 = ax_m.get_subplotspec().get_position(fig).x0
        fig.text(x0 - 0.06, ax_d.get_position().y1 + 0.025, title, fontsize=8, fontweight="bold", va="bottom")
    handles = [Line2D([], [], color=RUNS[k][1], lw=1.5, label=RUNS[k][0]) for k in RUNS] + [
        Line2D([], [], marker="v", color="white", mec=INK, ls="", ms=6, label="Takeover request (dashed line)"),
        Line2D([], [], marker="^", color="white", mec=INK, ls="", ms=6, label="Hand-back"),
        Line2D([], [], marker="X", color=INK, ls="", ms=7, label="Contact"),
        Patch(color=RUNS["CL"][1], alpha=0.25, label="Manual control"),
        Line2D([], [], color=INK, lw=0.8, ls=(0, (4, 2)), label="Planned leg")]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.045), ncol=4, frameon=False)
    return fig


def states_at(run):
    """Evidence, worst-state posteriors, P(TakeoverRequested) and U before the injection and at the request."""
    data = H.load(os.path.join(BAGS, run), images=False)
    start, _ = mission_window(data)
    row = results_row(run)
    urg = [(t - start, v) for t, v in data["/blueye/takeover_request/urgency"]]
    thr = float(data["/blueye/takeover_request/threshold"][-1][1])
    t_req = next(t for t, v in urg if v >= thr)
    t_inj = t_req - float(row["t_req_s"])
    ev = [(t - start, e) for t, e in data["/blueye/takeover_request/evidence"]]
    out = {}
    for key, t in (("before", t_inj - 5.0), ("request", t_req)):
        e = min(ev, key=lambda x: abs(x[0] - t))[1]
        out[key] = {"evidence": e["evidence"], "post": e["posteriors"], "p_req": e["probabilities"]["TakeoverRequested"],
                    "U": min(urg, key=lambda x: abs(x[0] - t))[1]}
    return out


WORST = {"NavigationSystemPerformance": ("NSP", "Inadequate"), "DataQuality": ("DQ", "Inadequate"),
         "AutonomousControlPerformance": ("ACP", "Inadequate"), "CollisionWithObstacle": ("COL", "Likely"),
         "HumanDecisionReliability": ("HDR", "Low")}


def fig_explanation(s1_run, s2_run):
    """Two takeover requests explained: active path in ink, unchanged nodes in grey; arcs labelled with the mapping
    rule and the row of the arc trace (Table S2), whose source in the task analysis is listed under each panel."""
    from matplotlib.patches import FancyArrowPatch
    fig, axes = plt.subplots(2, 1, figsize=(7.2, 6.6), gridspec_kw={"height_ratios": [1.0, 1.65], "hspace": 0.08})
    sev = {"Critical": CRITICAL, "High": CRITICAL, "Lost": CRITICAL, "Weak": AMBER, "Moderate": AMBER, "Near": AMBER,
           "Medium": AMBER}

    def node(ax, x, y, text, kind, fill=None):
        style_ = {"evidence": dict(fc=fill, ec=INK, lw=0.8, color=INK),
                  "active": dict(fc="white", ec=INK, lw=1.1, color=INK),
                  "output": dict(fc="white", ec=CRITICAL, lw=1.6, color=INK),
                  "idle": dict(fc="#f3f2ee", ec="#b9b7b0", lw=0.7, color=MUTED)}[kind]
        return ax.text(x, y, text, ha="center", va="center", fontsize=6.5, color=style_.pop("color"), zorder=3,
                       linespacing=1.25, bbox=dict(boxstyle="round,pad=0.45", **style_))

    def arc(ax, a, b, label=None, active=True, frac=0.5):
        ax.add_patch(FancyArrowPatch(posA=a.get_position(), posB=b.get_position(), patchA=a.get_bbox_patch(),
                                     patchB=b.get_bbox_patch(), arrowstyle="-|>", mutation_scale=9, shrinkA=2,
                                     shrinkB=2, color=INK if active else "#b9b7b0", lw=1.0 if active else 0.7,
                                     ls="-" if active else (0, (3, 2)), zorder=3.5))
        if label:
            (xa, ya), (xb, yb) = a.get_position(), b.get_position()
            ax.text(xa + frac * (xb - xa), ya + frac * (yb - ya), label, ha="center", va="center", fontsize=6,
                    color=INK, zorder=4, bbox=dict(fc="white", ec="none", pad=0.8))

    def sources(ax, lines, y, columns=1):
        per = -(-len(lines) // columns)
        for c in range(columns):
            ax.text(0.0 + c * 5.15, y, "\n".join(lines[c * per:(c + 1) * per]), fontsize=6, color=INK, va="top",
                    linespacing=1.45)

    def worst(st, name):
        abbr, state = WORST[name]
        b, r = st["before"]["post"][name][state], st["request"]["post"][name][state]
        return f"{abbr} {state}\n{b:.2f} → {r:.2f}"

    def output(st):
        b, r = st["before"], st["request"]
        return f"TR TakeoverRequested\n{b['p_req']:.2f} → {r['p_req']:.2f}\n$U$ {b['U']:.2f} → {r['U']:.2f}"

    # (a) S1: the collision pathway
    ax = axes[0]
    st = states_at(s1_run)
    sr = st["request"]["evidence"]["SonarRange"]
    n_sr = node(ax, 0.9, 2.3, f"SR\n{sr}", "evidence", sev.get(sr, GOOD))
    n_col = node(ax, 4.4, 2.3, worst(st, "CollisionWithObstacle"), "active")
    n_dq = node(ax, 4.4, 3.45, worst(st, "DataQuality"), "idle")
    n_acp = node(ax, 4.4, 1.15, worst(st, "AutonomousControlPerformance"), "idle")
    n_hdr = node(ax, 6.7, 1.15, worst(st, "HumanDecisionReliability"), "idle")
    n_tr = node(ax, 8.9, 2.3, output(st), "output")
    arc(ax, n_sr, n_col, "A5 (26)")
    arc(ax, n_col, n_tr, "A5, N7 (39)")
    for n in (n_dq, n_acp, n_hdr):
        arc(ax, n, n_tr, active=False)
    sources(ax, ["(26) SR → COL: return in the safety box (C6, C8) on the path to collision (C3); transit task AT3",
                 "(39) COL → TR: collision pathway to the takeover constraint C1"], 0.45)
    ax.text(0.0, 4.15, "(a) S1: obstacle on the transit leg", fontsize=8, fontweight="bold", va="center")
    ax.set_ylim(-0.25, 4.4)

    # (b) S2: the navigation and data-quality pathway
    ax = axes[1]
    st = states_at(s2_run)
    ev = st["request"]["evidence"]
    n_usbl = node(ax, 0.9, 4.6, f"USBL\n{ev['USBLStrength']}", "evidence", sev.get(ev["USBLStrength"], GOOD))
    n_wte = node(ax, 0.9, 3.4, f"WTE\n{ev['WaypointTrackingError']}", "evidence", sev.get(ev["WaypointTrackingError"], GOOD))
    n_cur = node(ax, 0.9, 2.2, f"Current\n{ev['Current']}", "evidence", sev.get(ev["Current"], GOOD))
    n_nsp = node(ax, 3.5, 3.4, worst(st, "NavigationSystemPerformance"), "active")
    n_dq = node(ax, 6.2, 4.6, worst(st, "DataQuality"), "active")
    n_acp = node(ax, 6.2, 3.4, worst(st, "AutonomousControlPerformance"), "active")
    n_col = node(ax, 6.2, 2.2, worst(st, "CollisionWithObstacle"), "idle")
    n_hdr = node(ax, 8.9, 2.0, worst(st, "HumanDecisionReliability"), "idle")
    n_tr = node(ax, 8.9, 3.4, output(st), "output")
    arc(ax, n_usbl, n_nsp, "A1, A2 (15)", frac=0.42)
    arc(ax, n_wte, n_nsp, "A1, A2 (18)", frac=0.42)
    arc(ax, n_cur, n_nsp, "N4 (17)", frac=0.42)
    arc(ax, n_nsp, n_dq, "A5 (34)", frac=0.48)
    arc(ax, n_nsp, n_acp, "A1 (30)", frac=0.5)
    arc(ax, n_dq, n_tr, "A5, N7, A4 (40, 41)", frac=0.5)
    arc(ax, n_acp, n_tr, "A3 (37)", frac=0.38)
    arc(ax, n_nsp, n_col, active=False)
    for n in (n_col, n_hdr):
        arc(ax, n, n_tr, active=False)
    sources(ax, ["(15) USBL → NSP: position fix AI1.1 feeds the navigation decision AI1.2 (C5)",
                 "(18) WTE → NSP: tracking error read in AI1.1 for the on-track decision AI1.2 (I-2)",
                 "(17) Current → NSP: read with the vehicle state (AI2.1); degrades position keeping",
                 "(30) NSP → ACP: estimation precedes execution (AU2.3, AT1.3, AH1.3, AD3.3 publish the state "
                 "used by AU3, AT2, AH2, AD5)",
                 "(34) NSP → DQ: valid position required for adequate data (I-2, I-4 → I-6 → I-X3)",
                 "(40, 41) DQ → TR: data-quality pathway to C1, active in inspection only",
                 "(37) ACP → TR: request-and-confirm interaction A-TO.3 → H-TO.1"], 1.3)
    ax.text(0.0, 5.3, "(b) S2: cross-current and loss of the USBL signal during inspection", fontsize=8,
            fontweight="bold", va="center")
    ax.set_ylim(-0.45, 5.55)
    for ax in axes:
        ax.set_xlim(-0.1, 10.1)
        ax.axis("off")
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--nominal", default="nominal_run01_shadow_20261018")
    parser.add_argument("--s1", default="s1_obstacle_run01_shadow_20261018")
    parser.add_argument("--s2", default="s2_current_usbl_run01_shadow_20261019")
    parser.add_argument("--s1-cl", default="s1_obstacle_run01_closedloop_20261018")
    parser.add_argument("--s2-cl", default="s2_current_usbl_run01_closedloop_20261019")
    parser.add_argument("--only", choices=["nominal", "traces", "closed_loop", "explanation"], default=None)
    args = parser.parse_args()
    os.makedirs(OUT, exist_ok=True)
    figures = {"nominal": ("sim_nominal_mission", lambda: fig_nominal(os.path.join(BAGS, args.nominal))),
               "traces": ("sim_traces", lambda: fig_traces(args.s1, args.s2)),
               "closed_loop": ("sim_closed_loop", lambda: fig_closed_loop(args.s1, args.s1_cl, args.s2, args.s2_cl)),
               "explanation": ("explanation_example", lambda: fig_explanation(args.s1, args.s2))}
    for key, (name, make) in figures.items():
        if args.only and key != args.only:
            continue
        fig = make()
        for ext in ("pdf", "png"):
            fig.savefig(os.path.join(OUT, f"{name}.{ext}"), dpi=300, bbox_inches="tight")
        plt.close(fig)
        print(f"saved {os.path.join(OUT, name)}.pdf/.png")


if __name__ == "__main__":
    main()
