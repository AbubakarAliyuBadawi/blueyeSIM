#!/usr/bin/env python3
"""
Results of the field runs (src/plots/field/<run>/, made by plot_field_run.py) for the paper, in src/plots/field/field_results/:
  field_runs.csv/.md          one row per run: mode, mission time, distance, survey completed, requests, battery
  field_nominal_phases.csv/.md  urgency per phase of the nominal run, next to the simulation's SN (max U per phase of
                              paper/<S0_RUN>/phase_summary.csv)
  field_takeovers.csv/.md     one row per takeover (TO1, TO2, ...) with the metrics of the simulation table
  field_takeovers_rows.tex    the same as LaTeX table rows
  field_twin_takeovers.csv/.md  the takeovers of the fault run next to those of its simulator twin (paper/sf_field_twin_*,
  field_twin_takeovers_rows.tex   scenario SC), with LaTeX rows
  text_numbers.md             numbers for the text
  field_overview.pdf/.png     urgency over the nominal run and over the fault run, with the takeovers
  field_twin_overview.pdf/.png  urgency over the fault run and over its simulator twin, with the takeovers

Usage (after plot_field_run.py on every run, the simulator twin included):
  source ~/Desktop/auto-pilot/install/setup.bash
  python3 src/plots/analyse_field_runs.py
"""

import csv
import os
import re
import statistics as st
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plot_field_run as pfr  # noqa: E402
import plot_nominal_run as pnr  # noqa: E402

import yaml  # noqa: E402

FIELD = os.path.join(HERE, "field")
BAGS = os.path.join(HERE, "..", "rosbags", "field")
OUT = os.path.join(FIELD, "field_results")
PAPER = os.path.join(HERE, "paper")
PAPER_BAGS = os.path.join(HERE, "..", "rosbags", "paper")
S0_RUN = "nominal_run01_shadow_20261027"          # the simulation's nominal run (layout v4)
TWIN_PREFIX = "sf_field_twin_"                     # the simulator twin of the fault run (scenario SC)


def read_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def summary_value(lines, key):
    return next((l.split(":", 1)[1].strip() for l in lines if l.startswith(key + ":")), "")


def runs():
    names = sorted(d for d in os.listdir(FIELD) if os.path.exists(os.path.join(FIELD, d, "run_summary.txt")))
    out = []
    for name in names:
        lines = open(os.path.join(FIELD, name, "run_summary.txt")).read().splitlines()
        window = re.search(r"\(([\d.]+) s\)", summary_value(lines, "analysis window"))
        batt = re.findall(r"([\d.]+) %", summary_value(lines, "battery"))
        takeovers = os.path.join(FIELD, name, "takeover_events.csv")
        out.append({
            "run": name, "mode": summary_value(lines, "takeover mode"),
            "mission_time_s": float(window[1]) if window else "",
            "distance_m": summary_value(lines, "distance travelled").replace(" m", ""),
            "result": summary_value(lines, "mission result"),
            "alarms": summary_value(lines, "alarms (rising crossings, re-armed below 0.8 x threshold)"),
            "requests_answered": len(read_csv(takeovers)) if os.path.exists(takeovers) else 0,
            "battery_used_pct": round(float(batt[0]) - float(batt[1]), 1) if len(batt) == 2 else "",
        })
    return out


def sim_s0_max_u():
    """{phase: max U} of the simulation's SN run (paper/<S0_RUN>/phase_summary.csv)."""
    return {p["phase"]: f"{float(p['urgency_max']):.2f}"
            for p in read_csv(os.path.join(PAPER, S0_RUN, "phase_summary.csv"))}


def twin_run():
    """The simulator twin of the fault run: the latest paper/sf_field_twin_* with a takeover table, or None."""
    names = sorted(d for d in os.listdir(PAPER) if d.startswith(TWIN_PREFIX)
                   and os.path.exists(os.path.join(PAPER, d, "takeover_events.csv")))
    return names[-1] if names else None


def write_table(rows, name, header=None):
    with open(os.path.join(OUT, f"{name}.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(OUT, f"{name}.md"), "w") as f:
        if header:
            f.write(header + "\n\n")
        f.write("| " + " | ".join(rows[0]) + " |\n|" + "---|" * len(rows[0]) + "\n")
        for r in rows:
            f.write("| " + " | ".join(str(v) for v in r.values()) + " |\n")


def overview_figure(panels, name, change_label="Injected evidence change"):
    """Urgency over each run, with the takeovers. panels: [(bag, run folder with field_run.yaml, tag)]."""
    fig, axes = plt.subplots(2, 1, figsize=(7.2, 4.1), gridspec_kw={"hspace": 0.62})
    for ax, (bag, folder, tag) in zip(axes, panels):
        run = yaml.safe_load(open(os.path.join(folder, "field_run.yaml")))
        data, t0 = pfr.load_bag(bag)
        sim = pfr.is_sim(data)
        cuts = pfr.exclude_windows(data, t0, run.get("exclude", []))
        start, end, _ = pfr.mission_window(data)
        segs = pnr.phase_segments(data, start, end)
        total = segs[-1][1] - segs[0][0]
        for a, b, p in segs:
            phase_name, colour = pnr.PHASES.get(p, (str(p), "grey"))
            ax.axvspan(a - start, b - start, color=colour, alpha=0.12, lw=0)
            if (b - a) >= 0.07 * total:
                ax.text((a + b) / 2 - start, 1.02, phase_name, transform=ax.get_xaxis_transform(), ha="center",
                        va="bottom", fontsize=7, color=colour)
        urg = [(t - start, v) for t, v in data["/blueye/takeover_request/urgency"] if start <= t <= end]
        ax.plot(*zip(*urg), color="black", lw=1.0, label="Urgency")
        thr = data["/blueye/takeover_request/threshold"]
        ax.axhline(float(thr[-1][1]) if thr else 0.65, color="#d62728", ls="--", lw=1.0, label="Threshold")
        for t, removed in cuts:
            for dx in (-0.8, 0.8):
                ax.axvline(t - start + dx, color="#52514e", lw=0.6)
        groups = pfr.group_takeovers(pfr.takeover_events(data, start, end), run.get("takeovers", []))
        for g in groups:
            ax.axvline(g["request"] - start, color="#d62728", lw=0.9, ls="-.")
            if "hand-back" in g:
                ax.axvline(g["hand-back"] - start, color="#2a78d6", lw=0.9, ls="-.")
                ax.axvspan(g["request"] - start, g["hand-back"] - start, color="#d62728", alpha=0.06, lw=0)
        pfr.label_takeovers(ax, groups, start, 0.93)
        for t, label, colour, _ in pfr.injections(data, start, end):
            if not sim or label.startswith("USBL"):          # the twin's sonar readings came from the rocks
                ax.axvline(t - start, color=colour, lw=0.8, ls=(0, (1, 2)))
        ax.set_xlim(0, end - start)
        ax.set_ylim(0, 1)
        ax.set_ylabel("Urgency")
        ax.text(0.0, 1.13, tag, transform=ax.transAxes, fontsize=7.5, fontweight="bold", va="bottom")
    axes[1].set_xlabel("Mission time [s]")
    handles = [plt.Line2D([], [], color="black", lw=1.0, label="Urgency"),
               plt.Line2D([], [], color="#d62728", ls="--", lw=1.0, label="Threshold"),
               plt.Line2D([], [], color="#52514e", lw=0.8, ls=(0, (1, 2)), label=change_label),
               plt.Line2D([], [], color="#d62728", lw=0.9, ls="-.", label="Takeover request"),
               plt.Line2D([], [], color="#2a78d6", lw=0.9, ls="-.", label="Hand-back")]
    axes[1].legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.32), ncol=5, frameon=False, fontsize=7)
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(OUT, f"{name}.{ext}"), dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"  {name}.pdf/.png")


def twin_takeovers(faults, twin):
    """The takeovers of the fault run and of its simulator twin, one row each, TO1 field, TO1 twin, ..."""
    columns = ["phase", "injected", "max_U_before", "max_U_after", "t_att_s", "t_req_s", "t_req_after_trigger_s",
               "decision", "decision_s", "manual_control_s", "handback_after_cleared_s"]
    field = read_csv(os.path.join(FIELD, faults, "takeover_events.csv"))
    sim = {t["ID"]: t for t in read_csv(os.path.join(PAPER, twin, "takeover_events.csv"))}
    rows = []
    for f in field:
        for run, t in (("FC", f), ("SC", sim.get(f["ID"]))):
            if t is not None:
                rows.append({"ID": f["ID"], "run": run, **{c: t[c] for c in columns}})
    write_table(rows, "field_twin_takeovers",
                f"Takeovers of the closed-loop field run ({faults}) and of its simulator twin ({twin}). Times in s; "
                "t_att and t_req from the first change of the takeover (field: injected; twin: USBL injected, sonar "
                "from the rocks), t_req after trigger from the change that crossed the threshold.")
    with open(os.path.join(OUT, "field_twin_takeovers_rows.tex"), "w") as f:
        f.write(f"%% Generated by src/plots/analyse_field_runs.py from src/plots/field/{faults}/ and "
                f"src/plots/paper/{twin}/takeover_events.csv\n")
        f.write("%% ID & Run & Phase & Evidence changes & max U before & max U after & t_att [s] & t_req [s] & "
                "t_req after trigger [s] & decision [s] & manual control [s]\n")
        for t in rows:
            # each degraded state once, in order; the returns to Clear / Strong (a flickering echo) are in the .md
            changes = ", ".join(dict.fromkeys(c for c in re.sub(r" \(\d+ s\)", "", t["injected"]).split(", ")
                                              if not c.endswith(("Clear", "Strong"))))
            f.write(f"{t['ID']} & {t['run']} & {t['phase']} & {changes} & ${float(t['max_U_before']):.2f}$ & "
                    f"${float(t['max_U_after']):.2f}$ & ${float(t['t_att_s']):.1f}$ & ${float(t['t_req_s']):.1f}$ & "
                    f"${float(t['t_req_after_trigger_s']):.1f}$ & ${float(t['decision_s']):.1f}$ & "
                    f"${float(t['manual_control_s']):.1f}$ \\\\\n")
    return rows


def main():
    os.makedirs(OUT, exist_ok=True)
    rs = runs()
    write_table(rs, "field_runs")
    nominal = next(r["run"] for r in rs if r["mode"] == "shadow")
    faults = next(r["run"] for r in rs if r["mode"] == "closed_loop")

    sim = sim_s0_max_u()
    phases = read_csv(os.path.join(FIELD, nominal, "phase_summary.csv"))
    nominal_rows = [{"phase": p["phase"], "duration_s": p["duration_s"], "mean_U": p["urgency_mean"],
                     "max_U": p["urgency_max"], "alarms": p["alarms"],
                     "sim_SN_max_U": sim.get(p["phase"], "")} for p in phases]
    write_table(nominal_rows, "field_nominal_phases",
                f"Urgency per phase of the nominal field run (shadow mode) and the simulation's SN max U per phase "
                f"({S0_RUN}).")

    tk = read_csv(os.path.join(FIELD, faults, "takeover_events.csv"))
    takeover_rows = [{"ID": t["ID"], "phase": t["phase"], "injected": t["injected"],
                      "max_U_before": t["max_U_before"], "max_U_after": t["max_U_after"], "t_att_s": t["t_att_s"],
                      "t_req_s": t["t_req_s"], "t_req_after_trigger_s": t["t_req_after_trigger_s"],
                      "decision": t["decision"], "decision_s": t["decision_s"],
                      "manual_control_s": t["manual_control_s"],
                      "handback_after_cleared_s": t["handback_after_cleared_s"]} for t in tk]
    write_table(takeover_rows, "field_takeovers",
                "Takeovers of the closed-loop field run. Times in s; t_att and t_req from the first injected change "
                "of the takeover, t_req after trigger from the change that crossed the threshold.")
    with open(os.path.join(OUT, "field_takeovers_rows.tex"), "w") as f:
        f.write("%% Generated by src/plots/analyse_field_runs.py from src/plots/field/" + faults + "/takeover_events.csv\n")
        f.write("%% ID & Phase & Injected & max U before & max U after & t_att [s] & t_req [s] & decision [s] & "
                "manual control [s]\n")
        for t in takeover_rows:
            injected = re.sub(r" \(\d+ s\)", "", t["injected"])
            f.write(f"{t['ID']} & {t['phase']} & {injected} & ${float(t['max_U_before']):.2f}$ & "
                    f"${float(t['max_U_after']):.2f}$ & ${float(t['t_att_s']):.1f}$ & ${float(t['t_req_s']):.1f}$ & "
                    f"${float(t['decision_s']):.1f}$ & ${float(t['manual_control_s']):.1f}$ \\\\\n")

    def ms(values):
        v = [float(x) for x in values if x != ""]
        return f"{st.mean(v):.1f} ± {st.stdev(v):.1f}" if len(v) > 1 else (f"{v[0]:.1f}" if v else "n/a")
    nom = next(r for r in rs if r["run"] == nominal)
    flt = next(r for r in rs if r["run"] == faults)
    nominal_lines = open(os.path.join(FIELD, nominal, "run_summary.txt")).read().splitlines()
    by_phase = {p["phase"]: p for p in phases}
    lines = [
        "# Numbers for the text (generated by analyse_field_runs.py)", "",
        "## FN: nominal mission (shadow mode)", "",
        f"- run {nominal}: {float(nom['mission_time_s']) / 60:.1f} min of mission, {nom['distance_m']} m travelled, "
        f"{nom['result']}",
        f"- takeover requests that the network would have raised: {nom['alarms']} "
        f"(peak urgency {summary_value(nominal_lines, 'peak urgency')})",
        "- urgency per phase (max): " + ", ".join(f"{p['phase']} {float(p['urgency_max']):.2f}" for p in phases),
        "- transit and inspection stayed at or below "
        f"{max(float(by_phase[p]['urgency_max']) for p in ('Transit', 'Inspection') if p in by_phase):.2f}",
        "",
        "## FC: injected disturbances (closed loop)", "",
        f"- run {faults}: {float(flt['mission_time_s']) / 60:.1f} min of mission, {flt['distance_m']} m travelled, "
        f"{flt['result']}",
        f"- {len(tk)} takeover requests, all answered "
        f"({sum(1 for t in tk if t['decision'] == 'accepted')} accepted) and handed back",
        "- request after the change that crossed the threshold: "
        + ", ".join(f"{t['ID']} {float(t['t_req_after_trigger_s']):.2f} s" for t in tk),
        f"- operator decision time (request to answer): {ms(t['decision_s'] for t in tk)} s",
        f"- manual control (answer to hand-back): {ms(t['manual_control_s'] for t in tk)} s",
        f"- hand-back after the injected fault was cleared: {ms(t['handback_after_cleared_s'] for t in tk)} s",
        "- peak urgency: " + ", ".join(f"{t['ID']} {float(t['max_U_after']):.2f}" for t in tk),
    ]
    twin = twin_run()
    if twin:
        tw = read_csv(os.path.join(PAPER, twin, "takeover_events.csv"))
        twin_lines = open(os.path.join(PAPER, twin, "run_summary.txt")).read().splitlines()
        window = re.search(r"\(([\d.]+) s\)", summary_value(twin_lines, "analysis window"))
        lines += [
            "",
            "## SC: simulator twin of FC (closed loop)", "",
            f"- run {twin}: {float(window[1]) / 60:.1f} min of mission, "
            f"{summary_value(twin_lines, 'distance travelled')} travelled, {summary_value(twin_lines, 'mission result')}",
            f"- {len(tw)} takeover requests, all answered "
            f"({sum(1 for t in tw if t['decision'] == 'accepted')} accepted) and handed back",
            "- request after the change that crossed the threshold: "
            + ", ".join(f"{t['ID']} {float(t['t_req_after_trigger_s']):.2f} s" for t in tw),
            "- request after the first change of the takeover: "
            + ", ".join(f"{t['ID']} {float(t['t_req_s']):.2f} s" for t in tw)
            + " (field: " + ", ".join(f"{t['ID']} {float(t['t_req_s']):.2f} s" for t in tk) + ")",
            f"- operator decision time (request to answer): {ms(t['decision_s'] for t in tw)} s",
            f"- manual control (answer to hand-back): {ms(t['manual_control_s'] for t in tw)} s",
            "- peak urgency: " + ", ".join(f"{t['ID']} {float(t['max_U_after']):.2f}" for t in tw),
        ]
    with open(os.path.join(OUT, "text_numbers.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))
    overview_figure([(os.path.join(BAGS, nominal), os.path.join(FIELD, nominal), "(a) FN, nominal mission, shadow mode"),
                     (os.path.join(BAGS, faults), os.path.join(FIELD, faults), "(b) FC, closed loop with injected disturbances")],
                    "field_overview")
    if twin:
        twin_takeovers(faults, twin)
        overview_figure([(os.path.join(BAGS, faults), os.path.join(FIELD, faults), "(a) FC, field, closed loop"),
                         (os.path.join(PAPER_BAGS, twin), os.path.join(PAPER, twin),
                          "(b) SC, simulator twin, closed loop (sonar from the rocks)")],
                        "field_twin_overview", "Injected change (field: sonar, USBL; twin: USBL)")


if __name__ == "__main__":
    main()
