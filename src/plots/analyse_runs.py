#!/usr/bin/env python3
"""
Results table of the simulation study (tab:sim-results) from all runs in src/rosbags/paper/.

S1 and S2 come from the per-run results of plot_hazard_run.py (plots/paper/<run>/hazard_results.csv; run that first
for every hazard run, with --config <the run's obstacle_config.yaml> for S1). S0 is computed here from the nominal
recordings. Values are mean +- sample standard deviation over the runs. t_att, t_req and max U are pooled over open
and closed loop: the behaviour tree acts on the recommendation only after the request, so both modes are the same
until then. The lead time comes from the open-loop runs and success from the closed-loop runs.

Usage:
  source ~/Desktop/auto-pilot/install/setup.bash
  python3 src/plots/analyse_runs.py

Output (src/plots/paper/sim_results/):
  sim_results_table.tex   the table (table* environment of the paper, caption with the S0 and S2 numbers)
  sim_results_rows.tex    the rows only, for \\input
  sim_results_runs.csv    one line per run with the values used
  text_numbers.md         numbers for the text: S0 per phase, requests per hour, operator times, closest approach
"""

import csv
import glob
import math
import os

import numpy as np

import plot_hazard_run as H

HERE = os.path.dirname(os.path.abspath(__file__))
BAGS = os.path.join(H.SRC, "rosbags", "paper")
PLOTS = os.path.join(HERE, "paper")
OUT = os.path.join(PLOTS, "sim_results")
SCENARIOS = [("S1", "Transit", "s1_obstacle_run*"), ("S2", "Inspection", "s2_current_usbl_run*")]
PHASE_ORDER = ["Undocking", "Transit", "Inspection", "Homing", "Docking"]


def num(v):
    return None if v in ("", "n/a", None) else float(v)


def mean_sd(values, digits):
    values = [v for v in values if v is not None]
    if not values:
        return "n/a"
    if len(values) == 1:
        return f"${values[0]:.{digits}f}$"
    return f"${np.mean(values):.{digits}f} \\pm {np.std(values, ddof=1):.{digits}f}$"


def nominal_run(bag):
    """S0 values of one nominal recording, over the mission (first /mission_state > 0 to 'Mission finished')."""
    data = H.load(bag, images=False)
    start = next(t for t, v in data["/mission_state"] if v > 0)
    end = next((t for t, n, m in data["/rosout"] if "Mission finished" in m), data["/mission_state"][-1][0])
    thr = float(data["/blueye/takeover_request/threshold"][-1][1]) if data["/blueye/takeover_request/threshold"] else 0.65
    states = data["/mission_state"]

    def phase_at(t):
        state = 0
        for ts, v in states:
            if ts > t:
                break
            state = v
        return H.PHASES.get(state)

    urg = [(t, v) for t, v in data["/blueye/takeover_request/urgency"] if start <= t <= end]
    per_phase = {}
    for t, v in urg:
        p = phase_at(t)
        per_phase[p] = max(per_phase.get(p, 0.0), v)
    requests, armed = 0, True                      # rising crossings, re-armed below 0.8 x threshold (as the BT)
    for t, v in urg:
        if armed and v >= thr:
            requests, armed = requests + 1, False
        elif not armed and v < 0.8 * thr:
            armed = True
    ev = [e for t, e in data["/blueye/takeover_request/evidence"] if start <= t <= end]
    top = [max(e["probabilities"], key=e["probabilities"].get) for e in ev]
    return {"run": os.path.basename(bag), "duration_s": end - start, "max_U": max(v for _, v in urg),
            "per_phase": per_phase, "requests": requests,
            "attention_top": top.count("IncreasedAttentionRequired") / len(top),
            "takeover_top": top.count("TakeoverRequested") / len(top)}


def main():
    os.makedirs(OUT, exist_ok=True)
    runs_csv = []

    # S0
    s0 = []
    for bag in sorted(glob.glob(os.path.join(BAGS, "nominal_run*"))):
        r = nominal_run(bag)
        s0.append(r)
        runs_csv.append({"ID": "S0", "run": r["run"], "mode": "OL", "max_U_before": f"{r['max_U']:.3f}",
                         "requests": r["requests"], "duration_s": f"{r['duration_s']:.0f}"})
        print(f"S0 {r['run']}: {r['duration_s']:.0f} s, max U {r['max_U']:.3f}, requests {r['requests']}")
    hours = sum(r["duration_s"] for r in s0) / 3600.0
    n_requests = sum(r["requests"] for r in s0)
    bound = -math.log(0.05) / hours                # 95 % upper bound on the rate with zero events
    rows = [f"S0 & All & {len(s0)}/-- & {mean_sd([r['max_U'] for r in s0], 2)} & n/a & n/a & n/a & n/a & n/a \\\\"]

    # S1, S2
    extra = {}
    for sid, phase, pattern in SCENARIOS:
        results = []
        for folder in sorted(glob.glob(os.path.join(PLOTS, pattern))):
            path = os.path.join(folder, "hazard_results.csv")
            if not os.path.exists(path):
                print(f"  missing {path}: run plot_hazard_run.py on {os.path.basename(folder)} first")
                continue
            with open(path) as fh:
                row = next(csv.DictReader(fh))
            results.append(row)
            runs_csv.append({k: row.get(k, "") for k in ("ID", "run", "mode", "max_U_before", "max_U_after", "t_att_s",
                                                          "t_req_s", "t_req_after_usbl_cut_s", "lead_time_s",
                                                          "closest_approach_m", "max_off_leg_m", "decision_s",
                                                          "manual_control_s", "success_CL")})
        ol = [r for r in results if r["mode"] == "OL"]
        cl = [r for r in results if r["mode"] == "CL"]
        success = sum(r["success_CL"] == "yes" for r in cl)
        lead = mean_sd([num(r.get("lead_time_s")) for r in ol], 1) if sid == "S1" else "n/a"
        rows.append(f"{sid} & {phase} & {len(ol)}/{len(cl)} & "
                    f"{mean_sd([num(r['max_U_before']) for r in results], 2)} & "
                    f"{mean_sd([num(r['max_U_after']) for r in results], 2)} & "
                    f"{mean_sd([num(r['t_att_s']) for r in results], 1)} & "
                    f"{mean_sd([num(r['t_req_s']) for r in results], 1)} & {lead} & {success}/{len(cl)} \\\\")
        extra[sid] = {"ol": ol, "cl": cl, "all": results}
        print(f"{sid}: {len(ol)} OL, {len(cl)} CL")

    s2_cut = mean_sd([num(r.get("t_req_after_usbl_cut_s")) for r in extra["S2"]["all"]], 1)
    with open(os.path.join(OUT, "sim_results_rows.tex"), "w") as fh:
        fh.write("\n".join(rows) + "\n")
    caption = (
        "Simulation results (mean $\\pm$ standard deviation over the runs). OL: open loop (the BT logs the takeover "
        "request but does not act on it); CL: closed loop (the BT holds the vehicle and the supervisor takes over). "
        "$t_{\\mathrm{att}}$, $t_{\\mathrm{req}}$ and $\\max U$ are pooled over both modes, which behave identically "
        "until the request. Times are measured from the injection: the first sonar echo from the obstacle in S1 and the "
        "onset of the cross-current in S2. In S2 the USBL signal was cut 20.5\\,s after the onset of the current, and "
        f"the request followed {s2_cut}\\,s after the cut. The lead time is "
        "$t_{\\mathrm{haz}}-t_{\\mathrm{req}}$ in the open-loop runs; S2 has no undesired event. Success is the number "
        "of closed-loop runs in which the vehicle held before the undesired event and resumed the mission after the "
        f"hand-back. For S0, $\\max U$ is taken over the whole mission; no takeover request occurred in {len(s0)} "
        f"missions ({hours * 60:.0f}\\,min).")
    with open(os.path.join(OUT, "sim_results_table.tex"), "w") as fh:
        fh.write("\\begin{table*}[width=\\textwidth,cols=9,pos=t]\n")
        fh.write(f"\\caption{{{caption}}}\n\\label{{tab:sim-results}}\n\\small\n")
        fh.write("\\begin{tabular*}{\\textwidth}{@{\\extracolsep{\\fill}}llccccccc@{}}\n\\toprule\n")
        fh.write("ID & Phase & Runs (OL/CL) & $\\max U$ before & $\\max U$ after & $t_{\\mathrm{att}}$ [s] & "
                 "$t_{\\mathrm{req}}$ [s] & Lead time [s] & Success (CL) \\\\\n\\midrule\n")
        fh.write("%% Generated by src/plots/analyse_runs.py from the runs in src/rosbags/paper/\n")
        fh.write("\n".join(rows) + "\n\\bottomrule\n\\end{tabular*}\n\\end{table*}\n")

    fields = sorted({k for r in runs_csv for k in r}, key=lambda k: (k not in ("ID", "run", "mode"), k))
    with open(os.path.join(OUT, "sim_results_runs.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        w.writerows(runs_csv)

    # numbers for the text
    lines = ["# Numbers for the text (generated by analyse_runs.py)", "", "## S0 nominal", "",
             f"- {len(s0)} missions, {hours * 60:.1f} min in total; takeover requests: {n_requests}"
             + (f" (95 % upper bound on the rate: {bound:.1f} per hour)" if n_requests == 0 else
                f" ({n_requests / hours:.1f} per hour)"),
             f"- time with IncreasedAttentionRequired most probable: "
             f"{mean_sd([100 * r['attention_top'] for r in s0], 1).strip('$')} % of the mission "
             f"(TakeoverRequested: {mean_sd([100 * r['takeover_top'] for r in s0], 1).strip('$')} %)", "",
             "| Phase | max U (mean ± SD over runs) | highest |", "|---|---|---|"]
    for p in PHASE_ORDER:
        vals = [r["per_phase"][p] for r in s0 if p in r["per_phase"]]
        lines.append(f"| {p} | {mean_sd(vals, 2).strip('$').replace(chr(92) + 'pm', '±')} | {max(vals):.2f} |")
    for sid in ("S1", "S2"):
        cl, ol = extra[sid]["cl"], extra[sid]["ol"]
        lines += ["", f"## {sid}", "",
                  f"- operator decision time (popup to accept): {mean_sd([num(r['decision_s']) for r in cl], 1).strip('$')} s",
                  f"- manual control (accept to hand-back): {mean_sd([num(r['manual_control_s']) for r in cl], 1).strip('$')} s"]
        if sid == "S1":
            lines += [f"- closest approach, open loop: {mean_sd([num(r['closest_approach_m']) for r in ol], 2).strip('$')} m "
                      f"(contact in {len(ol)}/{len(ol)} runs); closed loop: "
                      f"{mean_sd([num(r['closest_approach_m']) for r in cl], 2).strip('$')} m (no contact)"]
        else:
            lines += [f"- request after the USBL cut: {s2_cut.strip('$')} s (all runs)",
                      f"- max distance off the leg while autonomous, open loop: "
                      f"{mean_sd([num(r['max_off_leg_m']) for r in ol], 2).strip('$')} m"]
    text = "\n".join(lines).replace("\\pm", "±") + "\n"
    with open(os.path.join(OUT, "text_numbers.md"), "w") as fh:
        fh.write(text)
    print()
    print(open(os.path.join(OUT, "sim_results_table.tex")).read())
    print(text)
    print(f"Saved to {OUT}")


if __name__ == "__main__":
    main()
