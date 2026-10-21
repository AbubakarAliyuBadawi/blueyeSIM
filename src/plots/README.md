# Plots

Scripts stay in this folder; their output goes into one folder per run, named exactly like the run in
`src/rosbags/`.

| Folder | What is in it |
|---|---|
| `paper/<run>/` | Figures and tables for a run used in the RESS paper (bag in `src/rosbags/paper/<run>/`). |
| `paper/<run>/diagnostics/` | Engineering plots of the same run (trajectory, depth, heading, velocity tracking, thrusters). |
| `paper/mission_plan/` | Plan view of the mission (`mission_plan.pdf/.png`), drawn from `TestMission.xml`. |
| `paper/s1_obstacle_comparison/` | S1 open loop against closed loop (`s1_ol_vs_cl.pdf/.png`, `.csv/.md`). |
| `paper/sim_results/` | The results table of the paper from all runs (`sim_results_table.tex`, `sim_results_rows.tex`), one line per run (`sim_results_runs.csv`) numbers for the text (`text_numbers.md`), the Simulation study subsection (`sec_res_sim.tex`) and its two figures. Made by `analyse_runs.py` and `plot_paper_figures.py`. |
| `paper/s2_current_usbl_comparison/` | S2 open loop against closed loop (`s2_ol_vs_cl.pdf/.png`, `.csv/.md`). |
| `development_2026-10_bn_v4/` | Plots of superseded development runs (bags in `src/rosbags/development_2026-10_bn_v4/`). |
| `archive_2026-07_to_08/` | Plots of the older takeover tests (bags in `src/rosbags/archive_2026-06_to_08/`). |

## Scripts

| Script | Use |
|---|---|
| `plot_nominal_run.py` | Paper figures and tables for one run. |
| `plot_mission_plan.py` | Plan view of the mission: waypoints read from the behaviour tree, markers from `pipe.scn`. Rerun after changing the mission. |
| `plot_hazard_run.py` | Results of one hazard-scenario run: `hazard_timeline` (Fig. 9 panel), `hazard_results.csv/.md` (one row of the results table), `hazard_summary.txt`. Run with `--scenario S1` (injection found automatically from the first sonar echo off the rock), `--scenario S2` (injection = first current onset, read from `/scenario/events` or the `sim_current` log; also reports the time from the USBL cut and the distance off the leg; the S2 window ends 40 s after the last disturbance, or 30 s after the hand-back when that is later) or `--injection <s>`. |
| `plot_paper_figures.py` | The two simulation figures of the paper, in `paper/sim_results/`: `sim_nominal_mission` (phase, recommendation posteriors, U for one nominal run) `sim_traces` (S1 and S2 open-loop traces: evidence strip and U), `sim_closed_loop` (open against closed loop, S1 and S2) and `explanation_example` (the S1 and S2 requests traced through the network with the rows of the arc trace). `--only <figure>` makes one of them. Run `plot_hazard_run.py` on the S1 and S2 runs first. |
| `analyse_runs.py` | The results table (mean ± SD over all runs in `rosbags/paper/`) and the S0 numbers for the text, in `paper/sim_results/`. Run `plot_hazard_run.py` on every hazard run first; S1 runs with a moved rock keep their rock config in `paper/<run>/obstacle_config.yaml`, pass it with `--config`. |
| `plot_ol_cl_comparison.py` | Open loop against closed loop (`--ol <bag> --cl <bag>`). S1 (default): plan view around the rock, distance to the rock and urgency, in `paper/s1_obstacle_comparison/`. `--scenario S2`: plan view of the inspection leg, the current and USBL loss as injected, distance off the leg and urgency, in `paper/s2_current_usbl_comparison/`. Distance off the leg in the tables is while the vehicle steered itself (closed loop: until the operator accepted); the distance under manual control is a separate column, since the BN enters no tracking error then. When both runs got the same disturbances (a replayed run), panel (b) draws them once. |
| `plot_sim_mission.py` | Engineering plots of a simulated mission. |
| `plot_real_mission.py` | Engineering plots of a field mission. |
| `view_3d_map.py` | Interactive 3D view of a run. |
| `record_sim_mission.sh`, `record_full_mission.sh` | Older recording helpers. |

## Making the plots for a new paper run

```bash
source ~/Desktop/auto-pilot/install/setup.bash
cd ~/Desktop/auto-pilot/src
python3 plots/plot_nominal_run.py rosbags/paper/<run>        # add --obstacle-config <blueye_obstacle_config.yaml> for S1 runs
python3 plots/plot_sim_mission.py rosbags/paper/<run> --out plots/paper/<run>/diagnostics
python3 plots/plot_hazard_run.py rosbags/paper/<run> --scenario S1    # or S2; hazard runs only
```

## Output of `plot_nominal_run.py` (PDF for the paper, PNG for viewing)

| File | Content |
|---|---|
| `fig1_mission_map` | Track coloured by mission phase, waypoints, docking station and pipeline footprints; the rock (S1), takeover request / hand-back markers (closed loop) and the water-current direction (S2). |
| `fig2_timeline` | (a) urgency against the threshold, (b) TakeoverRequest posterior, (c) evidence entered in the BN by severity (grey: not entered by an evidence rule), (d) operator fatigue and stress. Dotted lines and the "Disturbances:" line mark injected current changes and USBL cuts; dash-dot lines and the "Takeover:" line mark the operator events. |
| `fig3_hidden_nodes` | Posterior of the intermediate BN nodes over time (the explanation layer). |
| `phase_summary.csv` / `.md` | Per phase: start, duration, mean and max urgency, time above threshold, alarms. |
| `run_summary.txt` | Mission and docking result, alarms, peak urgency with the evidence at the peak, battery, distance. |

The analysis window runs from the first `/mission_state` to "Mission finished", so BN output from before the
behaviour tree starts the mission is left out (and counted in `run_summary.txt`).
