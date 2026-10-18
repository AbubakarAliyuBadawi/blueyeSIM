# Plots

Scripts stay in this folder; their output goes into one folder per run, named exactly like the run in
`src/rosbags/`.

| Folder | What is in it |
|---|---|
| `paper/<run>/` | Figures and tables for a run used in the RESS paper (bag in `src/rosbags/paper/<run>/`). |
| `paper/<run>/diagnostics/` | Engineering plots of the same run (trajectory, depth, heading, velocity tracking, thrusters). |
| `paper/mission_plan/` | Plan view of the mission (`mission_plan.pdf/.png`), drawn from `TestMission.xml`. |
| `paper/s1_obstacle_comparison/` | S1 open loop against closed loop (`s1_ol_vs_cl.pdf/.png`, `.csv/.md`). |
| `development_2026-10_bn_v4/` | Plots of superseded development runs (bags in `src/rosbags/development_2026-10_bn_v4/`). |
| `archive_2026-07_to_08/` | Plots of the older takeover tests (bags in `src/rosbags/archive_2026-06_to_08/`). |

## Scripts

| Script | Use |
|---|---|
| `plot_nominal_run.py` | Paper figures and tables for one run. |
| `plot_mission_plan.py` | Plan view of the mission: waypoints read from the behaviour tree, markers from `pipe.scn`. Rerun after changing the mission. |
| `plot_hazard_run.py` | Results of one hazard-scenario run: `hazard_timeline` (Fig. 9 panel), `hazard_results.csv/.md` (one row of the results table), `hazard_summary.txt`. Run with `--scenario S1` (injection found automatically from the first sonar echo off the rock) or `--injection <s>`. |
| `plot_ol_cl_comparison.py` | S1 open loop against closed loop (`--ol <bag> --cl <bag>`): plan view around the rock, distance to the rock and urgency; output in `paper/s1_obstacle_comparison/`. |
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
```

## Output of `plot_nominal_run.py` (PDF for the paper, PNG for viewing)

| File | Content |
|---|---|
| `fig1_mission_map` | Track coloured by mission phase, waypoints, docking station and pipeline footprints. |
| `fig2_timeline` | (a) urgency against the threshold, (b) TakeoverRequest posterior, (c) evidence entered in the BN by severity (grey: not entered by an evidence rule), (d) operator fatigue and stress. |
| `fig3_hidden_nodes` | Posterior of the intermediate BN nodes over time (the explanation layer). |
| `phase_summary.csv` / `.md` | Per phase: start, duration, mean and max urgency, time above threshold, alarms. |
| `run_summary.txt` | Mission and docking result, alarms, peak urgency with the evidence at the peak, battery, distance. |

The analysis window runs from the first `/mission_state` to "Mission finished", so BN output from before the
behaviour tree starts the mission is left out (and counted in `run_summary.txt`).
