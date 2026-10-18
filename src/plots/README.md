# Plots

Scripts stay in this folder; their output goes into one folder per run, named exactly like the run in
`src/rosbags/`.

| Folder | What is in it |
|---|---|
| `paper/<run>/` | Figures and tables for a run used in the RESS paper (bag in `src/rosbags/paper/<run>/`). |
| `paper/<run>/diagnostics/` | Engineering plots of the same run (trajectory, depth, heading, velocity tracking, thrusters). |
| `archive_2026-07_to_08/` | Plots of the older takeover tests (bags in `src/rosbags/archive_2026-06_to_08/`). |

## Scripts

| Script | Use |
|---|---|
| `plot_nominal_run.py` | Paper figures and tables for one run. |
| `plot_sim_mission.py` | Engineering plots of a simulated mission. |
| `plot_real_mission.py` | Engineering plots of a field mission. |
| `view_3d_map.py` | Interactive 3D view of a run. |
| `record_sim_mission.sh`, `record_full_mission.sh` | Older recording helpers. |

## Making the plots for a new paper run

```bash
source ~/Desktop/auto-pilot/install/setup.bash
cd ~/Desktop/auto-pilot/src
python3 plots/plot_nominal_run.py rosbags/paper/<run>
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
