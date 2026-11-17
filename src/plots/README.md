# Plots

Scripts stay in this folder; their output goes into one folder per run, named exactly like the run in
`src/rosbags/`. A run's folder also keeps the simulator config it was recorded with (`scenario_config.yaml`, or `obstacle_config.yaml` for S1); the scripts use it, so runs of the earlier layout are drawn in their own layout.

Names in the paper: **SN** and **SC** are the simulated missions (nominal in shadow mode; closed loop with disturbances, the twin of FC), **FN** and **FC** the field missions, **TO1–TO3** the three takeovers of SC and FC. Bags and folders keep their file names: SN `nominal_run01_shadow_20261027`, SC `sf_field_twin_run01_closedloop_20261027`, FN `pier_nominal_run01_shadow_20261025`, FC `pier_faults_run01_closedloop_20261025`. (Before 2026-10-30 these were S0, SF, N1, D1 and F1–F3.)

| Folder | What is in it |
|---|---|
| `paper/<run>/` | Figures and tables for a run used in the RESS paper (bag in `src/rosbags/paper/<run>/`). |
| `paper/<run>/diagnostics/` | Engineering plots of the same run (trajectory, depth, heading, velocity tracking, thrusters). |
| `paper/state_frequencies/` | How often each evidence state occurred in normal operation, next to the priors of `UnifiedMissionDBN_v4.xdsl` (`nominal_state_frequencies.csv`, `state_frequencies_summary.md`), and the bag inventory (`bag_inventory.csv`). Made by `state_frequencies.py`. |
| `paper/diagrams/` | `system_overview.drawio`: the system overview for the paper (supervisor, ROS 2 execution and decision support, simulated or real vehicle); open and export in draw.io. |
| `paper/mission_plan/` | Plan view of the mission (`mission_plan.pdf/.png`), drawn from `TestMission.xml`. |
| `paper/s1_obstacle_comparison/` | S1 open loop against closed loop (`s1_ol_vs_cl.pdf/.png`, `.csv/.md`). |
| `paper/sim_results/` | Simulation figures of the paper (layout v4): `sim_mission_maps` (SN and SC tracks, close-ups of the two rocks), from `plot_sim_maps.py`. The v3 results table and figures are in `archive_v3_layout_2026-10-20/sim_results/`. |
| `paper/s2_current_usbl_comparison/` | S2 open loop against closed loop (`s2_ol_vs_cl.pdf/.png`, `.csv/.md`). |
| `paper/nominal_run01_shadow_20261027/` | SN (v4). Drawn with `plot_field_run.py` (2026-10-27), so its timeline matches the SC and field figures; the earlier `plot_nominal_run.py` style differs only in the legends. |
| `paper/sf_field_twin_run01_closedloop_20261027/` | The simulator twin of the closed-loop field run (scenario SC): drawn by `plot_field_run.py` like the field run, with `takeover_events.csv/.md` for TO1–TO3; `scenario_config.yaml` holds both rocks, `field_run.yaml` the takeover labels. |
| `field/<run>/` | Figures and tables for a field run of the real Blueye (bag in `src/rosbags/field/<run>/`); `field_run.yaml` keeps what is particular to the run (site, left-out windows, takeover labels). |
| `field/mission_plan/` | The pier mission on the satellite image (`pier_layout.pdf/.png`), from `plot_pier_layout.py`. |
| `field/field_results/` | The field results for the paper: runs, nominal urgency per phase next to the simulation's SN (`nominal_run01_shadow_20261027`), the takeovers TO1–TO3 (`.csv/.md`, LaTeX rows), the takeovers next to the simulator twin's (`field_twin_takeovers`, `.csv/.md`, LaTeX rows), numbers for the text, `field_overview` and `field_twin_overview` (urgency of the field run above its twin). Made by `analyse_field_runs.py`. |
| `archive_v3_layout_2026-10-20/` | Figures and tables of the earlier paper runs in layout v3 (18–20 October 2026), including their `sim_results/` and comparisons. |
| `development_2026-10_bn_v4/` | Plots of superseded development runs (bags in `src/rosbags/development_2026-10_bn_v4/`). |
| `archive_2026-07_to_08/` | Plots of the older takeover tests (bags in `src/rosbags/archive_2026-06_to_08/`). |

## Scripts

| Script | Use |
|---|---|
| `plot_nominal_run.py` | Paper figures and tables for one run. |
| `plot_mission_plan.py` | Plan view of the mission: waypoints read from the behaviour tree, markers from `pipe.scn`, structures placed by the simulator config (the pipe mesh from its `pipeline_mesh` key). The view follows the route. Rerun after changing the mission. |
| `plot_layout_proposal.py` | The v4 layout as proposed, over the seabed depth of the Stonefish world, with the estimated mission timeline (`paper/mission_plan/layout_proposal`). |
| `plot_hazard_run.py` | Results of one hazard-scenario run: `hazard_timeline` (Fig. 9 panel), `hazard_results.csv/.md` (one row of the results table), `hazard_summary.txt`. Run with `--scenario S1` (injection found automatically from the first sonar echo off the rock), `--scenario S2` (injection = first current onset, read from `/scenario/events` or the `sim_current` log; also reports the time from the USBL cut and the distance off the leg; the S2 window ends 40 s after the last disturbance, or 30 s after the hand-back when that is later) or `--injection <s>`. |
| `plot_paper_figures.py` | The two simulation figures of the paper, in `paper/sim_results/`: `sim_nominal_mission` (phase, recommendation posteriors, U for one nominal run) `sim_traces` (S1 and S2 open-loop traces: evidence strip and U), `sim_closed_loop` (open against closed loop, S1 and S2) and `explanation_example` (now in `field/field_results/`: the F1 and F3 requests of the field run D1 traced through the network with the rows of the arc trace, `--only explanation`). `--only <figure>` makes one of them. Run `plot_hazard_run.py` on the S1 and S2 runs first. |
| `analyse_runs.py` | The results table (mean ± SD over all runs in `rosbags/paper/`) and the S0 numbers for the text, in `paper/sim_results/`. Run `plot_hazard_run.py` on every hazard run first; S1 runs with a moved rock keep their rock config in `paper/<run>/obstacle_config.yaml`, pass it with `--config`. |
| `plot_ol_cl_comparison.py` | Open loop against closed loop (`--ol <bag> --cl <bag>`). S1 (default): plan view around the rock, distance to the rock and urgency, in `paper/s1_obstacle_comparison/`. `--scenario S2`: plan view of the inspection leg, the current and USBL loss as injected, distance off the leg and urgency, in `paper/s2_current_usbl_comparison/`. Distance off the leg in the tables is while the vehicle steered itself (closed loop: until the operator accepted); the distance under manual control is a separate column, since the BN enters no tracking error then. When both runs got the same disturbances (a replayed run), panel (b) draws them once. |
| `state_frequencies.py` | Observed state frequencies of the evidence nodes against their priors, from SN and FN (without its 13-min hold) and SC and FC up to 10 s before their first disturbance; nodes fixed by design in a run are reported but not pooled. Reads with rosbag2_py, or the rosbags package when ROS 2 is not sourced. Analysis only. |
| `plot_sim_maps.py` | `paper/sim_results/sim_mission_maps`: (a) SN, (b) SC with rocks A and B and the takeovers TO1–TO3, (c, d) close-ups of the rocks with the planned transit leg. |
| `plot_field_run.py` | Figures and tables for one field run, laid out like the simulation runs (it uses `plot_nominal_run.py`'s timeline and hidden-node figures): `fig1_mission_map` (track in the drone's own position estimate, from the reset at the dock), `fig2_timeline`, `fig3_hidden_nodes`, `phase_summary`, `run_summary.txt`, and for closed-loop runs `takeover_events.csv/.md` with the metrics of the simulation table. Reads `field_run.yaml` in the output folder. Also draws the simulator twin of a field run (a bag of `blueye_stonefish_bt`, output in `paper/<run>/`): the simulation map with the rocks of the run's `scenario_config.yaml`, the window from the first mission state to "Mission finished", and only the injected USBL changes on the line above the timeline (the rocks give the sonar readings). |
| `analyse_field_runs.py` | `field/field_results/` from all field runs and the simulator twin. Run `plot_field_run.py` on every field run and on the twin first. |
| `plot_pier_layout.py` | The pier layout from `mission_sites.py` on the satellite image (`field/mission_plan/`). |
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
| `fig1_mission_map` | Track coloured by mission phase, waypoints, docking station and pipeline footprints; the rocks (S1, SC), takeover request / hand-back markers (closed loop) and the water-current direction (S2). |
| `fig2_timeline` | (a) urgency against the threshold, (b) TakeoverRequest posterior, (c) operator fatigue and stress, (d) evidence entered in the BN by severity (grey: not entered by an evidence rule). Dotted lines and the "Disturbances:" line mark injected current changes and USBL cuts; dash-dot lines and the "Takeover:" line mark the operator events. |
| `fig3_hidden_nodes` | Posterior of the intermediate BN nodes over time (the explanation layer). |
| `phase_summary.csv` / `.md` | Per phase: start, duration, mean and max urgency, time above threshold, alarms. |
| `run_summary.txt` | Mission and docking result, alarms, peak urgency with the evidence at the peak, battery, distance. |

The analysis window runs from the first `/mission_state` to "Mission finished", so BN output from before the
behaviour tree starts the mission is left out (and counted in `run_summary.txt`).

## Making the plots for a field run

```bash
source ~/Desktop/auto-pilot/install/setup.bash
cd ~/Desktop/auto-pilot/src
python3 plots/plot_field_run.py rosbags/field/<run>     # write plots/field/<run>/field_run.yaml first if needed
python3 plots/analyse_field_runs.py                     # after all field runs and the twin
```

The simulator twin of the fault run (scenario SC), after copying `blueye_field_twin_config.yaml` to `plots/paper/<run>/scenario_config.yaml` and writing `field_run.yaml` (`takeovers: [TO1, TO2, TO3]`):

```bash
python3 plots/plot_field_run.py rosbags/paper/sf_field_twin_run01_closedloop_20261027
```

Field runs of 2026-10-25 at the pier: `pier_nominal_run01_shadow_20261025` (nominal, shadow mode) and
`pier_faults_run01_closedloop_20261025` (closed loop; TO1 sonar Critical in transit, TO2 USBL Weak with sonar Near and
Critical in transit, TO3 USBL Lost in inspection). The attempts that were not used are in
`src/rosbags/field/excluded_20261025/`, with the reason for each in its README.
