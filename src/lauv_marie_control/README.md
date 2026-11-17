# lauv_marie_control

LAUV Marie (NTNU AUR-Lab) in Stonefish, flown either by DUNE, the LAUV's own onboard software, or by a simple
heading, depth and speed autopilot of this package.

## Run with DUNE (plans from Neptus or the command line)

DUNE runs as on the vehicle, with its own vehicle simulation switched off: Stonefish provides Marie's motion, and
DUNE's navigation, controllers, manoeuvres and safety fly her. A bridge node passes the vehicle state to DUNE and
DUNE's fin and propeller commands to Stonefish.

```bash
source ~/Desktop/auto-pilot/install/setup.bash
ros2 launch lauv_marie_control lauv_marie_dune.launch.py              # 3D window; rendering:=false for headless
```

Marie starts at the surface. In Neptus she is `lauv-simulator-1`, and plans and manoeuvres are sent as to any LAUV.
From a second terminal, without Neptus:

```bash
source ~/Desktop/auto-pilot/install/setup.bash
ros2 run lauv_marie_control dune_plan goto 100 50 --depth 5 --popup    # metres north, east of the start
ros2 run lauv_marie_control dune_plan yoyo 150 0 --depth 10 --amplitude 5 --pitch 15
ros2 run lauv_marie_control dune_plan loiter 0 0 --depth 4 --radius 20 --duration 60
ros2 run lauv_marie_control dune_plan station 30 20 --radius 8 --duration 40     # at the surface
ros2 run lauv_marie_control dune_plan popup 0 0 --duration 30
ros2 run lauv_marie_control dune_plan stop
```

Setup, once: DUNE is the copy in `~/dune-simulator/dune-marie` (of `breach_software_stack/dune-ntnu-os-2025`, DUNE
2025.04); another folder can be given with `dune_dir:=`. It is OceanScan's software: keep it out of git.

- Stop your own `./dune -c lauv-simulator-1` first: both are named `lauv-simulator-1`.
- Do not start the autopilot of this package at the same time: both would command the fins.
- `dune/lauv-marie-stonefish.ini` is DUNE's configuration (copied to `~/dune-simulator/dune-marie/etc` at every
  launch): the simulator's settings with Marie's depth and roll gains from her logs, its own physics off, the
  water depth of its simulated sea set to the 100 m of the Stonefish scenario, and a second UDP link for the
  bridge (ports 6970 and 6971; Neptus keeps 6002). DUNE's logs go to `~/dune-simulator/dune-marie/log`.
- DUNE's sensors (DVL, AHRS, depth, GPS at the surface) are its own simulated ones, computed from Stonefish's motion.
- `/lauv_marie/dune/estimated_state` (DUNE's navigation) and `/lauv_marie/dune/plan_state` are published for
  other nodes.

Checked headless (17 November 2026): Goto at 5 m (depth within 0.1 m, bearing within 0.1 deg), PopUp, YoYo between
5 and 15 m at 15 deg pitch, Loiter of 20 m radius, station keeping at the surface, stopping a plan. DUNE's position
estimate stays within 0.2 m of Stonefish's. Not yet checked with Neptus.

## Run with the autopilot of this package

```bash
source ~/Desktop/auto-pilot/install/setup.bash
ros2 launch lauv_marie_control lauv_marie.launch.py                     # 3D window; rendering:=false for headless
```

In the 3D window, choose `LauvMarie` under *Trackball center* to make the camera follow the vehicle.

Second terminal, keyboard control (`w/s` speed, `a/d` heading, `r/f` depth, `space` stop, `q` quit):

```bash
source ~/Desktop/auto-pilot/install/setup.bash
ros2 run lauv_marie_control teleop_keyboard
```

Or set the autopilot by topic:

```bash
ros2 topic pub --once /lauv_marie/cmd/rpm std_msgs/msg/Float64 "{data: 1100.0}"       # about 1.2 m/s
ros2 topic pub --once /lauv_marie/cmd/heading std_msgs/msg/Float64 "{data: 90.0}"     # degrees, 90 = east
ros2 topic pub --once /lauv_marie/cmd/depth std_msgs/msg/Float64 "{data: 10.0}"       # m
ros2 topic pub --once /lauv_marie/cmd/goto geometry_msgs/msg/Point "{x: 40.0, y: -30.0, z: 8.0}"   # north, east, depth
```

Marie has one propeller and four fins: she cannot hover or move sideways, and the fins work only with forward speed.
She is slightly buoyant (0.4 kg), like the LAUV: with the propeller stopped she rises to the surface.

## Where things are

| What | Where |
|---|---|
| Vehicle (mass, drag, propeller, fins, sensors) and what is assumed | `vortex-stonefish-sim/stonefish_sim/drones/lauv_marie.scn` |
| World (open water, flat seabed) | `stonefish_sim/scenarios/lauv_marie.scn` (`lauv_marie_no_gpu.scn` is the headless copy) |
| Start position, seabed depth | `stonefish_sim/config/lauv_marie_config.yaml` (30 m); with DUNE `dune/stonefish_start.yaml` (surface, 100 m) |
| Meshes, made from the AUR-Lab Blender model | `stonefish_sim/data/lauv_marie/`, by `vortex-stonefish-sim/scripts/make_lauv_marie_meshes.py` |
| Autopilot, keyboard control | `lauv_marie_control/autopilot.py`, `teleop_keyboard.py` |
| DUNE bridge, plan tool, IMC messages | `lauv_marie_control/dune_bridge.py`, `dune_plan.py`, `imc.py`; `dune/IMC.xml` (IMC 5.5.4.9) |
| DUNE configuration, Stonefish start for DUNE | `dune/lauv-marie-stonefish.ini`, `dune/stonefish_start.yaml` |
| Marie's manoeuvres from her logs, 10 Hz (fin angles, propeller speed, attitude, rates, depth, speed) | `field_data/` |
| Tools for tuning against them | `tools/` |

## Topics

| Topic | Type | Content |
|---|---|---|
| `/lauv_marie/thrusters` | `Float64MultiArray` | `[rpm]`, direct propeller command |
| `/lauv_marie/rudders` | `Float64MultiArray` | `[top, bottom, port, starboard, 0, 0]` fin angles in rad (±0.436, 25°) |
| `/lauv_marie/odom` | `Odometry` | ground truth, NED |
| `/lauv_marie/imu`, `/dvl`, `/pressure`, `/gps` | | simulated sensors (GPS only at the surface) |
| `/lauv_marie/cmd/heading`, `/depth`, `/rpm`, `/goto` | `Float64`, `Point` | autopilot setpoints |

Do not publish to `/lauv_marie/thrusters` or `/lauv_marie/rudders` while the autopilot runs: it publishes them at 20 Hz.

## Tuning against Marie's logs

`tools/extract_field_manoeuvres.py` reads raw DUNE logs (`Data.lsf.gz`, `IMC.xml.gz`) with the reader of
`auv-sbi-navigation/analysis` and writes one 10 Hz table per log to `field_data/`. Four logs of 7-8 September 2026
are there (106 min): the star manoeuvre at the surface, the glacier dive, the sidescan survey and a rows survey.

`tools/sim_manoeuvre.py` runs a headless simulator on its own ROS domain, feeds it steps or the measured fin angles
and propeller speed of a log window, and writes the response to CSV. `--set` tries vehicle values without editing
`lauv_marie.scn`, so several runs can go in parallel:

```bash
cd ~/Desktop/auto-pilot/src/lauv_marie_control
python3 tools/sim_manoeuvre.py --out /tmp/turn.csv 25,1150,0,0 15,1150,0.342,0
python3 tools/sim_manoeuvre.py --out /tmp/dive.csv --warmup 20 --elevator-trim -7.5 --window 1140 1300 \
    --replay field_data/120215_07092026_Billefjord_Rows_SSS_MBES_glacier1.csv.gz
```

What the logs gave:

| | Marie | Model |
|---|---|---|
| Servos | 0 and 3 rudders (positive: to port), 1 and 2 elevators (positive: nose down), differences within a pair: roll | same four fins; the simulator's rudder sign is opposite |
| Fin travel | 25° | 25° |
| Speed | 1.13e-3 m/s per rpm under water | 1.08e-3 |
| Yaw rate in replays under water | | correlation 0.97-0.98, amplitude 0.93-1.05 of Marie's |
| Pitch in dives and climbs, replays | | correlation 0.88-0.97, 3.5-6° rms error |
| Speed lost at 20° of rudder | about 12 % | 24 % |
| Elevator in level flight | about -7.5° | 0° when neutral; not measured since the 0.4 kg buoyancy was added |

The replays are open loop, so the simulated depth and heading drift away from the log within minutes; rates and
pitch are what is compared.
