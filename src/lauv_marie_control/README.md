# lauv_marie_control

LAUV Marie (NTNU AUR-Lab) in Stonefish: a base model with a heading, depth and speed autopilot.

## Run

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

## Where things are

| What | Where |
|---|---|
| Vehicle (mass, drag, propeller, fins, sensors) and what is assumed | `vortex-stonefish-sim/stonefish_sim/drones/lauv_marie.scn` |
| World (open water, seabed at 30 m) | `stonefish_sim/scenarios/lauv_marie.scn` (`lauv_marie_no_gpu.scn` is the headless copy) |
| Start position | `stonefish_sim/config/lauv_marie_config.yaml` |
| Meshes, made from the AUR-Lab Blender model | `stonefish_sim/data/lauv_marie/`, by `vortex-stonefish-sim/scripts/make_lauv_marie_meshes.py` |
| Autopilot, keyboard control | `lauv_marie_control/autopilot.py`, `teleop_keyboard.py` |
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
| Elevator in level flight | about -7.5° | 0° |

The replays are open loop, so the simulated depth and heading drift away from the log within minutes; rates and
pitch are what is compared.
