# blueye_takeover_bn_real

Real-mission Bayesian network takeover inference for the Blueye ROV.

This package is the real-drone version of the simulation package `blueye_takeover_bn`.
It keeps the same Bayesian network model and the same takeover output topics, but it
feeds the BN from real ROS topics and from helper monitor nodes that convert raw
vehicle/sensor data into the discrete states expected by the BN.

The package also contains local copies of the helper scripts in:

```bash
blueye_takeover_bn_real/scripts
```

That means the package does not depend on standalone scripts left in another workspace.
After building this package, the helpers can be launched with `ros2 run`.

## Main Idea

The Bayesian network expects symbolic/discrete evidence such as:

```text
BatteryLevel = Low
DVLStatus = Nominal
USBLStrength = Strong
SonarRange = Clear
CameraQuality = Good
MissionPhase = Inspection
```

Most real sensors publish numeric or raw messages instead. The helper nodes convert
those real messages into the discrete BN states. The BN node then sets evidence in
the PySMILE network, updates the belief state, and publishes the takeover decision.

## Build

From the auto-pilot workspace:

```bash
cd /home/badawi/Desktop/auto-pilot
colcon build --packages-select blueye_takeover_bn_real
source install/setup.bash
```

## Run the BN

```bash
ros2 launch blueye_takeover_bn_real takeover_bn_real.launch.py
```

or:

```bash
ros2 run blueye_takeover_bn_real takeover_bn_real
```

The BN can run even if not all evidence topics are present, but the result is more
meaningful when the helper monitors are publishing.

## BN Output Topics

The real package keeps the same takeover output interface as the simulation package.

```text
/blueye/takeover_request/state
/blueye/takeover_request/no_takeover_prob
/blueye/takeover_request/attention_required_prob
/blueye/takeover_request/takeover_requested_prob
/blueye/takeover_request/max_probability
/blueye/takeover_request/threshold
/blueye/takeover_request/evidence
```

Useful checks:

```bash
ros2 topic echo /blueye/takeover_request/state
ros2 topic echo /blueye/takeover_request/evidence
```

The `state` topic publishes the most likely BN outcome. The `evidence` topic publishes
a JSON summary containing the latest state, probabilities, and active evidence.

## Takeover States

The target BN node is `TakeoverRequest`. The expected output states are:

```text
NoTakeoverRequired
IncreasedAttentionRequired
TakeoverRequested
```

The launch parameter `takeover_threshold` is published on:

```text
/blueye/takeover_request/threshold
```

It is currently a reporting threshold. The published `state` is the BN state with the
highest probability.

## Mission Phase

The real BN subscribes to both:

```text
/mission_phase
/mission_state
```

Both are expected as:

```text
std_msgs/msg/Int32
```

The mapping is:

```text
1 = Undocking
2 = Transit
3 = Inspection
4 = Docking
```

This was chosen to match the simplified real mission phase mapping:

```text
undocking -> transit -> inspection -> transit -> docking
```

The BN default is:

```text
MissionPhase = Transit
```

If no mission phase topic is available, the BN will continue using `Transit`.

## Explicit BN Defaults

The real BN only sets these default evidence values:

```text
MissionPhase = Transit
Fatigue = Low
Stress = Low
```

Everything else comes from topics, from helper nodes, or is left unknown.

Human state topics are optional:

```text
/blueye/human/fatigue  std_msgs/msg/Float32
/blueye/human/stress   std_msgs/msg/Float32
```

The BN maps both values like this:

```text
value < 0.3  -> Low
value < 0.6  -> Medium
otherwise    -> High
```

If those topics are not published, the BN assumes `Low` fatigue and `Low` stress.

## BN Input Evidence

The BN node uses these evidence topics:

| BN node | Source topic | Type | How value is obtained |
| --- | --- | --- | --- |
| `MissionPhase` | `/mission_phase` or `/mission_state` | `std_msgs/Int32` | Integer mission phase mapped to `Undocking`, `Transit`, `Inspection`, `Docking` |
| `Current` | `/blueye/current` | `std_msgs/Float32` | Numeric water current discretized inside the BN |
| `BatteryLevel` | `/blueye/battery_state` | `std_msgs/String` | Published by `real_mission_state_monitors` |
| `USBLStrength` | `/blueye/usbl_strength` | `std_msgs/String` | Published by `usbl_strength_monitor` |
| `DVLStatus` | `/blueye/dvl_status` | `std_msgs/String` | Published by `real_mission_state_monitors` |
| `Speed` | `/blueye/speed_state` | `std_msgs/String` | Published by `real_mission_state_monitors` |
| `Altitude` | `/blueye/altitude_state` | `std_msgs/String` | Published by `real_mission_state_monitors` |
| `CameraQuality` | `/blueye/camera_quality` | `std_msgs/String` | Published by `camera_quality_monitor` |
| `SonarRange` | `/blueye/sonar_range` | `std_msgs/String` | Published by `sonar_range_monitor` |
| `ArUcoVisibility` | `/blueye/aruco_visibility` | `std_msgs/String` | Published by `aruco_visibility_monitor` |
| `WaypointTrackingError` | `/blueye/waypoint_tracking_error` | `std_msgs/String` | Published by `real_mission_state_monitors` |
| `DataQuality` | `/blueye/inspection_data_quality` | `std_msgs/String` | Published by `inspection_data_quality_monitor` |
| `RangeToTarget` | `/blueye/pose` + `/blueye/current_waypoint` | `PoseStamped` + `Pose` | Computed inside the BN from pose-to-waypoint distance |
| `Fatigue` | `/blueye/human/fatigue` | `std_msgs/Float32` | Optional human input, default `Low` |
| `Stress` | `/blueye/human/stress` | `std_msgs/Float32` | Optional human input, default `Low` |

## Current

Helper:

```bash
ros2 run blueye_takeover_bn_real real_current_publisher
```

Publishes:

```text
/blueye/current  std_msgs/msg/Float32
```

The helper supports two modes:

```text
manual
stormglass
```

Default mode is `manual`, with:

```text
manual_current_mps = 0.2
noise_mps = 0.005
```

The BN discretizes `/blueye/current` internally:

```text
current < 0.3 m/s -> Low
current < 0.7 m/s -> Medium
otherwise         -> High
```

Example manual run:

```bash
ros2 run blueye_takeover_bn_real real_current_publisher --ros-args \
  -p mode:=manual \
  -p manual_current_mps:=0.25
```

## USBL Strength

Helper:

```bash
ros2 run blueye_takeover_bn_real usbl_strength_monitor
```

Subscribes:

```text
/blueye/evologics/raw_data  std_msgs/msg/String
```

Publishes:

```text
/blueye/usbl_strength       std_msgs/msg/String
```

The helper parses Evologics `RECVIM` raw packets and extracts:

```text
RSSI
Integrity
```

Example raw packet:

```text
+++AT:47:RECVIM,7,6,4,ack,221308,-51,140,-0.0098,E
```

In that packet:

```text
RSSI = -51 dB
Integrity = 140
```

Discrete states:

```text
Strong
Moderate
Weak
Lost
```

Default thresholds:

```text
Strong:   RSSI >= -55 dB and integrity >= 100
Moderate: RSSI >= -65 dB and integrity >= 70
Weak:     RSSI >= -75 dB and integrity >= 40
Lost:     anything worse, or packet cannot be parsed
```

RSSI means received signal strength indicator. For these negative dB values, closer
to zero is stronger. For example, `-51 dB` is stronger than `-75 dB`.

## Sonar Range

Helper:

```bash
ros2 run blueye_takeover_bn_real sonar_range_monitor
```

Subscribes:

```text
/blueye/sonar_3d/pointcloud_intensity  sensor_msgs/msg/PointCloud2
```

Publishes:

```text
/blueye/sonar_range          std_msgs/msg/String
/blueye/sonar_nearest_range  std_msgs/msg/Float32
```

The helper filters sonar points in a forward safety box:

```text
min_x_m = 0.05
max_x_m = 5.0
max_abs_y_m = 1.0
max_abs_z_m = 0.8
min_valid_points = 5
```

Then it finds the nearest valid obstacle point.

Discrete states:

```text
Clear
Near
Critical
```

Default thresholds:

```text
nearest < 0.7 m       -> Critical
0.7 m to < 2.0 m      -> Near
nearest >= 2.0 m      -> Clear
too few valid points  -> Clear
```

## Real Mission State Monitors

Helper:

```bash
ros2 run blueye_takeover_bn_real real_mission_state_monitors
```

This helper converts several real vehicle topics into BN-friendly discrete topics.

### Depth

Subscribes:

```text
/blueye/depth  std_msgs/msg/Float32
```

Publishes:

```text
/blueye/depth_state  std_msgs/msg/String
```

States:

```text
depth < 0.5 m -> Shallow
depth > 5.0 m -> Deep
otherwise     -> Nominal
```

Depth is currently published for monitoring, but the takeover BN model does not use
`Depth` as evidence.

### Altitude

Subscribes:

```text
/blueye/altitude  std_msgs/msg/Float32
```

Publishes:

```text
/blueye/altitude_state  std_msgs/msg/String
```

States:

```text
altitude > 2.0 m -> Safe
altitude > 1.0 m -> Marginal
otherwise        -> Unsafe
```

### Battery

Subscribes:

```text
/blueye/battery  geometry_msgs/msg/Pose
```

Publishes:

```text
/blueye/battery_level             std_msgs/msg/Float32
/blueye/battery_state             std_msgs/msg/String
/blueye/battery_charge_state      std_msgs/msg/String
/blueye/battery_runtime_to_empty  std_msgs/msg/Float32
/blueye/battery_current           std_msgs/msg/Float32
```

The helper interprets the battery pose message as:

```text
position.x    charging current
position.y    state of charge, if usable
position.z    battery current
orientation.x runtime to empty, seconds
```

If `position.y` is usable, it is used as battery state of charge. If not, the helper
falls back to `orientation.x` runtime to empty.

Battery states from percentage:

```text
SOC > 75% -> High
SOC > 50% -> Medium
SOC > 25% -> Low
otherwise -> Critical
```

Runtime fallback:

```text
runtime > 1800 s -> High
runtime > 900 s  -> Medium
runtime > 300 s  -> Low
otherwise        -> Critical
```

Charging state:

```text
charging_current > 0.1 A -> Charging
otherwise                -> Discharging
```

The BN uses:

```text
/blueye/battery_state
```

### DVL Status

Subscribes:

```text
/blueye/sensor/dvl  marine_acoustic_msgs/msg/Dvl
```

Publishes:

```text
/blueye/dvl_status  std_msgs/msg/String
```

States:

```text
Failed
Degraded
Nominal
```

Logic:

```text
velocity invalid         -> Failed
num_good_beams < 3       -> Degraded
otherwise                -> Nominal
no DVL message for 2.0 s -> Failed
```

### Speed

Subscribes:

```text
/blueye/speed  std_msgs/msg/Float32
```

Publishes:

```text
/blueye/speed_state  std_msgs/msg/String
```

States:

```text
abs(speed) < 0.3 m/s -> Safe
abs(speed) < 0.7 m/s -> Moderate
otherwise            -> High
```

### Waypoint Tracking Error

Subscribes:

```text
/blueye/pose                    geometry_msgs/msg/PoseStamped
/blueye/current_waypoint        geometry_msgs/msg/Pose
/blueye/guidance/desired_state  nav_msgs/msg/Odometry
```

Publishes:

```text
/blueye/waypoint_tracking_error    std_msgs/msg/String
/blueye/waypoint_tracking_error_m  std_msgs/msg/Float32
```

The helper computes the distance between the current pose and the latest waypoint or
desired state pose.

States:

```text
error < 0.5 m -> Low
error > 1.5 m -> High
otherwise     -> Medium
```

## Camera Quality

Helper:

```bash
ros2 run blueye_takeover_bn_real camera_quality_monitor
```

Publishes:

```text
/blueye/camera_quality    std_msgs/msg/String
/blueye/camera_latency    std_msgs/msg/Float32
/blueye/camera_frame_gap  std_msgs/msg/Float32
```

It can read either an RTSP stream or a ROS image topic.

Default RTSP mode:

```text
source_mode = rtsp
rtsp_url = rtsp://192.168.1.101:8554/test
```

ROS image mode:

```bash
ros2 run blueye_takeover_bn_real camera_quality_monitor --ros-args \
  -p source_mode:=ros \
  -p camera_topic:=/blueye/image
```

The helper measures:

```text
sharpness   Laplacian variance of grayscale image
brightness  mean grayscale intensity
latency     ROS image header stamp delay, if using ROS image mode
frame_gap   time since previous received frame
```

States:

```text
Excellent
Good
Poor
Failed
```

Default image thresholds:

```text
Excellent: sharpness > 200 and brightness between 40 and 100
Good:      sharpness > 50  and brightness between 20 and 120
Poor:      sharpness > 10
Failed:    worse than Poor, no stream, bad conversion, or severe timing issue
```

Timing downgrade:

```text
latency >= 1.0 s or frame_gap >= 0.7 s -> Failed
latency >= 0.25 s or frame_gap >= 0.2 s -> downgrade Excellent/Good to Poor
```

## ArUco Visibility

Helper:

```bash
ros2 run blueye_takeover_bn_real aruco_visibility_monitor
```

Subscribes:

```text
/blueye/aruco/status  blueye_interfaces/msg/ArucoDetectionStatus
```

Publishes:

```text
/blueye/aruco_visibility          std_msgs/msg/String
/blueye/aruco_marker_count        std_msgs/msg/Int32
/blueye/docking_station_detected  std_msgs/msg/Bool
```

States:

```text
All
Some
None
```

Default thresholds:

```text
num_tags >= 6 -> All
num_tags >= 1 -> Some
otherwise     -> None
```

If no ArUco status message arrives for `1.5 s`, the helper publishes:

```text
ArUcoVisibility = None
```

## Inspection Data Quality

Helper:

```bash
ros2 run blueye_takeover_bn_real inspection_data_quality_monitor
```

Publishes:

```text
/blueye/inspection_data_quality         std_msgs/msg/String
/blueye/inspection_data_quality_score   std_msgs/msg/Float32
/blueye/inspection_data_quality_reason  std_msgs/msg/String
```

The discrete states are:

```text
Adequate
Marginal
Inadequate
```

It combines:

```text
/blueye/camera_quality
/blueye/sonar_range
/blueye/dvl_status
/blueye/usbl_strength
/blueye/position_valid
/mission/phase
```

Each input must be fresh. Default freshness timeout:

```text
fresh_timeout_s = 3.0
```

The score is built from freshness plus data quality:

```text
camera freshness   0.20
sonar freshness    0.20
dvl freshness      0.15
usbl freshness     0.15
position freshness 0.15
mission phase      0.05 optional
camera quality     up to 0.15
navigation quality up to 0.15
```

Final state:

```text
score >= 0.75 -> Adequate
score >= 0.45 -> Marginal
otherwise     -> Inadequate
```

By default, `require_inspection_phase` is `False`, so this monitor can publish quality
throughout the mission. If set to `True`, it publishes `Inadequate` outside inspection.

Note: this helper's default mission phase input is `/mission/phase` as a string. If
your mission phase is only available as `/mission_phase` or `/mission_state` Int32,
either leave `require_inspection_phase:=False` or provide/remap a string mission phase
topic for this helper.

## Range To Target

The BN computes `RangeToTarget` internally. It subscribes to:

```text
/blueye/pose              geometry_msgs/msg/PoseStamped
/blueye/current_waypoint  geometry_msgs/msg/Pose
```

It computes Euclidean distance between current pose and waypoint.

States:

```text
distance > 5.0 m -> Far
distance < 1.0 m -> Close
otherwise        -> Mid
```

## Stale Data and Timeout Behavior

"Stale" means no fresh message has arrived on a topic within the configured timeout.

The BN has timeout behavior for selected evidence:

```text
DVL stale for more than 2.0 s          -> DVLStatus = Failed
ArUco stale for more than 1.5 s        -> ArUcoVisibility = None
target data stale for more than 2.0 s  -> clear RangeToTarget and WaypointTrackingError
DataQuality stale for more than 3.0 s  -> clear DataQuality
```

"Clear evidence" means the BN removes that observation and treats it as unknown.
It does not force a good or bad state.

The DVL and ArUco timeouts are different: they force explicit bad states because stale
DVL/ArUco data usually means those sensing functions are unavailable.

## Installed Executables

After building the package, these commands are available:

```bash
ros2 run blueye_takeover_bn_real takeover_bn_real
ros2 run blueye_takeover_bn_real usbl_strength_monitor
ros2 run blueye_takeover_bn_real sonar_range_monitor
ros2 run blueye_takeover_bn_real real_current_publisher
ros2 run blueye_takeover_bn_real camera_quality_monitor
ros2 run blueye_takeover_bn_real real_mission_state_monitors
ros2 run blueye_takeover_bn_real inspection_data_quality_monitor
ros2 run blueye_takeover_bn_real aruco_visibility_monitor
```

## Suggested Startup Order

Start the real sensor/parsing nodes first:

```bash
ros2 run evologic_usbl evologics_usbl
ros2 launch sonar_3d sonar3d_drive.launch.py
ros2 run waterlinked_dvl waterlinked_dvl
```

Then start helper monitors:

```bash
ros2 run blueye_takeover_bn_real real_mission_state_monitors
ros2 run blueye_takeover_bn_real usbl_strength_monitor
ros2 run blueye_takeover_bn_real sonar_range_monitor
ros2 run blueye_takeover_bn_real real_current_publisher
ros2 run blueye_takeover_bn_real camera_quality_monitor
ros2 run blueye_takeover_bn_real inspection_data_quality_monitor
ros2 run blueye_takeover_bn_real aruco_visibility_monitor
```

Then start the BN:

```bash
ros2 launch blueye_takeover_bn_real takeover_bn_real.launch.py
```

## Useful Debug Commands

Check the raw evidence topics:

```bash
ros2 topic echo /blueye/battery_state
ros2 topic echo /blueye/dvl_status
ros2 topic echo /blueye/usbl_strength
ros2 topic echo /blueye/sonar_range
ros2 topic echo /blueye/camera_quality
ros2 topic echo /blueye/inspection_data_quality
ros2 topic echo /blueye/aruco_visibility
ros2 topic echo /blueye/waypoint_tracking_error
ros2 topic echo /blueye/current
```

Check the BN result:

```bash
ros2 topic echo /blueye/takeover_request/state
ros2 topic echo /blueye/takeover_request/evidence
```

Check topic rates:

```bash
ros2 topic hz /blueye/dvl_status
ros2 topic hz /blueye/sonar_range
ros2 topic hz /blueye/camera_quality
ros2 topic hz /blueye/takeover_request/state
```

## What Changed From the Simulation BN

The simulation package produced BN evidence from simulated/autopilot topics. This
real package keeps the same BN model and same output topics, but changes how evidence
is provided:

```text
MissionPhase uses real mission phase integer topics.
Current is read from /blueye/current and discretized inside the BN.
Battery, altitude, speed, DVL, and waypoint tracking are produced by real_mission_state_monitors.
USBL strength is parsed from real Evologics raw RECVIM packets.
Sonar range is computed from the real 3D sonar point cloud with intensity.
Camera quality is computed from RTSP or ROS image frames.
DataQuality uses the standalone inspection data quality monitor.
ArUcoVisibility uses real ArUco detection status.
RangeToTarget is computed internally from real pose and waypoint.
Fatigue and Stress remain optional human inputs, defaulting to Low.
```

## Notes and Known Assumptions

The BN uses a PySMILE `.xdsl` model copied from the simulation package:

```text
config/full_mission.xdsl
```

The package requires PySMILE to be available in the Python environment.

Some helper thresholds are conservative starting points. They should be tuned from
real mission data once enough bags are available.

The helper nodes are independent. If one helper is not running, the BN can still run,
but the relevant evidence will be unknown or timeout-controlled.

The loose scripts in `/home/badawi/blueye_ws/src` are not required by this package
after the package has been built. Copies are installed from this package itself.
