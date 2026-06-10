# blueyeROV_BT (simulation)
A behaviourTree package for the BlueyeROV mission planning for a short video on how to simulation works, here is a youtube video link [Video Link] (https://www.youtube.com/watch?v=vxIXswrNpQY)

![Behavior Tree](./images/bt.png)
*The behaviourTree visualized with Groot2*

## Dependencies

- [BehaviourTree.cpp (v4)](https://github.com/BehaviorTree/BehaviorTree.CPP)

- [Groot2 Visualizer App:](https://www.behaviortree.dev/groot/) If you want to monitor the behaviorTree in real time (Note: since the behaviourTree consist morethan 20 ndoes you need to subscribe to the premium package after the 1 month free trains ends)

This is a separate package that works together with the AURlab Gazebo simulator, to run the simulation you need to clone the simulation package in [mundus_mir_simulator](https://gitlab.com/aurlab/mundus-mir-project/mundus_mir_simulator.git), make sure to switch the branch to "mission_panner" to use the current world the planner uses 

### BehaviorTree.CPP
for more about the documentation of the library 
[BehaviourTree.cpp](https://www.behaviortree.dev/) 

Behaviour tree is implemented with BehaviorTree.CPP is implemented with ROS2 and subscribes to the topics it needs access to and publishes mission states (e.g. waypoint)

## How To Run Mission Planner

### 1st Terminal Simulator

Create a workspace and build both the mission planner and the simulator package

- Run the simulator ros2 "launch mundus_mir_simulator_launch mundus_mir_pipeline_world.launch.py"

### 2nd Terminal Sonar
(you need both the battery and sonar packages to run the mission planner), both packages are included in the source folder, currently a standard sonar based 2d horizontal obstacle avoildance is used for static obstacles (obstacle avoidance is implemented with conventional way) -> would be interesting to get Sam's code in here instead. 

- ros2 launch blueye_bt blueye_bt.launch.py

### 3th Terminal Visualization

- ros2 run blueye_visualization blueye_visualization 

## How To Define Waypoints
Missions are based on XML files in the behaviorTree folder in the bt workspace, you can create your constume missions, but if you need any additional behaviour apart from the behaviours, actions or conditions from the current available once, you need to create a cpp script and register the node in the main file.

The controller is based on the waypoint control Ambjorn implemented in the mundus simulator package, the waypoints uses service call

The overall Mission is defined in Missioncontrol.xml file, but the MissionControl xml file contains subtrees which are included in the behaviorTree folder.

## In Real World
The Blueye Missionplanner SDK is used to create a mission, the BehaviorTree.CPP is used to determine the sequences and calls the according files. rightnow I am working to make this work, and will provide an updated readme.

PyTrees would be other opensource option 
USBL can be used to refer to GPS location in Missionplanner SDK

# blueyeROV_BT (real)
![Behavior Tree](./images/bt_real.png)

## How the Mission Planner Works

The real Blueye mission planner has two layers: a **C++ Behavior Tree** as the high-level coordinator, and a **Python mission script** as the low-level executor that talks directly to the drone via the Blueye SDK.

---

### 1. Entry Point — C++ Behavior Tree (`src/main.cpp`)

When launched, the ROS2 node `blueye_bt_real` loads an XML behavior tree file (path passed as a ROS parameter), registers all BT node types, then ticks the tree in a tight loop (1ms interval) until it completes or is interrupted. A Groot2 live monitor is published on port `6677` so you can watch the tree execute graphically in real time.

---

### 2. The Behavior Tree (`behavior_trees/Mission.xml`)

```
Fallback
├── Sequence (normal mission with battery check)
│   └── ReactiveSequence (continuous battery monitoring)
│       ├── Inverter → BatteryLevelCondition (threshold: 20%)
│       └── Sequence (mission steps)
│           ├── LaunchUndockingProcedure   ← reverse out of dock
│           ├── ParallelAll
│           │   ├── ActivateAutoModes      ← depth + heading hold
│           │   └── LaunchMissionProcedure ← runs mission.py
│           └── LaunchDockingProcedure     ← blocks until charging_current > 0
└── Sequence (emergency fallback)
    ├── GoToWaypoint (hardcoded emergency GPS coordinates)
    └── LaunchDockingProcedure
```

Key logic:
- **Battery check is reactive**: if battery drops below 20% at any point during the mission sequence, the `ReactiveSequence` aborts immediately and falls through to the emergency sequence.
- **Emergency fallback**: drives the ROV to a hardcoded GPS position and docks it safely.
- **Parallel stabilization**: `ActivateAutoModes` (depth hold + heading hold) runs in parallel with the mission script after undocking, giving the drone 5 seconds to stabilize before the mission begins.

---

### 3. LaunchMissionProcedure (`src/behaviors/launch_mission_procedure.cpp`)

This BT node calls `system()` to shell out to the bash script `launch_mission.sh`, passing the drone IP and max retries. It blocks until the script exits and returns `SUCCESS` or `FAILURE` based on the exit code.

---

### 4. The Python Mission Script (`scripts/mission_planner_scripts/mission.py`)

This is where the actual drone commands happen, using the **Blueye SDK** (`blueye.sdk`):

**Step 1 — Connect**: connects to the drone at `192.168.1.101`, takes control, and attaches a `reset_position()` method to the drone's control client.

**Step 2 — Reset position**: sends a `ResetPositionCtrl` message to anchor the drone's internal dead-reckoning to a known GPS position (hardcoded: `63.4414548, 10.3482882`).

**Step 3 — Build mission**: constructs a `bp.Mission` object with these instructions in order:

| ID | Type | Detail |
|----|------|--------|
| 1 | Control Mode | Auto-depth + Auto-heading |
| 2 | Depth Set Point | Go to 1.0m depth at 0.5 m/s |
| 3 | Waypoint | Pipeline Point 1 @ 0.2 m/s |
| 4 | Wait | 10 seconds |
| 5 | Waypoint | Pipeline Point 2 @ 0.2 m/s |
| 6 | Wait | 10 seconds |
| 7 | Waypoint | Pipeline Point 3 @ 0.2 m/s |
| 8 | Wait | 10 seconds |
| 9 | Waypoint | Docking Station @ 0.5 m/s |

All waypoints use a **0.5m circle of acceptance** (how close the ROV must get before a waypoint counts as reached).

**Step 4 — Execute with retry**: sends the mission with `drone.mission.send_new()` then starts it with `drone.mission.run()`. Status is polled every 2 seconds. If the mission is `ABORTED`, it instantly calls `drone.mission.run()` again (resuming from where it left off) up to `max_retries` times. On `COMPLETED`, the script exits successfully.

---

### Summary Flow

```
BT ticks
  → LaunchUndockingProcedure
       undocking.py: connect → take_control → reverse out → DISCONNECT
  → (parallel) ActivateAutoModes + LaunchMissionProcedure
       mission.py: connect → take_control → reset GPS → build mission → run mission
       drone navigates: waypoint 1 → wait → waypoint 2 → wait → waypoint 3 → wait → docking GPS point
       mission.py: DISCONNECT
  → LaunchDockingProcedure (BLOCKING)
       launch_docking_real.sh: launch ArUco visual docking controller
       monitor /blueye/battery until charging_current > 0
       kill docking controller → exit SUCCESS
  → BT complete
```

Each Python script (undocking, mission) follows the same pattern: connect → take_control → do work → disconnect. They never overlap. `blueye_telemetry.py` stays connected throughout for telemetry and services but does not hold control.

---

- Connect to the Blueye WiFi

## How To Run

### Terminal 1 — Behavior Tree (required)
```bash
cd ~/Desktop/blueyeROV_BT/
source install/setup.bash
ros2 launch blueye_bt_real blueye_bt_real.launch.py
```
This starts two things: `blueye_telemetry.py` immediately (drone connection, battery, GPS, depth/heading hold services), then the behavior tree node after a 5-second delay.

### Terminal 2 — Trajectory Visualization (optional)
```bash
ros2 launch blueye_visualization_real blueye_visualization_real.launch.py
```

### Terminal 3 — Battery Monitor (optional)
```bash
ros2 topic echo /blueye/battery
```

### To Test Emergency Behavior (Low Battery Simulation)
```bash
ros2 topic pub --once /blueye/battery geometry_msgs/msg/Pose "{position: {x: 10.4, y: 0.0, z: -0.95}, orientation: {x: 3000.0, y: 0.0, z: 0.0, w: 1.0}}"
```

# Battery Simulation for Blueye ROV Emergency Behavior Testing

The battery condition node subscribes to the `/blueye/battery` topic which uses a `geometry_msgs/msg/Pose` message type as a container for battery information. While this may seem like an unusual message type for battery data, it's used as a convenient container with multiple fields.

### Message Field Mappings

| Pose Field | Battery Information | Description |
|------------|---------------------|-------------|
| `orientation.x` | Runtime to empty | Time remaining in seconds until battery is depleted |
| `position.x` | Charging current | Current flowing into the battery while charging (Amps) |
| `position.z` | Current draw | Current flowing from the battery while discharging (Amps) |

## Battery Level Calculation

The battery percentage is calculated using the following formula:

```cpp
battery_percentage = (runtime_to_empty / full_runtime) * 100.0


## Testing Low Battery Condition

To simulate a low battery condition and trigger the emergency behavior in the behavior tree, you can publish a message with a runtime value that results in a battery percentage below the threshold (default 20%).

### Example Command

```bash
# Simulate approximately 15% battery remaining
ros2 topic pub --once /blueye/battery geometry_msgs/msg/Pose "{position: {x: 10.4, y: 0.0, z: -0.95}, orientation: {x: 3000.0, y: 0.0, z: 0.0, w: 1.0}}"

## Expected Behavior

When the battery level falls below the threshold:

1. The `BatteryLevelCondition` node returns `SUCCESS`.
2. The `ReactiveSequence` in the behavior tree detects this condition.
3. The emergency sequence is triggered, causing the ROV to:
   - Activate auto depth and heading hold.
   - Navigate to the emergency waypoint which are the docking station
   - Dock at the docking station.
