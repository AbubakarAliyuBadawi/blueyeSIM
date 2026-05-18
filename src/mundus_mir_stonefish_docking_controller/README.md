# Mundus MIR Stonefish Docking Controller

This package runs Blueye docking in Stonefish using the ArUco docking rig, `robot_localization`, the existing 4-DOF velocity controller, and the existing Stonefish Blueye thrust allocator.

## What It Starts

The launch file starts this chain:

```text
/blueye/cam/image_color
  -> stonefish_aruco_pose
  -> /blueye/pose_estimated_board_stamped

/blueye/imu/data_raw + /blueye/dvl/sim
  -> stonefish_sensor_republisher
  -> /blueye/imu_enu + /blueye/dvl_enu

/blueye/pose_estimated_board_stamped + /blueye/imu_enu + /blueye/dvl_enu
  -> robot_localization ekf_node
  -> /odometry/filtered

/odometry/filtered
  -> stonefish_docking_controller
  -> /blueye/ref_vel

The docking controller prefers the direct ArUco pose for docking, because this is the most direct docking-station-relative measurement. `/odometry/filtered` is still launched for inspection and later tuning. If direct visual pose is not available, the controller can fall back according to its parameters.

```text
/blueye/pose_estimated_board_stamped + /blueye/odom
  -> stonefish_docking_controller
  -> /blueye/ref_vel
```

/blueye/ref_vel + /blueye/odom
  -> velocity_controller_4dof
  -> /blueye/cmd_force

/blueye/cmd_force
  -> blueye_simulator_interface
  -> /blueye/thrusters
```

## Build

From the Stonefish control workspace:

```bash
cd /home/badawi/Desktop/auto-pilot/src/mundus_stonefish_ws
source /opt/ros/humble/setup.bash
source /home/badawi/Desktop/auto-pilot/src/install/setup.bash
colcon build --symlink-install --packages-select mundus_mir_stonefish_docking_controller
source install/setup.bash
```

If `velocity_controller_4dof_ros2` or `mundus_mir_vehicle_interfaces` are not already built in this workspace, build them too:

```bash
colcon build --symlink-install --packages-select \
  mundus_mir_stonefish_docking_controller \
  velocity_controller_4dof_ros2 \
  mundus_mir_vehicle_interfaces
source install/setup.bash
```

## Start The Stonefish Simulator

In a first terminal, start the docking scenario. Use your normal Stonefish launch command if it differs, but the important part is that the scenario includes:

```text
stonefish_sim/scenarios/docking.scn
```

Example:

```bash
cd /home/badawi/Desktop/auto-pilot/src
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch stonefish_ros2 stonefish_simulator.launch.py \
  scenario:=/home/badawi/Desktop/auto-pilot/src/vortex-stonefish-sim/stonefish_sim/scenarios/docking.scn
```

Check that Stonefish is publishing:

```bash
ros2 topic list | grep blueye
```

You should see at least:

```text
/blueye/cam/image_color
/blueye/imu/data_raw
/blueye/dvl/sim
/blueye/odom
/blueye/thrusters
```

## Run Docking

In a second terminal:

```bash
cd /home/badawi/Desktop/auto-pilot/src/mundus_stonefish_ws
source /opt/ros/humble/setup.bash
source /home/badawi/Desktop/auto-pilot/src/install/setup.bash
source install/setup.bash
ros2 launch mundus_mir_stonefish_docking_controller stonefish_docking.launch.py
```

This launch uses wall time (`use_sim_time: false`) because the current Stonefish simulator launch does not publish `/clock`. If `/clock` has no publisher and `use_sim_time` is true, timer-based nodes such as `stonefish_docking_controller` and `velocity_controller_4dof` will appear alive but will not publish commands.

By default the docking stack now starts disarmed so you can open PlotJuggler and inspect the topics before the ROV moves. Start docking with:

```bash
ros2 service call /blueye/start_docking std_srvs/srv/SetBool "{data: true}"
```

Stop/disarm docking again with:

```bash
ros2 service call /blueye/start_docking std_srvs/srv/SetBool "{data: false}"
```

If you intentionally want the old behavior where docking starts as soon as the launch is ready:

```bash
ros2 launch mundus_mir_stonefish_docking_controller stonefish_docking.launch.py auto_start:=true
```

For visual debugging of ArUco detection:

```bash
ros2 launch mundus_mir_stonefish_docking_controller stonefish_docking.launch.py debug_view:=true
```

The Stonefish launch treats one visible marker as enough to enable docking motion. If the vehicle is not moving because detection is still false, you can temporarily allow motion without the detection gate:

```bash
ros2 launch mundus_mir_stonefish_docking_controller stonefish_docking.launch.py require_aruco_detection:=false
```

## Useful Debug Commands

Check ArUco detection:

```bash
ros2 topic echo /blueye/docking_station_detected
ros2 topic echo /blueye/aruco_visibility
ros2 topic hz /blueye/pose_estimated_board_stamped
```

Check EKF output:

```bash
ros2 topic hz /odometry/filtered
ros2 topic echo /odometry/filtered
```

Check control output:

```bash
ros2 topic echo /blueye/docking_controller_status
ros2 topic echo /blueye/ref_vel
ros2 topic echo /blueye/cmd_force
ros2 topic echo /blueye/thrusters
```

If `/blueye/docking_controller_status` and `/blueye/ref_vel` are silent, check that `/clock` has a publisher only if you intentionally changed the launch back to simulated time:

```bash
ros2 topic info /clock
```

## Notes For Tuning

The ArUco camera matrix is initialized from the Stonefish Blueye camera resolution `1920x1080` and `60 deg` horizontal FOV:

```text
fx = fy = 1662.8
cx = 960.0
cy = 540.0
```

If the estimated pose is biased, tune these parameters in `launch/stonefish_docking.launch.py`.

The DVL frame conversion in `stonefish_sensor_republisher.py` assumes the DVL mounting in `blueye.scn`:

```xml
<origin rpy="0.0 0.0 1.5708" xyz="-0.032 -0.005 0.096"/>
```

If `/odometry/filtered` moves sideways or backwards compared with the camera image, the DVL axis mapping is the first thing to check.

The final docking waypoints are copied from the successful Gazebo controller. If the Stonefish docking rig origin differs from the Gazebo rig origin, tune the waypoint list in:

```text
mundus_mir_stonefish_docking_controller/stonefish_docking_controller.py
```
