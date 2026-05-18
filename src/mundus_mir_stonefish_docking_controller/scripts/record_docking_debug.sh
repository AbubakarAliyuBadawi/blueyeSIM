#!/usr/bin/env bash
set -euo pipefail

BAG_ROOT="${1:-$HOME/docking_debug_bags}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_DIR="$BAG_ROOT/docking_debug_$STAMP"

TOPICS=(
  /blueye/docking_controller_status
  /blueye/docking_progress
  /blueye/docking_station_detected
  /blueye/aruco_visibility
  /blueye/pose_estimated_board
  /blueye/pose_estimated_board_stamped
  /odometry/filtered
  /blueye/odom
  /blueye/current_waypoint
  /blueye/current_reference
  /blueye/ref_vel
  /blueye/cmd_force
  /blueye/thrusters
  /blueye/dvl_enu
  /blueye/imu_enu
  /blueye/dvl/sim
  /blueye/imu/data_raw
  /tf
  /tf_static
)

cleanup() {
  # Prevent cleanup from running multiple times
  trap - EXIT INT TERM

  echo
  echo "Stopping docking and rosbag..."

  # Stop docking safely without hanging forever
  timeout 3 ros2 service call /blueye/start_docking \
    std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 || true

  # Stop rosbag cleanly
  if [[ -n "${BAG_PID:-}" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" >/dev/null 2>&1 || true
    wait "$BAG_PID" >/dev/null 2>&1 || true
  fi

  echo "Saved bag: $BAG_DIR"
}

# Cleanup only once when script exits
trap cleanup EXIT

mkdir -p "$BAG_ROOT"

echo "Recording docking debug bag:"
echo "  $BAG_DIR"
echo
echo "Topics:"
printf '  %s\n' "${TOPICS[@]}"
echo

ros2 bag record -o "$BAG_DIR" "${TOPICS[@]}" &
BAG_PID=$!

echo "Waiting for /blueye/start_docking service..."
until ros2 service list | grep -qx "/blueye/start_docking"; do
  sleep 0.5
done

echo
echo "Rosbag is recording. Open PlotJuggler now if you want."
read -r -p "Press Enter to ARM docking and continue recording..."

echo
echo "Arming docking..."

timeout 3 ros2 service call /blueye/start_docking \
  std_srvs/srv/SetBool "{data: true}"

echo
echo "Docking armed. Recording continues."
echo "Press Ctrl+C here when the test is finished."

wait "$BAG_PID"