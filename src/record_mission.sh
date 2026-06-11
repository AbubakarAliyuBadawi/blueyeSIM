#!/bin/bash
# Records all relevant topics for mission + takeover/handover analysis.
# Usage:
#   ./record_mission.sh              # saves to ./rosbags/mission_<timestamp>/
#   ./record_mission.sh my_label     # saves to ./rosbags/my_label_<timestamp>/

LABEL=${1:-"mission"}
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
OUTPUT_DIR="$(dirname "$0")/rosbags/${LABEL}_${TIMESTAMP}"

echo "Recording to: $OUTPUT_DIR"
echo "Press Ctrl+C to stop."

ros2 bag record \
  --output "$OUTPUT_DIR" \
  \
  /mission_state \
  /mission_phase \
  \
  /blueye/takeover_request/urgency \
  /blueye/takeover_request/threshold \
  /blueye/takeover_request/human_decision \
  /blueye/takeover_request/handback \
  /blueye/takeover_request/attention_required_prob \
  /blueye/takeover_request/takeover_requested_prob \
  /blueye/takeover_request/no_takeover_prob \
  /blueye/takeover_request/state \
  /blueye/takeover_request/evidence \
  \
  /blueye/depth \
  /blueye/pose \
  /blueye/sensor/ekf/pose \
  /blueye/sensor/ekf/lat_long \
  /blueye/gps \
  /blueye/speed \
  /blueye/commands \
  \
  /blueye/battery \
  /blueye/battery_level \
  /blueye/battery_state \
  /blueye/battery_current \
  \
  /blueye/depth_state \
  /blueye/speed_state \
  /blueye/altitude_state \
  /blueye/current_waypoint \
  /blueye/waypoint_tracking_error_m \
  \
  /blueye/camera_quality \
  /blueye/usbl_strength \
  /blueye/sonar_range \
  /blueye/aruco_visibility \
  /blueye/inspection_data_quality \
  \
  /blueye/human/stress \
  /blueye/human/fatigue
