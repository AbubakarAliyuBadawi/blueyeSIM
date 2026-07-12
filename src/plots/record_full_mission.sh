#!/bin/bash
# Record all topics needed for full mission analysis and journal paper plots.
# Run this BEFORE starting the BT mission in a separate terminal.
#
# Usage:
#   bash src/plots/record_full_mission.sh
#   bash src/plots/record_full_mission.sh my_experiment_name

source ~/blueye_ws/install/setup.bash

LABEL=${1:-"takeover_real"}
BAG_DIR="$HOME/Desktop/auto-pilot/src/rosbags/${LABEL}_$(date +%Y%m%d_%H%M%S)"

echo "Recording to: $BAG_DIR"
echo "Stop with Ctrl+C when mission is complete."
echo ""

# Position / trajectory
TOPICS="/blueye/sensor/ekf/lat_long"
TOPICS="$TOPICS /blueye/sensor/ekf/pose"
TOPICS="$TOPICS /blueye/pose"
TOPICS="$TOPICS /blueye/sensor/depth"
TOPICS="$TOPICS /blueye/sensor/attitude"
TOPICS="$TOPICS /blueye/speed"
TOPICS="$TOPICS /blueye/altitude"

# Control commands
TOPICS="$TOPICS /blueye/commands"
TOPICS="$TOPICS /joy"

# Mission state
TOPICS="$TOPICS /mission_state"
TOPICS="$TOPICS /blueye/mission/control"

# Takeover / handover events
TOPICS="$TOPICS /blueye/takeover_request/urgency"
TOPICS="$TOPICS /blueye/takeover_request/threshold"
TOPICS="$TOPICS /blueye/takeover_request/human_decision"
TOPICS="$TOPICS /blueye/takeover_request/handback"

# Docking
TOPICS="$TOPICS /blueye/battery"

# IMU
TOPICS="$TOPICS /blueye/sensor/imu"

ros2 bag record --output "$BAG_DIR" $TOPICS
