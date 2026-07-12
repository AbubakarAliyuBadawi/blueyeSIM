#!/bin/bash
# Record simulation topics for mission analysis and journal paper plots.
# Run BEFORE starting the BT mission in a separate terminal.
#
# Usage:
#   bash record_sim_mission.sh
#   bash record_sim_mission.sh my_experiment_name

source ~/Desktop/auto-pilot/install/setup.bash

LABEL=${1:-"sim"}
BAG_DIR="$HOME/rosbags/${LABEL}_$(date +%Y%m%d_%H%M%S)"

echo "Recording to: $BAG_DIR"
echo "Stop with Ctrl+C when mission is complete."
echo ""

# Trajectory / position
TOPICS="/odometry/filtered"
TOPICS="$TOPICS /blueye/odom"

# Mission state
TOPICS="$TOPICS /mission_state"
TOPICS="$TOPICS /mission/phase"

# Control commands
TOPICS="$TOPICS /blueye/cmd_force"
TOPICS="$TOPICS /blueye/ref_vel"
TOPICS="$TOPICS /joy"

# Takeover / handover events
TOPICS="$TOPICS /blueye/takeover_request/urgency"
TOPICS="$TOPICS /blueye/takeover_request/threshold"
TOPICS="$TOPICS /blueye/takeover_request/human_decision"
TOPICS="$TOPICS /blueye/takeover_request/handback"

# Bayesian network outputs (unique to simulation)
TOPICS="$TOPICS /blueye/takeover_request/max_probability"
TOPICS="$TOPICS /blueye/takeover_request/no_takeover_prob"
TOPICS="$TOPICS /blueye/takeover_request/takeover_requested_prob"
TOPICS="$TOPICS /blueye/takeover_request/attention_required_prob"
TOPICS="$TOPICS /blueye/takeover_request/evidence"
TOPICS="$TOPICS /blueye/takeover_request/state"

# Human model
TOPICS="$TOPICS /blueye/human/fatigue"
TOPICS="$TOPICS /blueye/human/stress"

# Waypoints / pipeline
TOPICS="$TOPICS /blueye/current_waypoint"
TOPICS="$TOPICS /blueye/current_reference"

# Docking
TOPICS="$TOPICS /blueye/docking_controller_status"
TOPICS="$TOPICS /blueye/docking_progress"
TOPICS="$TOPICS /blueye/docking_station_detected"

ros2 bag record --output "$BAG_DIR" $TOPICS
