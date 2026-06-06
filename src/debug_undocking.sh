#!/bin/bash
# Record topics needed to diagnose undocking heading deviation.
# Run this in a separate terminal BEFORE launching the BT.

STAMP=$(date +"%Y%m%d_%H%M%S")
BAG="undocking_debug_${STAMP}"

echo "Recording undocking debug bag: ${BAG}"
echo "Start the BT launch now, then Ctrl+C this script when undocking completes."
echo ""

source /home/badawi/Desktop/auto-pilot/install/setup.bash

ros2 bag record \
    /blueye/odom \
    /blueye/imu/data_raw \
    /blueye/ref_vel \
    /blueye/cmd_force \
    /blueye/thruster_state \
    -o "${BAG}"
