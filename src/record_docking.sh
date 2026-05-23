#!/bin/bash
# Record docking diagnostic bag.
# Run BEFORE launching the docking stack, or immediately after.
# Stop with Ctrl+C when docking finishes (or fails).

source /opt/ros/humble/setup.bash
source /home/badawi/Desktop/auto-pilot/install/setup.bash

STAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT="docking_bag_${STAMP}"

echo "Recording to: ${OUTPUT}"
echo "Stop with Ctrl+C"

ros2 bag record \
    --output "${OUTPUT}" \
    /blueye/odom \
    /blueye/ref_vel \
    /blueye/cmd_force \
    /blueye/thrusters \
    /blueye/pose_estimated_board_stamped \
    /odometry/filtered \
    /blueye/imu_enu \
    /blueye/dvl_enu
