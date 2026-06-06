#!/bin/bash
# Records a debug bag for pipeline inspection.
# Run this after the simulator starts, then trigger the inspection.
# Bag is saved to the same directory as this script.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_NAME="inspection_debug_$(date +%Y%m%d_%H%M%S)"
BAG_PATH="${SCRIPT_DIR}/${BAG_NAME}"

echo "Recording to: ${BAG_PATH}"
echo "Press Ctrl+C to stop."

ros2 bag record \
    /blueye/pipeline_inspection_status \
    /blueye/pipeline_inspection_progress \
    /blueye/pipeline_marker_detected \
    /blueye/pipeline_marker_ids \
    /blueye/pipeline_marker_visibility \
    /blueye/pipeline_pose_estimated \
    /blueye/pipeline_pose_estimated_stamped \
    /blueye/pipeline_current_waypoint \
    /blueye/pipeline_current_reference \
    /blueye/ref_vel \
    /blueye/odom \
    /odometry/filtered \
    /blueye/dvl_enu \
    /blueye/imu_enu \
    /blueye/inspection/debug_image \
    /blueye/cam_down/image_color/compressed \
    /tf \
    /tf_static \
    -o "${BAG_PATH}"
