#!/bin/bash

SCRIPT_PATH=/home/badawi/Desktop/auto-pilot/src/blueye_real_bt/scripts/mission_planner_scripts/mission.py

# Execute the script with the provided arguments
python3 $SCRIPT_PATH "$@"

# Return the exit code from the Python script
exit $?