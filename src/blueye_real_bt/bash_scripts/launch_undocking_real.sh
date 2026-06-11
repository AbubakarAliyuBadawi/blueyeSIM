#!/bin/bash

# launch_undocking_real.sh
# Simple undocking script that calls the SDK undocking script

SCRIPT_PATH=/home/badawi/Desktop/auto-pilot/src/blueye_real_bt/scripts/mission_planner_scripts/undocking.py

# Execute the undocking script with the provided arguments
python3 $SCRIPT_PATH "$@"

# Return the exit code from the Python script
exit $?