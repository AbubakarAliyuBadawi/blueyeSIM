#!/bin/bash

source ~/Desktop/auto-pilot/install/setup.bash

echo "Starting docking procedure..."

# TF, the DVL driver and the ArUco detection normally run from the start of the mission
# (ros2 launch mundus_mir_docking_controller real_sensors.launch.py) so the takeover network has their
# evidence throughout; start them here only if they are not running yet.
NODES=$(ros2 node list 2>/dev/null)
if echo "$NODES" | grep -q "/waterlinked_dvl$" && echo "$NODES" | grep -q "/pose_estimation_aruco_real_life$"; then
    WITH_SENSORS=false
    echo "DVL and ArUco detection already running: starting the EKF only"
else
    WITH_SENSORS=true
fi

# Launch only the docking-specific nodes (EKF and docking controller, and TF, DVL and ArUco detection if needed).
# blueye_handler, joy_node, and joystick_controller remain running so the
# operator can take over via joystick at any point during docking.
# A takeover halts this script: the tree stops this process group, then zeroes the thrusters and holds.
ros2 launch mundus_mir_docking_controller docking_bt.launch.py with_sensors:=$WITH_SENSORS &
DOCKING_PID=$!

echo "Docking nodes started (PID: $DOCKING_PID)"

# DOCKING_ENGAGE=0 (in the environment of the behaviour tree) leaves the docking controller idle: the operator docks
# with the joystick and the step still ends on charging. For tests while the dock-board pose is not trusted.
if [ "${DOCKING_ENGAGE:-1}" = "0" ]; then
    echo "Automatic docking disabled (DOCKING_ENGAGE=0): dock with the joystick"
else

# Engage the docking controller as the joystick's docking-mode button does: depth and heading hold off, docking
# mode on — once the EKF has a pose (it needs the dock markers in view).
python3 - <<'EOF'
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool

POSE_TIMEOUT = 30.0   # s for the first EKF pose

rclpy.init()
node = rclpy.create_node('docking_engage')
poses = []
node.create_subscription(Odometry, '/odometry/filtered', poses.append, 1)
start = time.time()
while rclpy.ok() and not poses and time.time() - start < POSE_TIMEOUT:
    rclpy.spin_once(node, timeout_sec=0.2)
if not poses:
    node.get_logger().error(f'No EKF pose on /odometry/filtered after {POSE_TIMEOUT:.0f} s: dock markers not in view?')
    sys.exit(1)


def call(name, value):
    client = node.create_client(SetBool, name)
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(f'{name} not available')
        return False
    future = client.call_async(SetBool.Request(data=value))
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.result() is None:
        node.get_logger().error(f'{name} {value}: no response')
        return False
    node.get_logger().info(f'{name} {value}')
    return True


# station keeping first: the mission or the undocking may have left it on, and it would fight the docking controller
engaged = call('/blueye/stationkeep', False) and call('/blueye/depth_hold', False) and \
    call('/blueye/heading_hold', False) and call('/blueye/docking_mode', True)
sys.exit(0 if engaged else 1)
EOF

if [ $? -ne 0 ]; then
    echo "Docking controller could not be engaged"
    kill $DOCKING_PID 2>/dev/null
    wait $DOCKING_PID 2>/dev/null
    exit 1
fi

echo "Docking controller engaged"
fi
echo "Monitoring /blueye/battery for charging signal..."

# Block until charging_current (position.x) > 0 for CONFIRM_SECS consecutive
# seconds, or until timeout (10 min). Sustained charging prevents false positives
# from brief contact during approach.
python3 - <<'EOF'
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import sys
import time

CONFIRM_SECS = 5   # must be charging this long before declaring success
# position.z = msg.battery.current: positive=charging, negative=discharging.
# position.x = msg.battery.charging_current: the BQ40Z50 chip's REQUESTED rate —
# always ~5.6 A regardless of dock connection, so it cannot be used for detection.
CHARGE_THRESHOLD = 0.5  # A — must exceed this to count as charging

class ChargingMonitor(Node):
    def __init__(self):
        super().__init__('docking_charging_monitor')
        self.confirmed = False
        self.charging_since = None
        self.start = time.time()
        self.timeout = 600  # 10 minutes
        self.create_subscription(Pose, '/blueye/battery', self.battery_cb, 10)

    def battery_cb(self, msg):
        current = msg.position.z  # actual battery current (A), positive = charging
        if current > CHARGE_THRESHOLD:
            if self.charging_since is None:
                self.charging_since = time.time()
                self.get_logger().info(
                    f'Charging detected ({current:.3f} A) — confirming for {CONFIRM_SECS}s...')
            elapsed = time.time() - self.charging_since
            self.get_logger().info(
                f'Charging: {current:.3f} A  ({elapsed:.1f}/{CONFIRM_SECS}s)')
            if elapsed >= CONFIRM_SECS:
                self.get_logger().info('Docking confirmed — drone has been charging.')
                self.confirmed = True
        else:
            if self.charging_since is not None:
                self.get_logger().warn(
                    f'Charging lost ({current:.3f} A) — resetting confirmation timer')
            self.charging_since = None

rclpy.init()
node = ChargingMonitor()

while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=1.0)
    if node.confirmed:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    if time.time() - node.start > node.timeout:
        node.get_logger().error(f'Docking timed out after {node.timeout}s')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

node.destroy_node()
rclpy.shutdown()
sys.exit(1)
EOF

MONITOR_RESULT=$?

# Stop the docking nodes (the tree then zeroes the thrusters)
kill $DOCKING_PID 2>/dev/null
wait $DOCKING_PID 2>/dev/null

if [ $MONITOR_RESULT -eq 0 ]; then
    echo "Docking procedure completed successfully"
    exit 0
else
    echo "Docking procedure failed or timed out"
    exit 1
fi
