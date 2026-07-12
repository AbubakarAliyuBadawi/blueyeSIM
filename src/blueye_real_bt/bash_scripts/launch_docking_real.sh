#!/bin/bash

source ~/blueye_ws/install/setup.bash

echo "Starting docking procedure..."

# Launch only the docking-specific nodes (ArUco detection, EKF, DVL, TF).
# blueye_handler, joy_node, and joystick_controller remain running so the
# operator can take over via joystick at any point during docking.
ros2 launch mundus_mir_docking_controller docking_bt.launch.py &
DOCKING_PID=$!

echo "Docking controller started (PID: $DOCKING_PID)"
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

# Stop the docking controller
kill $DOCKING_PID 2>/dev/null
wait $DOCKING_PID 2>/dev/null

if [ $MONITOR_RESULT -eq 0 ]; then
    echo "Docking procedure completed successfully"
    exit 0
else
    echo "Docking procedure failed or timed out"
    exit 1
fi
