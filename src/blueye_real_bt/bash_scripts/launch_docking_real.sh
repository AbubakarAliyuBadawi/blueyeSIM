#!/bin/bash

source ~/Desktop/mundus_mir_simulator/install/setup.bash

echo "Starting docking procedure..."

# Launch docking controller in background
ros2 launch mundus_mir_docking_controller docking.launch.py &
DOCKING_PID=$!

echo "Docking controller started (PID: $DOCKING_PID)"
echo "Monitoring /blueye/battery for charging signal..."

# Block until charging_current (position.x) > 0 or timeout (10 min)
python3 - <<'EOF'
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import sys
import time

class ChargingMonitor(Node):
    def __init__(self):
        super().__init__('docking_charging_monitor')
        self.charging = False
        self.start = time.time()
        self.timeout = 600  # 10 minutes
        self.create_subscription(Pose, '/blueye/battery', self.battery_cb, 10)

    def battery_cb(self, msg):
        charging_current = msg.position.x
        self.get_logger().info(f'Charging current: {charging_current:.3f} A')
        if charging_current > 0:
            self.get_logger().info('Drone is charging — docking complete.')
            self.charging = True

rclpy.init()
node = ChargingMonitor()

while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=1.0)
    if node.charging:
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
