#!/usr/bin/env python3
"""
Test: can the operator control surge/sway while auto_depth + auto_heading are active?

What this script does:
  1. Connects to the drone (single SDK connection)
  2. Subscribes to /blueye/commands (WrenchStamped) — published by joystick_controller
  3. Activates auto_depth, then commands heave until drone reaches target depth
  4. Once at depth: sets heave=0 to hold, activates auto_heading
  5. For --hold-duration seconds: forwards joystick surge+sway to drone, logs depth
  6. Reports whether depth stayed stable (hold worked) and whether surge/sway moved the drone

Run alongside:
  Terminal 1: ros2 run joy joy_node
  Terminal 2: ros2 run blueye_joystick_cpp joystick_controller

Usage:
  python3 test_depth_hold_joystick.py [--drone-ip 192.168.1.101] [--depth 1.0] [--hold-duration 20]
"""

import argparse
import sys
import time
import logging
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import blueye.protocol as bp
from blueye.sdk import Drone
from blueye.protocol.types.mission_planning import DepthZeroReference


def setup_logging():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s",
        handlers=[logging.StreamHandler(sys.stdout)],
    )
    return logging.getLogger("depth_hold_joystick_test")


class JoystickBridge(Node):
    """Subscribes to /blueye/commands and makes latest surge/sway available."""
    def __init__(self):
        super().__init__("depth_hold_joystick_test")
        self.surge = 0.0
        self.sway  = 0.0
        self._lock = threading.Lock()
        self.create_subscription(WrenchStamped, "/blueye/commands", self._cb, 10)

    def _cb(self, msg: WrenchStamped):
        with self._lock:
            self.surge = msg.wrench.force.x
            self.sway  = msg.wrench.force.y

    def get(self):
        with self._lock:
            return self.surge, self.sway


def main():
    parser = argparse.ArgumentParser(description="Test depth-hold + manual surge/sway")
    parser.add_argument("--drone-ip",       default="192.168.1.101")
    parser.add_argument("--depth",          type=float, default=1.0,
                        help="Target hold depth in metres")
    parser.add_argument("--hold-duration",  type=float, default=20.0,
                        help="How long to hold and accept joystick input (s)")
    parser.add_argument("--dive-speed",     type=float, default=0.15,
                        help="Heave speed setpoint while diving (m/s)")
    parser.add_argument("--depth-tolerance",type=float, default=0.08,
                        help="Depth band around target considered 'at depth' (m)")
    args = parser.parse_args()

    logger = setup_logging()
    logger.info("=== Depth-hold + joystick surge/sway test ===")
    logger.info(f"Target depth:  {args.depth} m")
    logger.info(f"Hold duration: {args.hold_duration} s")
    logger.info("")
    logger.info("Make sure these are running in other terminals:")
    logger.info("  ros2 run joy joy_node")
    logger.info("  ros2 run blueye_joystick_cpp joystick_controller")
    logger.info("")

    # ── ROS init ─────────────────────────────────────────────────────────────
    rclpy.init()
    bridge = JoystickBridge()
    spin_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    spin_thread.start()

    # ── Connect to drone ──────────────────────────────────────────────────────
    logger.info("Connecting to drone...")
    drone = Drone(ip=args.drone_ip, auto_connect=True, timeout=30)
    logger.info(f"Connected: {drone.serial_number}  SW: {drone.software_version}")
    if not drone.in_control:
        drone.take_control()
        logger.info("Control acquired")

    try:
        # ── 1. Clear any previous state ───────────────────────────────────────
        drone.motion.station_keeping_active = False
        drone.motion.auto_depth_active      = False
        drone.motion.auto_heading_active    = False
        drone.motion.surge = 0.0
        drone.motion.sway  = 0.0
        drone.motion.heave = 0.0
        drone.motion.yaw   = 0.0
        drone.mission.clear()
        time.sleep(1.0)

        # ── 2. Use mission planner to dive to target depth ────────────────────
        logger.info(f"Loading DepthSetPoint mission → {args.depth} m ...")
        depth_sp  = bp.DepthSetPoint(
            depth=args.depth,
            speed_to_depth=args.dive_speed,
            depth_zero_reference=DepthZeroReference.DEPTH_ZERO_REFERENCE_SURFACE,
        )
        instruction = bp.Instruction(
            depth_set_point_command=bp.DepthSetPointCommand(depth_set_point=depth_sp)
        )
        drone.mission.send_new(bp.Mission(instructions=[instruction]))
        time.sleep(0.5)
        drone.mission.run()
        logger.info("Mission running — waiting for drone to reach target depth...")

        # Wait until depth is close enough OR mission completes
        DONE_STATES = {
            bp.MissionState.MISSION_STATE_COMPLETED,
            bp.MissionState.MISSION_STATE_ABORTED,
        }
        while True:
            status = drone.mission.get_status()
            current = drone.depth
            logger.info(f"  depth={current:.3f} m  target={args.depth} m")
            if current >= args.depth - args.depth_tolerance:
                logger.info("Target depth reached via mission.")
                break
            if status and status.state in DONE_STATES:
                logger.info(f"Mission completed (state={status.state.name}).")
                break
            time.sleep(0.5)

        # ── 3. Switch to manual depth+heading hold ────────────────────────────
        logger.info("Clearing mission and activating auto_depth + auto_heading...")
        drone.mission.clear()
        time.sleep(0.5)
        drone.motion.auto_depth_active   = True
        drone.motion.auto_heading_active = True
        drone.motion.heave = 0.0
        time.sleep(0.5)

        hold_depth = drone.depth
        logger.info(f"At depth: {hold_depth:.3f} m")
        logger.info(f"auto_depth={drone.motion.auto_depth_active}  "
                    f"auto_heading={drone.motion.auto_heading_active}  "
                    f"station_keeping={drone.motion.station_keeping_active}")

        # ── 3. Joystick phase ─────────────────────────────────────────────────
        logger.info("")
        logger.info(f">>> Joystick active for {args.hold_duration} s")
        logger.info("    Use your controller to move the drone in surge/sway.")
        logger.info("    Depth should stay stable — buoyancy or joystick drift will show here.")
        logger.info("")

        start     = time.time()
        last_log  = start
        max_depth_delta = 0.0
        min_depth = hold_depth
        max_depth = hold_depth

        while time.time() - start < args.hold_duration:
            surge, sway = bridge.get()

            # Forward joystick surge+sway; heave=0 lets auto_depth hold
            drone.motion.surge = surge
            drone.motion.sway  = sway
            drone.motion.heave = 0.0

            now = time.time()
            if now - last_log >= 0.5:
                try:
                    d = drone.depth
                    delta = d - hold_depth
                    max_depth_delta = max(max_depth_delta, abs(delta))
                    min_depth = min(min_depth, d)
                    max_depth = max(max_depth, d)
                    remaining = args.hold_duration - (now - start)
                    logger.info(f"  t={now-start:4.1f}s  depth={d:.3f} m  Δ={delta:+.3f} m  "
                                f"surge={surge:+.2f}  sway={sway:+.2f}  "
                                f"({remaining:.0f}s left)")
                except Exception:
                    pass
                last_log = now
            time.sleep(0.1)

        # ── 4. Stop and report ────────────────────────────────────────────────
        drone.motion.surge = 0.0
        drone.motion.sway  = 0.0
        drone.motion.heave = 0.0

        logger.info("")
        logger.info("=== RESULT ===")
        logger.info(f"Hold depth:       {hold_depth:.3f} m")
        logger.info(f"Depth range:      {min_depth:.3f} m → {max_depth:.3f} m")
        logger.info(f"Max depth delta:  {max_depth_delta:.3f} m")

        if max_depth_delta < 0.15:
            logger.info("DEPTH HOLD: STABLE ✓  (auto_depth kept drone within 15 cm)")
        else:
            logger.info(f"DEPTH HOLD: DRIFTED  (max {max_depth_delta:.3f} m from target)")

        logger.info("")
        logger.info("SURGE/SWAY: check if the drone physically moved horizontally while you")
        logger.info("            used the joystick. If yes → manual surge/sway works with depth hold.")
        logger.info("            If the drone didn't respond to joystick → it does not.")

        return 0

    except KeyboardInterrupt:
        logger.info("Test interrupted by user")
        return 130

    except Exception as e:
        logger.error(f"Unexpected error: {e}", exc_info=True)
        return 1

    finally:
        logger.info("Stopping all motion...")
        try:
            drone.motion.surge = 0.0
            drone.motion.sway  = 0.0
            drone.motion.heave = 0.0
            drone.motion.yaw   = 0.0
            drone.motion.auto_depth_active   = False
            drone.motion.auto_heading_active = False
        except Exception:
            pass
        try:
            drone.disconnect()
        except Exception:
            pass
        try:
            bridge.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
