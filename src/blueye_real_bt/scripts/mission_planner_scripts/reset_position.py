#!/usr/bin/env python3
"""
Reset the drone's position estimate to the dock of a site while the drone is in the dock; run it before every
mission (before starting the behaviour tree), with blueye_handler running: the reset goes through the handler
(/blueye/reset_position), the only process connected to the drone. It saves the dock reference
(mission_sites.REFERENCE_FILE): mission.py refuses to run without a recent one, and places its waypoints from the dock
along the direction the undocking moved the drone. The drone's compass heading in the dock is saved too, for
comparison only.

The reset leaves the estimate invalid, and the DVL silent, for several seconds; this script waits until the drone
reports a steady valid estimate at the dock again, so this happens before the mission and the takeover network, not
during them.

Usage, drone docked:
  python3 reset_position.py [--site pool|pier]     (default: the MISSION_SITE environment variable, else pool)
"""

import argparse
import logging
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException

import mission_sites
from drone_link import DroneLink

AT_DOCK_M = 1.0     # the reported position must come within this of the dock coordinate
HEADING_SAMPLE_S = 2.0


def setup_logging(log_file="log/reset_position.log"):
    Path(log_file).parent.mkdir(exist_ok=True, parents=True)
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s',
                        handlers=[logging.FileHandler(log_file), logging.StreamHandler(sys.stdout)])
    return logging.getLogger("reset_position")


def reset_to_dock(link, site, logger, timeout_s=60.0):
    """Reset the estimate to the dock and wait until the drone reports it there, valid; False after timeout_s."""
    lat0, lon0 = site["dock"]
    ok, message = link.call("/blueye/reset_position", {"lat": lat0, "lon": lon0})
    if not ok:
        logger.error(f"Position reset failed: {message}")
        return False
    sent = time.time()
    logger.info(f"Position reset sent: dock at {lat0:.7f}, {lon0:.7f}")
    time.sleep(1.0)                                   # the estimate still shows the old position for a moment
    while time.time() - sent < timeout_s:
        offset = link.offset_from_dock((lat0, lon0), timeout_s=0.5)
        if offset is not None and offset[0] < AT_DOCK_M and link.position_ready():
            logger.info(f"Position estimate valid at the dock ({offset[0]:.2f} m from the dock coordinate, "
                        f"{time.time() - sent:.1f} s after the reset)")
            return True
        time.sleep(0.5)
    logger.error(f"No valid position estimate at the dock {timeout_s:.0f} s after the reset")
    return False


def docked_heading(link, logger, sample_s=HEADING_SAMPLE_S):
    """The drone's compass heading [deg] averaged over sample_s (circular mean); None without a pose. Kept for
    comparison: in the pool it is not reliable enough to place the waypoints."""
    samples, end = [], time.time() + sample_s
    while time.time() < end:
        if link.pose is not None:
            samples.append(math.radians(link.pose[3]))
        time.sleep(0.1)
    if not samples:
        logger.warning("No pose from the handler: docked compass heading unknown")
        return None
    heading = math.degrees(math.atan2(sum(math.sin(a) for a in samples), sum(math.cos(a) for a in samples))) % 360
    logger.info(f"Docked compass heading {heading:.1f} deg")
    return heading


def main():
    parser = argparse.ArgumentParser(description="Reset the drone's position estimate to the dock (drone docked)")
    parser.add_argument('--site', choices=sorted(mission_sites.SITES), default=mission_sites.DEFAULT_SITE)
    args = parser.parse_args()
    logger = setup_logging()

    rclpy.init()
    node = rclpy.create_node("reset_position_client")
    link = DroneLink(node, logger)
    spin = threading.Thread(target=lambda: _spin(node), daemon=True)
    spin.start()
    try:
        if not reset_to_dock(link, mission_sites.SITES[args.site], logger):
            return 1
        reference = mission_sites.save_reference(args.site, docked_heading(link, logger))
        logger.info(f"Dock reference saved to {mission_sites.REFERENCE_FILE}: {reference}")
        return 0
    finally:
        rclpy.shutdown()
        spin.join(timeout=2.0)


def _spin(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


if __name__ == "__main__":
    sys.exit(main())
