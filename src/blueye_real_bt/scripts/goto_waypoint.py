#!/usr/bin/env python3
"""
Single waypoint mission for the takeover REJECT path of the behaviour tree: the drone goes to one waypoint on its
onboard mission planner, then the tree starts the docking step.

The mission runs through blueye_handler (mission_planner_scripts/drone_link.py), the only process connected to the
drone; this script never connects itself. Interrupted, or failed, it pauses the mission: the drone holds position.

Usage:
  goto_waypoint.py <lat> <lon> <depth> [drone_ip]        a fixed coordinate (drone_ip is accepted and unused)
  goto_waypoint.py --site-point S [--site pool|pier]      a waypoint of the site layout (mission_sites.py), placed with
                                                          the dock reference and the dock's axis saved by mission.py
                                                          (site: MISSION_SITE environment variable, else pool)
"""

import argparse
import logging
import sys
import threading
import time
from pathlib import Path

import blueye.protocol as bp
import rclpy
from blueye.protocol.types.mission_planning import DepthZeroReference
from blueye.protocol.types.message_formats import LatLongPosition
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions

sys.path.insert(0, str(Path(__file__).resolve().parent / "mission_planner_scripts"))
import mission_sites                     # noqa: E402
from drone_link import DroneLink         # noqa: E402

MAX_DISTANCE_M = 50.0    # a fixed coordinate farther than this from the drone is refused (wrong site)
MAX_DURATION_S = 600.0
MAX_RETRIES = 3


def mission_to(lat, lon, depth, speed, circle_of_acceptance):
    depth_set_point = bp.DepthSetPoint(depth=depth, speed_to_depth=0.2,
                                       depth_zero_reference=DepthZeroReference.DEPTH_ZERO_REFERENCE_SURFACE)
    instructions = [
        bp.Instruction(id=1, auto_continue=True, control_mode_command=bp.ControlModeCommand(
            control_mode_vertical=bp.ControlModeVertical.CONTROL_MODE_VERTICAL_AUTO_DEPTH,
            control_mode_horizontal=bp.ControlModeHorizontal.CONTROL_MODE_HORIZONTAL_AUTO_HEADING)),
        bp.Instruction(id=2, auto_continue=True,
                       depth_set_point_command=bp.DepthSetPointCommand(depth_set_point=depth_set_point)),
        bp.Instruction(id=3, auto_continue=True, waypoint_command=bp.WaypointCommand(waypoint=bp.Waypoint(
            id=3, name="Target Waypoint", global_position=LatLongPosition(latitude=lat, longitude=lon),
            circle_of_acceptance=circle_of_acceptance, speed_to_target=speed, depth_set_point=depth_set_point))),
    ]
    return bp.Mission(id=1, name="Waypoint Mission", instructions=instructions, default_surge_speed=speed,
                      default_heave_speed=0.2, default_circle_of_acceptance=circle_of_acceptance)


def run(link, mission, logger):
    """Load and run the mission until it completes; False on failure or timeout."""
    ok, message = link.load_mission(mission)
    if not ok:
        logger.error(f"Loading the mission failed: {message}")
        return False
    logger.info(message)
    if not link.wait_for_valid_position():
        return False

    retries = 0
    ok, message = link.call("/blueye/mission/run")
    if not ok:
        logger.error(message)
        return False
    time.sleep(1.0)                 # the status topic (2 Hz) shows the result from here on
    start = time.time()
    while time.time() - start < MAX_DURATION_S:
        status = link.mission_status()
        if status is None:
            logger.warning("No mission status from blueye_handler")
            time.sleep(2.0)
            continue
        state = status["state"].replace("MISSION_STATE_", "")
        depth = f", Depth: {link.pose[2]:.1f}m" if link.pose else ""
        logger.info(f"Mission: {state}, Progress: {len(status['completed'])}/{status['total']} instructions{depth}")
        if state == "COMPLETED":
            logger.info("Mission completed successfully")
            return True
        if state == "ABORTED":
            if retries >= MAX_RETRIES:
                logger.warning("Maximum retry attempts reached")
                return False
            retries += 1
            logger.info(f"Mission aborted, resuming (attempt {retries}/{MAX_RETRIES})")
            link.call("/blueye/mission/run")
            time.sleep(3.0)
            continue
        if state in ("FAILED_TO_LOAD_MISSION", "FAILED_TO_START_MISSION"):
            logger.error(f"Mission failed with state: {state}")
            return False
        time.sleep(2.0)
    logger.warning(f"Mission timeout after {MAX_DURATION_S:.0f} s")
    return False


def main():
    parser = argparse.ArgumentParser(description="Single waypoint mission through blueye_handler")
    parser.add_argument("coordinates", nargs="*", help="<lat> <lon> <depth> [drone_ip]")
    parser.add_argument("--site-point", help="waypoint of the site layout instead, e.g. S")
    parser.add_argument("--site", choices=sorted(mission_sites.SITES), default=mission_sites.DEFAULT_SITE)
    args = parser.parse_args()
    if args.site_point is None and not 3 <= len(args.coordinates) <= 4:
        parser.error("give <lat> <lon> <depth> [drone_ip], or --site-point")

    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                        handlers=[logging.StreamHandler(sys.stdout)])
    logger = logging.getLogger("waypoint_mission")

    if args.site_point is not None:
        try:
            lat, lon, depth = mission_sites.site_point(args.site, args.site_point)
        except ValueError as exc:
            logger.error(f"No target: {exc}")
            return 1
        site = mission_sites.SITES[args.site]
        speed, circle = site["speed"], site["circle_of_acceptance"]
        logger.info(f"Target: {args.site_point} of the {args.site} layout")
    else:
        lat, lon, depth = (float(v) for v in args.coordinates[:3])
        speed, circle = 0.3, 1.0

    # rclpy must not shut down on SIGINT: pausing the mission still needs it
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("goto_waypoint_client")
    link = DroneLink(node, logger)
    spin = threading.Thread(target=lambda: _spin(node), daemon=True)
    spin.start()
    started = completed = False
    try:
        offset = link.offset_from_dock((lat, lon))     # from the target to the drone
        if offset is None:
            logger.error("No valid position estimate from blueye_handler")
            return 1
        logger.info(f"Waypoint mission to ({lat:.7f}, {lon:.7f}) at {depth} m: {offset[0]:.1f} m away")
        if args.site_point is None and offset[0] > MAX_DISTANCE_M:
            logger.error(f"Not going: the target is more than {MAX_DISTANCE_M:.0f} m away (a coordinate of another "
                         "site?)")
            return 1
        started = True
        completed = run(link, mission_to(lat, lon, depth, speed, circle), logger)
        return 0 if completed else 1
    except KeyboardInterrupt:
        logger.info("Waypoint mission interrupted")
        return 130
    finally:
        if started and not completed:
            ok, message = link.call("/blueye/mission/pause", timeout_s=5.0)
            logger.info(f"Mission paused, the drone holds position ({message})" if ok
                        else f"Pausing the mission failed: {message}")
        rclpy.shutdown()
        spin.join(timeout=2.0)


def _spin(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


if __name__ == "__main__":
    sys.exit(main())
