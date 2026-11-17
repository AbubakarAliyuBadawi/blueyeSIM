#!/usr/bin/env python3
"""
Survey mission for the Blueye on its onboard mission planner, built from a site layout in mission_sites.py
(waypoints in metres from the dock): transit to the first inspection point, a straight inspection line with a wait at
each point, homing to a stop point in front of the dock. The behaviour tree then starts the docking step.

Before each mission, reset_position.py resets the position estimate to the dock while the drone is docked and saves
the dock reference. This script runs after the undocking: the direction from the dock to the drone then gives the
dock's axis, along which the layout is placed (outside AXIS_MIN_M-AXIS_MAX_M from the dock that direction says
nothing about the dock, and the docked compass heading saved by the reset is used instead). It refuses to run without
a recent reference, or when the drone is more than MAX_START_DISTANCE_M from the dock.

For the takeover network it publishes the mission phase on /mission_state (2 transit, 3 inspection,
4 homing) and the active waypoint on /blueye/waypoint_goal, in the frame of the drone's position estimate.
The drone's notifications are logged, among them the mission planner's reason for an abort.

The mission runs through blueye_handler (drone_link.py), the only process connected to the drone: this script never
connects itself, so the handler keeps control for the joystick, the holds and the docking.

Usage:
  python3 mission.py [--site pool|pier] [--drone-ip IP_ADDRESS] [--timeout SECONDS]
  (the site defaults to the MISSION_SITE environment variable, else pool)
"""
import json
import argparse
import os
import sys
import time
import logging
import threading
from pathlib import Path
from datetime import datetime
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32
import blueye.protocol as bp
from blueye.protocol.types.mission_planning import DepthZeroReference
from blueye.protocol.types.message_formats import LatLongPosition

import mission_sites
from drone_link import DroneLink

# ── Mission control channel ──────────────────────────────────────────────────
# The BT publishes "pause" / "resume" / "stop" here.
_pause_event  = threading.Event()
_resume_event = threading.Event()
_stop_event   = threading.Event()
_control_node = None
_control_thread = None
_state_pub    = None

# ── Phase and waypoint goal for the takeover network ─────────────────────────
# Phases as in the simulator and the network, one per mission instruction (from the site layout): the phase of
# the first instruction not yet completed. The active waypoint goes to /blueye/waypoint_goal (PointStamped, frame odom: x north, y east in metres,
# z depth) once per waypoint, because the network starts a new tracking leg on every goal it receives.
TRANSIT, INSPECTION, HOMING = 2, 3, 4
PHASE_NAMES = {TRANSIT: "TRANSIT", INSPECTION: "INSPECTION", HOMING: "HOMING"}
PHASES = {"transit": TRANSIT, "inspection": INSPECTION, "homing": HOMING}
_goal_pub = None
_pose = None        # (time, north, east) from the handler's /blueye/sensor/ekf/pose
_latlon = None      # (time, latitude, longitude) from the handler's /blueye/sensor/ekf/lat_long


def _setup_control_channel(logger):
    """Subscribe to /blueye/mission/control, create the state and goal publishers and the link to blueye_handler;
    returns the link."""
    global _control_node, _state_pub, _goal_pub
    if not rclpy.ok():
        # rclpy must not shut down on SIGINT (the tree stopping the script): pausing the mission still needs it
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    _control_node = rclpy.create_node('mission_planner_control')

    def _cb(msg: String):
        if msg.data == "pause":
            _pause_event.set()
        elif msg.data == "resume":
            _resume_event.set()
        elif msg.data == "stop":
            _stop_event.set()

    def _pose_cb(msg: Odometry):
        global _pose
        _pose = (time.time(), msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _latlon_cb(msg: Point):
        global _latlon
        _latlon = (time.time(), msg.x, msg.y)

    _control_node.create_subscription(String, '/blueye/mission/control', _cb, 10)
    _control_node.create_subscription(Odometry, '/blueye/sensor/ekf/pose', _pose_cb, 10)
    _control_node.create_subscription(Point, '/blueye/sensor/ekf/lat_long', _latlon_cb, 10)
    _state_pub = _control_node.create_publisher(Int32, '/mission_state', 10)
    _goal_pub = _control_node.create_publisher(PointStamped, '/blueye/waypoint_goal', 10)
    link = DroneLink(_control_node, logger)
    global _control_thread
    _control_thread = threading.Thread(target=_spin_control, daemon=True)
    _control_thread.start()
    return link


def _spin_control():
    try:
        rclpy.spin(_control_node)
    except ExternalShutdownException:
        pass


def _shutdown_control_channel():
    """Stop the ROS spin thread before the process ends."""
    if rclpy.ok():
        rclpy.shutdown()
    if _control_thread is not None:
        _control_thread.join(timeout=2.0)


def _publish_state(state: int):
    """Publish mission state integer to /mission_state."""
    if _state_pub is not None:
        msg = Int32()
        msg.data = state
        _state_pub.publish(msg)


def _local_from_latlon(lat, lon):
    """(north, east) of a lat/long in the frame of the drone's position estimate, or None without a recent pair.

    Anchored on the latest estimate and lat/long, which the handler publishes from the same telemetry message,
    so the origin of the estimate (set by the position reset) need not be known. Flat earth: fine over the
    hundred metres of a mission."""
    if _pose is None or _latlon is None or abs(_pose[0] - _latlon[0]) > 0.5 or time.time() - _pose[0] > 2.0:
        return None
    _, lat0, lon0 = _latlon
    m_per_deg_lat, m_per_deg_lon = mission_sites.metres_per_degree(lat0)
    return _pose[1] + (lat - lat0) * m_per_deg_lat, _pose[2] + (lon - lon0) * m_per_deg_lon


def _mission_layout(mission, phases):
    """Instruction ids in order, their phases, and all waypoints in order as (instruction id, waypoint)."""
    ids = [i.id for i in mission.instructions]
    waypoints = [(i.id, i.waypoint_command.waypoint) for i in mission.instructions if "waypoint_command" in i]
    return {"ids": ids, "phases": phases, "waypoints": waypoints}


def _progress(completed, layout):
    """(phase, active waypoint as (instruction id, waypoint) or None) from the completed instruction ids.

    A waypoint is active once every instruction before it is done, so its leg starts where the drone sets off
    for it (at depth after the descent, after the wait at the previous waypoint)."""
    done = set(completed)
    phase = next((layout["phases"][i] for i in layout["ids"] if i not in done), HOMING)
    pending = next(((i, w) for i, w in layout["waypoints"] if i not in done), None)
    if pending is not None and not all(j in done for j in layout["ids"][:layout["ids"].index(pending[0])]):
        pending = None
    return phase, pending


def _publish_goal(waypoint, logger):
    """Publish a waypoint as the network's goal; False if its local position cannot be computed yet."""
    local = _local_from_latlon(waypoint.global_position.latitude, waypoint.global_position.longitude)
    if local is None or _goal_pub is None:
        return False
    msg = PointStamped()
    msg.header.stamp = _control_node.get_clock().now().to_msg()
    msg.header.frame_id = "odom"
    msg.point.x, msg.point.y = local
    msg.point.z = float(waypoint.depth_set_point.depth)
    _goal_pub.publish(msg)
    logger.info(f"Goal: {waypoint.name} at north {local[0]:.1f} m, east {local[1]:.1f} m, depth {msg.point.z:.1f} m")
    return True


# From the dock at the mission start: the undocking reverses about 1.2 m. Less is too short a baseline for the dock's
# axis; more means the reference is not from this docking (or the drone drifted far)
AXIS_MIN_M, AXIS_MAX_M = 0.5, 3.0
# TEMPORARY for the pier tests (2026-10-25, the user's request): missions start up to this far from the dock (was 3 m)
MAX_START_DISTANCE_M = 50.0


def setup_logging(log_file="log/mission.log", log_level="INFO"):
    """Set up logging configuration."""
    # Create log directory if needed
    log_path = Path(log_file)
    log_path.parent.mkdir(exist_ok=True, parents=True)
    
    # Set up logging level
    level = getattr(logging, log_level.upper())
    
    # Configure logging
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler(sys.stdout)
        ]
    )
    
    return logging.getLogger("pool_test")


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Run a pipeline inspection and docking mission with position reset')
    
    # Drone settings
    parser.add_argument('--drone-ip', type=str, default="192.168.1.101",
                        help='IP address of the Blueye drone')
    parser.add_argument('--timeout', type=int, default=30,
                        help='Connection timeout for the drone in seconds')
    
    parser.add_argument('--site', choices=sorted(mission_sites.SITES), default=mission_sites.DEFAULT_SITE,
                        help='Site layout in mission_sites.py')

    # Mission retry settings
    parser.add_argument('--max-retries', type=int, default=5,
                        help='Maximum number of retry attempts for aborted missions')
    
    # Parse arguments
    return parser.parse_args()


def create_mission(site_name, out_bearing_deg, logger):
    """The mission of a site layout (mission_sites.py), placed along the dock's axis (bearing out of the dock), and
    the phase of each of its instructions by id."""
    site = mission_sites.SITES[site_name]
    logger.info(f"Creating the {site_name} survey (out of the dock: {out_bearing_deg:.0f} deg)")
    instructions, phases = [], {}

    def add(phase, **command):
        instruction = bp.Instruction(id=len(instructions) + 1, auto_continue=True, **command)
        instructions.append(instruction)
        phases[instruction.id] = phase
        return instruction

    def depth_set_point(depth):
        return bp.DepthSetPoint(depth=depth, speed_to_depth=0.5,
                                depth_zero_reference=DepthZeroReference.DEPTH_ZERO_REFERENCE_SURFACE)

    # auto depth and heading, then the depth of the first waypoint before setting off
    add(TRANSIT, control_mode_command=bp.ControlModeCommand(
        control_mode_vertical=bp.ControlModeVertical.CONTROL_MODE_VERTICAL_AUTO_DEPTH,
        control_mode_horizontal=bp.ControlModeHorizontal.CONTROL_MODE_HORIZONTAL_AUTO_HEADING))
    add(TRANSIT, depth_set_point_command=bp.DepthSetPointCommand(
        depth_set_point=depth_set_point(site["waypoints"][0][3])))

    for name, forward, right, depth, phase, inspect in site["waypoints"]:
        lat, lon = mission_sites.dock_frame_to_latlon(site, forward, right, out_bearing_deg)
        add(PHASES[phase], waypoint_command=bp.WaypointCommand(waypoint=bp.Waypoint(
            id=len(instructions) + 1,
            name=name,
            global_position=LatLongPosition(latitude=lat, longitude=lon),
            circle_of_acceptance=site["circle_of_acceptance"],
            speed_to_target=site["speed"],
            depth_set_point=depth_set_point(depth))))
        logger.info(f"Added waypoint {name}: {forward:.1f} m out, {right:+.1f} m right of the dock, depth {depth} m "
                    f"({lat:.7f}, {lon:.7f}), {phase}")
        if inspect:
            add(INSPECTION, wait_for_command=bp.WaitForCommand(wait_for_seconds=site["wait_s"]))

    mission = bp.Mission(
        id=1,
        name=f"{site_name} survey",
        instructions=instructions,
        default_surge_speed=site["speed"],
        default_heave_speed=0.5,
        default_circle_of_acceptance=site["circle_of_acceptance"],
    )
    return mission, phases

def save_mission_to_json(mission, filename="mission.json", logger=None):
    """Save the mission to a JSON file for cross-checking."""
    try:
        # Convert mission to dictionary format
        mission_dict = {
            "id": mission.id,
            "name": mission.name,
            "default_surge_speed": mission.default_surge_speed,
            "default_heave_speed": mission.default_heave_speed,
            "default_circle_of_acceptance": mission.default_circle_of_acceptance,
            "instructions": []
        }
        
        # Convert each instruction to dictionary
        for instruction in mission.instructions:
            instr_dict = {
                "id": instruction.id,
                "auto_continue": instruction.auto_continue
            }
            
            # Handle different instruction types
            if instruction.control_mode_command:
                instr_dict["type"] = "control_mode"
                instr_dict["control_mode_vertical"] = instruction.control_mode_command.control_mode_vertical.name
                instr_dict["control_mode_horizontal"] = instruction.control_mode_command.control_mode_horizontal.name
                
            elif instruction.depth_set_point_command:
                instr_dict["type"] = "depth_set_point"
                instr_dict["depth"] = instruction.depth_set_point_command.depth_set_point.depth
                instr_dict["speed_to_depth"] = instruction.depth_set_point_command.depth_set_point.speed_to_depth
                instr_dict["depth_zero_reference"] = instruction.depth_set_point_command.depth_set_point.depth_zero_reference.name
                
            elif instruction.waypoint_command:
                instr_dict["type"] = "waypoint"
                waypoint = instruction.waypoint_command.waypoint
                instr_dict["name"] = waypoint.name
                instr_dict["latitude"] = waypoint.global_position.latitude
                instr_dict["longitude"] = waypoint.global_position.longitude
                instr_dict["circle_of_acceptance"] = waypoint.circle_of_acceptance
                instr_dict["speed_to_target"] = waypoint.speed_to_target
                instr_dict["depth"] = waypoint.depth_set_point.depth
                
            elif instruction.wait_for_command:
                instr_dict["type"] = "wait"
                instr_dict["wait_for_seconds"] = instruction.wait_for_command.wait_for_seconds
            
            mission_dict["instructions"].append(instr_dict)
        
        # Save to JSON file
        with open(filename, 'w') as f:
            json.dump(mission_dict, f, indent=2)
        
        if logger:
            logger.info(f"Mission saved to {filename}")
        else:
            print(f"Mission saved to {filename}")
            
        return True
        
    except Exception as e:
        error_msg = f"Failed to save mission to JSON: {str(e)}"
        if logger:
            logger.error(error_msg)
        else:
            print(error_msg)
        return False


RETRY_DELAY_S = 3.0     # after an abort: before run() again, and before polling the status after it
START_RETRIES = 3       # FAILED_TO_START_MISSION: run() again once the position estimate is valid


def _state(status):
    return status["state"].replace("MISSION_STATE_", "") if status else "NO STATUS"


def run_mission_with_instant_resume(link, mission, phases, max_duration, logger, max_retries=10):
    """Run the mission and retry if aborted. The drone needs a moment to take the run command: polling at once
    reads the same abort again, which used up every retry within milliseconds."""
    start_time = time.time()
    retry_count = 0
    start_retries = 0
    layout = _mission_layout(mission, phases)
    last_phase = last_goal_id = None    # what was last published for the takeover network
    loaded = completed = False

    def command(name):
        ok, message = link.call(f"/blueye/mission/{name}")
        if not ok:
            raise RuntimeError(message)
        time.sleep(1.0)                 # the status topic (2 Hz) shows the result from here on
        return ok

    try:
        logger.info(f"Previous mission state: {_state(link.mission_status())}")
        logger.info(f"Sending new mission: {mission.name}")
        ok, message = link.load_mission(mission)
        if not ok:
            logger.error(f"Loading the mission failed: {message}")
            return False
        logger.info(message)
        loaded = True

        logger.info("Waiting for a steady valid position estimate before starting")
        if not link.wait_for_valid_position():
            return False

        # Main mission execution loop with retries
        while retry_count <= max_retries:
            if retry_count == 0:
                logger.info("Starting mission execution")
            else:
                logger.info(f"Resuming mission execution (attempt {retry_count + 1}/{max_retries + 1})")
            command("run")              # continues from where it stopped
            if retry_count > 0:
                time.sleep(RETRY_DELAY_S)

            # Monitor mission progress
            while True:
                # BT sent "stop" (REJECT path) — abort immediately
                if _stop_event.is_set():
                    logger.info("Stop signal received — terminating mission script")
                    return False

                # BT sent "pause" (takeover ACCEPT path) — freeze, wait for resume
                if _pause_event.is_set():
                    _pause_event.clear()
                    logger.info("Pause signal received — pausing mission (the drone holds position)")
                    command("pause")
                    logger.info("Mission paused. Waiting for resume signal...")
                    while not _resume_event.wait(timeout=1.0):
                        if _stop_event.is_set():
                            logger.info("Stop received while paused — terminating")
                            return False
                    _resume_event.clear()
                    logger.info("Resume signal received — resuming mission")
                    command("run")
                    # the tree published other phases during the takeover: send phase and goal again (the
                    # network starts a new leg from where the vehicle is, as after a hand-back in simulation)
                    last_phase = last_goal_id = None
                    continue

                # Check if maximum duration exceeded
                elapsed = time.time() - start_time
                if elapsed > max_duration:
                    logger.warning(f"Mission timeout after {elapsed:.1f} seconds")
                    return False

                status = link.mission_status()
                if status is None:
                    logger.warning("No mission status from blueye_handler")
                    time.sleep(2)
                    continue

                # Phase and active waypoint for the takeover network, published when they change
                phase, active = _progress(status["completed"], layout)
                if phase != last_phase:
                    _publish_state(phase)
                    logger.info(f"Phase: {PHASE_NAMES[phase]} ({phase})")
                    last_phase = phase
                if active is not None and active[0] != last_goal_id and _publish_goal(active[1], logger):
                    last_goal_id = active[0]

                state = _state(status)
                depth = f", Depth: {link.pose[2]:.1f}m" if link.pose else ""
                logger.info(f"Mission: {state}, Progress: {len(status['completed'])}/{status['total']} instructions, "
                            f"Time: {status.get('time_elapsed', 0)}s/"
                            f"{status.get('time_elapsed', 0) + status.get('eta', 0)}s{depth}")

                if state == "COMPLETED":
                    logger.info("Mission completed successfully")
                    completed = True
                    return True
                elif state == "ABORTED":
                    logger.warning(f"Mission was aborted (attempt {retry_count + 1}/{max_retries + 1})")
                    if retry_count < max_retries:
                        retry_count += 1
                        time.sleep(RETRY_DELAY_S)
                        break  # Break out of the inner loop to retry run
                    logger.warning(f"Maximum retry attempts ({max_retries}) reached. Mission failed.")
                    return False
                elif state == "FAILED_TO_START_MISSION" and start_retries < START_RETRIES:
                    start_retries += 1
                    logger.warning(f"Mission failed to start (retry {start_retries}/{START_RETRIES} once the "
                                   "position estimate is valid)")
                    if not link.wait_for_valid_position():
                        return False
                    command("run")
                    time.sleep(RETRY_DELAY_S)
                    continue
                elif state in ("FAILED_TO_LOAD_MISSION", "FAILED_TO_START_MISSION"):
                    logger.error(f"Mission failed with state: {state}")
                    return False

                time.sleep(2)  # Poll every 2 seconds

    except Exception as e:
        logger.error(f"Error during mission execution: {str(e)}")
        return False
    finally:
        # The handler keeps the drone after this script ends, so a mission left running would carry on without the
        # tree (before, the script's own connection closing stopped it): pause it, and the drone holds position
        if loaded and not completed:
            ok, message = link.call("/blueye/mission/pause", timeout_s=5.0)
            logger.info(f"Mission left paused, the drone holds position ({message})" if ok
                        else f"Pausing the mission on exit failed: {message}")


def main():
    """Main function."""
    # Parse command line arguments
    args = parse_arguments()
    
    # Set up logging
    logger = setup_logging()
    logger.info(f"Starting the {args.site} survey mission")

    # ROS control channel (pause/resume/stop from the BT) and the link to blueye_handler
    link = _setup_control_channel(logger)
    logger.info("Mission control channel ready on /blueye/mission/control")

    success = False

    try:
        reference = mission_sites.load_reference(args.site)
    except ValueError as exc:
        logger.error(f"Not starting: {exc}")
        return 1
    logger.info(f"Dock reference of {reference['time']}")
    
    try:
        offset = link.offset_from_dock(reference["dock"])
        if offset is None or offset[0] > MAX_START_DISTANCE_M:
            logger.error("Not starting: the drone's position estimate is "
                         + ("missing" if offset is None else f"{offset[0]:.1f} m from the dock")
                         + f" (at most {MAX_START_DISTANCE_M:.0f} m): run reset_position.py with the drone docked, "
                         "then the tree")
            return 1
        distance, out_bearing = offset
        compass = reference.get("docked_compass_heading_deg")
        if AXIS_MIN_M <= distance <= AXIS_MAX_M:
            logger.info(f"Undocked {distance:.1f} m toward {out_bearing:.0f} deg: dock axis"
                        + (f" (docked compass heading {compass:.0f} deg suggested {(compass + 180) % 360:.0f})"
                           if compass is not None else ""))
        else:
            # the direction to the drone says nothing about the dock here (at 16:31 it pointed into the pier)
            nominal = mission_sites.SITES[args.site].get("nominal_out_bearing_deg")
            if compass is not None:
                out_bearing, source = (compass + 180) % 360, f"the docked compass heading {compass:.0f} deg reversed"
            elif nominal is not None:
                out_bearing, source = nominal, "the site's nominal bearing"
            else:
                source = "the direction to the drone (no other)"
            logger.warning(f"The drone is {distance:.1f} m from the dock, not {AXIS_MIN_M}-{AXIS_MAX_M:.0f} m after "
                           f"an undocking: the route is placed around the dock coordinate in the drone's own position "
                           f"estimate, which may be off by that much. Dock axis {out_bearing:.0f} deg, from {source}")
        
        mission_sites.save_out_bearing(args.site, out_bearing)    # for the return to the stop point on a reject

        # Create the mission
        mission, phases = create_mission(args.site, out_bearing, logger)
        
        save_mission_to_json(mission, "mission.json", logger)
        
        # Run the mission with instant auto-resume
        success = run_mission_with_instant_resume(link, mission, phases, 1800, logger, args.max_retries)
        
        logger.info(f"Mission {'completed successfully' if success else 'failed'}")
        return 0 if success else 1
        
    except KeyboardInterrupt:
        logger.info("Mission aborted by user")
        return 130  # Standard exit code for SIGINT
        
    except Exception as e:
        logger.error(f"Unexpected error: {str(e)}", exc_info=True)
        return 1
        
    finally:
        return 0 if success else 1


if __name__ == "__main__":
    code = main()
    # End without the interpreter's teardown: with the SDK's and the ROS threads still alive it aborted the process
    # after a finished run (exit code 134 instead of 0 or 1, 2026-10-22)
    _shutdown_control_channel()
    logging.shutdown()
    sys.stdout.flush()
    os._exit(code)