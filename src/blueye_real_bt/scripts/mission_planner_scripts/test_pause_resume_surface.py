#!/usr/bin/env python3
"""
Full pause/resume cycle test:
  1. Mission planner dives to --depth (default 2.0 m)
  2. Pauses MID-DIVE (after 5 s of RUNNING) — mission is still RUNNING, not COMPLETED
  3. Holds paused for --pause-secs (default 20 s) — user tries joystick
  4. Resumes the dive to --depth
  5. Waits for depth reached, then sends new mission: DepthSetPoint(0.2 m) to resurface

Run alongside (in separate terminals):
  ros2 run blueye_handler blueye_handler
  ros2 run joy joy_node
  ros2 run blueye_joystick_cpp joystick_controller

Record bag (in another terminal BEFORE running this script):
  ros2 bag record /blueye/commands /joy /blueye/sensor/depth /blueye/sensor/imu /blueye/sensor/compass \
    -o ~/rosbags/pause_resume_test_$(date +%Y%m%d_%H%M%S)
"""

import argparse, sys, time, logging
import blueye.protocol as bp
from blueye.sdk import Drone
from blueye.protocol.types.mission_planning import DepthZeroReference


def log_setup():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(message)s",
        handlers=[logging.StreamHandler(sys.stdout)],
    )
    return logging.getLogger("pause_resume_test")


def wait_for_state(drone, target, timeout=20.0, log=None):
    name = target.name if hasattr(target, "name") else str(target)
    deadline = time.time() + timeout
    while time.time() < deadline:
        s = drone.mission.get_status()
        if s and s.state == target:
            if log:
                log.info(f"Mission reached {name}")
            return True
        time.sleep(0.2)
    if log:
        log.error(f"Timeout waiting for {name}")
    return False


def depth_setpoint_mission(depth, speed=0.3):
    return bp.Mission(instructions=[bp.Instruction(
        depth_set_point_command=bp.DepthSetPointCommand(
            depth_set_point=bp.DepthSetPoint(
                depth=depth,
                speed_to_depth=speed,
                depth_zero_reference=DepthZeroReference.DEPTH_ZERO_REFERENCE_SURFACE,
            )
        )
    )])


def run_mission(drone, mission, log):
    drone.mission.clear()
    time.sleep(0.5)
    drone.mission.send_new(mission)
    time.sleep(0.3)
    drone.mission.run()


def log_modes(drone, log, tag=""):
    log.info(f"  [{tag}] depth={drone.depth:.3f} m  "
             f"auto_depth={drone.motion.auto_depth_active}  "
             f"auto_heading={drone.motion.auto_heading_active}  "
             f"station_keeping={drone.motion.station_keeping_active}")


def joystick_window(drone, log, duration, label):
    log.info("")
    log.info(f">>> {label} — {duration:.0f} s window — push joystick now!")
    log.info("    Depth logged every second. Watch for changes.")
    log.info("")
    t0 = time.time()
    depths = []
    while time.time() - t0 < duration:
        d = drone.depth
        depths.append(d)
        s = drone.mission.get_status()
        state = s.state.name if s else "unknown"
        log.info(f"  t={time.time()-t0:4.1f}s  depth={d:.3f} m  state={state}")
        time.sleep(1.0)
    return depths


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--drone-ip",    default="192.168.1.101")
    ap.add_argument("--depth",       type=float, default=2.0)
    ap.add_argument("--dive-pause-after", type=float, default=5.0,
                    help="Seconds after RUNNING before pausing (mid-dive)")
    ap.add_argument("--pause-secs",  type=float, default=20.0)
    ap.add_argument("--surface-depth", type=float, default=0.2)
    args = ap.parse_args()

    log = log_setup()
    log.info(f"Target dive depth : {args.depth} m")
    log.info(f"Pause mid-dive after : {args.dive_pause_after} s of RUNNING")
    log.info(f"Pause duration : {args.pause_secs} s")
    log.info(f"Surface target : {args.surface_depth} m")
    log.info("")

    log.info("Connecting...")
    drone = Drone(ip=args.drone_ip, auto_connect=True, timeout=30)
    log.info(f"Connected: {drone.serial_number}  SW: {drone.software_version}")
    if not drone.in_control:
        drone.take_control()

    try:
        # ── PHASE 1: Dive ─────────────────────────────────────────────────────
        log.info(f"=== PHASE 1: Diving to {args.depth} m ===")
        run_mission(drone, depth_setpoint_mission(args.depth), log)

        if not wait_for_state(drone, bp.MissionState.MISSION_STATE_RUNNING, timeout=10, log=log):
            log.error("Mission never reached RUNNING — aborting")
            return 1

        log.info(f"Mission RUNNING — waiting {args.dive_pause_after} s before pausing...")
        for i in range(int(args.dive_pause_after)):
            log.info(f"  t={i+1}s  depth={drone.depth:.3f} m")
            time.sleep(1.0)

        # ── PHASE 2: Pause ────────────────────────────────────────────────────
        log.info("")
        log.info("=== PHASE 2: Pausing mission MID-DIVE ===")
        drone.mission.pause()
        time.sleep(0.5)

        s = drone.mission.get_status()
        state_after_pause = s.state.name if s else "unknown"
        log.info(f"Mission state after pause(): {state_after_pause}")
        log_modes(drone, log, tag="PAUSED")

        depth_at_pause = drone.depth
        depths_during_pause = joystick_window(drone, log, args.pause_secs, state_after_pause)

        max_delta = max(abs(d - depth_at_pause) for d in depths_during_pause)
        log.info(f"During pause: depth range {min(depths_during_pause):.3f}–{max(depths_during_pause):.3f} m  "
                 f"max Δ={max_delta:.3f} m")
        if max_delta > 0.15:
            log.info(">>> DRONE MOVED during pause (joystick or buoyancy affected depth)")
        else:
            log.info(">>> DRONE DID NOT MOVE during pause")

        # ── PHASE 3: Resume ───────────────────────────────────────────────────
        log.info("")
        log.info("=== PHASE 3: Resuming mission ===")
        try:
            drone.mission.run()
            log.info("Resume command sent")
        except Exception as e:
            log.warning(f"Resume failed ({e}) — mission may have been COMPLETED already")

        DONE = {bp.MissionState.MISSION_STATE_COMPLETED, bp.MissionState.MISSION_STATE_ABORTED}
        log.info(f"Waiting for drone to reach {args.depth} m ...")
        deadline = time.time() + 60
        while time.time() < deadline:
            s = drone.mission.get_status()
            d = drone.depth
            state = s.state.name if s else "unknown"
            log.info(f"  depth={d:.3f} m  state={state}")
            if s and s.state in DONE:
                log.info(f"Mission {state} at depth {d:.3f} m")
                break
            time.sleep(1.5)

        # ── PHASE 4: Resurface ────────────────────────────────────────────────
        log.info("")
        log.info(f"=== PHASE 4: Resurfacing to {args.surface_depth} m ===")
        run_mission(drone, depth_setpoint_mission(args.surface_depth, speed=0.2), log)

        if not wait_for_state(drone, bp.MissionState.MISSION_STATE_RUNNING, timeout=10, log=log):
            log.warning("Surface mission didn't reach RUNNING")

        deadline = time.time() + 120
        while time.time() < deadline:
            d = drone.depth
            s = drone.mission.get_status()
            state = s.state.name if s else "unknown"
            log.info(f"  depth={d:.3f} m  state={state}")
            if d <= args.surface_depth + 0.1:
                log.info(f"Surfaced to {d:.3f} m")
                break
            if s and s.state in DONE:
                log.info(f"Surface mission {state} at depth {d:.3f} m")
                break
            time.sleep(1.5)

        # ── Final summary ──────────────────────────────────────────────────────
        log.info("")
        log.info("=== FINAL SUMMARY ===")
        log.info(f"Paused in state       : {state_after_pause}")
        log.info(f"Depth at pause        : {depth_at_pause:.3f} m")
        log.info(f"Max depth Δ in pause  : {max_delta:.3f} m")
        if state_after_pause == "MISSION_STATE_PAUSED":
            if max_delta > 0.15:
                log.info("RESULT: Joystick / buoyancy moved drone during MISSION_STATE_PAUSED")
                log.info("        → Check /blueye/commands bag to see if it was joystick input")
                log.info("          or just positive buoyancy (monotonic upward drift)")
            else:
                log.info("RESULT: Drone held position during MISSION_STATE_PAUSED")
                log.info("        → Firmware blocked all motion commands")
        else:
            log.info(f"NOTE: Mission was {state_after_pause} when pause was called (not PAUSED)")
            log.info("      Joystick behaviour observed in COMPLETED state (not PAUSED)")
        return 0

    except KeyboardInterrupt:
        log.info("Interrupted")
        return 130
    finally:
        log.info("Cleanup...")
        try:
            drone.motion.surge = 0.0
            drone.motion.heave = 0.0
            drone.mission.clear()
            drone.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
