#!/usr/bin/env python3
"""
Test: can the drone accept motion commands while a mission is PAUSED?

What this script does:
  1. Connects to the drone
  2. Loads a minimal 1-instruction mission (DepthSetPoint to 0.5 m)
  3. Runs the mission — waits for MISSION_STATE_RUNNING
  4. Waits 5 s so the mission autopilot is clearly active
  5. Calls drone.mission.pause() — waits for MISSION_STATE_PAUSED
  6. Logs the control modes (auto_depth, auto_heading, station_keeping)
  7. Sends drone.motion.heave = -0.3 for 4 s, logging depth every 0.5 s
     → if depth increases the firmware accepted the command
     → if depth is unchanged (or only drifts due to buoyancy) it rejected it
  8. Stops motion, logs final state, resumes then aborts the mission

Run:
  python3 test_motion_during_pause.py [--drone-ip 192.168.1.101]

Record a bag alongside to confirm with /blueye/commands:
  ros2 bag record /blueye/commands -o motion_pause_test
"""

import argparse
import sys
import time
import logging
import blueye.protocol as bp
from blueye.sdk import Drone
from blueye.protocol.types.mission_planning import DepthZeroReference


def setup_logging():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s",
        handlers=[logging.StreamHandler(sys.stdout)],
    )
    return logging.getLogger("pause_motion_test")


def wait_for_state(drone, target_state, timeout=15.0, logger=None):
    """Block until the mission reaches target_state or timeout expires."""
    state_name = target_state.name if hasattr(target_state, "name") else str(target_state)
    deadline = time.time() + timeout
    while time.time() < deadline:
        status = drone.mission.get_status()
        if status and status.state == target_state:
            if logger:
                logger.info(f"Mission reached {state_name}")
            return True
        time.sleep(0.2)
    if logger:
        logger.error(f"Timed out waiting for {state_name}")
    return False


def log_control_modes(drone, logger, label=""):
    """Log all active control modes."""
    prefix = f"[{label}] " if label else ""
    logger.info(f"{prefix}auto_depth={drone.motion.auto_depth_active}  "
                f"auto_heading={drone.motion.auto_heading_active}  "
                f"station_keeping={drone.motion.station_keeping_active}")


def log_mission_state(drone, logger, label=""):
    """Log current mission state."""
    prefix = f"[{label}] " if label else ""
    status = drone.mission.get_status()
    if status:
        state_name = status.state.name if hasattr(status.state, "name") else str(status.state)
        completed = len(status.completed_instruction_ids)
        total     = status.total_number_of_instructions
        logger.info(f"{prefix}mission_state={state_name}  "
                    f"completed={completed}/{total}  "
                    f"elapsed={status.time_elapsed}s")
    else:
        logger.warning(f"{prefix}no mission status available")


def main():
    parser = argparse.ArgumentParser(description="Test motion commands during mission pause")
    parser.add_argument("--drone-ip", default="192.168.1.101")
    parser.add_argument("--depth",    type=float, default=0.5,
                        help="Target depth for the test mission (m)")
    parser.add_argument("--heave",    type=float, default=-0.3,
                        help="Heave command to send during pause (negative = down)")
    parser.add_argument("--test-duration", type=float, default=4.0,
                        help="How long to apply the motion command (s)")
    args = parser.parse_args()

    logger = setup_logging()
    logger.info("=== Motion-during-pause test ===")
    logger.info(f"Drone IP:      {args.drone_ip}")
    logger.info(f"Mission depth: {args.depth} m")
    logger.info(f"Heave command: {args.heave}  (negative = downward)")
    logger.info(f"Test duration: {args.test_duration} s")

    # ── 1. Connect ───────────────────────────────────────────────────────────
    logger.info("Connecting to drone...")
    drone = Drone(ip=args.drone_ip, auto_connect=True, timeout=30)
    logger.info(f"Connected: {drone.serial_number}  SW: {drone.software_version}")
    if not drone.in_control:
        drone.take_control()
        logger.info("Control acquired")

    try:
        # ── 2. Build a minimal mission ────────────────────────────────────────
        depth_sp = bp.DepthSetPoint(
            depth=args.depth,
            speed_to_depth=0.3,
            depth_zero_reference=DepthZeroReference.DEPTH_ZERO_REFERENCE_SURFACE,
        )
        instruction = bp.Instruction(
            depth_set_point_command=bp.DepthSetPointCommand(depth_set_point=depth_sp)
        )
        mission = bp.Mission(instructions=[instruction])

        # ── 3. Load & run ─────────────────────────────────────────────────────
        logger.info("Clearing any previous mission...")
        drone.mission.clear()
        time.sleep(1.0)

        logger.info("Sending mission (1 instruction: DepthSetPoint %.1f m)..." % args.depth)
        drone.mission.send_new(mission)
        time.sleep(0.5)

        logger.info("Running mission...")
        drone.mission.run()

        if not wait_for_state(drone, bp.MissionState.MISSION_STATE_RUNNING, timeout=10, logger=logger):
            logger.error("Mission did not reach RUNNING — aborting test")
            return 1

        log_control_modes(drone, logger, "RUNNING")
        log_mission_state(drone, logger, "RUNNING")
        logger.info("Waiting 5 s with mission active...")
        for i in range(5):
            time.sleep(1)
            try:
                logger.info(f"  t={i+1}s  depth={drone.depth:.3f} m")
            except Exception:
                pass

        # ── 4. Pause ──────────────────────────────────────────────────────────
        logger.info(">>> Pausing mission...")
        drone.mission.pause()

        if not wait_for_state(drone, bp.MissionState.MISSION_STATE_PAUSED, timeout=5, logger=logger):
            logger.error("Mission did not reach PAUSED — aborting test")
            return 1

        log_mission_state(drone, logger, "PAUSED")
        log_control_modes(drone, logger, "PAUSED")

        depth_at_pause = drone.depth
        logger.info(f"Depth at pause: {depth_at_pause:.3f} m")

        # ── 5. Send motion command ────────────────────────────────────────────
        logger.info(f">>> Sending heave={args.heave} for {args.test_duration} s...")
        logger.info("    Watch depth: if it changes → firmware accepted the command")
        logger.info("    If depth only drifts slowly (buoyancy) → command was rejected")

        start = time.time()
        last_log = start
        while time.time() - start < args.test_duration:
            drone.motion.heave = args.heave      # re-send every 100 ms (watchdog)
            now = time.time()
            if now - last_log >= 0.5:
                try:
                    delta = drone.depth - depth_at_pause
                    logger.info(f"  t={now-start:.1f}s  depth={drone.depth:.3f} m  "
                                f"Δ={delta:+.3f} m")
                except Exception:
                    logger.info(f"  t={now-start:.1f}s  (depth unavailable)")
                last_log = now
            time.sleep(0.1)

        # ── 6. Stop motion ────────────────────────────────────────────────────
        logger.info(">>> Stopping motion...")
        drone.motion.heave = 0.0
        drone.motion.surge = 0.0

        depth_after_phase1 = drone.depth
        delta_phase1 = depth_after_phase1 - depth_at_pause
        logger.info(f"Depth at pause:        {depth_at_pause:.3f} m")
        logger.info(f"Depth after phase 1:   {depth_after_phase1:.3f} m")
        logger.info(f"Phase 1 Δ depth:       {delta_phase1:+.3f} m")

        phase1_accepted = delta_phase1 > 0.10
        if phase1_accepted:
            logger.info("Phase 1: MOTION ACCEPTED (depth increased with downward heave)")
        else:
            logger.info("Phase 1: MOTION REJECTED — station_keeping likely overriding commands")

        # ── 7. Disable station keeping and retry ──────────────────────────────
        logger.info("")
        logger.info(">>> Phase 2: disabling station_keeping and retrying motion command...")
        drone.motion.station_keeping_active = False
        time.sleep(0.5)
        log_control_modes(drone, logger, "SK-OFF")

        depth_before_phase2 = drone.depth
        logger.info(f"Depth before phase 2: {depth_before_phase2:.3f} m")
        logger.info(f"Sending heave={args.heave} for {args.test_duration} s (station_keeping OFF)...")

        start = time.time()
        last_log = start
        while time.time() - start < args.test_duration:
            drone.motion.heave = args.heave
            now = time.time()
            if now - last_log >= 0.5:
                try:
                    delta = drone.depth - depth_before_phase2
                    logger.info(f"  t={now-start:.1f}s  depth={drone.depth:.3f} m  Δ={delta:+.3f} m")
                except Exception:
                    logger.info(f"  t={now-start:.1f}s  (depth unavailable)")
                last_log = now
            time.sleep(0.1)

        drone.motion.heave = 0.0
        drone.motion.surge = 0.0
        depth_after_phase2 = drone.depth
        delta_phase2 = depth_after_phase2 - depth_before_phase2
        logger.info(f"Depth before phase 2: {depth_before_phase2:.3f} m")
        logger.info(f"Depth after phase 2:  {depth_after_phase2:.3f} m")
        logger.info(f"Phase 2 Δ depth:      {delta_phase2:+.3f} m")

        phase2_accepted = delta_phase2 > 0.10

        # ── 8. Final verdict ──────────────────────────────────────────────────
        logger.info("")
        logger.info("=== FINAL RESULT ===")
        logger.info(f"Phase 1 (station_keeping ON):  {'ACCEPTED' if phase1_accepted else 'REJECTED'}  Δ={delta_phase1:+.3f} m")
        logger.info(f"Phase 2 (station_keeping OFF): {'ACCEPTED' if phase2_accepted else 'REJECTED'}  Δ={delta_phase2:+.3f} m")
        logger.info("")
        if not phase1_accepted and phase2_accepted:
            logger.info("CONCLUSION: station_keeping was blocking joystick control.")
            logger.info("FIX: EnableJoystick must also disable station_keeping.")
            logger.info("     Motion commands work fine during MISSION_STATE_PAUSED once SK is off.")
        elif not phase1_accepted and not phase2_accepted:
            logger.info("CONCLUSION: firmware blocks ALL motion commands during MISSION_STATE_PAUSED.")
            logger.info("FIX: drone.mission.pause() cannot be used for operator takeover.")
            logger.info("     A different approach is needed (e.g. clear mission, re-upload on handback).")
        elif phase1_accepted:
            logger.info("CONCLUSION: motion commands work even with station_keeping ON during pause.")
            logger.info("Current design is fine — no changes needed to EnableJoystick.")

        # ── 9. Clean up ───────────────────────────────────────────────────────
        logger.info("")
        logger.info("Resuming mission briefly then clearing...")
        drone.mission.run()
        time.sleep(1.0)
        drone.mission.clear()
        logger.info("Test complete.")
        return 0

    except KeyboardInterrupt:
        logger.info("Test interrupted by user")
        return 130

    except Exception as e:
        logger.error(f"Unexpected error: {e}", exc_info=True)
        return 1

    finally:
        logger.info("Stopping all motion and disconnecting...")
        try:
            drone.motion.heave = 0.0
            drone.motion.surge = 0.0
            drone.motion.sway = 0.0
            drone.motion.yaw = 0.0
        except Exception:
            pass
        try:
            drone.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
