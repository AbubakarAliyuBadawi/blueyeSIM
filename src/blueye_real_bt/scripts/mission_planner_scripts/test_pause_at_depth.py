#!/usr/bin/env python3
"""
Test: dive to target depth via mission planner, hold 20s, pause mission,
then wait 30s while you try the joystick — logs depth the whole time.

Run:
  python3 test_pause_at_depth.py [--depth 2.0]

Make sure blueye_handler + joy_node + joystick_controller are running.
"""
import argparse, sys, time, logging
import blueye.protocol as bp
from blueye.sdk import Drone
from blueye.protocol.types.mission_planning import DepthZeroReference


def log_setup():
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s - %(message)s",
                        handlers=[logging.StreamHandler(sys.stdout)])
    return logging.getLogger("pause_depth_test")


def wait_for_depth(drone, target, tolerance, logger):
    logger.info(f"Waiting for depth >= {target - tolerance:.1f} m ...")
    while True:
        d = drone.depth
        logger.info(f"  depth = {d:.3f} m  (target {target} m)")
        if d >= target - tolerance:
            return d
        time.sleep(1.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--drone-ip", default="192.168.1.101")
    ap.add_argument("--depth",    type=float, default=2.0,
                    help="Target depth in metres")
    ap.add_argument("--hold",     type=float, default=20.0,
                    help="Seconds to hold at depth before pausing")
    ap.add_argument("--test",     type=float, default=30.0,
                    help="Seconds to keep paused for joystick test")
    args = ap.parse_args()

    log = log_setup()
    log.info(f"Target depth : {args.depth} m")
    log.info(f"Hold before pause : {args.hold} s")
    log.info(f"Joystick test window : {args.test} s")

    log.info("Connecting...")
    drone = Drone(ip=args.drone_ip, auto_connect=True, timeout=30)
    log.info(f"Connected: {drone.serial_number}")
    if not drone.in_control:
        drone.take_control()

    try:
        # ── 1. Build a single DepthSetPoint mission ───────────────────────────
        drone.mission.clear()
        time.sleep(1.0)

        instr = bp.Instruction(
            depth_set_point_command=bp.DepthSetPointCommand(
                depth_set_point=bp.DepthSetPoint(
                    depth=args.depth,
                    speed_to_depth=0.3,
                    depth_zero_reference=DepthZeroReference.DEPTH_ZERO_REFERENCE_SURFACE,
                )
            )
        )
        drone.mission.send_new(bp.Mission(instructions=[instr]))
        time.sleep(0.5)

        log.info("Running mission — diving to %.1f m ..." % args.depth)
        drone.mission.run()

        # wait until we are at (or near) target depth
        wait_for_depth(drone, args.depth, tolerance=0.15, logger=log)
        at_depth = drone.depth
        log.info(f"At depth: {at_depth:.3f} m  — holding for {args.hold} s")

        # ── 2. Hold at depth (mission still RUNNING) ──────────────────────────
        t0 = time.time()
        while time.time() - t0 < args.hold:
            s = drone.mission.get_status()
            state = s.state.name if s else "unknown"
            log.info(f"  [HOLDING] depth={drone.depth:.3f} m  state={state}")
            time.sleep(2.0)

        # ── 3. Pause mission ──────────────────────────────────────────────────
        log.info(">>> Pausing mission now ...")
        drone.mission.pause()
        time.sleep(0.5)

        s = drone.mission.get_status()
        log.info(f"Mission state after pause: {s.state.name if s else 'unknown'}")
        log.info(f"auto_depth={drone.motion.auto_depth_active}  "
                 f"auto_heading={drone.motion.auto_heading_active}  "
                 f"station_keeping={drone.motion.station_keeping_active}")

        depth_at_pause = drone.depth
        log.info(f"Depth at pause: {depth_at_pause:.3f} m")
        log.info("")
        log.info(f">>> JOYSTICK TEST — {args.test} s window")
        log.info("    Push your joystick now and watch if the drone moves.")
        log.info("    Depth is logged every second.")
        log.info("")

        # ── 4. Joystick test window ───────────────────────────────────────────
        t0 = time.time()
        depths = []
        while time.time() - t0 < args.test:
            d = drone.depth
            depths.append(d)
            delta = d - depth_at_pause
            log.info(f"  t={time.time()-t0:4.1f}s  depth={d:.3f} m  Δ={delta:+.3f} m")
            time.sleep(1.0)

        # ── 5. Result ─────────────────────────────────────────────────────────
        max_delta = max(abs(d - depth_at_pause) for d in depths)
        log.info("")
        log.info("=== RESULT ===")
        log.info(f"Depth at pause : {depth_at_pause:.3f} m")
        log.info(f"Depth range    : {min(depths):.3f} – {max(depths):.3f} m")
        log.info(f"Max deviation  : {max_delta:.3f} m")
        if max_delta > 0.15:
            log.info("DRONE MOVED — motion commands accepted during MISSION_STATE_PAUSED")
        else:
            log.info("DRONE DID NOT MOVE — motion commands blocked during MISSION_STATE_PAUSED")
        log.info("(check physically whether you felt any response to the joystick)")

    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        log.info("Clearing mission and stopping motion ...")
        try:
            drone.motion.surge = 0.0
            drone.motion.heave = 0.0
            drone.mission.clear()
            drone.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
