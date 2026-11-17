#!/usr/bin/env python3
"""
Undocking for the behaviour tree: asks blueye_handler (the only process connected to the drone) to run the undocking
sequence on /blueye/undock — station keeping, hold, a timed reverse out of the dock, then station keeping again, which
lasts until the mission starts because the handler keeps control.

Interrupted (the tree halts it on a takeover): asks /blueye/undock/abort, so the drone stops reversing and holds.

Usage: undocking.py [--reverse-duration S] [--reverse-power P]   (--drone-ip and --timeout are accepted and unused)
"""

import argparse
import logging
import sys
import threading
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions

from drone_link import DroneLink


def setup_logging(log_file="log/undocking.log"):
    """Set up logging to the terminal and to a file (kept, so a failed undocking can be traced afterwards)."""
    Path(log_file).parent.mkdir(exist_ok=True, parents=True)
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[logging.FileHandler(log_file), logging.StreamHandler(sys.stdout)]
    )
    return logging.getLogger("undocking")


def main():
    parser = argparse.ArgumentParser(description='Undocking through blueye_handler')
    parser.add_argument('--drone-ip', type=str, default="192.168.1.101", help='unused (the handler is connected)')
    parser.add_argument('--timeout', type=int, default=30, help='unused')
    parser.add_argument('--reverse-duration', type=float, default=6.0, help='Duration to move backwards in seconds')
    parser.add_argument('--reverse-power', type=float, default=0.4, help='Power for backwards movement (0.1 to 1.0)')
    args = parser.parse_args()
    if not 0.1 <= args.reverse_power <= 1.0:
        print("Error: reverse-power must be between 0.1 and 1.0")
        return 1

    logger = setup_logging()
    # rclpy must not shut down on Ctrl+C / SIGINT: the abort call below still needs it
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("undocking_client")
    link = DroneLink(node, logger)
    spin = threading.Thread(target=lambda: _spin(node), daemon=True)
    spin.start()
    try:
        logger.info(f"Undocking through blueye_handler: reverse at {args.reverse_power} for {args.reverse_duration} s")
        ok, message = link.call("/blueye/undock", {"power": args.reverse_power, "duration": args.reverse_duration},
                                timeout_s=args.reverse_duration + 40.0)
        logger.info(f"Undocking {'completed' if ok else 'failed'}: {message}")
        return 0 if ok else 1
    except KeyboardInterrupt:
        ok, message = link.call("/blueye/undock/abort", timeout_s=5.0)
        logger.info(f"Undocking interrupted: {message}")
        return 130
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
