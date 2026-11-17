#!/usr/bin/env python3
"""
Send DUNE a one-manoeuvre plan, as Neptus would (IMC PlanControl START with a PlanSpecification), or stop the plan.
For Marie in Stonefish flown by DUNE (lauv_marie_dune.launch.py); Neptus does the same and more.

Positions are metres north and east of the Stonefish origin, the reference the dune_bridge node gives DUNE.

  ros2 run lauv_marie_control dune_plan goto 100 50 --depth 5            # to the point at 5 m depth
  ros2 run lauv_marie_control dune_plan goto 100 50 --depth 5 --popup    # ... then surface there for a GPS fix
  ros2 run lauv_marie_control dune_plan popup 0 0 --duration 30          # to the point, surface, wait 30 s
  ros2 run lauv_marie_control dune_plan yoyo 150 0 --depth 10 --amplitude 5 --pitch 15
  ros2 run lauv_marie_control dune_plan loiter 0 0 --depth 5 --radius 20 --duration 120
  ros2 run lauv_marie_control dune_plan station 0 0 --radius 10 --duration 60      # keep station at the surface
  ros2 run lauv_marie_control dune_plan stop

Speeds in m/s (--speed, default 1.2), depths in m, angles in degrees. The plan state is printed until the plan ends
(Ctrl+C stops watching, not the plan). DUNE's answer to the request is printed by the bridge.
"""

import argparse
import math
import socket
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from lauv_marie_control import imc

SOURCE = 0x4D0A              # this tool's IMC address (a console address, not used by any DUNE)
WGS84_A, WGS84_E2 = 6378137.0, 6.69437999014e-3


def to_wgs84(origin_lat, origin_lon, north, east):
    """latitude and longitude [rad] of a point north and east [m] of the origin [deg] (ellipsoid radii at the origin)"""
    lat0 = math.radians(origin_lat)
    k = 1.0 - WGS84_E2 * math.sin(lat0) ** 2
    meridian, prime = WGS84_A * (1.0 - WGS84_E2) / k ** 1.5, WGS84_A / math.sqrt(k)
    return lat0 + north / meridian, math.radians(origin_lon) + east / (prime * math.cos(lat0))


def manoeuvre(args, lat, lon):
    common = {"lat": lat, "lon": lon, "speed": args.speed, "speed_units": 0, "z_units": 1}     # m/s; depth
    if args.kind == "goto":
        return "Goto", dict(common, timeout=args.timeout, z=args.depth)
    if args.kind == "popup":
        return "PopUp", popup(args, lat, lon)
    if args.kind == "yoyo":
        return "YoYo", dict(common, timeout=args.timeout, z=args.depth, amplitude=args.amplitude,
                            pitch=math.radians(args.pitch))
    if args.kind == "loiter":
        return "Loiter", dict(common, timeout=args.timeout, z=args.depth, duration=args.duration, type=1,
                              radius=args.radius, direction=1 if args.clockwise else 2)
    if args.kind == "station":
        return "StationKeeping", dict(common, z=0.0, radius=args.radius, duration=args.duration)
    raise ValueError(args.kind)


def popup(args, lat, lon):
    # wait at the surface for `duration` s (GPS fix), then the plan goes on
    return {"timeout": args.timeout, "lat": lat, "lon": lon, "z": args.depth if args.kind == "popup" else 0.0,
            "z_units": 1, "speed": args.speed, "speed_units": 0, "duration": args.duration, "radius": args.radius,
            "flags": 0x02}


class Watcher(Node):
    def __init__(self):
        super().__init__("lauv_marie_dune_plan")
        self.state = None
        self.create_subscription(String, "/lauv_marie/dune/plan_state", self._cb, 10)

    def _cb(self, msg):
        self.state = msg.data


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("kind", choices=["goto", "popup", "yoyo", "loiter", "station", "stop"])
    parser.add_argument("north", type=float, nargs="?", default=0.0, help="m north of the Stonefish origin")
    parser.add_argument("east", type=float, nargs="?", default=0.0, help="m east of the Stonefish origin")
    parser.add_argument("--depth", type=float, default=2.0, help="m (yoyo: the centre depth; popup: depth on the way)")
    parser.add_argument("--speed", type=float, default=1.2, help="m/s")
    parser.add_argument("--amplitude", type=float, default=3.0, help="yoyo: m above and below --depth")
    parser.add_argument("--pitch", type=float, default=15.0, help="yoyo: pitch angle, degrees")
    parser.add_argument("--radius", type=float, default=20.0, help="loiter, station, popup: m")
    parser.add_argument("--duration", type=int, default=60, help="loiter, station, popup: s")
    parser.add_argument("--clockwise", action="store_true", help="loiter: clockwise (default counter-clockwise)")
    parser.add_argument("--popup", action="store_true", help="after the manoeuvre, surface at its point and wait "
                        "--duration s")
    parser.add_argument("--timeout", type=int, default=1000, help="s, for the manoeuvre")
    parser.add_argument("--plan-id", default="", help="default: the manoeuvre's name")
    parser.add_argument("--origin", type=float, nargs=2, default=[63.441006, 10.351081], metavar=("LAT", "LON"),
                        help="the dune_bridge origin, degrees")
    parser.add_argument("--dune", default="127.0.0.1:6971", help="DUNE's IMC link for the bridge (host:port)")
    parser.add_argument("--dune-id", type=lambda x: int(x, 0), default=0x00D1, help="DUNE's IMC address "
                        "(lauv-simulator-1: 0x00D1)")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    host, port = args.dune.split(":")
    plan_id = args.plan_id or f"sf_{args.kind}"
    if args.kind == "stop":
        request = {"type": 0, "op": 1, "request_id": int(time.time()) & 0xFFFF, "plan_id": ""}
    else:
        lat, lon = to_wgs84(*args.origin, args.north, args.east)
        maneuvers = [("PlanManeuver", {"maneuver_id": "1", "data": manoeuvre(args, lat, lon)})]
        transitions = []
        if args.popup and args.kind != "popup":
            maneuvers.append(("PlanManeuver", {"maneuver_id": "2", "data": ("PopUp", popup(args, lat, lon))}))
            transitions.append(("PlanTransition", {"source_man": "1", "dest_man": "2", "conditions": "ManeuverIsDone"}))
        plan = ("PlanSpecification", {"plan_id": plan_id, "start_man_id": "1", "maneuvers": maneuvers,
                                      "transitions": transitions})
        request = {"type": 0, "op": 0, "request_id": int(time.time()) & 0xFFFF, "plan_id": plan_id, "arg": plan}
    socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(
        imc.encode("PlanControl", SOURCE, request, destination=args.dune_id), (host, int(port)))
    print(f"Sent: {'stop' if args.kind == 'stop' else plan_id} to DUNE at {host}:{port}")
    if args.kind == "stop":
        return

    rclpy.init()
    node = Watcher()
    shown, started, start, last_print = None, False, time.time(), 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            state = node.state
            if state and (plan_id in state or started):
                stage = state.rsplit(" ", 1)[0]                   # without the progress
                if stage != shown or time.time() - last_print >= 10.0:
                    shown, last_print = stage, time.time()
                    print(f"  {time.time() - start:5.0f} s  {state}")
                if state.startswith("EXECUTING") and plan_id in state:
                    started = True
                elif started and not state.startswith("EXECUTING"):
                    print("Plan finished.")
                    break
            if not started and time.time() - start > 8.0:
                print("DUNE has not started the plan: see the dune_bridge output for its answer.")
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
