#!/usr/bin/env python3
"""
Open-loop manoeuvre of LAUV Marie in a headless Stonefish, logged to CSV: for tuning drones/lauv_marie.scn.

  python3 tools/sim_manoeuvre.py --out run.csv 25,1150,0,0 15,1150,0.342,0        # steps: duration,rpm,rudder,elevator
  python3 tools/sim_manoeuvre.py --out run.csv --replay field_data/<log>.csv.gz --window 300 420
  ... --set fin_lift=2.5 --set hull_lift_area=0.3       # try values without editing the vehicle file

Steps: rudder and elevator in rad, in the simulator's convention (rudder positive: turn to starboard; elevator
positive: nose down). --replay feeds the measured propeller speed and fin angles of a field log (the output of
extract_field_manoeuvres.py), converted from the vehicle's convention (rudder positive: turn to port; elevator
positive: nose down), each fin on its own, so the roll commands are replayed too.

--set changes a value of the vehicle file for this run only (a copy is written to a temporary folder):
  fin_lift, fin_area, fin_max, fin_stall, hull_lift_area, hull_lift_x (both surfaces), hull_yaw_x, hull_pitch_x,
  drag_surge, drag_cross, thrust_coeff, mass, cg_x, cg_z, depth (start depth)

The simulator runs on its own ROS domain (--domain, default 80), so several runs can go on at once with different
domains and none of them touches a simulation you have open. Output columns: t, rpm, rudder, elevator, north,
east, depth, roll, pitch, yaw, u, v, w, p, q, r (rad, m, m/s, rad/s).
"""

import argparse
import csv
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import time

SET = {   # name: (regular expression with one group for the value, occurrences expected)
    "fin_lift": (r'(?<=drag_coeff="0.1" lift_coeff=")[^"]+', 4),
    "fin_area": (r'(?<=max_angle="FINMAX" area=")[^"]+', 4),
    "fin_max": (r'(?<=drag_coeff="0.1" lift_coeff="FINLIFT" max_angle=")[^"]+', 4),
    "fin_stall": (r'(?<=area="FINAREA" stall_angle=")[^"]+', 4),
    "hull_lift_area": (r'(?<=max_angle="0.0" area=")[^"]+', 2),
    "drag": (r'(?<=quadratic_drag=")[^"]+', 1),
    "thrust_coeff": (r'(?<=<thrust_coeff value=")[^"]+', 1),
    "mass": (r'(?<=<mass value=")[^"]+', 1),
    "cg": (r'(?<=<cg xyz=")[^"]+', 1),
}


def vehicle_file(source, changes):
    """the vehicle file with the --set values"""
    text = open(source).read()

    def current(pattern):
        return re.search(pattern, text).group(0)

    fin_lift = current(SET["fin_lift"][0])
    fin_max = current(SET["fin_max"][0].replace("FINLIFT", re.escape(fin_lift)))
    fin_area = current(SET["fin_area"][0].replace("FINMAX", re.escape(fin_max)))
    drag = current(SET["drag"][0]).split()
    cg = current(SET["cg"][0]).split()

    def sub(pattern, value, count):
        nonlocal text
        text, n = re.subn(pattern, str(value), text)
        assert n == count, f"{pattern}: {n} replacements, expected {count}"

    c = dict(changes)
    if "fin_stall" in c:
        sub(SET["fin_stall"][0].replace("FINAREA", re.escape(fin_area)), c.pop("fin_stall"), 4)
    if "fin_area" in c:
        sub(SET["fin_area"][0].replace("FINMAX", re.escape(fin_max)), c.pop("fin_area"), 4)
    if "fin_max" in c:
        sub(SET["fin_max"][0].replace("FINLIFT", re.escape(fin_lift)), c.pop("fin_max"), 4)
    if "fin_lift" in c:
        sub(SET["fin_lift"][0], c.pop("fin_lift"), 4)
    if "hull_lift_area" in c:
        sub(SET["hull_lift_area"][0], c.pop("hull_lift_area"), 2)
    for key, names in (("hull_lift_x", ("HullLiftYaw", "HullLiftPitch")), ("hull_yaw_x", ("HullLiftYaw",)),
                       ("hull_pitch_x", ("HullLiftPitch",))):
        if key in c:
            x = c.pop(key)
            for name in names:
                sub(rf'(?<=<actuator name="{name}" type="rudder">\n      <link name="Base"/>\n      <origin xyz=")[^ ]+',
                    x, 1)
    if "drag_surge" in c or "drag_cross" in c:
        surge, cross = c.pop("drag_surge", drag[0]), c.pop("drag_cross", drag[1])
        sub(SET["drag"][0], f"{surge} {cross} {cross}", 1)
    if "thrust_coeff" in c:
        sub(SET["thrust_coeff"][0], c.pop("thrust_coeff"), 1)
    if "mass" in c:
        sub(SET["mass"][0], c.pop("mass"), 1)
    if "cg_x" in c or "cg_z" in c:
        sub(SET["cg"][0], f'{c.pop("cg_x", cg[0])} {cg[1]} {c.pop("cg_z", cg[2])}', 1)
    c.pop("depth", None)
    assert not c, f"unknown --set names: {sorted(c)}"
    return text


def commands_from_steps(steps):
    """[(t, rpm, [top, bottom, port, starboard])] at 20 Hz"""
    out, t = [], 0.0
    for duration, rpm, rudder, elevator in steps:
        for _ in range(int(round(duration * 20))):
            out.append((t, rpm, [rudder, rudder, elevator, elevator]))
            t += 0.05
    return out


def commands_from_log(path, t0, t1, elevator_trim=0.0, elevators=True):
    import numpy as np
    import pandas as pd
    d = pd.read_csv(path)
    d = d[(d.t >= t0) & (d.t <= t1)].interpolate().bfill()
    for name in ("fin1", "fin2"):                    # the elevators, less their trim angle in the logs
        d[name] = d[name] - elevator_trim if elevators else 0.0
    t = np.arange(0.0, t1 - t0, 0.05)
    col = lambda name: np.interp(t, d.t.to_numpy() - t0, d[name].to_numpy())          # noqa: E731
    # vehicle (DUNE, etc/common/vsim-models.ini): servo 0 the lower and 3 the upper rudder (positive: to port), 1 the port
    # and 2 the starboard elevator (positive: nose down); simulator order: top, bottom, port, starboard
    return [(float(a), float(b), [float(-c), float(-d_), float(e), float(f)])
            for a, b, c, d_, e, f in zip(t, col("rpm"), col("fin3"), col("fin0"), col("fin1"), col("fin2"))], d


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("steps", nargs="*", help="duration,rpm,rudder,elevator")
    parser.add_argument("--out", required=True)
    parser.add_argument("--replay")
    parser.add_argument("--window", nargs=2, type=float, default=[0.0, 120.0])
    parser.add_argument("--set", action="append", default=[], metavar="name=value")
    parser.add_argument("--elevator-trim", type=float, default=0.0,
                        help="--replay: degrees taken off the measured elevator angles (Marie's logs: about -7.9, "
                             "the elevator angle she holds in level flight)")
    parser.add_argument("--no-elevators", action="store_true", help="--replay: rudders only, elevators at zero")
    parser.add_argument("--domain", type=int, default=80)
    parser.add_argument("--warmup", type=float, default=0.0,
                        help="--replay: seconds at the window's first propeller speed with zero fins before it")
    args = parser.parse_args()
    changes = dict(s.split("=") for s in args.set)

    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float64MultiArray

    share = get_package_share_directory("stonefish_sim")
    work = tempfile.mkdtemp(prefix="lauv_marie_")
    drone = os.path.join(work, "lauv_marie.scn")
    open(drone, "w").write(vehicle_file(os.path.join(share, "drones", "lauv_marie.scn"), changes))
    scenario = open(os.path.join(share, "scenarios", "lauv_marie_no_gpu.scn")).read()
    scenario = scenario.replace("$(find stonefish_sim)/drones/lauv_marie.scn", drone)
    assert drone in scenario
    scenario_path = os.path.join(work, "scenario.scn")
    open(scenario_path, "w").write(scenario)

    if args.replay:
        commands, _ = commands_from_log(args.replay, *args.window, math.radians(args.elevator_trim),
                                        not args.no_elevators)
        if args.warmup:
            lead = commands_from_steps([(args.warmup, commands[0][1], 0.0, 0.0)])
            commands = lead + [(t + args.warmup, rpm, fins) for t, rpm, fins in commands]
    else:
        commands = commands_from_steps([[float(x) for x in s.split(",")] for s in args.steps])

    sim = subprocess.Popen(
        ["ros2", "run", "stonefish_ros2", "stonefish_simulator_nogpu", os.path.join(share, "data"), scenario_path,
         "100.0", "--ros-args", "-r", "__ns:=/stonefish_ros2",
         "-p", f"lauv_marie_position:=0.0 0.0 {changes.get('depth', '10.0')}", "-p", "lauv_marie_orientation:=0.0 0.0 0.0",
         "-p", "lauv_marie_seabed_depth:=100.0"],
        stdout=open(os.path.join(work, "sim.log"), "w"), stderr=subprocess.STDOUT, start_new_session=True)
    try:
        rclpy.init()
        node = rclpy.create_node("lauv_marie_manoeuvre")
        state = {}

        def odom(m):
            q, p, v, w = m.pose.pose.orientation, m.pose.pose.position, m.twist.twist.linear, m.twist.twist.angular
            state["row"] = [p.x, p.y, p.z,
                            math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
                            math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
                            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
                            v.x, v.y, v.z, w.x, w.y, w.z]
        node.create_subscription(Odometry, "/lauv_marie/odom", odom, 10)
        thrusters = node.create_publisher(Float64MultiArray, "/lauv_marie/thrusters", 10)
        rudders = node.create_publisher(Float64MultiArray, "/lauv_marie/rudders", 10)
        deadline = time.time() + 40
        while "row" not in state and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if "row" not in state:
            sys.exit(f"no odometry from the simulator: see {work}/sim.log")
        rows, start = [], time.time()
        for t, rpm, fins in commands:
            while time.time() - start < t:
                rclpy.spin_once(node, timeout_sec=0.005)
            thrusters.publish(Float64MultiArray(data=[rpm]))
            rudders.publish(Float64MultiArray(data=fins + [0.0, 0.0]))
            rclpy.spin_once(node, timeout_sec=0.0)
            rows.append([round(t, 3), rpm, (fins[0] + fins[1]) / 2, (fins[2] + fins[3]) / 2] + state["row"])
        with open(args.out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow("t rpm rudder elevator north east depth roll pitch yaw u v w p q r".split())
            w.writerows([[round(x, 5) for x in row] for row in rows])
    finally:
        os.killpg(sim.pid, signal.SIGKILL)


if __name__ == "__main__":
    main()
