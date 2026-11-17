#!/usr/bin/env python3
"""
Manoeuvre data of LAUV Marie from raw DUNE logs, for tuning the Stonefish model: the measured fin angles and
propeller speed (the inputs) with the attitude, rates, depth and speed (the response), on one 10 Hz time base.

Usage:
  python3 tools/extract_field_manoeuvres.py --reader ~/Desktop/auv-sbi-navigation/analysis \
      ~/Desktop/auv-sbi-navigation/more_data/billefjorden/101134_20260907_star [<raw log folder> ...]

A raw log folder holds Data.lsf.gz and IMC.xml.gz. The logs are only read. Output: field_data/<log>.csv.gz with
  t                  s from the start of the log
  rpm                propeller speed (Rpm)
  fin0 .. fin3       measured fin angles in rad (ServoPosition, servo id 0-3)
  cmd0 .. cmd3       commanded fin angles in rad (SetServoPosition)
  roll, pitch, yaw   rad (EulerAngles: phi, theta, psi)
  p, q, r            rad/s (AngularVelocity)
  depth              m (Depth)
  u_water, v_water   m/s, body frame, speed through the water (WaterVelocity, valid samples only)
  u_ground           m/s (GroundVelocity, valid samples only)
  medium             VehicleMedium: 0 ground, 1 air, 2 water (surface), 4 underwater

--reader is the folder of lsf_reader.py (auv-sbi-navigation/analysis).
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "field_data")
WANTED = {"ServoPosition", "SetServoPosition", "Rpm", "EulerAngles", "AngularVelocity", "Depth", "WaterVelocity",
          "GroundVelocity", "VehicleMedium"}
RATE = 10.0


def series(df, column, grid, t0, hold=False, max_gap=2.0):
    """`column` of a message table on the time grid: interpolated (held for commands and flags), NaN in gaps"""
    if df is None or len(df) == 0:
        return np.full(len(grid), np.nan)
    df = df.sort_values("timestamp")
    t, x = df["timestamp"].to_numpy() - t0, df[column].to_numpy(dtype=float)
    i = np.clip(np.searchsorted(t, grid, side="right") - 1, 0, len(t) - 1)
    value = x[i] if hold else np.interp(grid, t, x)
    j = np.clip(i + 1, 0, len(t) - 1)
    gap = np.minimum(np.abs(grid - t[i]), np.abs(t[j] - grid))
    value = value.astype(float)
    value[gap > max_gap] = np.nan
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logs", nargs="+")
    parser.add_argument("--reader", required=True, help="folder of lsf_reader.py")
    parser.add_argument("--out", default=OUT)
    args = parser.parse_args()
    sys.path.insert(0, os.path.expanduser(args.reader))
    from lsf_reader import IMCDefs, read_lsf
    os.makedirs(args.out, exist_ok=True)

    for src in args.logs:
        src = os.path.abspath(os.path.expanduser(src))
        lsf = os.path.join(src, "Data.lsf.gz")
        if not os.path.exists(lsf):
            lsf = os.path.join(src, "Data.lsf")
        r = read_lsf(lsf, IMCDefs(os.path.join(src, "IMC.xml.gz")), wanted=WANTED, progress=False)
        tb = r.tables
        t0 = r.t_start
        grid = np.arange(0.0, r.t_end - r.t_start, 1.0 / RATE)
        out = {"t": grid, "rpm": series(tb.get("Rpm"), "value", grid, t0)}
        for name, table, hold in (("fin", "ServoPosition", False), ("cmd", "SetServoPosition", True)):
            df = tb.get(table)
            for k in range(4):
                out[f"{name}{k}"] = series(None if df is None else df[df["id"] == k], "value", grid, t0, hold)
        # the IMU entity with the most samples (some logs carry a second attitude source)
        for table, columns in (("EulerAngles", {"roll": "phi", "pitch": "theta", "yaw": "psi"}),
                               ("AngularVelocity", {"p": "x", "q": "y", "r": "z"})):
            df = tb.get(table)
            if df is not None:
                df = df[df["src_ent"] == df["src_ent"].value_counts().idxmax()]
            for name, column in columns.items():
                if name == "yaw" and df is not None:            # unwrapped for the interpolation, wrapped again
                    df = df.sort_values("timestamp").assign(psi=np.unwrap(df.sort_values("timestamp")["psi"]))
                out[name] = series(df, column, grid, t0)
        out["yaw"] = np.arctan2(np.sin(out["yaw"]), np.cos(out["yaw"]))
        out["depth"] = series(tb.get("Depth"), "value", grid, t0)
        for name, table, column in (("u_water", "WaterVelocity", "x"), ("v_water", "WaterVelocity", "y"),
                                    ("u_ground", "GroundVelocity", "x")):
            df = tb.get(table)
            out[name] = series(None if df is None else df[df["validity"] == 7], column, grid, t0)
        out["medium"] = series(tb.get("VehicleMedium"), "medium", grid, t0, hold=True, max_gap=10.0)
        table = pd.DataFrame(out).round(5)
        path = os.path.join(args.out, os.path.basename(src) + ".csv.gz")
        table.to_csv(path, index=False, compression={"method": "gzip", "mtime": 0})
        moving = table["rpm"] > 500
        print(f"{os.path.basename(src)}: {grid[-1] / 60:.1f} min, {moving.mean() * grid[-1] / 60:.1f} min under way, "
              f"fin data {table['fin0'].notna().mean():.0%}, CRC failures {r.crc_failed}/{r.crc_checked}")


if __name__ == "__main__":
    main()
