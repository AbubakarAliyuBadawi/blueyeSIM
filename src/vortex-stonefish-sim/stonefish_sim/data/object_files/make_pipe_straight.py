#!/usr/bin/env python3
"""
Writes pipe_straight.obj: a straight pipe section for the pipeline scenario (objects/pipe.scn).

The pipe lies along +y (east when the pipeline orientation is 0), from y = 0 to y = LENGTH, centred on x = 0, z = 0
(NED: z down), with end caps. Texture coordinates repeat the "pipe" look every metre along the pipe.

  python3 make_pipe_straight.py [length_m] [radius_m]
"""

import math
import os
import sys

LENGTH = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
RADIUS = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
SEGMENTS = 32
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pipe_straight.obj")

v, vt, vn, f = [], [], [], []
for end, y in enumerate((0.0, LENGTH)):                     # side rings, one per end
    for k in range(SEGMENTS + 1):
        a = 2 * math.pi * k / SEGMENTS
        v.append((RADIUS * math.cos(a), y, RADIUS * math.sin(a)))
        vt.append((k / SEGMENTS, y))
        vn.append((math.cos(a), 0.0, math.sin(a)))
ring = SEGMENTS + 1
for k in range(SEGMENTS):                                   # side quads as two triangles
    a, b, c, d = k, k + 1, ring + k + 1, ring + k
    f.append((a, b, c))
    f.append((a, c, d))
for y, sign in ((0.0, -1.0), (LENGTH, 1.0)):                # caps
    centre = len(v)
    v.append((0.0, y, 0.0))
    vt.append((0.5, 0.5))
    vn.append((0.0, sign, 0.0))
    first = len(v)
    for k in range(SEGMENTS):
        a = 2 * math.pi * k / SEGMENTS
        v.append((RADIUS * math.cos(a), y, RADIUS * math.sin(a)))
        vt.append((0.5 + 0.5 * math.cos(a), 0.5 + 0.5 * math.sin(a)))
        vn.append((0.0, sign, 0.0))
    for k in range(SEGMENTS):
        a, b = first + k, first + (k + 1) % SEGMENTS
        f.append((centre, b, a) if sign < 0 else (centre, a, b))

with open(OUT, "w") as fh:
    fh.write(f"# straight pipe, length {LENGTH} m, radius {RADIUS} m, along +y; written by make_pipe_straight.py\n")
    fh.write("o PipeStraight\n")
    fh.writelines(f"v {x:.5f} {y:.5f} {z:.5f}\n" for x, y, z in v)
    fh.writelines(f"vt {u:.5f} {w:.5f}\n" for u, w in vt)
    fh.writelines(f"vn {x:.5f} {y:.5f} {z:.5f}\n" for x, y, z in vn)
    fh.writelines(f"f {a+1}/{a+1}/{a+1} {b+1}/{b+1}/{b+1} {c+1}/{c+1}/{c+1}\n" for a, b, c in f)
print(f"wrote {OUT}: {len(v)} vertices, {len(f)} faces")
