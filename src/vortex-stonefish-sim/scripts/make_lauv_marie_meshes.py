#!/usr/bin/env python3
"""
Meshes of LAUV Marie for Stonefish, from the AUR-Lab Blender export (blender_models/lauv/Marie).

Usage:
  python3 scripts/make_lauv_marie_meshes.py ~/Desktop/breach_software_stack/blender_models/lauv/Marie

Output (stonefish_sim/data/lauv_marie/):
  hull_visual.obj    the Blender hull (lauv_body.obj), coloured through palette.png (one swatch per Blender material)
  palette.png        the material colours of lauv_body.mtl
  hull_physics.obj   closed body of revolution with the hull's radius along its length (no mast, handle or sonars):
                     Stonefish takes the buoyancy, added mass and drag areas from it
  fin.obj            one fin: chord along x (leading edge forward), span along +z from the hull outwards, turning
                     about z at the origin
  propeller.obj      the propeller, shaft along x
  mesh_info.yaml     numbers for drones/lauv_marie.scn: displaced volume, stations of the fins and the propeller

Frames. Blender export: nose towards -y, z up, tail at y = 0. Stonefish body: x forward, y starboard, z down, so
(x, y, z) = (-y_b, -x_b, -z_b), with the origin on the hull axis at the centre of buoyancy of hull_physics.obj.
"""

import argparse
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "stonefish_sim", "data", "lauv_marie")
HULL_RADIUS = 0.092          # of the cylindrical section; larger radii belong to the mast, handle and sonars
SEGMENTS = 32                # around the physics hull
PIVOT_CHORD = 0.35           # the fin turns about this fraction of its chord behind the leading edge
UNNAMED = "unnamed"          # faces the export left without a material: drawn grey


def read_obj(path):
    """vertices, normals, [(material, [(v, vn), (v, vn), (v, vn)])] of a triangulated OBJ"""
    v, vn, faces, material = [], [], [], None
    for line in open(path):
        p = line.split()
        if not p:
            continue
        if p[0] == "v":
            v.append([float(x) for x in p[1:4]])
        elif p[0] == "vn":
            vn.append([float(x) for x in p[1:4]])
        elif p[0] == "usemtl":
            material = p[1] if len(p) > 1 else UNNAMED
        elif p[0] == "f":
            corners = [(int(c.split("/")[0]) - 1, int(c.split("/")[2]) - 1) for c in p[1:]]
            for k in range(1, len(corners) - 1):               # fan, in case a face is not a triangle
                faces.append((material, [corners[0], corners[k], corners[k + 1]]))
    return np.array(v), np.array(vn), faces


def read_mtl(path):
    colours, name = {}, None
    for line in open(path):
        p = line.split()
        if p and p[0] == "newmtl":
            name = p[1]
        elif p and p[0] == "Kd":
            colours[name] = [float(x) for x in p[1:4]]
    return colours


def to_body(points, origin_station=0.0):
    """Blender export to the Stonefish body frame, x measured from the station `origin_station` [m from the tail]"""
    return np.column_stack([-points[:, 1] - origin_station, -points[:, 0], -points[:, 2]])


def write_obj(path, name, v, vn, faces, uv=None, comment=""):
    """faces: [(uv index or None, [(v, vn), ...])]; Stonefish reads 'f v/vt/vn' with a texture and 'f v//vn' without"""
    with open(path, "w") as f:
        f.write(f"# {comment}; written by make_lauv_marie_meshes.py\no {name}\n")
        f.writelines(f"v {x:.5f} {y:.5f} {z:.5f}\n" for x, y, z in v)
        if uv is not None:
            f.writelines(f"vt {a:.4f} {b:.4f}\n" for a, b in uv)
        f.writelines(f"vn {x:.4f} {y:.4f} {z:.4f}\n" for x, y, z in vn)
        for t, corners in faces:
            if uv is not None:
                f.write("f " + " ".join(f"{a + 1}/{t + 1}/{b + 1}" for a, b in corners) + "\n")
            else:
                f.write("f " + " ".join(f"{a + 1}//{b + 1}" for a, b in corners) + "\n")


def hull_profile(v):
    """(station from the tail [m], radius [m]) of the bare hull: the median radius of the vertices in 2 cm slices,
    limited to the cylinder's radius (the mast, handle and sonars stand out of it)"""
    s, r = -v[:, 1], np.hypot(v[:, 0], v[:, 2])
    stations, radii = [], []
    for a in np.arange(s.min(), s.max(), 0.02):
        inside = (s >= a) & (s < a + 0.02)
        if inside.sum() >= 8:
            stations.append(float(s[inside].mean()))
            radii.append(min(float(np.median(r[inside])), HULL_RADIUS))
    # the cylinder between the first and the last full-radius slice has no smaller radius (sonar windows, DVL)
    full = [i for i, x in enumerate(radii) if x > 0.95 * HULL_RADIUS]
    for i in range(full[0], full[-1] + 1):
        radii[i] = HULL_RADIUS
    return [float(s.min())] + stations + [float(s.max())], [0.0] + radii + [0.0]


def revolve(stations, radii):
    """closed surface of revolution about x (stations along x, first and last radius 0), normals outwards"""
    rings, v = [], []
    for x, r in zip(stations, radii):
        if r == 0.0:
            rings.append([len(v)])
            v.append([x, 0.0, 0.0])
        else:
            rings.append(list(range(len(v), len(v) + SEGMENTS)))
            v += [[x, r * np.cos(a), r * np.sin(a)] for a in np.linspace(0, 2 * np.pi, SEGMENTS, endpoint=False)]
    v = np.array(v)
    tris = []
    for a, b in zip(rings[:-1], rings[1:]):                       # a is aft of b
        for k in range(SEGMENTS):
            a0, a1 = a[k % len(a)], a[(k + 1) % len(a)]
            b0, b1 = b[k % len(b)], b[(k + 1) % len(b)]
            if len(a) > 1:
                tris.append([a0, a1, b0])
            if len(b) > 1:
                tris.append([a1, b1, b0])
    tris = np.array(tris)
    return v, tris


def volume_and_centroid(v, tris):
    a, b, c = v[tris[:, 0]], v[tris[:, 1]], v[tris[:, 2]]
    signed = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6.0
    return signed.sum(), ((a + b + c) / 4.0 * signed[:, None]).sum(axis=0) / signed.sum()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source", help="folder of the Blender export (lauv_body.obj, up_down_fins.obj, propeller.obj)")
    parser.add_argument("--out", default=OUT)
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    hull_v, hull_n, hull_f = read_obj(os.path.join(args.source, "lauv_body.obj"))
    colours = read_mtl(os.path.join(args.source, "lauv_body.mtl"))
    colours[UNNAMED] = [0.5, 0.5, 0.5]

    # physics hull, and from it the body origin: the centre of buoyancy
    stations, radii = hull_profile(hull_v)
    v, tris = revolve(stations, radii)
    volume, centroid = volume_and_centroid(v, tris)
    if volume < 0:                                              # normals inwards: turn the triangles round
        tris = tris[:, ::-1]
        volume, centroid = volume_and_centroid(v, tris)
    origin = float(centroid[0])
    v[:, 0] -= origin
    normals = np.cross(v[tris[:, 1]] - v[tris[:, 0]], v[tris[:, 2]] - v[tris[:, 0]])
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    write_obj(os.path.join(args.out, "hull_physics.obj"), "LauvMarieHullPhysics", v, normals,
              [(None, [(int(i), k) for i in t]) for k, t in enumerate(tris)],
              comment="LAUV Marie, bare hull as a body of revolution (physics)")

    # visual hull, one palette swatch per Blender material
    names = sorted(colours)
    palette = np.zeros((8, 8 * len(names), 3), np.uint8)
    for i, name in enumerate(names):
        palette[:, 8 * i:8 * i + 8] = [int(round(255 * c)) for c in reversed(colours[name])]       # BGR
    cv2.imwrite(os.path.join(args.out, "palette.png"), palette)
    uv = [((i + 0.5) / len(names), 0.5) for i in range(len(names))]
    write_obj(os.path.join(args.out, "hull_visual.obj"), "LauvMarieHull", to_body(hull_v, origin),
              to_body(hull_n), [(names.index(m), corners) for m, corners in hull_f], uv,
              comment="LAUV Marie hull (visual), from the AUR-Lab Blender model by Enio Krizman")

    # one fin: the upper one of up_down_fins.obj, about its pivot on the hull axis
    fin_v, fin_n, fin_f = read_obj(os.path.join(args.source, "up_down_fins.obj"))
    upper = fin_v[:, 2] > 0
    s = -fin_v[upper, 1]
    leading, trailing = float(s.max()), float(s.min())
    pivot = leading - PIVOT_CHORD * (leading - trailing)
    root, tip = float(fin_v[upper, 2].min()), float(fin_v[upper, 2].max())
    # chord along x, span along +z (Blender z), thickness along y (Blender x): a rotation, so the normals keep
    fin = np.column_stack([-fin_v[:, 1] - pivot, fin_v[:, 0], fin_v[:, 2]])
    fin_normals = np.column_stack([-fin_n[:, 1], fin_n[:, 0], fin_n[:, 2]])
    new_index = np.cumsum(upper) - 1                           # the lower fin's vertices are left out
    faces = [(None, [(int(new_index[a]), b) for a, b in corners]) for _, corners in fin_f
             if all(upper[a] for a, _ in corners)]
    write_obj(os.path.join(args.out, "fin.obj"), "LauvMarieFin", fin[upper], fin_normals, faces,
              comment="LAUV Marie fin: chord along x, span along +z, turning about z at the origin")

    # propeller, shaft along x, centred
    prop_v, prop_n, prop_f = read_obj(os.path.join(args.source, "propeller.obj"))
    prop_station = float(-(prop_v[:, 1].min() + prop_v[:, 1].max()) / 2)
    write_obj(os.path.join(args.out, "propeller.obj"), "LauvMariePropeller", to_body(prop_v, prop_station),
              to_body(prop_n), [(None, corners) for _, corners in prop_f],
              comment="LAUV Marie propeller, shaft along x")

    span, chord = tip - root, leading - trailing
    info = {
        "hull_length_m": round(float(stations[-1] - stations[0]), 3),
        "hull_radius_m": HULL_RADIUS,
        "displaced_volume_m3": round(float(volume), 5),
        "origin_station_from_tail_m": round(origin, 4),
        "nose_x_m": round(float(stations[-1]) - origin, 4),
        "tail_x_m": round(float(stations[0]) - origin, 4),
        "fin_pivot_x_m": round(pivot - origin, 4),
        "fin_chord_m": round(chord, 4),
        "fin_root_radius_m": round(root, 4),
        "fin_tip_radius_m": round(tip, 4),
        "fin_area_m2": round(span * chord, 5),
        "propeller_x_m": round(prop_station - origin, 4),
        "propeller_diameter_m": round(float(np.ptp(prop_v[:, 0])), 4),
        "palette_materials": names,
    }
    with open(os.path.join(args.out, "mesh_info.yaml"), "w") as f:
        f.write("# written by make_lauv_marie_meshes.py; body frame x forward, y starboard, z down, origin at the\n"
                "# centre of buoyancy of hull_physics.obj\n")
        for key, value in info.items():
            f.write(f"{key}: {value}\n")
    for key, value in info.items():
        print(f"  {key}: {value}")


if __name__ == "__main__":
    main()
