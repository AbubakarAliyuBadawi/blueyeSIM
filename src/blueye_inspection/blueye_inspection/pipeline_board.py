import math

import numpy as np


MARKER_LENGTH = 0.15

# Marker centers and orientation from stonefish_sim/objects/pipe.scn in the pipe base frame.
# Marker IDs match the Aruco5x5_N looks assigned in pipe.scn.
MARKERS = [
    (21, "aruco_01", 0.0, 0.0, -0.15, 0.0, 0.0, 0.0),
    (22, "aruco_02", 0.0, -1.5, -0.15, 0.0, 0.0, 0.0),
    (23, "aruco_03", 1.0, -2.0, -0.15, 0.0, 0.0, 0.0),
    (24, "aruco_04", 0.0, -7.5, -0.15, 0.0, 0.0, 0.0),
    (25, "aruco_05", 0.0, -13.0, -0.15, 0.0, 0.0, 0.0),
    (26, "aruco_06", -3.0, -10.5, -0.15, 0.0, 0.0, 0.0),
]

BOARD_ORIGIN = np.array([MARKERS[0][2], MARKERS[0][3], MARKERS[0][4]], dtype="float32")


def _rotation_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype="float32")
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype="float32")
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype="float32")
    return rz @ ry @ rx


def _marker_corners(x, y, z, roll, pitch, yaw):
    half = MARKER_LENGTH / 2.0
    local = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype="float32",
    )
    rotation = _rotation_from_rpy(roll, pitch, yaw)
    center = np.array([x, y, z], dtype="float32") - BOARD_ORIGIN
    return (local @ rotation.T) + center


pos_board = [_marker_corners(x, y, z, roll, pitch, yaw) for _, _, x, y, z, roll, pitch, yaw in MARKERS]
id_board = np.array([marker_id for marker_id, *_ in MARKERS], dtype="int32").reshape((-1, 1))

inspection_waypoints = [
    {
        "id": marker_id,
        "name": name,
        "x": float(x - BOARD_ORIGIN[0]),
        "y": float(y - BOARD_ORIGIN[1]),
        "z": float(z - BOARD_ORIGIN[2]),
        "yaw": float(yaw),
        "radius": 0.5,
    }
    for marker_id, name, x, y, z, _, _, yaw in MARKERS
]
