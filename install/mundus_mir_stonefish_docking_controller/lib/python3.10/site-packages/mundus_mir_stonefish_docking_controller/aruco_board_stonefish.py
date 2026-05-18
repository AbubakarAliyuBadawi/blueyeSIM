import numpy as np


MARKER_LENGTHS = {
    0: 0.169,
    1: 0.169,
    2: 0.169,
    3: 0.25,
    4: 0.169,
    5: 0.12,
    6: 0.12,
    7: 0.08,
    8: 0.08,
    9: 0.08,
    10: 0.08,
    11: 0.08,
    12: 0.08,
    13: 0.08,
    14: 0.08,
    15: 0.08,
    16: 0.25,
    17: 0.25,
    18: 0.25,
    19: 0.25,
    20: 0.25,
}

# Blender/Gazebo dock-frame marker centers from the successful Gazebo controller.
T_DOCK_TAG_LIST = [
    [1.4925, 1.9199, 1.176],
    [1.8327, 1.4464, 2.1781],
    [1.1698, 1.46, 2.1781],
    [1.4925, 0.95922, 1.176],
    [1.4925, 1.4939, 2.1781],
    [0.86261, 1.1461, 1.176],
    [1.4925, 1.2135, 2.1781],
    [1.782, 1.875, 1.176],
    [1.972, 1.684, 1.176],
    [0.19152, 1.0447, 1.176],
    [1.203, 1.875, 1.176],
    [0.16104, 1.875, 2.1781],
    [0.19152, 1.875, 1.176],
    [1.972, 1.0447, 1.176],
    [1.5325, 1.6156, 2.1781],
    [1.4525, 1.6156, 2.1781],
    [0.79951, 1.6993, 1.176],
    [0.008167, 1.6951, 1.68],
    [2.0181, 1.6983, 1.68],
    [0.88297, 1.6953, 2.1781],
    [1.8796, 1.6975, 2.1781],
]


def _marker_corners(center, marker_id):
    length = MARKER_LENGTHS[marker_id]
    half = length / 2.0
    x, y, z = center

    if marker_id in (17, 18):
        return np.array(
            [
                [x, y + half, z + half],
                [x, y + half, z - half],
                [x, y - half, z - half],
                [x, y - half, z + half],
            ],
            dtype="float32",
        )

    if marker_id in (11, 19, 20):
        return np.array(
            [
                [x + half, y + half, z],
                [x - half, y + half, z],
                [x - half, y - half, z],
                [x + half, y - half, z],
            ],
            dtype="float32",
        )

    return np.array(
        [
            [x - half, y + half, z],
            [x + half, y + half, z],
            [x + half, y - half, z],
            [x - half, y - half, z],
        ],
        dtype="float32",
    )


R_BOARD_DOCK = np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
T_DOCK_TAG_ARRAY = [np.array(value, dtype="float32") for value in T_DOCK_TAG_LIST]
T_BOARD_TAG_ARRAY = [R_BOARD_DOCK @ (tag - T_DOCK_TAG_ARRAY[0]) for tag in T_DOCK_TAG_ARRAY]

pos_board = [_marker_corners(center, marker_id) for marker_id, center in enumerate(T_BOARD_TAG_ARRAY)]
id_board = np.arange(21, dtype="int32").reshape((-1, 1))
