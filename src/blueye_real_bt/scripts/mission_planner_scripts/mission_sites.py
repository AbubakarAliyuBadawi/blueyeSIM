"""
Mission layouts per test site, in metres from the dock.

A waypoint is given in the dock frame: metres straight out of the dock opening ("forward") and metres to the right
looking out of the dock ("right"). Before each mission reset_position.py resets the drone's position estimate to the
dock coordinate while the drone is docked (the dock reference, below). The undocking then reverses the drone straight
out, so when the mission starts the direction from the dock coordinate to the drone's position estimate is the dock's
axis, in the frame the drone navigates in. The waypoints are placed along that axis: no compass value is used (in the
pool the compass read 197° and 169° for the same docked drone, and disagreed with the drone's own motion by 138°),
and the dock coordinate itself only needs to be roughly right.

Phases (for /mission_state): the leg to a waypoint has the phase given with it; the wait at an inspection point is
inspection.

The site is chosen with the environment variable MISSION_SITE (default pool), which the behaviour tree passes on to
the scripts it starts, e.g. MISSION_SITE=pier for reset_position.py and for the tree.
"""

import json
import math
import os
import time
from pathlib import Path

SITES = {
    # AUR-lab pool, Trondheim Biological Station: 13.8 x 8.7 m, 2.3 m deep; dock on the south wall, its opening
    # facing a 1.55 m square obstacle (reaching the surface) about 8 m out on the dock axis.
    # Layout: plots/real_drone/pool_layout_proposal.png
    "pool": {
        "dock": (63.44137187695592, 10.348215616597997),
        "speed": 0.2,                 # m/s to each waypoint
        "circle_of_acceptance": 0.5,  # m
        "wait_s": 5.0,                # at each inspection point
        # (name, forward m, right m, depth m, phase of the leg to it, inspection point)
        "waypoints": [
            ("I1", 3.0, 2.5, 1.2, "transit", True),
            ("I2", 5.0, 2.5, 1.2, "inspection", True),
            ("I3", 7.0, 2.5, 1.2, "inspection", True),
            ("I4", 9.0, 2.5, 1.2, "inspection", True),
            ("I5", 11.0, 2.5, 1.2, "inspection", True),
            ("H1", 11.0, -2.5, 1.2, "homing", False),
            ("H2", 4.5, -2.5, 1.2, "homing", False),
            ("H3", 4.5, 0.0, 1.7, "homing", False),
            ("S", 2.5, 0.0, 1.7, "homing", False),    # stop point facing the dock: docking starts here
        ],
    },
    # Wooden pier near the AUR-lab pool: dock hung on the pier's east face, its opening facing east (open water).
    # Inspection of an assumed pipeline along a straight 18.3 m line heading 091°, the inspection start and end
    # given by the user (63.4408125, 10.3492829 and 63.4408095, 10.3496510); the dock coordinate is read off the
    # satellite image (about 1-2 m), on the line of the user's measured route. Same settings as the pool; the dock
    # is to hang at about 2 m so the stop point at 1.7 m faces it. Layout: plots/field/mission_plan/pier_layout.png
    "pier": {
        "dock": (63.4407614, 10.3491130),
        "nominal_out_bearing_deg": 91.0,   # for plots only: the mission takes the axis from the undocking
        "speed": 0.2,
        "circle_of_acceptance": 0.5,
        "wait_s": 5.0,
        "waypoints": [
            ("I1", 8.4, -5.8, 1.2, "transit", True),
            ("I2", 14.5, -5.8, 1.2, "inspection", True),
            ("I3", 20.6, -5.8, 1.2, "inspection", True),
            ("I4", 26.7, -5.8, 1.2, "inspection", True),
            ("H1", 13.2, 0.0, 1.2, "homing", False),    # back onto the dock axis
            ("S", 3.7, 0.0, 1.7, "homing", False),      # stop point facing the dock: docking starts here
        ],
    },
}
DEFAULT_SITE = os.environ.get("MISSION_SITE", "pool")
if DEFAULT_SITE not in SITES:
    raise ValueError(f"MISSION_SITE={DEFAULT_SITE!r} is not a site; use one of {', '.join(sorted(SITES))}")

# Dock reference written by reset_position.py; the mission refuses an older one (the estimate drifts, and the drone
# may have been flown and docked again since)
REFERENCE_FILE = Path(__file__).resolve().parent / "dock_reference.json"
REFERENCE_MAX_AGE_S = 3600.0


def save_reference(site_name, docked_compass_heading_deg=None):
    """Save the dock reference after a reset at the dock (the compass heading is kept for comparison only)."""
    reference = {"site": site_name, "dock": list(SITES[site_name]["dock"]),
                 "docked_compass_heading_deg": docked_compass_heading_deg,
                 "time": time.strftime("%Y-%m-%d %H:%M:%S"), "unix_time": time.time()}
    REFERENCE_FILE.write_text(json.dumps(reference, indent=2))
    return reference


def load_reference(site_name):
    """The dock reference of a site; ValueError (with what to do) if it is missing, of another site or too old."""
    if not REFERENCE_FILE.exists():
        raise ValueError("no dock reference: run reset_position.py with the drone docked")
    reference = json.loads(REFERENCE_FILE.read_text())
    if reference["site"] != site_name:
        raise ValueError(f"the dock reference is for site {reference['site']!r}: run reset_position.py --site {site_name}")
    age = time.time() - reference["unix_time"]
    if age > REFERENCE_MAX_AGE_S:
        raise ValueError(f"the dock reference is {age / 60:.0f} min old: run reset_position.py with the drone docked")
    return reference


def save_out_bearing(site_name, out_bearing_deg):
    """Add the dock's axis found by mission.py (bearing out of the dock) to the reference, for site_point."""
    reference = load_reference(site_name)
    reference["out_bearing_deg"] = out_bearing_deg
    REFERENCE_FILE.write_text(json.dumps(reference, indent=2))
    return reference


def site_point(site_name, point_name):
    """(latitude, longitude, depth) of a waypoint of the site layout, e.g. the stop point S in front of the dock;
    ValueError without a reference holding the dock's axis (saved by mission.py at the mission start)."""
    reference = load_reference(site_name)
    if reference.get("out_bearing_deg") is None:
        raise ValueError("the dock's axis is not known yet (mission.py saves it at the mission start)")
    site = SITES[site_name]
    for name, forward, right, depth, _phase, _inspect in site["waypoints"]:
        if name == point_name:
            return (*dock_frame_to_latlon(site, forward, right, reference["out_bearing_deg"]), depth)
    raise ValueError(f"no waypoint {point_name!r} in the {site_name} layout")


def metres_per_degree(lat):
    """Metres per degree of latitude and of longitude at a latitude (flat earth over a few hundred metres)."""
    phi = math.radians(lat)
    return (111132.954 - 559.822 * math.cos(2 * phi) + 1.175 * math.cos(4 * phi),
            111412.84 * math.cos(phi) - 93.5 * math.cos(3 * phi))


def dock_frame_to_north_east(forward, right, out_bearing_deg):
    """(north, east) in metres from the dock of a point given in the dock frame; out_bearing_deg: bearing straight out
    of the dock opening."""
    out = math.radians(out_bearing_deg)
    return (forward * math.cos(out) - right * math.sin(out),
            forward * math.sin(out) + right * math.cos(out))


def dock_frame_to_latlon(site, forward, right, out_bearing_deg):
    """(latitude, longitude) of a point given in the dock frame."""
    north, east = dock_frame_to_north_east(forward, right, out_bearing_deg)
    lat0, lon0 = site["dock"]
    m_lat, m_lon = metres_per_degree(lat0)
    return lat0 + north / m_lat, lon0 + east / m_lon
