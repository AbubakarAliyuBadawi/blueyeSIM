"""
ROS link to blueye_handler for the mission scripts (mission.py, undocking.py, reset_position.py, goto_waypoint.py).

blueye_handler is the only process connected to the drone and keeps control for the whole mission. The scripts must
not connect themselves: when a second SDK client takes control, the drone drops the handler (its joystick, holds and
docking thrust then go nowhere), and as soon as the controlling client disconnects the drone releases every hold and
floats up (tests on the drone, 2026-10-23). So the scripts ask the handler, through its services, and follow the
drone through its topics:

  /blueye/sensor/ekf/lat_long   published by the handler only while the position estimate is valid
  /blueye/sensor/ekf/pose       position estimate (north, east, depth) and heading
  /blueye/notification          the drone's notifications (JSON)
  /blueye/mission/status        onboard mission status (JSON)

The link's node must be spun by the caller (e.g. rclpy.spin in a thread); its methods are called from other threads.
"""

import base64
import json
import math
import time

import blueye.protocol as bp
from blueye_interfaces.srv import JsonCommand
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger

import mission_sites

VALID_GAP_S = 0.5     # lat/long older than this: the estimate is not valid (the handler stops publishing it)
# Notifications that hold up the readiness wait. POSITION_ESTIMATE_IS_INACCURATE (level info) is only logged: at the
# pier the drone sent it every second throughout, with DVL bottom lock and a valid estimate (2026-10-25)
POSITION_WARNINGS = ("NOTIFICATION_TYPE_DRONE_POSITION_IS_UNKNOWN",)
SERVICES = {"/blueye/undock": JsonCommand, "/blueye/undock/abort": Trigger, "/blueye/reset_position": JsonCommand,
            "/blueye/mission/load": JsonCommand, "/blueye/mission/run": Trigger, "/blueye/mission/pause": Trigger,
            "/blueye/mission/clear": Trigger}


class DroneLink:
    def __init__(self, node, logger):
        self.node = node
        self.logger = logger
        self.latlon = None              # latest valid (latitude, longitude)
        self.latlon_time = 0.0
        self.valid_since = None         # start of the current stretch of valid estimates
        self.last_warning = 0.0         # last notification that the position is unknown or inaccurate
        self.pose = None                # (north, east, depth, heading deg)
        self.status = None              # latest /blueye/mission/status
        self.status_time = 0.0
        self._last_notification = None
        node.create_subscription(Point, "/blueye/sensor/ekf/lat_long", self._latlon_cb, 10)
        node.create_subscription(Odometry, "/blueye/sensor/ekf/pose", self._pose_cb, 10)
        node.create_subscription(String, "/blueye/notification", self._notification_cb, 10)
        node.create_subscription(String, "/blueye/mission/status", self._status_cb, 10)
        self.clients = {name: node.create_client(srv, name) for name, srv in SERVICES.items()}

    # ── drone state ──────────────────────────────────────────────────────────

    def _latlon_cb(self, msg):
        now = time.time()
        if now - self.latlon_time > VALID_GAP_S:
            self.valid_since = now
        self.latlon, self.latlon_time = (msg.x, msg.y), now

    def _pose_cb(self, msg):
        q = msg.pose.pose.orientation
        heading = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))) % 360
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, p.z, heading)

    def _notification_cb(self, msg):
        try:
            n = json.loads(msg.data)
        except ValueError:
            return
        if n.get("type") in POSITION_WARNINGS:
            self.last_warning = time.time()
        text = f"{n.get('type')} [{n.get('level')}]" + (f" value {n['value']}" if n.get("value") is not None else "")
        if text != self._last_notification:        # the drone repeats a notification every second
            self._last_notification = text
            self.logger.info(f"Drone notification: {text}")

    def _status_cb(self, msg):
        try:
            self.status, self.status_time = json.loads(msg.data), time.time()
        except ValueError:
            pass

    def position_valid(self):
        return time.time() - self.latlon_time <= VALID_GAP_S

    def position_ready(self, steady_s=3.0):
        """Valid estimate, without position warnings from the drone, for steady_s."""
        now = time.time()
        return self.position_valid() and now - self.valid_since >= steady_s and now - self.last_warning >= steady_s

    def wait_for_valid_position(self, steady_s=3.0, timeout_s=60.0):
        start = time.time()
        while time.time() - start < timeout_s:
            if self.position_ready(steady_s):
                return True
            time.sleep(0.2)
        self.logger.error(f"No steady valid position estimate after {timeout_s:.0f} s")
        return False

    def offset_from_dock(self, dock, timeout_s=10.0):
        """(distance m, bearing deg) from the dock coordinate to the drone's position estimate; None without a valid
        estimate in timeout_s."""
        start = time.time()
        while not self.position_valid() and time.time() - start < timeout_s:
            time.sleep(0.2)
        if not self.position_valid():
            return None
        m_lat, m_lon = mission_sites.metres_per_degree(dock[0])
        north = (self.latlon[0] - dock[0]) * m_lat
        east = (self.latlon[1] - dock[1]) * m_lon
        return math.hypot(north, east), math.degrees(math.atan2(east, north)) % 360

    def mission_status(self, max_age_s=3.0):
        """Latest onboard mission status (dict) not older than max_age_s, or None."""
        return self.status if time.time() - self.status_time <= max_age_s else None

    def fresh_status(self, after, timeout_s=5.0):
        """The first mission status received after the time `after` (e.g. a command), or None."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.status is not None and self.status_time > after:
                return self.status
            time.sleep(0.1)
        return None

    # ── handler services ─────────────────────────────────────────────────────

    def call(self, name, params=None, timeout_s=15.0):
        """Call a handler service (JsonCommand with params, Trigger without); (success, message)."""
        client = self.clients[name]
        if not client.wait_for_service(timeout_sec=5.0):
            return False, f"{name} not available: is blueye_handler running?"
        request = client.srv_type.Request()
        if params is not None:
            request.params_json = json.dumps(params)
        future = client.call_async(request)
        deadline = time.time() + timeout_s
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done():
            return False, f"{name}: no answer within {timeout_s:.0f} s"
        result = future.result()
        return result.success, result.message

    def load_mission(self, mission):
        """Clear the drone's mission and send this one (a blueye.protocol.Mission)."""
        payload = base64.b64encode(bp.Mission.serialize(mission)).decode("ascii")
        return self.call("/blueye/mission/load", {"mission_b64": payload}, timeout_s=30.0)
