"""
Drone services of blueye_handler, which owns the drone for the whole mission: the mission scripts and the behaviour
tree ask the handler instead of opening their own SDK connection (a second client taking control drops the handler,
and the drone releases every hold once its controlling client disconnects).

Services (blueye_interfaces/JsonCommand takes its parameters as a JSON object, std_srvs/Trigger none):
  /blueye/undock            JsonCommand {"power": 0.4, "duration": 6.0}  timed reverse out of the dock, ending in
                                                                         station keeping; answers when done (~20 s)
  /blueye/undock/abort      Trigger      stop the undocking: thrust zeroed, station keeping on
  /blueye/reset_position    JsonCommand {"lat": .., "lon": .., "heading": deg (optional, else the drone compass)}
  /blueye/mission/load      JsonCommand {"mission_b64": base64 of a serialized blueye.protocol.Mission}  clear + send
  /blueye/mission/run       Trigger      run, or resume a paused mission
  /blueye/mission/pause     Trigger
  /blueye/mission/clear     Trigger
Topics:
  /blueye/mission/status    std_msgs/String (JSON) {"state", "completed", "total", "time_elapsed", "eta"}, 2 Hz
  /blueye/notification      std_msgs/String (JSON) {"type", "level", "value"} for each drone notification
"""

import base64
import json
import threading
import time

import blueye.protocol as bp
from blueye_interfaces.srv import JsonCommand
from google.protobuf import wrappers_pb2
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from std_srvs.srv import Trigger


def notification_value(value):
    """Value attached to a drone notification (a protobuf Any), e.g. a distance; None without one."""
    for cls in (wrappers_pb2.FloatValue, wrappers_pb2.DoubleValue, wrappers_pb2.Int32Value,
                wrappers_pb2.UInt32Value, wrappers_pb2.BoolValue, wrappers_pb2.StringValue):
        if value.Is(cls.DESCRIPTOR):
            unpacked = cls()
            value.Unpack(unpacked)
            return unpacked.value
    return value.type_url.split("/")[-1] or None


class DroneServices:
    def __init__(self, node, drone):
        """drone: callable returning the handler's current blueye.sdk.Drone (it changes when control is reclaimed)."""
        self.node = node
        self._drone = drone
        self._sdk_lock = threading.Lock()          # request/reply calls to the drone, one at a time
        self._undocking = threading.Lock()
        self._undock_cancel = threading.Event()
        group = ReentrantCallbackGroup()           # undocking takes ~20 s; the other services stay available
        node.create_service(JsonCommand, "/blueye/undock", self._undock, callback_group=group)
        node.create_service(Trigger, "/blueye/undock/abort", self._undock_abort, callback_group=group)
        node.create_service(JsonCommand, "/blueye/reset_position", self._reset_position, callback_group=group)
        node.create_service(JsonCommand, "/blueye/mission/load", self._mission_load, callback_group=group)
        for name in ("run", "pause", "clear"):
            node.create_service(Trigger, f"/blueye/mission/{name}",
                                lambda request, response, name=name: self._mission_command(name, response),
                                callback_group=group)
        self.status_pub = node.create_publisher(String, "/blueye/mission/status", 10)
        self.notification_pub = node.create_publisher(String, "/blueye/notification", 10)
        node.create_timer(0.5, self._publish_status, callback_group=group)

    def notification_callback(self, _msg_type, msg):
        n = msg.notification
        value = notification_value(n.value)
        out = String()
        out.data = json.dumps({"type": n.type_.name, "level": n.level.name,
                               "value": value if isinstance(value, (int, float, bool, str)) or value is None
                               else str(value)})
        self.notification_pub.publish(out)

    # ── undocking ────────────────────────────────────────────────────────────

    def _wait(self, seconds):
        """Sleep; False if the undocking was aborted meanwhile."""
        return not self._undock_cancel.wait(seconds)

    def _undock(self, request, response):
        params = json.loads(request.params_json or "{}")
        power, duration = float(params.get("power", 0.4)), float(params.get("duration", 6.0))
        if not self._undocking.acquire(blocking=False):
            response.success, response.message = False, "undocking already running"
            return response
        self._undock_cancel.clear()
        try:
            response.success, response.message = self._undock_sequence(power, duration)
        except Exception as exc:
            self._hold()
            response.success, response.message = False, f"undocking failed: {exc}"
        finally:
            self._undocking.release()
        self.node.get_logger().info(f"Undocking: {response.message}")
        return response

    def _undock_sequence(self, power, duration):
        motion = self._drone().motion
        log = self.node.get_logger()
        log.info("Undocking: station keeping")
        motion.station_keeping_active = True
        if not self._wait(3.0):
            return self._aborted()
        if not motion.station_keeping_active:
            log.warning("Undocking: station keeping did not activate, auto depth and heading instead")
            motion.auto_depth_active = True
            motion.auto_heading_active = True
            if not self._wait(2.0):
                return self._aborted()
        log.info("Undocking: holding for 5 s")
        if not self._wait(5.0):
            return self._aborted()
        motion.station_keeping_active = False
        motion.auto_depth_active = True
        motion.auto_heading_active = True
        if not self._wait(1.0):
            return self._aborted()
        log.info(f"Undocking: reversing at {power} for {duration:.0f} s")
        end = time.time() + duration
        while time.time() < end and not self._undock_cancel.is_set():
            motion.surge = -power                  # re-sent every 0.1 s: the drone stops on stale motion input
            time.sleep(0.1)
        motion.surge = 0.0
        if self._undock_cancel.is_set():
            return self._aborted()
        time.sleep(1.0)
        motion.station_keeping_active = True       # stays on: the handler keeps control, so the hold lasts
        time.sleep(3.0)
        return True, "undocked, station keeping until the next command"

    def _hold(self):
        motion = self._drone().motion
        motion.surge = motion.sway = motion.heave = motion.yaw = 0.0
        motion.station_keeping_active = True

    def _aborted(self):
        self._hold()
        return False, "undocking aborted: thrust zeroed, station keeping on"

    def _undock_abort(self, request, response):
        self._undock_cancel.set()
        self._hold()
        response.success, response.message = True, "undocking abort requested: station keeping on"
        return response

    # ── position reset ───────────────────────────────────────────────────────

    def _reset_position(self, request, response):
        params = json.loads(request.params_json or "{}")
        heading = params.get("heading")
        settings = {
            "heading_source_during_reset": bp.HeadingSource.HEADING_SOURCE_MANUAL_INPUT if heading is not None
            else bp.HeadingSource.HEADING_SOURCE_DRONE_COMPASS,
            "manual_heading": float(heading) if heading is not None else 0.0,
            "reset_coordinate_source": bp.ResetCoordinateSource.RESET_COORDINATE_SOURCE_MANUAL,
            "reset_coordinate": {"latitude": float(params["lat"]), "longitude": float(params["lon"])},
        }
        self._drone()._ctrl_client._messages_to_send.put(bp.ResetPositionCtrl(settings=settings))
        response.success = True
        response.message = f"position reset sent: {params['lat']:.7f}, {params['lon']:.7f}"
        self.node.get_logger().info(response.message)
        return response

    # ── onboard mission ──────────────────────────────────────────────────────

    def _mission_load(self, request, response):
        mission = bp.Mission.deserialize(base64.b64decode(json.loads(request.params_json)["mission_b64"]))
        with self._sdk_lock:
            drone = self._drone()
            drone.mission.clear()
            time.sleep(2.0)
            for attempt in range(3):
                try:
                    drone.mission.send_new(mission)
                    break
                except Exception as exc:
                    if attempt == 2:
                        response.success, response.message = False, f"sending the mission failed: {exc}"
                        return response
                    time.sleep(2.0)
        response.success = True
        response.message = f"mission '{mission.name}' loaded ({len(mission.instructions)} instructions)"
        self.node.get_logger().info(response.message)
        return response

    def _mission_command(self, name, response):
        if not self._sdk_lock.acquire(timeout=10.0):
            response.success, response.message = False, "drone busy"
            return response
        try:
            getattr(self._drone().mission, name)()
            response.success, response.message = True, f"mission {name}"
        except Exception as exc:
            response.success, response.message = False, f"mission {name} failed: {exc}"
        finally:
            self._sdk_lock.release()
        self.node.get_logger().info(response.message)
        return response

    def _publish_status(self):
        if not self._sdk_lock.acquire(timeout=0.05):
            return
        try:
            status = self._drone().mission.get_status()
        except Exception as exc:
            self.node.get_logger().warning(f"Mission status unavailable: {exc}", throttle_duration_sec=10.0)
            return
        finally:
            self._sdk_lock.release()
        out = String()
        if status is None:
            out.data = json.dumps({"state": "MISSION_STATE_NONE", "completed": [], "total": 0})
        else:
            out.data = json.dumps({"state": status.state.name, "completed": list(status.completed_instruction_ids),
                                   "total": status.total_number_of_instructions,
                                   "time_elapsed": status.time_elapsed, "eta": status.estimated_time_to_complete})
        self.status_pub.publish(out)
