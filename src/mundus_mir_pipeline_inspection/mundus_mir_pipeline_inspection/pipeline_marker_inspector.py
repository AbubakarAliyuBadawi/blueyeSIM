import math
from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.node import Node

from mundus_mir_msgs.srv import (
    AddWaypoint,
    ClearWaypoints,
    GoToWaypoints,
    RunWaypointController,
)


@dataclass(frozen=True)
class Marker:
    name: str
    x: float
    y: float
    z: float
    yaw: float = 0.0


class PipelineMarkerInspector(Node):
    def __init__(self):
        super().__init__("pipeline_marker_inspector")

        self.declare_parameter("pipeline_position", [1.0, 0.5, 3.5])
        self.declare_parameter("pipeline_yaw", 0.0)
        self.declare_parameter("inspection_z_offset", -0.6)
        self.declare_parameter("desired_velocity", 0.25)
        self.declare_parameter("fixed_heading", False)
        self.declare_parameter("clear_existing_waypoints", True)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("marker_order", ["aruco_01", "aruco_02", "aruco_03", "aruco_04", "aruco_06", "aruco_05"])
        self.declare_parameter("run_waypoint_controller_service", "/blueye/run_waypoint_controller")
        self.declare_parameter("clear_waypoints_service", "/blueye/clear_waypoints")
        self.declare_parameter("add_waypoint_service", "/blueye/add_waypoint")
        self.declare_parameter("go_to_waypoints_service", "/blueye/go_to_waypoints")

        self.marker_map = {
            "aruco_01": Marker("aruco_01", 0.0, -1.0, -0.11, 0.0),
            "aruco_02": Marker("aruco_02", 2.0, -3.0, -0.11, 0.0),
            "aruco_03": Marker("aruco_03", 1.0, -5.0, -0.11, 0.775),
            "aruco_04": Marker("aruco_04", 0.0, -7.5, -0.11, 0.0),
            "aruco_05": Marker("aruco_05", 0.0, -13.0, -0.11, 0.0),
            "aruco_06": Marker("aruco_06", -3.0, -10.5, -0.11, 0.0),
        }

        self.run_client = self.create_client(
            RunWaypointController,
            self.get_parameter("run_waypoint_controller_service").value,
        )
        self.clear_client = self.create_client(
            ClearWaypoints,
            self.get_parameter("clear_waypoints_service").value,
        )
        self.add_client = self.create_client(
            AddWaypoint,
            self.get_parameter("add_waypoint_service").value,
        )
        self.go_client = self.create_client(
            GoToWaypoints,
            self.get_parameter("go_to_waypoints_service").value,
        )

    def run_once(self):
        if not self.get_parameter("auto_start").value:
            self.get_logger().info("auto_start is false; set it true or restart when ready.")
            return

        clients = [self.run_client, self.clear_client, self.add_client, self.go_client]
        while rclpy.ok() and not all(client.service_is_ready() for client in clients):
            self.get_logger().info("Waiting for waypoint controller services...")
            for client in clients:
                client.wait_for_service(timeout_sec=0.5)

        self._send_mission()

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()

    def _world_marker(self, marker: Marker) -> Marker:
        pipeline_position = self.get_parameter("pipeline_position").value
        pipeline_yaw = float(self.get_parameter("pipeline_yaw").value)

        c = math.cos(pipeline_yaw)
        s = math.sin(pipeline_yaw)
        x = pipeline_position[0] + c * marker.x - s * marker.y
        y = pipeline_position[1] + s * marker.x + c * marker.y
        z = pipeline_position[2] + marker.z
        yaw = pipeline_yaw + marker.yaw
        return Marker(marker.name, x, y, z, yaw)

    def _inspection_waypoints(self) -> List[Marker]:
        marker_order = self.get_parameter("marker_order").value
        z_offset = float(self.get_parameter("inspection_z_offset").value)

        waypoints = []
        for marker_name in marker_order:
            if marker_name not in self.marker_map:
                self.get_logger().warn(f"Unknown marker '{marker_name}', skipping.")
                continue
            world_marker = self._world_marker(self.marker_map[marker_name])
            waypoints.append(
                Marker(
                    world_marker.name,
                    world_marker.x,
                    world_marker.y,
                    world_marker.z + z_offset,
                    world_marker.yaw,
                )
            )
        return waypoints

    def _send_mission(self):
        desired_velocity = float(self.get_parameter("desired_velocity").value)
        fixed_heading = bool(self.get_parameter("fixed_heading").value)

        run_req = RunWaypointController.Request()
        run_req.run = True
        self._call(self.run_client, run_req)
        self.get_logger().info("Waypoint controller enabled.")

        if self.get_parameter("clear_existing_waypoints").value:
            clear_req = ClearWaypoints.Request()
            clear_req.clear = True
            self._call(self.clear_client, clear_req)
            self.get_logger().info("Cleared existing waypoints.")

        waypoints = self._inspection_waypoints()
        for waypoint in waypoints:
            add_req = AddWaypoint.Request()
            add_req.x = float(waypoint.x)
            add_req.y = float(waypoint.y)
            add_req.z = float(waypoint.z)
            add_req.desired_velocity = desired_velocity
            add_req.fixed_heading = fixed_heading
            add_req.heading = float(waypoint.yaw)
            response = self._call(self.add_client, add_req)
            if not response.accepted:
                self.get_logger().warn(f"Waypoint rejected for {waypoint.name}.")
            self.get_logger().info(
                f"Added {waypoint.name}: x={waypoint.x:.2f}, y={waypoint.y:.2f}, z={waypoint.z:.2f}, yaw={waypoint.yaw:.2f}"
            )

        go_req = GoToWaypoints.Request()
        go_req.run = True
        go_response = self._call(self.go_client, go_req)
        self.get_logger().info(f"Started pipeline inspection: {go_response.status_code}")


def main(args=None):
    rclpy.init(args=args)
    node = PipelineMarkerInspector()
    try:
        node.run_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
