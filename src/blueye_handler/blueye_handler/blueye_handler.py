import sys
import threading
import rclpy
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import blueye.sdk
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import blueye.protocol as bp
from blueye_interfaces.msg import BatteryStatus, FloatStamped, ErrorFlags
from geometry_msgs.msg import QuaternionStamped
from scipy.spatial.transform import Rotation as R
from std_srvs.srv import SetBool, Trigger
from blueye_handler.drone_services import DroneServices
from std_msgs.msg import Float64
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float32


def quaternion_from_euler(roll, pitch, yaw):
    # Convert to radians if needed
    yaw = np.radians(yaw)
    pitch = np.radians(pitch)
    roll = np.radians(roll)

    # Create rotation object with intrinsic (ZYX) order: yaw -> pitch -> roll
    r = R.from_euler("ZYX", [yaw, pitch, roll])

    # Return quaternion (x, y, z, w)
    return r.as_quat()


class BlueyeHandler(Node):

    def __init__(self):
        super().__init__("blueye_commands_node")

        # We do depth
        self.depth = 0.0
        self.heave_rate = 0.0
        self.last_time = self.get_clock().now()

        # We also do heading
        self.heading = 0.0

        # Attempts to connect to the blueye
        self.connect_to_blueye()

        # Define subscribers
        self.command_subscriber = self.create_subscription(
            WrenchStamped, "/blueye/commands", self.blueye_commands_callback, 1
        )

        self.light_subscriber = self.create_subscription(
            Bool, "/blueye/lights", self.light_callback, 1
        )

        self.camera_tilt_subscriber = self.create_subscription(
            Float64, "/blueye/camera_tilt", self.camera_tilt_callback, 1
        )

        # Define publishers
        self.battery_publisher = self.create_publisher(
            BatteryStatus, "blueye/sensor/battery", 1
        )

        self.imu_publisher = self.create_publisher(Imu, "blueye/sensor/imu", 1)

        self.attitude_publisher = self.create_publisher(
            QuaternionStamped, "blueye/sensor/attitude", 1
        )

        self.depth_publisher = self.create_publisher(
            FloatStamped, "blueye/sensor/depth", 1
        )

        self.error_publisher = self.create_publisher(
            ErrorFlags, "blueye/sensor/error_status", 1
        )

        self.position_publisher = self.create_publisher(
            Odometry, "/blueye/sensor/ekf/pose", 1
        )

        self.lat_long_publisher = self.create_publisher(
            Point, "/blueye/sensor/ekf/lat_long", 1
        )

        # BT-compatible publishers
        self.bt_battery_publisher = self.create_publisher(Pose, "/blueye/battery", 1)
        self.bt_speed_publisher = self.create_publisher(Float32, "/blueye/speed", 1)
        self.bt_depth_publisher = self.create_publisher(Float32, "/blueye/depth", 1)
        self.bt_pose_publisher = self.create_publisher(PoseStamped, "/blueye/pose", 1)
        self.bt_altitude_publisher = self.create_publisher(Float32, "/blueye/altitude", 1)

        # Control Mode Services
        self.depth_hold = self.create_service(
            SetBool, "/blueye/depth_hold", self.depth_hold_callback
        )
        self.heading_hold = self.create_service(
            SetBool, "/blueye/heading_hold", self.heading_hold_callback
        )
        self.stationkeep = self.create_service(
            SetBool, "/blueye/stationkeep", self.stationkeep_callback
        )

        # The handler owns the drone for the whole mission: undocking, position reset and the onboard mission run
        # through its services, so no other client takes control (that would drop the handler)
        self.drone_services = DroneServices(self, lambda: self.blueye_drone)
        self.in_control_publisher = self.create_publisher(Bool, "/blueye/handler/in_control", 1)
        self.take_control_service = self.create_service(Trigger, "/blueye/take_control", self.take_control_callback)
        self._reconnect_lock = threading.Lock()
        self._lost_control_logged = False
        self.create_timer(1.0, self.control_watchdog)

        self._setup_drone()

        # Connected
        self.get_logger().info(f"Connected to Blueye as client {self.blueye_drone.client_id}, in control")

    def _setup_drone(self):
        """Telemetry rates and callbacks, and the camera stream, of the current drone connection."""
        # Define telemetry channels
        self.blueye_drone.telemetry.set_msg_publish_frequency(bp.CalibratedImuTel, 100)
        self.blueye_drone.telemetry.set_msg_publish_frequency(bp.DepthTel, 10)
        self.blueye_drone.telemetry.set_msg_publish_frequency(bp.AttitudeTel, 10)
        self.blueye_drone.telemetry.set_msg_publish_frequency(bp.BatteryBQ40Z50Tel, 0.1)
        self.blueye_drone.telemetry.set_msg_publish_frequency(bp.ErrorFlagsTel, 0.1)
        self.blueye_drone.telemetry.set_msg_publish_frequency(bp.PositionEstimateTel, 25)
        self.blueye_drone.telemetry.set_msg_publish_frequency(bp.AltitudeTel, 5)

        # Define callbacks
        self.blueye_drone.telemetry.add_msg_callback(
            [bp.CalibratedImuTel], self.imu_callback
        )
        self.blueye_drone.telemetry.add_msg_callback([bp.DepthTel], self.depth_callback)
        self.blueye_drone.telemetry.add_msg_callback(
            [bp.AttitudeTel], self.attitude_callback
        )
        self.blueye_drone.telemetry.add_msg_callback(
            [bp.BatteryBQ40Z50Tel], self.battery_callback
        )
        self.blueye_drone.telemetry.add_msg_callback(
            [bp.ErrorFlagsTel], self.error_callback
        )
        self.blueye_drone.telemetry.add_msg_callback(
            [bp.PositionEstimateTel], self.position_callback
        )
        self.blueye_drone.telemetry.add_msg_callback(
            [bp.AltitudeTel], self.altitude_callback
        )
        self.blueye_drone.telemetry.add_msg_callback(
            [bp.NotificationTel], self.drone_services.notification_callback
        )

        self._init_camera()

    def control_watchdog(self):
        """Publish whether the handler still controls the drone. Another client taking control drops the handler
        without its SDK noticing; the drone's telemetry says who is in control."""
        in_control = self.blueye_drone.client_in_control == self.blueye_drone.client_id
        msg = Bool()
        msg.data = in_control
        self.in_control_publisher.publish(msg)
        if not in_control and not self._lost_control_logged:
            self.get_logger().error(
                f"Lost control of the drone (client in control: {self.blueye_drone.client_in_control}, "
                f"handler: {self.blueye_drone.client_id}): commands, holds and the joystick have no effect. "
                "Call /blueye/take_control to reclaim it.")
        self._lost_control_logged = not in_control

    def take_control_callback(self, request, response):
        """Reconnect and take control: a client that lost control can only get it back on a new connection."""
        with self._reconnect_lock:
            try:
                try:
                    self.blueye_drone.disconnect()
                except Exception:
                    pass
                self.connect_to_blueye()
                self._setup_drone()
                response.success = True
                response.message = f"in control again as client {self.blueye_drone.client_id}"
            except Exception as exc:
                response.success = False
                response.message = f"reclaiming control failed: {exc}"
        self.get_logger().info(response.message)
        return response

    # Tilts blueye camera
    def camera_tilt_callback(self, msg):
        # Tilt camera
        if (msg.data) > 0.0:

            self.blueye_drone.camera.tilt.set_velocity(0.4)
            current_tilt = self.blueye_drone.camera.tilt.angle
            self.get_logger().info("Current tilt angle: {}".format(current_tilt))
        elif msg.data < 0.0:
            self.blueye_drone.camera.tilt.set_velocity(-0.4)

            current_tilt = self.blueye_drone.camera.tilt.angle
            self.get_logger().info("Current tilt angle: {}".format(current_tilt))
        else:
            self.blueye_drone.camera.tilt.set_velocity(0.0)

    # Activate depth hold
    def depth_hold_callback(self, request, response):
        print("Depth Hold Toogled")
        self.blueye_drone.motion.auto_depth_active = request.data
        response.success = True
        return response

    # Activate heading hold
    def heading_hold_callback(self, request, response):
        print("Heading Hold Toogled")
        self.blueye_drone.motion.auto_heading_active = request.data
        response.success = True
        return response

    # Activate
    def stationkeep_callback(self, request, response):
        print("Stationkeep Toogled")
        self.blueye_drone.motion.station_keeping_active = request.data
        response.success = True
        return response

    def light_callback(self, msg):

        # Turn lights based on
        if msg.data:
            self.blueye_drone.lights = 1.0
        else:
            self.blueye_drone.lights = 0.0

    def position_callback(self, msg_type, msg):

        # Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = msg.position_estimate.northing
        odom_msg.pose.pose.position.y = msg.position_estimate.easting
        odom_msg.pose.pose.position.z = self.depth

        # Yaw to quaternion
        quaternion = quaternion_from_euler(0, 0, self.heading)
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        # Velocity
        odom_msg.twist.twist.linear.x = msg.position_estimate.surge_rate
        odom_msg.twist.twist.linear.y = msg.position_estimate.sway_rate
        odom_msg.twist.twist.linear.z = self.heave_rate
        odom_msg.twist.twist.angular.z = msg.position_estimate.yaw_rate

        # Publish Blueye EKF
        self.position_publisher.publish(odom_msg)

        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = odom_msg.header.stamp
        pose_stamped_msg.header.frame_id = "odom"
        pose_stamped_msg.pose = odom_msg.pose.pose
        self.bt_pose_publisher.publish(pose_stamped_msg)

        # BT-compatible speed topic
        speed_msg = Float32()
        speed_msg.data = float(np.sqrt(
            msg.position_estimate.surge_rate**2 +
            msg.position_estimate.sway_rate**2
        ))
        self.bt_speed_publisher.publish(speed_msg)

        # Publish current estimate of lat long
        if msg.position_estimate.is_valid:
            latitude = msg.position_estimate.global_position.latitude
            longitude = msg.position_estimate.global_position.longitude

            point_msg = Point()
            point_msg.x = latitude
            point_msg.y = longitude
            point_msg.z = self.depth
            self.lat_long_publisher.publish(point_msg)

    def imu_callback(self, msg_type, msg):

        # Fill IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_sensor_frame"
        imu_msg.linear_acceleration.x = msg.imu.accelerometer.x
        imu_msg.linear_acceleration.y = msg.imu.accelerometer.y
        imu_msg.linear_acceleration.z = msg.imu.accelerometer.z
        imu_msg.angular_velocity.x = msg.imu.gyroscope.x
        imu_msg.angular_velocity.y = msg.imu.gyroscope.y
        imu_msg.angular_velocity.z = msg.imu.gyroscope.z

        # Publish message
        self.imu_publisher.publish(imu_msg)

    def attitude_callback(self, msg_type, msg):

        # yaw = msg.attitude.yaw * (math.pi / 180.0)
        self.heading = msg.attitude.yaw 

        # Fill QuaternionStamped message
        attitude_msg = QuaternionStamped()
        attitude_msg.header.stamp = self.get_clock().now().to_msg()
        attitude_msg.header.frame_id = "attitude_sensor_frame"
        quaternion = quaternion_from_euler(0.0, 0.0, self.heading)
        attitude_msg.quaternion.x = quaternion[0]
        attitude_msg.quaternion.y = quaternion[1]
        attitude_msg.quaternion.z = quaternion[2]
        attitude_msg.quaternion.w = quaternion[3]

        # Publish attitude message
        self.attitude_publisher.publish(attitude_msg)

    def depth_callback(self, msg_type, msg):

        # Fill FloatStamped message
        depth_msg = FloatStamped()
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.header.frame_id = "depth_sensor_frame"
        depth_msg.data = msg.depth.value

        # Publish depth message
        self.depth_publisher.publish(depth_msg)

        bt_depth_msg = Float32()
        bt_depth_msg.data = float(msg.depth.value)
        self.bt_depth_publisher.publish(bt_depth_msg)

        # Depth and heave is not part of the PositionEstimateTel message for some reason..
        dt = (self.get_clock().now() - self.last_time).nanoseconds / 1e9
        if dt > 0:
            self.heave_rate = (msg.depth.value - self.depth) / dt
        else:
            self.heave_rate = 0.0
        self.depth = msg.depth.value
        self.last_time = self.get_clock().now()

    def battery_callback(self, msg_type, msg):

        # Fill battery message
        # battery_msg = BatteryStatus()
        # battery_msg.header.stamp = self.get_clock().now().to_msg()
        # battery_msg.header.frame_id = "battery_sensor_frame"
        # battery_msg.total_voltage = msg.battery.voltage.total
        # battery_msg.cell_1_voltage = msg.battery.voltage.cell_1
        # battery_msg.cell_2_voltage = msg.battery.voltage.cell_2
        # battery_msg.cell_3_voltage = msg.battery.voltage.cell_3
        # battery_msg.cell_4_voltage = msg.battery.voltage.cell_4
        # battery_msg.average_temperature = msg.battery.temperature.average
        # battery_msg.cell_1_temperature = msg.battery.temperature.cell_1
        # battery_msg.cell_2_temperature = msg.battery.temperature.cell_2
        # battery_msg.cell_3_temperature = msg.battery.temperature.cell_3
        # battery_msg.cell_4_temperature = msg.battery.temperature.cell_4
        # battery_msg.initialization = msg.battery.status.initialization
        # battery_msg.error_status = msg.battery.status.error
        # battery_msg.current = msg.battery.current
        # battery_msg.average_current = msg.battery.average_current
        # battery_msg.calculated_state_of_charge = msg.battery.calculated_state_of_charge
        # battery_msg.remaining_capacity = msg.battery.remaining_capacity
        # battery_msg.full_charge_capacity = msg.battery.full_charge_capacity
        # battery_msg.runtime_to_empty = msg.battery.runtime_to_empty
        # battery_msg.average_time_to_empty = msg.battery.average_time_to_empty
        # battery_msg.average_time_to_full = msg.battery.average_time_to_full
        ##battery_msg.charging_current = msg.battery.charging_current
        # battery_msg.charging_voltage = msg.battery.charging_voltage
        # battery_msg.cycle_count = msg.battery.cycle_count
        # battery_msg.design_capacity = msg.battery.design_capacity
        # battery_msg.manufacturer_name = msg.battery.manufacturer_name
        # battery_msg.device_name = msg.battery.device_name
        # battery_msg.device_chemistry = msg.battery.device_chemistry
        # battery_msg.calculated_state_of_charge = msg.battery.calculated_state_of_charge

        # Publish battery message
        # self.battery_publisher.publish(battery_msg)

        # BT-compatible battery topic (geometry_msgs/Pose packing)
        soc = msg.battery.relative_state_of_charge
        if soc <= 0.0:
            soc = msg.battery.calculated_state_of_charge
        bt_battery_msg = Pose()
        bt_battery_msg.position.x = msg.battery.charging_current
        bt_battery_msg.position.y = soc
        bt_battery_msg.position.z = msg.battery.current
        bt_battery_msg.orientation.x = float(msg.battery.runtime_to_empty)
        self.bt_battery_publisher.publish(bt_battery_msg)

    def error_callback(self, msg_type, msg):

        # Fill error message
        # error_msg = ErrorFlags()
        # error_msg.header.stamp = self.get_clock().now().to_msg()
        # error_msg.header.frame_id = "error_sensor_frame"
        pass

    def altitude_callback(self, msg_type, msg):
        if not msg.altitude.is_valid:
            return
        altitude_msg = Float32()
        altitude_msg.data = float(msg.altitude.value)
        self.bt_altitude_publisher.publish(altitude_msg)

    def _init_camera(self):
        try:
            self.blueye_drone.camera.bitrate = 5000000
            self.blueye_drone.camera.resolution = 1080
            self.blueye_drone.camera.framerate = 30
            self.get_logger().info("Camera stream activated — 1080p@30fps, 5Mbps")
            self.get_logger().info(
                f"RTSP stream: rtsp://{self.blueye_drone._ip}:8554/test"
            )
        except Exception as exc:
            self.get_logger().warning(f"Camera init failed: {exc}")

    def connect_to_blueye(self):
        self.blueye_drone = blueye.sdk.Drone()
        if not self.blueye_drone:
            self.get_logger().error("Not able to communicate with Blueye. Exiting..")
            sys.exit()
        if not self.blueye_drone.in_control:
            self.blueye_drone.take_control()
        return True

    def blueye_commands_callback(self, msg):
        # Thresholding incoming values
        surge = max(-1, min(msg.wrench.force.x, 1))
        sway = max(-1, min(msg.wrench.force.y, 1))
        heave = max(-1, min(msg.wrench.force.z, 1))
        yaw = max(-1, min(msg.wrench.torque.z, 1))

        # Sending commands
        self.blueye_drone.motion.surge = surge
        self.blueye_drone.motion.sway = sway
        self.blueye_drone.motion.heave = heave
        self.blueye_drone.motion.yaw = yaw


def main(args=None):
    rclpy.init(args=args)
    blueye_handler = BlueyeHandler()
    # several threads: undocking (~20 s) and mission calls must not block the joystick commands or the holds
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(blueye_handler)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        blueye_handler.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
