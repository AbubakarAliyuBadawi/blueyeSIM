import numpy as np
import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from stonefish_ros2.msg import DVL


class StonefishSensorRepublisher(Node):
    def __init__(self):
        super().__init__("stonefish_sensor_republisher")

        self.declare_parameter("imu_in_topic", "/blueye/imu/data_raw")
        self.declare_parameter("dvl_in_topic", "/blueye/dvl/sim")
        self.declare_parameter("imu_out_topic", "/blueye/imu_enu")
        self.declare_parameter("dvl_out_topic", "/blueye/dvl_enu")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("imu_frame_id", "base_link")
        self.declare_parameter("dvl_velocity_stddev", 0.03)
        self.declare_parameter("imu_angular_velocity_stddev", 0.01)
        self.declare_parameter("imu_linear_acceleration_stddev", 0.02)

        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.imu_frame_id = self.get_parameter("imu_frame_id").value

        self.imu_sub = self.create_subscription(
            Imu, self.get_parameter("imu_in_topic").value, self.imu_callback, qos_profile_sensor_data
        )
        self.dvl_sub = self.create_subscription(
            DVL, self.get_parameter("dvl_in_topic").value, self.dvl_callback, qos_profile_sensor_data
        )
        self.imu_pub = self.create_publisher(Imu, self.get_parameter("imu_out_topic").value, 10)
        self.dvl_pub = self.create_publisher(TwistWithCovarianceStamped, self.get_parameter("dvl_out_topic").value, 10)

    def imu_callback(self, msg):
        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.imu_frame_id
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration

        gyro_std = self.get_parameter("imu_angular_velocity_stddev").value
        accel_std = self.get_parameter("imu_linear_acceleration_stddev").value
        out.angular_velocity_covariance = np.diag([gyro_std**2] * 3).flatten().tolist()
        out.linear_acceleration_covariance = np.diag([accel_std**2] * 3).flatten().tolist()
        self.imu_pub.publish(out)

    def dvl_callback(self, msg):
        out = TwistWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.base_frame_id

        # Stonefish Blueye DVL is mounted with rpy="0 0 1.5708".
        # This maps Stonefish DVL x/y into the Blueye body frame used by the controller.
        out.twist.twist.linear.x = float(msg.velocity.y)
        out.twist.twist.linear.y = float(-msg.velocity.x)
        out.twist.twist.linear.z = float(msg.velocity.z)

        std = self.get_parameter("dvl_velocity_stddev").value
        covariance = [0.0] * 36
        covariance[0] = std**2
        covariance[7] = std**2
        covariance[14] = std**2
        covariance[21] = 999.0
        covariance[28] = 999.0
        covariance[35] = 999.0
        out.twist.covariance = covariance
        self.dvl_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = StonefishSensorRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
