import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np

# Imports a msg type from the workspace: gazebo_garden_auv_simulator
# Note: Need to source this workspace in the terminal before running this node!!!
# from bluerov_interfaces.msg import DVL
from marine_acoustic_msgs.msg import Dvl

# This node adds covariances and timestamps to the imu and dvl messages and republishes them in enu frame, that robot_localization wants

class DvlAndImuRepublisherNode(Node):
    def __init__(self):
        super().__init__('repub_node')

        self.imu_subscriber = self.create_subscription(
        Imu, "/blueye/imu", self.imu_callback, 1)

        self.dvl_subscriber = self.create_subscription(
        Dvl, "/blueye/dvl", self.dvl_callback, 1)
        
        self.imu_publisher = self.create_publisher(Imu, "/blueye/imu_enu", 10)
        
        self.dvl_publisher = self.create_publisher(TwistWithCovarianceStamped, "/blueye/dvl_enu", 10)   
        
    def imu_callback(self, msg:Imu):
        
        # Note: the imu message from gazebo was already in enu frame
        
        enu_imu_msg = msg
        
        enu_imu_msg.header = msg.header
        enu_imu_msg.header.stamp = self.get_clock().now().to_msg()
        enu_imu_msg.header.frame_id = "imu_link"
        
        # Standard deviations for angular velocity, as set in the gazebo simulation
        std_dev_roll = 0.001
        std_dev_pitch = 0.001
        std_dev_yaw = 0.001
        
        # Earlier: (as in real life)
        # std_dev_roll = 0.01
        # std_dev_pitch = 0.01
        # std_dev_yaw = 0.01
        
        # Set angular velocity covariance
        angular_velocity_covariance = np.diag([std_dev_roll**2, std_dev_pitch**2, std_dev_yaw**2])  # [radians^2/s^2]
        enu_imu_msg.angular_velocity_covariance = angular_velocity_covariance.flatten().tolist()

        # Standard deviations for linear acceleration, as set in the gazebo simulation
        std_dev_ax = 0.01
        std_dev_ay = 0.01
        std_dev_az = 0.01
        
        # Earlier: (as in real life)
        # std_dev_ax = 0.02
        # std_dev_ay = 0.02
        # std_dev_az = 0.02

        # Set linear acceleration covariance
        linear_acceleration_covariance = np.diag([std_dev_ax**2, std_dev_ay**2, std_dev_az**2])  # [m^2/s^4]
        enu_imu_msg.linear_acceleration_covariance = linear_acceleration_covariance.flatten().tolist()
        
        self.imu_publisher.publish(enu_imu_msg)
        
    def dvl_callback(self, msg:Dvl):
        
        # Note: the dvl message is in ned frame from gazebo
        # x: forward, y: right, z: down
        # Want to change to x: forward y: left z: up

        dvl_stamped = TwistWithCovarianceStamped()
        
        dvl_stamped.header.stamp = self.get_clock().now().to_msg()
        dvl_stamped.header.frame_id = "base_link"
        
        dvl_stamped.twist.twist.linear.x = msg.velocity.x
        dvl_stamped.twist.twist.linear.y = -msg.velocity.y
        dvl_stamped.twist.twist.linear.z = -msg.velocity.z
        
        # Adding covariance matrix (R) for the measurements
        
        std_dev_vx = 0.01  # Example standard deviation for vx
        std_dev_vy = 0.01  # Example standard deviation for vy
        std_dev_vz = 0.01  # Example standard deviation for vz
        
        covariance = [ std_dev_vx**2, 0.0, 0.0, 0.0, 0.0, 0.0,  # Covariance for linear velocity x
                        0.0, std_dev_vy**2, 0.0, 0.0, 0.0, 0.0,  # Covariance for linear velocity y
                        0.0, 0.0, std_dev_vz**2, 0.0, 0.0, 0.0,  # Covariance for linear velocity z
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,            # Angular velocities are assumed to be unmeasured
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0            ]

        dvl_stamped.twist.covariance = covariance       
        
        
        self.dvl_publisher.publish(dvl_stamped)     
        
        

def main(args=None):
    rclpy.init(args=args)                       # Initialize the ROS communication
    repub_node = DvlAndImuRepublisherNode()     # Create the ROS node
    rclpy.spin(repub_node)                      # Keep the node alive
    repub_node.destroy_node()
    rclpy.shutdown()                            # Shutdown the ROS client library

if __name__ == '__main__':
    main()