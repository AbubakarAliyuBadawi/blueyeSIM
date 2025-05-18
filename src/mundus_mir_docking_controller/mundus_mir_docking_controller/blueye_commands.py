import rclpy
from rclpy.node import Node
import blueye.sdk
from blueye_interfaces.msg import BlueyeCommand
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3, TransformStamped
from sensor_msgs.msg import Image, Imu, MagneticField
import blueye.protocol as bp
import math
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float32
#import rospy
#from p2_drone.msg import SystemTime # Message type for time synchronization : ASK AMBJØRN FOR THIS

from tf2_ros import TransformBroadcaster
import time
import numpy as np




'''
This node does the following:

- Initialize Blueye Drone

- Subscribe to BlueyeCommand Messages

- Send blueye commands with the SDK to move the drone

- Subscribe to IMU sensor data via Blueye SDK and publish it on the ROS topic /blueye/imu_enu including the covariance matrices

- Define some Camera stream properties. The stream is initiated with the node blueye_camera_node.py in the blueye_sensors package

- Note that DVL sensor data is gathered by running a separate script called tcp_server.py (located in the top folder of this workspace) 
  which fetches the velocities vx, vy, vz and position xyz from the Waterlinked DVL attached to the Blueye via guestport
  These data are then published on the ROS topics /blueye/dvl_enu and /blueye/odometry respectively


'''


class BlueyeCommandNode(Node):

    def __init__(self):
        super().__init__("blueye_commands_node")

        # Attempts to connect to the blueye
        self.connect_to_blueye()
        
        # Turn on the lights
        self.blueye_drone.lights = 0.2
        
        # Ensure that the camera tilt doesnt change
        # self.blueye_drone.tilt.stabilization_enabled = False
        
        # Turn this on when tuning surge and sway PID controllers
        # self.blueye_drone.motion.auto_depth_active = True
        
        self.battery_current = -1
        
        # OBS: This is turned on in the blueye_commands_callback function
        # self.blueye_drone.motion.auto_heading_active = True
        
        # Create subscriber for a topic called /blueye/battery
        self.battery_subscriber = self.create_subscription(Pose, "/blueye/battery", self.battery_callback, 10)
        
        self.bridge = CvBridge()
        
        # How to use this shit
        self.br = TransformBroadcaster(self)  # TF2 Transform Broadcaster

        # Declaring ROS parameters for the sensors DVL, Camera, Depth, IMU 
        self.declare_parameter("use_dvl", False) # NOTE: fetch DVL sensor data via script tcp_server.py instead 
        self.declare_parameter("use_camera", True)
        self.declare_parameter("use_depth", False)
        self.declare_parameter("use_imu", True)

        # Fetching ROS parameters
        use_dvl = self.get_parameter("use_dvl").get_parameter_value().bool_value
        use_camera = self.get_parameter("use_camera").get_parameter_value().bool_value
        use_depth = self.get_parameter("use_depth").get_parameter_value().bool_value
        use_imu = self.get_parameter("use_imu").get_parameter_value().bool_value
        
        # Flags
        self.depth_valid = False
        self.dvl_valid = False
        self.imu_valid = False
        self.cam_valid = False

        # Define publishers, subscribers and callbacks based on which sensors are available        
        self.battery_publisher = self.create_publisher(Pose, "/blueye/battery", 100)
        self.blueye_drone.telemetry.set_msg_publish_frequency(bp.BatteryBQ40Z50Tel, 10)
        self.blueye_drone.telemetry.add_msg_callback([bp.BatteryBQ40Z50Tel], self.callback_battery)
        
        # Wil hente ut field charging_current eller absolute_state_of_charge
        
        # # DVL (and depth sensor)
        # if use_dvl:
        #     pass

        # Camera
        if use_camera:
            # Note: moved camera streamer to separate node: blueye_camera_node.py in the blueye_sensors package
            
            # TODO: Research these values!!
            # Setting camera settings
            
            for attempt in range(3):
                try:
                    self.blueye_drone.camera.bitrate =  5000000 # 10 Mbit bitrate #TODO: Test med 5 mill
                    self.blueye_drone.camera.resolution = 1080 # Vertical pixels. Can also choose 720
                    self.blueye_drone.camera.framerate = 30 # Can also choose 25
                    break  # Success, exit the retry loop
                except blueye.protocol.exceptions.ResponseTimeout as e:
                    if attempt < 2:  # Allow 2 retries
                        print("Retrying setting bitrate...")
                    else:
                        print("Failed to set camera settings after several attempts.")
                        raise

                
            
        # IMU
        if use_imu:
            self.imu_publisher = self.create_publisher(Imu, "/blueye/imu_enu", 100)
            
            # self.get_logger().info("IMU is used")
            
            self.blueye_drone.telemetry.set_msg_publish_frequency(bp.CalibratedImuTel, 10)
            self.blueye_drone.telemetry.add_msg_callback([bp.CalibratedImuTel], self.callback_imu) # Is this the best option?? Could choose IMU1 or IMU2 as well

            # self.blueye_drone.telemetry.set_msg_publish_frequency(bp.Imu1Tel, 10)
            # self.blueye_drone.telemetry.add_msg_callback([bp.Imu1Tel], self.callback_imu) # Is this the best option?? Could choose IMU1 or IMU2 as well

        # # Depth
        # if use_depth and not use_dvl:
        #     pass

        # Declare blueye protocol callback
        # self.blueye_drone.telemetry.add_msg_callback([bp.PositionEstimateTel], self.callback_dvl)
        # self.blueye_drone.telemetry.add_msg_callback([bp.DepthTel], self.callback_depth)
        
        
        # Declare their publish frequencies
        # self.blueye_drone.telemetry.set_msg_publish_frequency(bp.PositionEstimateTel, 10)
        # self.blueye_drone.telemetry.set_msg_publish_frequency(bp.DepthTel, 10)
        
        
        
        # Define subscriber for blueye commands
        self.command_subscriber = self.create_subscription(
            BlueyeCommand,
            "/blueye/commands",
            self.blueye_commands_callback,
            1)

        # Odometry callback with fixed rate
        # odometry_timer = 0.1
        # self.publish_dvl_odometry = self.create_timer(odometry_timer, self.dvl_and_depth_odometry_callback)
        # self.publish_visual_odometry = self.create_timer(odometry_timer, self.visual_odometry_callback)
        # self.publish_imu_odometry = self.create_timer(odometry_timer, self.imu_odometry_callback)

    
    # Callback for the battery topic
    def battery_callback(self, msg:Pose):
        self.battery_current = msg.position.z
    
    def connect_to_blueye(self):
        connected = False
        while not connected:
            try:
                self.blueye_drone = blueye.sdk.Drone()
                connected = True
            except ConnectionError as err:
                print("Connection error")
        self.get_logger().info("Connected to Blueye drone.")
        
        # Alternative way to connect to the blueye drone:
        # self.blueye_drone = blueye.sdk.Drone()
        # if not self.blueye_drone:
        #     return False
        # return True


     
    def mod(self, x, y):
        return x - math.floor(x/y) * y
    
    # SSA is the "smallest signed angle" or the smallest difference between two angles.
    # Maps the angle to the interval [-pi, pi)
    def ssa(self, angle):
        return self.mod(angle + math.pi, 2 * math.pi) - math.pi 
     
    
    # Sending the incoming commands to the blueye drone for actuation
    def blueye_commands_callback(self, msg):
        
        # Thresholding incoming values
        surge = max(-1, min(msg.surge, 1)) 
        sway = max(-1, min(msg.sway, 1)) 
        heave = max(-1, min(msg.heave, 1)) 
        yaw = max(-1, min(msg.yaw, 1)) 

        # Sending commands
        self.blueye_drone.motion.surge = surge
        self.blueye_drone.motion.sway = sway
        self.blueye_drone.motion.heave = heave
        self.blueye_drone.motion.yaw = yaw
        
        # Does this cause issues?? Does the frequent restart cause bad performance of the auto heading??
        # Purpose: to stop the propellers when we are docked and charging
        if self.battery_current > 0:
            self.blueye_drone.motion.auto_heading_active = False
        else:
            self.blueye_drone.motion.auto_heading_active = True

        
        
    def callback_battery(self, msg_type_name ,msg:bp.BatteryBQ40Z50Tel):
        
        # Log the message type if needed (optional)
        # self.get_logger().info(f'Received message type: {msg_type_name}')
        
        # self.get_logger().info("Battery callback executed")
        
        battery_msg = Pose()
        battery_msg.position.x = msg.battery.charging_current
        battery_msg.position.y = msg.battery.relative_state_of_charge
        battery_msg.position.z = msg.battery.current
        battery_msg.orientation.x = float(msg.battery.runtime_to_empty)
    
        #Publish to the /blueye/battery topic
        self.battery_publisher.publish(battery_msg)
        
       # Stores locally the incoming imu messages
    def callback_imu(self, msg_type_name ,msg:bp.CalibratedImuTel):
        
        # Log the message type if needed (optional)
        # self.get_logger().info(f'Received message type: {msg_type_name}')
        
        # self.get_logger().info("Imu callback executed")
        
        # Note since the Blueye gives IMU data x - forward y - right z - down ("NED") we need to convert to 
        # x - forward y - left z - up ("ENU") by changing the sign of the y and z values because that is what rob_loc EKF wants
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.angular_velocity.x = msg.imu.gyroscope.x
        imu_msg.angular_velocity.y = -msg.imu.gyroscope.y
        imu_msg.angular_velocity.z = -msg.imu.gyroscope.z 
        imu_msg.linear_acceleration.x = msg.imu.accelerometer.x
        imu_msg.linear_acceleration.y = -msg.imu.accelerometer.y
        imu_msg.linear_acceleration.z = -msg.imu.accelerometer.z
        
        # Adding covariance matrices for the measurement noise of the IMU (The R matrix in the Kalman filter)
        
        # Standard deviations for angular velocity
        std_dev_roll = 0.01
        std_dev_pitch = 0.01
        std_dev_yaw = 0.01
        
        # Set angular velocity covariance
        angular_velocity_covariance = np.diag([std_dev_roll**2, std_dev_pitch**2, std_dev_yaw**2])  # [radians^2/s^2]
        imu_msg.angular_velocity_covariance = angular_velocity_covariance.flatten().tolist()

        # Standard deviations for linear acceleration
        std_dev_ax = 0.02
        std_dev_ay = 0.02
        std_dev_az = 0.02

        # Set linear acceleration covariance
        linear_acceleration_covariance = np.diag([std_dev_ax**2, std_dev_ay**2, std_dev_az**2])  # [m^2/s^4]
        imu_msg.linear_acceleration_covariance = linear_acceleration_covariance.flatten().tolist()
        
        
        self.imu_publisher.publish(imu_msg)
    

        # # Now broadcast the transform
        # t = TransformStamped()
        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = 'base_link'
        # t.child_frame_id = 'imu_link'
        # t.transform.translation.x = 1.0  # Adjust these values based on the actual physical location of the IMU
        # t.transform.translation.y = 1.0
        # t.transform.translation.z = 1.0
        # # Assuming no rotation from base_link to imu_link; adjust as necessary
        # t.transform.rotation.x = 0.0
        # t.transform.rotation.y = 0.0
        # t.transform.rotation.z = 0.0
        # t.transform.rotation.w = 1.0
        # self.br.sendTransform(t)

        
        self.imu_valid = True

# ---------------Probably not needed-----------------

    # def dvl_and_depth_odometry_callback(self):
    #     # Checks that both depth and position estimate valid
    #     if (self.depth_valid and self.dvl_valid):
    #         odom_msg = Odometry()
    #         odom_msg.pose.pose.position.x = self.x
    #         odom_msg.pose.pose.position.y = self.y
    #         odom_msg.pose.pose.position.z = self.z
    #         odom_msg.pose.pose.orientation.w = self.heading
            
    #         self.dvl_publisher.publish(odom_msg)
            



    # Position and angular velocity from a mounted DVL
    # def callback_dvl(self, msg_type: str, msg: bp.PositionEstimateTel):
    #     self.x = msg.position_estimate.northing
    #     self.y = msg.position_estimate.easting
    #     print("Surge rate: {}". format(msg.position_estimate.surge_rate))
    #     print("Sway rate: {}". format(msg.position_estimate.sway_rate))
    #     print("Yaw rate: {}". format(msg.position_estimate.yaw_rate))
        
    #     print("Is valid: {}". format(msg.position_estimate.is_valid))

    #     #heading = (msg.position_estimate.heading*math.pi) / 180.0
    #     self.heading = self.ssa(msg.position_estimate.heading)
        
        
    #     self.dvl_valid = True
    
    # def callback_depth(self, msg_type: str, msg: bp.DepthTel):
    #     # print("Depth: {}".format(msg.depth.value))
    #     self.z = msg.depth.value
    
    #     #TODO: Publish to the /blueye/depth topic    
    #     self.depth_valid = True
        
# ---------------------------------------------------
        
        
 



def main(args=None):
    rclpy.init(args=args)
    blueye_commands_node = BlueyeCommandNode()
    rclpy.spin(blueye_commands_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()









    # def imu_odometry_callback(self):
    #     # Checks that valid IMU data is available
    #     if (self.imu_valid):
    #         imu_odom_msg = Odometry()
    #         imu_odom_msg.twist.twist.linear.x = self.u
    #         imu_odom_msg.twist.twist.linear.y = self.v
    #         imu_odom_msg.twist.twist.linear.z = self.w
    #         imu_odom_msg.twist.twist.angular.z = self.yaw_rate

           



    
    # Dont need this because the visual odometry is already published in the pose_estimation_aruco node
    
    # Publishes the visual odometry pose to the topic: /blueye/visual_odometry
    # def visual_odometry_callback(self):
    #     if (self.visual_est_valid):
    #         visual_odom_msg = Odometry()
    #         visual_odom_msg.pose.pose.position.x = self.x_visual_est
    #         visual_odom_msg.pose.pose.position.y = self.y_visual_est
    #         visual_odom_msg.pose.pose.position.z = self.z_visual_est
    #         visual_odom_msg.pose.pose.orientation.w = self.heading_visual_est # [rad]
            
    #         self.visual_odometry_publisher.publish(visual_odom_msg)

    # # Stores locally the incoming pose estimates from the topic /blueye/pose_estimated_board
    # def callback_pose_estimated_board(self, msg:Pose):
    #     self.x_visual_est = msg.position.x
    #     self.y_visual_est = msg.position.y
    #     self.z_visual_est = msg.position.z
    #     self.heading_visual_est = self.ssa(msg.orientation.z) # [rad]
        
    #     self.visual_est_valid = True