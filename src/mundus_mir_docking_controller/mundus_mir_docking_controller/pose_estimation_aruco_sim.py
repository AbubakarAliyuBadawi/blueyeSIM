import rclpy 
from rclpy.node import Node
import sys, time, math
import cv2

opencv_version = cv2.__version__
opencv_above_6 = int(opencv_version.split('.')[0]) >= 6

print(f"[INFO] OpenCV version: {opencv_version}")
if opencv_above_6:
    print("[INFO] Using OpenCV 6+ API for ArUco")
else:
    print("[INFO] Using legacy ArUco API (pre-6.0)")

import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Vector3
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
import pyquaternion
from nav_msgs.msg import Odometry
import datetime
from std_srvs.srv import SetBool
import threading
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32




#Importing the aruco board definition
# from image_processing.aruco_board_actual_real import pos_board, id_board

from mundus_mir_docking_controller.aruco_board_5x5 import pos_board_5x5, id_board_5x5
from mundus_mir_docking_controller.aruco_board_4x4 import pos_board_4x4, id_board_4x4
from mundus_mir_docking_controller.aruco_board import pos_board, id_board

#from src/blueye_control/mundus_mir_docking_controller/mundus_mir_docking_controller/aruco_board_4x4.py import pos_board_4x4, id_board_4x4

# Defining Image node
class PoseEstimationAruco(Node):
    ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    }

    # Specify which type of marker you want to detect
    aruco_type = "DICT_4X4_50"
    
    # Here you define your Camera matrix (K-matrix) and the distortion coefficients of your camera
    # Calibrated params for calibration 22/6 2024 
    intrinsic_camera = np.array(((1210.8424, 0, 895.1996),(0, 1212.7456, 511.1417),(0,0,1)))
    distortion = np.array((-0.1779, 0.0220, 0.0035, -0.0023, 0.0811))
    
    # Rotation matrix +180 deg around the y-axis
    rot_matrix_180_y = np.array([
        [-1, 0, 0],
        [0, 1, 0],
        [0, 0, -1]
    ])

    # Rotation matrix +90 deg around the x-axis
    rot_matrix_90_x = np.array([
        [1, 0, 0],
        [0, 0, -1],
        [0, 1, 0]
    ])
    
    # 180 deg rotation matrix around the x axis
    rot_matrix_180_x  = np.zeros((3,3), dtype=np.float32)
    rot_matrix_180_x[0,0] = 1.0
    rot_matrix_180_x[1,1] =-1.0
    rot_matrix_180_x[2,2] =-1.0
    
    #-- Font for the text in the image
    font = cv2.FONT_HERSHEY_PLAIN
    # font = cv2.FONT_HERSHEY_COMPLEX
    #font = cv2.FONT_HERSHEY_COMPLEX_SMALL
    
    # Statistical Counters for tag performance test
    #detection_time = []
    #tot_images = 0
    #tot_detections = 0 
    #tot_correct_detections = 0
    #TP = 0.0
    #FP = 0.0
    #FN = 0.0
    #id_precision = 0.0
    #detection_rate = 0.0
    #avg_detection_time = 0
    
    def __init__(self):
        super().__init__("pose_estimation_aruco_node")
        # Create subscribers
        self.image_subscriber = self.create_subscription(
                Image, "/blueye/camera_1/image_raw", self.image_callback, 1)
        
        
        # Create publisher of an image topic
        self.overlayed_image_publisher = self.create_publisher(Image, "/blueye/overlayed_image", 10)

        # Create publisher of how many Aruco frames are seen in an image
        self.aruco_count_publisher = self.create_publisher(Int32, "/blueye/aruco_count", 10)

        
        #self.overlayed_image_publisher = self.create_publisher(CompressedImage, "/blueye/overlayed_image/compressed", 10)
        #self.image_publisher = self.create_publisher(Image, "/blueye/image", 10)



        # Controller modes 
        self.dock_mode = False
        self.depth_hold = False
        self.heading_hold = False
        self.inspection_clock = False
        self.inspection_counter_clock = False

        self.dock_mode_service = self.create_service(SetBool, "/blueye/dock_mode/viz", self.dock_mode_callback) 
        self.inspection_clock_service = self.create_service(SetBool, "/blueye/inspection_clock/viz", self.inspection_clock_callback) 
        self.inspection_counter_clock_service = self.create_service(SetBool, "/blueye/inspection_counter_clock/viz", self.inspection_counter_clock_callback) 
        self.depth_hold_service = self.create_service(SetBool, "/blueye/depth_hold/viz", self.depth_hold_callback)
        self.heading_hold_service = self.create_service(SetBool, "/blueye/heading_hold/viz", self.heading_hold_callback)
        
        # self.pose_subscriber = self.create_subscription(
        #         Odometry, "/blueye/odometry", self.pose_callback, 1)
        # Create publishers
        # self.pose_euler_gt_pub_ = self.create_publisher(Pose, "/blueye/pose_euler_gt", 10)
        self.pose_estimated_board_pub_ = self.create_publisher(Pose, "/blueye/pose_estimated_board", 10) # Can delete this
        
        self.pose_estimated_board_stamped_pub_ = self.create_publisher(PoseWithCovarianceStamped, "/blueye/pose_estimated_board_stamped", 10)
        
        # Create subscriber to get the current waypoint
        self.current_waypoint_subscriber = self.create_subscription(Pose, "/blueye/current_waypoint", self.current_waypoint_callback, 10)
        #self.current_waypoint_subscriber = self.create_subscription(Odometry, "/blueye/guidance/desired_state", self.current_waypoint_callback, 1)
        
        self.current_waypoint = [0,0,0,0]
        
        self.battery_subscriber = self.create_subscription(Pose, "/blueye/battery", self.battery_callback, 10)
        
        self.battery_current = -1.0
        self.battery_status = 0.0
        
        # Starting subscriber to listen for filtered pose and velocity from EKF
        # self.filtered_odometry_subscriber = self.create_subscription(Odometry, "/odometry/filtered", self.filtered_odometry_callback, 1)
        self.filtered_odometry_subscriber = self.create_subscription(Odometry, "/blueye/navigation/current_state", self.filtered_odometry_callback, 1)
        
        # Aruco relevant stuff
        self.dict_5x5 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        if opencv_above_6:
            self.board_5x5 = cv2.aruco.Board(pos_board_5x5, self.dict_5x5, id_board_5x5)
        else: 
            self.board_5x5 = cv2.aruco.Board(pos_board_5x5, self.dict_5x5, id_board_5x5)

        self.dict_4x4 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if opencv_above_6:
            self.board_4x4 = cv2.aruco.Board(pos_board_4x4, self.dict_4x4, id_board_4x4)
        else:
            self.board_4x4 = cv2.aruco.Board(pos_board_4x4, self.dict_4x4, id_board_4x4)

        if opencv_above_6:
            self.board = cv2.aruco.Board(pos_board, self.dict_4x4, id_board)
        else: 
            self.board = cv2.aruco.Board(pos_board, self.dict_4x4, id_board)

        self.aruco_params = cv2.aruco.DetectorParameters()

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_psi = 0.0
        
        self.bridge = CvBridge()
    
        self.t_tag_camera_gt = np.zeros(3)
        self.tag_camera_euler_angles_gt = np.zeros(3)
        self.yaw_tag_camera_gt = 0.0
        
        #self.timer = self.create_timer(1,self.write_statistical_values_to_file)

        # Starting a camera node in its own thread to do aruco detection and estimation
        #self.rtsp_thread = threading.Thread(target=self.read_rtsp_stream)
        #self.rtsp_thread.daemon = True 
        #self.rtsp_thread.start()

    def image_callback(self, msg:Image):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        output_image = self.pose_estimation(image, self.ARUCO_DICT[self.aruco_type], self.intrinsic_camera, self.distortion)
        #-----------------------------------------------------------
        # Filtered fused pose and waypoint visualization on image:

            
        # OBS: axis is flipped because want to give pose wrt. aruco frame which is drawn in image
        str_position = "Estimated Pose  x=%4.3f  y=%4.3f  z=%4.3f  yaw=%4.3f" % (self.current_x, self.current_y, self.current_z, np.rad2deg(self.current_psi))
        cv2.putText(output_image, str_position, (60, 75), self.font, 1.0, (255, 0, 255), 2, cv2.LINE_AA)

        if self.depth_hold:
            cv2.putText(output_image, "Depth Hold", (60, 400), self.font, 1.0, (0, 0, 255), 2, cv2.LINE_AA)

        if self.heading_hold:
            cv2.putText(output_image, "Heading Hold", (60, 475), self.font, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
        
        if self.dock_mode:
            cv2.putText(output_image, "Dock Mode", (60, 550), self.font, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
        
        if self.inspection_clock:
            cv2.putText(output_image, "Clock Insp", (60, 550), self.font, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
        
        if self.inspection_counter_clock:
            cv2.putText(output_image, "Co.Clock Insp", (60, 550), self.font, 1.0, (0, 0, 255), 2, cv2.LINE_AA)

        if not self.current_waypoint == [0,0,0,0]:
            # Print the gt pose of the camera wrt the tag frame on screen 
            # OBS: axis is flipped because want to give pose wrt. aruco frame which is drawn in image
            str_position = "Current Waypoint  x=%4.3f  y=%4.3f  z=%4.3f  yaw=%4.3f" % (self.current_waypoint[0], self.current_waypoint[1], self.current_waypoint[2], self.current_waypoint[3])
            cv2.putText(output_image, str_position, (60, 150), self.font, 1.0, (0, 255, 255), 2, cv2.LINE_AA)
            
        if self.battery_current > 0:
            percentage = self.battery_status*100
            str_position = "Securely Docked and Charging With Current: %4.3f A, %2.0f %% " % (self.battery_current, percentage)
            cv2.putText(output_image, str_position, (60, 225), self.font, 1.0, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            percentage = self.battery_status*100
            str_position = "%2.0f %% " % (percentage)
            cv2.putText(output_image, str_position, (60, 225), self.font, 1.0, (0, 255, 0), 2, cv2.LINE_AA)
    
        #------- Show Image on Screen ------# 
        cv2.namedWindow("Estimated Pose Image", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Estimated Pose Image", 720, 540)
        cv2.imshow('Estimated Pose Image', output_image)
        # cv2.imwrite()
        key = cv2.waitKey(1) & 0xFF
        
        # Publish original and overlayed image
        #image_msg = self.bridge.cv2_to_imgmsg(og_frame, 'bgr8')
        overlayed_image_msg = self.bridge.cv2_to_imgmsg(output_image, 'bgr8')
        
        self.overlayed_image_publisher.publish(overlayed_image_msg)
        
        # Encode overlayed image as JPEG
        #compressed_msg = CompressedImage()
        #compressed_msg.header.stamp = self.get_clock().now().to_msg()
        #compressed_msg.format = "jpeg"
        #compressed_msg.data = np.array(cv2.imencode('.jpg', output_image)[1]).tobytes()

        #self.overlayed_image_publisher.publish(compressed_msg)

        #self.image_publisher.publish(image_msg)
        
        if key == ord('q'):
            # self.out.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()
    



    def dock_mode_callback(self, request, response):
        self.dock_mode = request.data
        response.success = True
        return response
    
    def inspection_clock_callback(self, request, response):
        self.inspection_clock = request.data
        response.success = True
        return response
    
    def inspection_counter_clock_callback(self, request, response):
        self.inspection_counter_clock = request.data
        response.success = True
        return response

    def depth_hold_callback(self, request, response):
        self.depth_hold = request.data
        response.success = True
        return response

    def heading_hold_callback(self, request, response):
        self.heading_hold = request.data
        response.success = True
        return response 
    
    
    def write_statistical_values_to_file(self):
        #statistical_values = f"True Positives: {self.TP}, False Positives: {self.FP}, False Negatives: {self.FN}, ID Precision: {self.id_precision}, Detection Rate: {self.detection_rate}, Avg. Detection Time: {self.avg_detection_time},\n"
        #with open("statistical_values.txt", "a") as file:
        #    file.write(statistical_values)
        pass
            
    def current_waypoint_callback(self, msg:Pose):
        self.current_waypoint[0] = msg.position.x
        self.current_waypoint[1] = msg.position.y
        self.current_waypoint[2] = msg.position.z
        self.current_waypoint[3] = msg.orientation.w # deg
        
    #def current_waypoint_callback(self, msg:Odometry):
    #    self.current_waypoint[0] = msg.pose.pose.position.x
    #    self.current_waypoint[1] = msg.pose.pose.position.y
    #    self.current_waypoint[2] = msg.pose.pose.position.z

    #    # Extract orientation quaternion
    #    qx = msg.pose.pose.orientation.x
    #    qy = msg.pose.pose.orientation.y
    #    qz = msg.pose.pose.orientation.z
    #    qw = msg.pose.pose.orientation.w
    #    
    #    # Extract yaw angle from the quaternion 
    #    siny_cosp = 2 * (qw * qz + qx * qy)
    #    cosy_cosp = 1 - 2 * (qy**2 + qz**2)
    #    self.current_waypoint[3] = math.atan2(siny_cosp, cosy_cosp)

    def battery_callback(self, msg:Pose):
        self.battery_current = msg.position.z # Current field [A]
        self.battery_status = msg.position.y # percentage battery 
        
    def filtered_odometry_callback(self, msg:Odometry):
        
        # Retriving the position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        
        # Extract orientation quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Extract yaw angle from the quaternion 
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy**2 + qz**2)
        self.current_psi = math.atan2(siny_cosp, cosy_cosp)

        # Print current pose from EKF to see if its flipped from aruco
        if (False):
            self.get_logger().info("Current Pose from EKF: X: {}, Y: {}, Z: {}, Yaw: {}".format(self.current_x, self.current_y, self.current_z, np.rad2deg(self.current_psi)))

    # Callback function that is executed when we receive gt pose fra gazebo topicet model/blueye/odometry       
    def pose_callback(self, msg:Odometry):
        
        # self.get_logger().info("Odometry Received")
        
        # Extract position data
        position = msg.pose.pose.position
        x = position.x
        y = position.y
        z = position.z

        # Extract orientation data
        orientation = msg.pose.pose.orientation
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w
        
        #Convert quaternion to Euler angles (roll, pitch, yaw)
        # Create a Rotation object from the quaternion
        rotation = Rotation.from_quat([qx, qy, qz, qw])
        

        # Get Euler angles in radians. euler is a numpy array containing roll, pitch yaw in specified order
        # euler[2] = yaw angle in radians wrt world frame
        euler = rotation.as_euler('xyz')  # Adjust order if needed
        
        #self.get_logger().info("GT Euler angles: X: {}, Y: {}, Z: {}".format(euler[0], euler[1], euler[2]))
        
        
        # -----------------------------------------------------------------------------------------------------
        #           Converts to gt pose of the camera relative to the tag frame 
        # -----------------------------------------------------------------------------------------------------
        
        
        R_world_blueye_gt = rotation.as_matrix()
        t_world_blueye_gt = np.array([x,y,z])
        T_world_blueye_gt =  np.zeros((4,4), dtype=float)
        T_world_blueye_gt[:3,:3] = R_world_blueye_gt
        T_world_blueye_gt[:3,3] = t_world_blueye_gt
        T_world_blueye_gt[3,3] = 1
        
        #t^world_dock: position of dock frame wrt world frame expressed in world frame
        R_world_dock =  self.rot_matrix_90_x
        t_world_dock =  np.array([0,0,-97.3])
        T_world_dock =  np.zeros((4,4), dtype=float)
        T_world_dock[:3,:3] = R_world_dock
        T_world_dock[:3,3] = t_world_dock
        T_world_dock[3,3] = 1
        
        R_dock_tag = self.rot_matrix_180_y
        t_dock_tag = np.array([1.4925, 1.9199, 1.176])  # Origo of tag board is in center of tag 0
        T_dock_tag =  np.zeros((4,4), dtype=float)
        T_dock_tag[:3,:3] = R_dock_tag
        T_dock_tag[:3,3] = t_dock_tag
        T_dock_tag[3,3] = 1
        
    
        R_camera_blueye = np.array([[0, 0, -1], # (R +90 deg around x-axis) @ (R -90 deg around y-axis)
                                    [-1, 0, 0],
                                    [0, 1, 0]
                                     ])
        t_camera_blueye = np.array([0.0015, 0.0875, -0.209])
        #t_camera_blueye = np.array([0.002325, 0.18, -0.3]) # New ambjørn model # LET THIS MATCH THE CAMERA POSE IN GAZEBO MODEL.SDF FILE, BUT TRANSFORMED FROM GAZEBO BLUEYE FRAME TO DEFINES BLUEYE FRAME AS CAMERA FRAME ORIENTATION 0.3 0.002325 0.18   -> 0.002325 0.18 -0.3
        T_camera_blueye =  np.zeros((4,4), dtype=float)
        T_camera_blueye[:3,:3] = R_camera_blueye
        T_camera_blueye[:3,3] = t_camera_blueye
        T_camera_blueye[3,3] = 1
        
        # fortsett og sjekk her!!
        T_tag_camera_gt = np.linalg.inv(T_dock_tag) @ np.linalg.inv(T_world_dock) @ T_world_blueye_gt @ np.linalg.inv(T_camera_blueye)
        
        R_tag_camera_gt = T_tag_camera_gt[:3,:3]
        self.t_tag_camera_gt = T_tag_camera_gt[:3,3]
        
        rotation_object = Rotation.from_matrix(R_tag_camera_gt)
        self.tag_camera_euler_angles_gt = rotation_object.as_euler('xyz', degrees=False)

        print(self.tag_camera_euler_angles_gt)
        
        self.yaw_tag_camera_gt = -self.tag_camera_euler_angles_gt[0] + math.pi/2 # The yaw angle wrt tag frame
        
        # self.get_logger().info("Tag-camera-gt: X: {}, Y: {}, Z: {}".format(self.t_tag_camera_gt[0], self.t_tag_camera_gt[1], self.t_tag_camera_gt[2]))
        # self.get_logger().info("Tag-camera-gt euler angles: roll: {}, pitch: {}, yaw: {}".format(self.tag_camera_euler_angles_gt[0], self.tag_camera_euler_angles_gt[1], self.tag_camera_euler_angles_gt[2]))
        
        
        # -----------------------------------------------------------------------------------------------------
        #               Publishing the ground truth (gt) pose of the camera relative to the tag frame
        # -----------------------------------------------------------------------------------------------------
        
        # Create a Pose message
        pose_euler_gt_msg = Pose()
        
        # Set the position
        pose_euler_gt_msg.position.x = self.t_tag_camera_gt[0]
        pose_euler_gt_msg.position.y = self.t_tag_camera_gt[1]
        pose_euler_gt_msg.position.z = self.t_tag_camera_gt[2]
        
        # Set the orientation with euler angles (xyz). The orientation attribute is made for quaternions, but use instead euler angles here
        pose_euler_gt_msg.orientation.x = self.tag_camera_euler_angles_gt[0]
        pose_euler_gt_msg.orientation.y = self.tag_camera_euler_angles_gt[1]
        pose_euler_gt_msg.orientation.z = self.tag_camera_euler_angles_gt[2] 
        pose_euler_gt_msg.orientation.w = self.yaw_tag_camera_gt # This is the yaw angle we're interested in (wrt tag frame)

        self.pose_euler_gt_pub_.publish(pose_euler_gt_msg)
        
        # -----------------------------------------------------------------------------------------------------
            
            
    """
    # Callback function that is executed when we receive an image from the gazebo topic blueye/front_camera   
    def image_callback(self, msg:Image):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        output_image = self.pose_estimation(image, self.ARUCO_DICT[self.aruco_type], self.intrinsic_camera, self.distortion)
        #-----------------------------------------------------------
        # Filtered fused pose and waypoint visualization on image:
        
        # OBS: axis is flipped because want to give pose wrt. aruco frame which is drawn in image
        str_position = "Estimated Pose  x=%4.3f  y=%4.3f  z=%4.3f  yaw=%4.3f" % (self.current_y, self.current_z, self.current_x, np.rad2deg(self.current_psi))
        cv2.putText(output_image, str_position, (260, 75), self.font, 2, (255, 0, 255), 2, cv2.LINE_AA)

        if self.depth_hold:
            cv2.putText(output_image, "Depth Hold", (260, 400), self.font, 2, (0, 0, 255), 2, cv2.LINE_AA)

        if self.heading_hold:
            cv2.putText(output_image, "Heading Hold", (260, 475), self.font, 2, (0, 0, 255), 2, cv2.LINE_AA)
        
        if self.dock_mode:
            cv2.putText(output_image, "Dock Mode", (260, 550), self.font, 2, (0, 0, 255), 2, cv2.LINE_AA)

        if not self.current_waypoint == [0,0,0,0]:
            # Print the gt pose of the camera wrt the tag frame on screen 
            # OBS: axis is flipped because want to give pose wrt. aruco frame which is drawn in image
            str_position = "Current Waypoint  x=%4.3f  y=%4.3f  z=%4.3f  yaw=%4.3f" % (self.current_waypoint[1], self.current_waypoint[2], self.current_waypoint[0], self.current_waypoint[3])
            cv2.putText(output_image, str_position, (260, 150), self.font, 2, (0, 255, 255), 2, cv2.LINE_AA)
            
        if self.battery_current > 0:
            percentage = self.battery_status*100
            str_position = "Securely Docked and Charging With Current: %4.3f A, %2.0f %% " % (self.battery_current, percentage)
            cv2.putText(output_image, str_position, (260, 225), self.font, 2, (0, 255, 0), 2, cv2.LINE_AA)
        else: 
            percentage = self.battery_status*100
            str_position = "%2.0f %% " % (percentage)
            cv2.putText(output_image, str_position, (260, 225), self.font, 2, (0, 255, 0), 2, cv2.LINE_AA)
            
            
        #------------------------------------------------------------
        
        
        cv2.imshow('Estimated Pose Image', output_image)
        # cv2.imwrite()
        key = cv2.waitKey(1) & 0xFF
        
        # Publish overlayed image
        overlayed_image_msg = self.bridge.cv2_to_imgmsg(output_image, 'bgr8')
        self.overlayed_image_publisher.publish(overlayed_image_msg)
        
        if key == ord('q'):
            # self.out.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()
        
        # Uncomment if want to save video of simulator
        # self.out.write(output_image)
        
        
        # Statistical parameters for tag performance test
        #self.tot_images +=1
        #self.TP = self.tot_correct_detections # True positives, because tag is present in every image
        #self.FP = self.tot_detections - self.tot_correct_detections # False positives
        #self.FN = self.tot_images - self.tot_correct_detections # False negatives
        #if self.tot_detections != 0: 
        #    self.id_precision = self.TP/self.tot_detections
        #else:
        #    self.id_precision = None
        #if self.tot_images != 0:
        #    self.detection_rate = self.TP/self.tot_images
        #else:
        #    self.detection_rate = None
    """
    
    # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6


    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ). So here the order is xyz??
    def rotationMatrixToEulerAngles(self, R):
        assert (self.isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])
    
    # with corner point coordinates
    @staticmethod
    def aruco_display(corners, ids, rejected, image, rvec, tvec):
        if len(corners) > 0:            
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # Convert to int for drawing
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))

                # Draw marker edges
                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

                # Label the ID
                cv2.putText(image, f"ID:{markerID}", (topLeft[0], topLeft[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Overlay coordinates of each corner
                cv2.putText(image, f"{topLeft}", (topLeft[0]-40, topLeft[1]-10),
                            cv2.FONT_HERSHEY_PLAIN, 0.6, (255, 0, 0), 1)
                cv2.putText(image, f"{topRight}", (topRight[0]+5, topRight[1]-10),
                            cv2.FONT_HERSHEY_PLAIN, 0.6, (255, 0, 0), 1)
                cv2.putText(image, f"{bottomRight}", (bottomRight[0]+5, bottomRight[1]+15),
                            cv2.FONT_HERSHEY_PLAIN, 0.6, (255, 0, 0), 1)
                cv2.putText(image, f"{bottomLeft}", (bottomLeft[0]-40, bottomLeft[1]+15),
                            cv2.FONT_HERSHEY_PLAIN, 0.6, (255, 0, 0), 1)

        return image
    """
    # Label the tags with ID
    @staticmethod
    def aruco_display(corners, ids, rejected, image, rvec, tvec):
        if len(corners) > 0:            
            
            # Transforms a multidimensional array into a 1d-array
            ids = ids.flatten()
            # Iterates over two sets of data simultaneously, 
            # automatically creating matching tuples with 4 corners and 1 Id
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # Takes out the corner coords casted to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                # Label the Aruco with ID
                cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                #-- Print the tag position in camera frame
                #str_position = "Marker Position x =%4.2f  y =%4.2f  z =%4.2f"%(tvec[0], tvec[1], tvec[2])
                #cv2.putText(image, str_position, (topLeft[0], topLeft[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
                
                #print("[Inference] ArUco marker ID: {}".format(markerID))
        return image
    """
    # This function is executed for every image received    
    def pose_estimation(self, frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
        
        # frame = image
        # Converting picture from BGR to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        corners_5x5, ids_5x5, rejected_5x5 = cv2.aruco.detectMarkers(gray, self.dict_5x5, parameters=self.aruco_params)
        corners_4x4, ids_4x4, rejected_4x4 = cv2.aruco.detectMarkers(gray, self.dict_4x4, parameters=self.aruco_params)

        # -----------------------------------------------------------------------------------
        #                     Aruco Board Pose Estimation
        #------------------------------------------------------------------------------------
    
        best_pose = None
        best_count = 0
        corners_all = []
        ids_all = []
        rejected_all = []

        # Defines initial guesses
        rvec_initial_guess = np.zeros((3, 1), dtype=np.float32)
        tvec_initial_guess = np.zeros((3, 1), dtype=np.float32)

        # Collect 5x5 detections
        if ids_5x5 is not None and len(ids_5x5) > 0:
            corners_all.extend(corners_5x5)
            ids_all.extend(ids_5x5)
            rejected_all.extend(rejected_5x5)

        # Collect 4x4 detections
        if ids_4x4 is not None and len(ids_4x4) > 0:
            corners_all.extend(corners_4x4)
            ids_all.extend(ids_4x4)
            rejected_all.extend(rejected_4x4)

        # Convert ids_all back into numpy array if not empty
        if len(ids_all) > 0:
            ids_all = np.array(ids_all, dtype=np.int32)
        
        # Publish the amount of observed Aruco boards here: 
        msg_aruco = Int32()
        msg_aruco.data = len(ids_all)
        self.aruco_count_publisher.publish(msg_aruco)

        """# --- Try 5x5 board ---
        
        if ids_5x5 is not None and len(ids_5x5) > 0:
            ok5, rvec5, tvec5 = cv2.aruco.estimatePoseBoard(
                corners_5x5, ids_5x5, self.board_5x5,
                matrix_coefficients, distortion_coefficients,
                rvec_initial_guess, tvec_initial_guess
            )
            if ok5 > 0:  # means some markers contributed
                best_pose = (rvec5, tvec5, "5x5")
                best_count = len(ids_5x5)
        
        # --- Try 4x4 board ---
        if ids_4x4 is not None and len(ids_4x4) > 0:
            ok4, rvec4, tvec4 = cv2.aruco.estimatePoseBoard(
                corners_4x4, ids_4x4, self.board_4x4,
                matrix_coefficients, distortion_coefficients,
                rvec_initial_guess, tvec_initial_guess
            )
            if ok4 > 0 and len(ids_4x4) > best_count:
                best_pose = (rvec4, tvec4, "4x4")
                best_count = len(ids_4x4)"""

        # --- Try both together ---
        if ids_all is not None and len(ids_all) > 0:
            ok, rvec, tvec = cv2.aruco.estimatePoseBoard(
                corners_all, ids_all, self.board,
                matrix_coefficients, distortion_coefficients,
                rvec_initial_guess, tvec_initial_guess
            )
            if ok > 0:
                best_pose = (rvec, tvec, "")
                best_count = len(ids_all)
        pose_blueye_camera_wrt_tag_est = np.zeros(4)
        
        if best_pose is not None:
            rvec_board, tvec_board, board_type = best_pose
        else: 
            return frame        
        
        # Obtain the rotation matrix tag->camera -- Board
        R_camera_tag    = np.matrix(cv2.Rodrigues(rvec_board)[0]) 
        R_tag_camera    = R_camera_tag.T  
        
        R_tag_camera = R_tag_camera
        t_tag_camera = (-R_tag_camera @ np.array(tvec_board).reshape(-1,1)).flatten()
        
        # Get the attitude of camera frame wrt flipped tag frame(or vice verca?????), in terms of euler 321 (Needs to be flipped first)    
        roll_camera_board, pitch_camera_board, yaw_camera_board = self.rotationMatrixToEulerAngles(self.rot_matrix_180_x*R_tag_camera)

        # Adjust yaw relative to world frame
        yaw_blueye_wrt_tag = -pitch_camera_board # - np.pi

        if (abs(roll_camera_board) > 0.5) and (abs(yaw_camera_board) > 0.5):
            if (t_tag_camera[0,0] < 0):
                yaw_blueye_wrt_tag = np.pi/2.0 + (np.pi/2.0 - yaw_blueye_wrt_tag)
            else:
                yaw_blueye_wrt_tag = -np.pi/2.0 - (np.pi/2.0 + yaw_blueye_wrt_tag) 

        if (yaw_blueye_wrt_tag < -np.pi):
            yaw_blueye_wrt_tag += 2 * np.pi
        elif (yaw_blueye_wrt_tag > np.pi):
            yaw_blueye_wrt_tag -= 2 * np.pi
        
        # Pitch_camera is the yaw angle because the y-axis of the aruco frame is upwards
        # yaw_blueye is wrt world frame. Minus pi/2 is because the aruco tags are rotated pi/2 relative to world frame
        # The minus is because our flipped tag frame has its y-axis(ref pitch) pointing opposite of world frames z-axis
        
        pose_blueye_camera_wrt_tag_est = np.array([t_tag_camera[0,0], t_tag_camera[0,1], t_tag_camera[0,2], yaw_blueye_wrt_tag], dtype=float)
        
        # ------------------------------------------------------------------------------------------
        #                Visualize the detected tags and their coordinate frame
        # ------------------------------------------------------------------------------------------
        
        cv2.aruco.drawDetectedMarkers(frame, corners_all) 
        # Adding Id-label
        frame = self.aruco_display(corners_all, ids_all, rejected_all, frame, rvec_board, tvec_board)
        #frame = self.aruco_display(corners_4x4, ids_4x4, rejected_4x4, frame, rvec_board, tvec_board)
        
        #-------------------------------------------------------------------------------------------
        
        
        #-------------------------------------------------------------------------------------------
        #               Publish the estimated pose of the camera wrt the tag frame
        #-------------------------------------------------------------------------------------------
        
        # Ensures that the list of arrays is not empty
        # This is for the aruco board method
        if not np.all(pose_blueye_camera_wrt_tag_est == 0): 
            
            # Publish the estimated pose as a pose msg object
            pose_estimated_board_msg = Pose()
            
            pose_estimated_board_msg.position.x = float(pose_blueye_camera_wrt_tag_est[0])
            pose_estimated_board_msg.position.y = float(pose_blueye_camera_wrt_tag_est[1])
            pose_estimated_board_msg.position.z = float(pose_blueye_camera_wrt_tag_est[2])
            pose_estimated_board_msg.orientation.z = float(pose_blueye_camera_wrt_tag_est[3]) #Yaw angle rel tag frame [rad]
            
            self.pose_estimated_board_pub_.publish(pose_estimated_board_msg)
            
            
            # Publish the estimated pose as a pose stamped msg object for the Robot_localization package (Kalmann filter)
            # Uncomment to see if flip happens in Aruco Code or ekf filter
            if (False):
                self.get_logger().info("Pose Blueye camera wrt tag: {}".format(pose_blueye_camera_wrt_tag_est))
                self.get_logger().info("Yaw Blueye camera wrt tag: {}".format(pose_blueye_camera_wrt_tag_est[3]))
            
            pose_stamped = PoseWithCovarianceStamped()
            
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "tag"
            
            pose_stamped.pose.pose.position.x = float(pose_blueye_camera_wrt_tag_est[2])
            pose_stamped.pose.pose.position.y = float(pose_blueye_camera_wrt_tag_est[0])
            pose_stamped.pose.pose.position.z = float(pose_blueye_camera_wrt_tag_est[1])
            
            # Only publish the yaw angle, but needs to be converted to quaternion
            # Note the - pi because the aruco tag is rotated 180 deg around the z-axis of the new rob_loc tag frame*
            # Also this new frame has z upwards and therefore the [0,0,1] in axis of rotation for yaw
            quat = pyquaternion.Quaternion(axis=[0,0,1], radians=(pose_blueye_camera_wrt_tag_est[3] + np.pi)).normalised # ????????????- np.pi or + np.pi # Stod også på null og funka
            
            pose_stamped.pose.pose.orientation.x = quat.x
            pose_stamped.pose.pose.orientation.y = quat.y
            pose_stamped.pose.pose.orientation.z = quat.z
            pose_stamped.pose.pose.orientation.w = quat.w
            
            # Adding covariance matrix (R) for the measurements
            
            # Standard deviations for position measurements (x, y, z) and yaw done by Computer Vision
            std_dev_pos = 0.00001  # Standard deviation for position (meters) Assuming equal for x, y and z
            std_dev_yaw = 0.00002  # Standard deviation for measured yaw (radians) OR 0.00003
            
            covariance = covariance = [ std_dev_pos**2, 0.0, 0.0, 0.0, 0.0, 0.0,  # Covariance for linear velocity x
                                        0.0, std_dev_pos**2, 0.0, 0.0, 0.0, 0.0,  # Covariance for linear velocity y
                                        0.0, 0.0, std_dev_pos**2, 0.0, 0.0, 0.0,  # Covariance for linear velocity z
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,             # Covariance for roll (unmeasured)
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,             # Covariance for pitch (unmeasured)
                                        0.0, 0.0, 0.0, 0.0, 0.0, std_dev_yaw**2]  # Covariance for yaw 
            
            pose_stamped.pose.covariance = covariance
            
            self.pose_estimated_board_stamped_pub_.publish(pose_stamped)
        #-------------------------------------------------------------------------------------------
                
            
                    
                
        return frame
    
def main():
    rclpy.init()
    pose_estimation_aruco_node = PoseEstimationAruco()
    rclpy.spin(pose_estimation_aruco_node)
    pose_estimation_aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
