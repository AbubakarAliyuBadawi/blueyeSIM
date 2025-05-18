import rclpy 
from rclpy.node import Node
import time, math
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import pyquaternion
import warnings
from std_msgs.msg import Bool

warnings.filterwarnings('error', category=DeprecationWarning)


#Importing the aruco board definition
from mundus_mir_docking_controller.aruco_board_actual_sim import pos_board, id_board

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
    aruco_type = "DICT_5X5_50"
    
    # Here you define your Camera matrix (K-matrix) and the distortion coefficients of your camera
    # These values are obtained from camera calibration (Here I use the ones listed in gz topic -e -t /blueye/camera_info)
    intrinsic_camera = np.array(((1211.7323684692383, 0, 960),(0,1211.7322969436646, 540),(0,0,1)))
    distortion = np.array((0.0,0.0,0.0,0.0,0.0))
    
    # 180 deg rotation matrix around the x axis
    rot_matrix_180_x  = np.zeros((3,3), dtype=float)
    rot_matrix_180_x[0,0] = 1.0
    rot_matrix_180_x[1,1] =-1.0
    rot_matrix_180_x[2,2] =-1.0
    
    #-- Font for the text in the image
    # font = cv2.FONT_HERSHEY_PLAIN
    font = cv2.FONT_HERSHEY_COMPLEX_SMALL
    
    
    # Statistical Counters for tag performance test
    detection_time = []
    tot_images = 0
    tot_detections = 0 
    tot_correct_detections = 0
    TP = 0.0
    FP = 0.0
    FN = 0.0
    id_precision = 0.0
    detection_rate = 0.0
    avg_detection_time = 0
    
    def __init__(self):
        super().__init__("pose_estimation_aruco_node")
        # Create subscribers
        self.image_subscriber = self.create_subscription(
                Image, "/blueye/camera_1/image_raw", self.image_callback, 1)
        
        self.current_waypoint_subscriber = self.create_subscription(Pose, "/blueye/current_waypoint", self.current_waypoint_callback, 10)
        self.current_waypoint = [0,0,0,0]
        
        self.filtered_odometry_subscriber = self.create_subscription(Odometry, "/odometry/filtered", self.filtered_odometry_callback, 1)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_psi = 0.0
        
        # Create publishers
        self.pose_euler_gt_pub_ = self.create_publisher(Pose, "/blueye/pose_euler_gt", 10)
        self.pose_estimated_board_pub_ = self.create_publisher(Pose, "/blueye/pose_estimated_board", 10) # Can delete this
        
        # 1) New publisher for docking detection
        self.docking_pub = self.create_publisher(Bool, "/blueye/docking_station_detected", 10)
        
        self.pose_estimated_board_stamped_pub_ = self.create_publisher(PoseWithCovarianceStamped, "/blueye/pose_estimated_board_stamped", 10)
        
        self.bridge = CvBridge()
    
        self.t_tag_camera_gt = np.zeros(3)
        self.tag_camera_euler_angles_gt = np.zeros(3)
        self.yaw_tag_camera_gt = 0.0
        
        self.timer = self.create_timer(1,self.write_statistical_values_to_file)
        
    
    def mod(self, x, y):
        return x - math.floor(x/y) * y
    
    # SSA is the "smallest signed angle" or the smallest difference between two angles.
    # Maps the angle to the interval [-pi, pi)
    def ssa(self, angle):
        return self.mod(angle + math.pi, 2 * math.pi) - math.pi 
    
    def write_statistical_values_to_file(self):
        statistical_values = f"True Positives: {self.TP}, False Positives: {self.FP}, False Negatives: {self.FN}, ID Precision: {self.id_precision}, Detection Rate: {self.detection_rate}, Avg. Detection Time: {self.avg_detection_time},\n"
        with open("statistical_values.txt", "a") as file:
            file.write(statistical_values)
            
    def current_waypoint_callback(self, msg:Pose):
        self.current_waypoint[0] = msg.position.x
        self.current_waypoint[1] = msg.position.y
        self.current_waypoint[2] = msg.position.z
        self.current_waypoint[3] = msg.orientation.w # deg
 
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
            
            
    # Callback function that is executed when we receive an image from the gazebo topic blueye/front_camera   
    def image_callback(self, msg:Image):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        output_image = self.pose_estimation(image, self.ARUCO_DICT[self.aruco_type], self.intrinsic_camera, self.distortion)
        
        #-----------------------------------------------------------
        # Filtered fused pose, Ground truth pose and waypoint visualization on image:
        
        # OBS: axis is flipped because want to give pose wrt. aruco frame which is drawn in image
        str_position = "Estimated Pose     x=%4.3f  y=%4.3f  z=%4.3f  yaw=%4.3f" % (self.current_y, self.current_z, self.current_x, np.rad2deg(self.current_psi))
        cv2.putText(output_image, str_position, (180, 75), self.font, 2, (215, 0, 215), 2, cv2.LINE_AA)
        
        if not self.current_waypoint == [0,0,0,0]:
            # Print the gt pose of the camera wrt the tag frame on screen 
            # OBS: axis is flipped because want to give pose wrt. aruco frame which is drawn in image
            str_position = "Current Waypoint   x=%4.3f  y=%4.3f  z=%4.3f  yaw=%4.3f" % (self.current_waypoint[1], self.current_waypoint[2], self.current_waypoint[0], self.current_waypoint[3])
            cv2.putText(output_image, str_position, (180, 150), self.font, 2, (0, 255, 255), 2, cv2.LINE_AA)
            
        
        #------------------------------------------------------------
        
    
        cv2.imshow('Estimated Pose Image', output_image)
        key = cv2.waitKey(1) & 0xFF
        
        
        if key == ord('q'):
            self.out.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()       
        
        # Statistical parameters for tag performance test
        self.tot_images +=1
        self.TP = self.tot_correct_detections # True positives, because tag is present in every image
        self.FP = self.tot_detections - self.tot_correct_detections # False positives
        self.FN = self.tot_images - self.tot_correct_detections # False negatives
        if self.tot_detections != 0: 
            self.id_precision = self.TP/self.tot_detections
        else:
            self.id_precision = None
        if self.tot_images != 0:
            self.detection_rate = self.TP/self.tot_images
        else:
            self.detection_rate = None
    
    
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

    
    def pose_estimation(self, frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
        # Convert image to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Define ArUco dictionary and detector parameters
        cv2.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        parameters = cv2.aruco.DetectorParameters()
        
        # Define ArUco board
        aruco_board = cv2.aruco.Board(pos_board, cv2.aruco_dict, id_board)

        # Perform marker detection
        start_time = time.perf_counter()
        # Perform marker detection
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(
                gray, 
                cv2.aruco_dict, 
                parameters=parameters
        )

        end_time = time.perf_counter()

        # Store detection time statistics
        self.detection_time.append(end_time - start_time)
        self.avg_detection_time = sum(self.detection_time) / len(self.detection_time)

        # Initialize a pose estimate array
        pose_blueye_camera_wrt_tag_est = np.zeros(4)

        # Check that we actually are detecting any tags
        if len(corners) > 0:
            
            self.tot_detections += len(ids)

            # Defines initial guesses
            rvec_initial_guess = np.zeros((3, 1), dtype=float)
            tvec_initial_guess = np.zeros((3, 1), dtype=float)
            
            # Estimate pose of the ArUco board
            _, rvec_board, tvec_board = cv2.aruco.estimatePoseBoard(
                corners, ids, aruco_board, matrix_coefficients, distortion_coefficients,
                rvec_initial_guess, tvec_initial_guess
            )
            

            # Obtain the rotation matrix tag->camera
            R_camera_tag = np.matrix(cv2.Rodrigues(rvec_board)[0])
            R_tag_camera = R_camera_tag.T  # Transpose to get tag->camera rotation
            # Compute translation
            
            R_tag_camera = R_tag_camera
            t_tag_camera = (-R_tag_camera @ np.array(tvec_board).reshape(-1,1)).flatten()


            # Get the attitude of camera frame wrt flipped tag frame(or vice verca?????), in terms of euler 321 (Needs to be flipped first)    
            _, pitch_camera_board, _ = self.rotationMatrixToEulerAngles(self.rot_matrix_180_x*R_tag_camera)

            # Adjust yaw relative to world frame
            yaw_blueye_wrt_tag = -pitch_camera_board
    
            pose_blueye_camera_wrt_tag_est = np.array([t_tag_camera[0,0], t_tag_camera[0,1], t_tag_camera[0,2], yaw_blueye_wrt_tag], dtype=float)

            # ------------------------------------------------------------------------------------------
            #                Visualize the detected tags and their coordinate frame
            # ------------------------------------------------------------------------------------------
            
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec_board, tvec_board, length=0.25)

            # Adding Id-label
            frame = self.aruco_display(corners, ids, rejected_img_points, frame, rvec_board, tvec_board)

            # ------------------------------------------------------------------------------------------
            #               Publish the estimated pose of the camera wrt the tag frame
            # ------------------------------------------------------------------------------------------
            if not np.all(pose_blueye_camera_wrt_tag_est == 0):
                # Publish estimated pose as a Pose message
                pose_estimated_board_msg = Pose()
                pose_estimated_board_msg.position.x    = float(pose_blueye_camera_wrt_tag_est[0])
                pose_estimated_board_msg.position.y    = float(pose_blueye_camera_wrt_tag_est[1])
                pose_estimated_board_msg.position.z    = float(pose_blueye_camera_wrt_tag_est[2])
                pose_estimated_board_msg.orientation.z = float(pose_blueye_camera_wrt_tag_est[3])  # Yaw angle

                self.pose_estimated_board_pub_.publish(pose_estimated_board_msg)

                # Publish estimated pose with covariance for robot localization
                pose_stamped = PoseWithCovarianceStamped()
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.header.frame_id = "tag"

                pose_stamped.pose.pose.position.x = float(pose_blueye_camera_wrt_tag_est[2])
                pose_stamped.pose.pose.position.y = float(pose_blueye_camera_wrt_tag_est[0])
                pose_stamped.pose.pose.position.z = float(pose_blueye_camera_wrt_tag_est[1])

                # Convert yaw to quaternion
                quat = pyquaternion.Quaternion(axis=[0, 0, 1], radians=(pose_blueye_camera_wrt_tag_est[3] - np.pi)).normalised
                pose_stamped.pose.pose.orientation.x = quat.x
                pose_stamped.pose.pose.orientation.y = quat.y
                pose_stamped.pose.pose.orientation.z = quat.z
                pose_stamped.pose.pose.orientation.w = quat.w

                # Add covariance matrix
                std_dev_pos = 0.0001  # Standard deviation for position (meters)
                std_dev_yaw = 0.001225  # Standard deviation for yaw (radians)
                pose_stamped.pose.covariance = [
                    std_dev_pos**2, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, std_dev_pos**2, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, std_dev_pos**2, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, std_dev_yaw**2
                ]

                self.pose_estimated_board_stamped_pub_.publish(pose_stamped)
                
        docking_msg = Bool()
        if (corners is not None) and ids is not None and len(ids) >= 3:
            docking_msg.data = True
        else:
            docking_msg.data = False
            
        self.docking_pub.publish(docking_msg)

        return frame
    
def main():
    rclpy.init()
    pose_estimation_aruco_node = PoseEstimationAruco()
    rclpy.spin(pose_estimation_aruco_node)
    pose_estimation_aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
