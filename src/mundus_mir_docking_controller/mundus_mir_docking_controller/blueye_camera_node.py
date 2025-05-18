#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

'''
This node does the following:

- Starts a videostream from the Blueye camera
- Displays the videostream
- Publish the images on the topic /blueye/image

'''


class BlueyeCameraNode(Node):

    def __init__(self):
        super().__init__("blueye_camera_node")
        self.bridge = CvBridge()
        self.camera_image_publisher = self.create_publisher(Image, "/blueye/image", 10)
        self.read_camera()

    def read_camera(self):
        
        # self.visual_odometry_publisher = self.create_publisher(Odometry, "/blueye/visual_odometry", 1)
        
        # self.pose_estimated_board_subscriber = self.create_subscription(Pose, "/blueye/pose_estimated_board", self.callback_pose_estimated_board, 10)
        
        
        #------------- Decide on method to use her. Also look at alex node for camera -----------------
        
        # ------- High Latency solution------------
        # rtsp_url = "rtsp://192.168.1.101:8554/test"
        # cap = cv2.VideoCapture(rtsp_url)
        
        # ------- Low Latency solution 1 ------------Brukt denne til nå
        gst_pipeline = 'rtspsrc latency=0 location=rtsp://192.168.1.101:8554/test ! decodebin ! videoconvert ! appsink'
        
        # ------- Low Latency solution 2 ------------
        # gst_pipeline = "rtspsrc latency=0 rtsp://192.168.1.101:8554/test"
        
        # ------- Low Latency solution 3 ------------
        # gst_pipeline = "rtspsrc location=rtsp://192.168.1.101:8554/test latency=0 ! " "rtph264depay ! ""avdec_h264 ! ""videoconvert ! ""appsink sync=false emit-signals=true drop=true max-buffers=1"
        
        # Create a VideoCapture object with the GStreamer pipeline
        cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        # Define the codec and create VideoWriter object to save the video
        # fourcc = cv2.VideoWriter_fourcc(*'I420')  # Codec definition for uncompressed AVI
        # out = cv2.VideoWriter('/home/aurlab/blueye-ros2-interface/videos/output.avi', fourcc, 20.0, (1920, 1080), True)
    
        if not cap.isOpened():
            self.get_logger().error("Unable to open camera stream.")
            return

        try:
            while rclpy.ok():
                #Capture frame by frame
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().error("Unable to fetch frame")
                    break
                
                # Write the frame into the file 'output.avi'
                # Uncomment the line below to save the video
                #out.write(frame)
                
                # View it via pose_est node instead!!!
                # Display the frame
                # cv2.imshow('Frame', frame)
                # if cv2.waitKey(1) == ord('q'):
                #     break

                small_frame = cv2.resize(frame, (640, 480))
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.camera_image_publisher.publish(ros_image)
            
        finally:
            cap.release()
            # out.release()
            cv2.destroyAllWindows()
    

def main(args=None):
    rclpy.init(args=args)
    blueye_camera_node = BlueyeCameraNode()
    rclpy.spin(blueye_camera_node)
    blueye_camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    