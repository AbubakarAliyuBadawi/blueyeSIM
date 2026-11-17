import socket
import json
import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from marine_acoustic_msgs.msg import Dvl

class WaterlinkedPublisher(Node):
    
    def __init__(self):
        super().__init__("waterlinked_publisher")
        self.publisher_velocity = self.create_publisher(Dvl, "/blueye/sensor/dvl", 1)
        self.publisher_position = self.create_publisher(Odometry, "/blueye/sensor/dvl/position", 1)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = ("192.168.1.99", 16171)
        self.socket.connect(self.server_address)
        self.timer = self.create_timer(0.05, self.dvl_callback)
        self.prev_vx = 0
        self.prev_vy = 0
        self.prev_vz = 0



    def quaternion_from_euler(self, roll, pitch, yaw):
       
            # Euler to Quaternion
            qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            
            return [qx, qy, qz, qw]

    def dvl_callback(self):
        # It seems like the entire message is not received at once, try increasing the buffer tomorrow and see if it works better.
        data = self.socket.recv(2048)

        if data:
            json_message = data.decode('utf-8')

            # TMP solution since I didnt capture a full message...
            #json_message_full = json_message.split("\n")[0] + ":true}"
            try:
                data = json.loads(json_message)
                data_type = data.get("type")

                # If Velocity message was received, tmp not checking type..
                if data_type == "velocity":
                    vx = data.get("vx")
                    vy = data.get("vy")
                    vz = data.get("vz")
                    covariance = data.get("covariance")
                    flattend_covariance = [item for sublist in covariance for item in sublist]
                    altitude = data.get("altitude")
                    beam_1, beam_2, beam_3, beam_4 = data.get("transducers")

                    # Fill DVL message
                    msg = Dvl()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "waterlinked_dvl"
                    msg.velocity_mode = 1
                    msg.dvl_type = 0
                    msg.velocity.x = vx
                    msg.velocity.y = vy
                    msg.velocity.z = vz
                    self.prev_vx = vx
                    self.prev_vy = vy
                    self.prev_vz = vz
                    msg.velocity_covar = flattend_covariance
                    msg.altitude = altitude
                    msg.course_gnd = math.atan2(vy, vx)
                    msg.speed_gnd = math.sqrt(vx**2 + vy**2)
                    msg.num_good_beams = sum([1 for beam in [beam_1, beam_2, beam_3, beam_4] if beam.get("beam_valid")])
                    msg.sound_speed = 1500.0 # TMP do I get this from the TCP communication?

                    # To-Do Beam Spesific Fields
                    msg.beam_ranges_valid = all([beam.get("beam_valid") for beam in [beam_1, beam_2, beam_3, beam_4]])
                    msg.beam_velocities_valid = all([beam.get("beam_valid") for beam in [beam_1, beam_2, beam_3, beam_4]])
                    msg.range = [beam_1.get("distance"), beam_2.get("distance"), beam_3.get("distance"), beam_4.get("distance")]
                    msg.beam_velocity = [beam_1.get("velocity"), beam_2.get("velocity"), beam_3.get("velocity"), beam_4.get("velocity")]

                    # Publish DVL message
                    self.publisher_velocity.publish(msg)
                    

                # If position message was received 
                elif data_type == "position_local":
                    time = data.get("ts")
                    x = data.get("x")
                    y = data.get("y")
                    z = data.get("z")
                    std = data.get("std")
                    roll = data.get("roll")
                    pitch = data.get("pitch")
                    yaw = data.get("yaw")


                    # Fill Odometry message
                    msg = Odometry()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "waterlinked_odom_frame"
                    msg.pose.pose.position.x = x
                    msg.pose.pose.position.y = y
                    msg.pose.pose.position.z = z

                    # Fill velocities
                    msg.twist.twist.linear.x = self.prev_vx
                    msg.twist.twist.linear.y = self.prev_vy
                    msg.twist.twist.linear.z = self.prev_vz
                    msg.twist.twist.angular.z = 0.0

                    # Turn orientation to quaternion
                    quaternions = self.quaternion_from_euler(roll, pitch, yaw)
                    msg.pose.pose.orientation.x = quaternions[0]
                    msg.pose.pose.orientation.y = quaternions[1]
                    msg.pose.pose.orientation.z = quaternions[2]
                    msg.pose.pose.orientation.w = quaternions[3]

                    # Publish odometry message
                    self.publisher_position.publish(msg)
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    waterlinked_dvl = WaterlinkedPublisher()
    rclpy.spin(waterlinked_dvl)
    waterlinked_dvl.socket.close()
    waterlinked_dvl.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
