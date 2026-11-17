import socket
import json
import math
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from marine_acoustic_msgs.msg import Dvl
from scipy.spatial.transform import Rotation as R

class WaterlinkedPublisher(Node):
    
    def __init__(self):
        super().__init__("waterlinked_publisher")
        
        # Publishers
        self.publisher_velocity = self.create_publisher(Dvl, "/blueye/sensor/dvl", 1)
        self.publisher_position = self.create_publisher(Odometry, "/blueye/sensor/dvl/position", 1)
        
        # Internal State
        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.prev_vz = 0.0
        
        # Socket Configuration
        self.server_address = ("192.168.1.99", 16171)
        self.running = True
        
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect(self.server_address)
            # Use makefile to treat the TCP stream as a line-by-line text file
            self.socket_file = self.socket.makefile('r', encoding='utf-8')
            self.get_logger().info(f"Connected to DVL at {self.server_address}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to DVL: {e}")
            raise e

        # Threading: Decouple socket reading from ROS main loop
        self.thread = threading.Thread(target=self._socket_reader_thread, daemon=True)
        self.thread.start()

    def _socket_reader_thread(self):
        """ Background thread to process blocking TCP reads. """
        while self.running and rclpy.ok():
            try:
                # readline() is ideal for JSON-per-line protocols
                line = self.socket_file.readline()
                if not line:
                    self.get_logger().warn("DVL connection closed by server.")
                    break
                
                self.process_incoming_data(line)
                
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"Error in DVL thread: {e}")
                break

    def process_incoming_data(self, json_string):
        try:
            data = json.loads(json_string)
            # to debug and check what's available
            data_type = data.get("type")

            # Extract NTP-synced timestamp from sensor
            # from velocity data:
            t_usec = data.get("time_of_validity")
            # from position data:
            t_sec = data.get("ts")

            if data_type == "velocity" and t_usec:
                # Handle Velocity packets (Microseconds)
                msg_sec = int(t_usec // 1_000_000)
                msg_nanosec = int((t_usec % 1_000_000) * 1_000)
                stamp = Time(seconds=msg_sec, nanoseconds=msg_nanosec).to_msg()


                now = self.get_clock().now().to_msg()
                diff_nanos = now.nanosec - msg_nanosec
                diff_ms = diff_nanos / 1_000_000.0
                # self.get_logger().info(f"Latency: {diff_ms:.2f} ms")

            elif data_type == "position_local" and t_sec:
                # Handle Position packets (Seconds as float)
                msg_sec = int(t_sec)
                msg_nanosec = int((t_sec - msg_sec) * 1_000_000_000)
                stamp = Time(seconds=msg_sec, nanoseconds=msg_nanosec).to_msg()
            else:
                # Only warn if it's a message we actually care about
                stamp = self.get_clock().now().to_msg()
                if data_type in ["velocity", "position_local"]:
                    self.get_logger().warn(f"No timestamp in {data_type}. Using ROS time.", throttle_duration_sec=10.0)

            if data_type == "velocity":
                self._handle_velocity(data, stamp)
            elif data_type == "position_local":
                self._handle_position(data, stamp)

        except json.JSONDecodeError:
            # This handles cases where the buffer might have partial data
            pass

    def _handle_velocity(self, data, stamp):
        # check if velocity is valid, skip if not
        if not data.get("velocity_valid", False): return

        vx = data.get("vx", 0.0)  # Default to 0.0 if keys are missing to avoid None errors
        vy = data.get("vy", 0.0)
        vz = data.get("vz", 0.0)
        
        # Cache for Odometry message
        self.prev_vx, self.prev_vy, self.prev_vz = vx, vy, vz

        msg = Dvl()
        msg.header.stamp = stamp
        msg.header.frame_id = "waterlinked_dvl"
        msg.velocity_mode = 1 # Dvl.DVL_MODE_BOTTOM
        msg.dvl_type = 0 #0 Dvl.DVL_TYPE_PISTON
        
        msg.velocity.x = vx
        msg.velocity.y = vy
        msg.velocity.z = vz
        
        # Covariance processing
        cov = data.get("covariance", [])
        msg.velocity_covar = [float(item) for sublist in cov for item in sublist]
        
        msg.altitude = data.get("altitude", -1.0)
        msg.course_gnd = math.atan2(vy, vx)
        msg.speed_gnd = math.sqrt(vx**2 + vy**2)
        
        # transducers = data.get("transducers", [])
        # msg.num_good_beams = sum(1 for t in transducers if t.get("beam_valid"))
        # msg.range = [t.get("distance", 0.0) for t in transducers]
        # msg.beam_velocity = [t.get("velocity", 0.0) for t in transducers]
        msg.sound_speed = 1480.0 # this is the value in the dvl
        beam_1, beam_2, beam_3, beam_4 = data.get("transducers")
        msg.num_good_beams = sum([1 for beam in [beam_1, beam_2, beam_3, beam_4] if beam.get("beam_valid")])
                    # To-Do Beam Spesific Fields
        msg.beam_ranges_valid = all([beam.get("beam_valid") for beam in [beam_1, beam_2, beam_3, beam_4]])
        msg.beam_velocities_valid = all([beam.get("beam_valid") for beam in [beam_1, beam_2, beam_3, beam_4]])

        # Convert to floats and ensure list is length 4
        msg.range = [float(x) for x in [beam_1.get("distance", 0.0), 
                                        beam_2.get("distance", 0.0), 
                                        beam_3.get("distance", 0.0), 
                                        beam_4.get("distance", 0.0)]]

        msg.beam_velocity = [float(x) for x in [beam_1.get("velocity", 0.0), 
                                                beam_2.get("velocity", 0.0), 
                                                beam_3.get("velocity", 0.0), 
                                                beam_4.get("velocity", 0.0)]]


        self.publisher_velocity.publish(msg)

    def _handle_position(self, data, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "waterlinked_odom_frame"
        msg.child_frame_id = "base_link"

        # Position
        msg.pose.pose.position.x = data.get("x", 0.0)
        msg.pose.pose.position.y = data.get("y", 0.0)
        msg.pose.pose.position.z = data.get("z", 0.0)

        # Orientation (Scipy handles deg-to-rad conversion if needed)
        # Note: Scipy from_euler expects degrees by default. 
        # If DVL sends radians, set degrees=False.
        r = R.from_euler('xyz', [data.get("roll", 0), data.get("pitch", 0), data.get("yaw", 0)], degrees=True)
        quat = r.as_quat() # [x, y, z, w]

        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        # Twist (Velocities)
        msg.twist.twist.linear.x = self.prev_vx
        msg.twist.twist.linear.y = self.prev_vy
        msg.twist.twist.linear.z = self.prev_vz

        self.publisher_position.publish(msg)

    def cleanup(self):
        self.running = False
        self.get_logger().info("Closing DVL connection...")
        try:
            self.socket_file.close()
            self.socket.close()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = WaterlinkedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
