import socket
import struct
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from geometry_msgs.msg import PoseStamped
import numpy as np
import time
import math
from pyproj import Geod
from std_msgs.msg import String
import sys



class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        #self.publisher_ = self.create_publisher(Odometry, '/blueye/evologics/usbl', 10)
        self.publisher_ = self.create_publisher(PoseStamped, '/blueye/evologics/usbl', 10)
        self.raw_data_publisher_ = self.create_publisher(String, '/blueye/evologics/raw_data', 10)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = ('192.168.1.189', 9200)
        #self.socket.connect(self.server_address)
        try:
            self.socket.connect(self.server_address)
        except OSError as e:
            self.get_logger().warn(
                f"Could not connect to Evologics modem at "
                f"{self.server_address}: {e}. "
                "USBL disabled."
            )
            self.destroy_node()
            return
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_target_ip = '192.168.1.101'  # your machine IP
        self.udp_target_port = 10110


    def timer_callback(self):
        data = self.socket.recv(1024)
        #print(data)
        #b'+++AT:45:RECVIM,6,2,3,ack,221305,-49,189,0.0026,E\x1d\x00\xf4\xff%\r\n+++AT:45:RECVIM,6,2,3,ack,221305,-50,172,0.0228,E\x1d\x00\xf3\xff%\r\n+++AT:45:RECVIM,6,2,3,ack,221305,-50,192,0.0073,E\x1d\x00\xf4\xff%\r\n+++AT:46:RECVIM,6,2,3,ack,221305,-49,211,-0.0095,E\x1d\x00\xf4\xff%\r\n+++AT:45:RECVIM,6,2,3,ack,221306,-50,166,0.0208,E\x1d\x00\xf4\xff%\r\n+++AT:45:RECVIM,6,2,3,ack,221306,-50,163,0.0039,E\x1d\x00\xf4\xff%\r\n+++AT:46:RECVIM,6,2,3,ack,221305,-51,132,-0.0185,E\x1d\x00\xf4\xff$\r\n'
        if not data:
            return
        self.raw_data_publisher_.publish(String(data=data.decode(errors='ignore')))
        hex_byte_start = data.find(b'E')
        hex_byte_start_real = data[hex_byte_start+1:].find(b'E') + hex_byte_start + 2
        if hex_byte_start_real != -1 and len(data) > hex_byte_start_real + 5:
            x, y, heading = struct.unpack('<hhB', data[hex_byte_start_real:hex_byte_start_real+5])
            heading = heading*2
            x_ned, y_ned, heading = self.publish_odometry(np.float64(x), np.float64(y), np.float64(heading))
            
            self.publish_to_UDP(np.float64(x_ned), np.float64(y_ned), np.float64(heading))
            

    def publish_to_UDP(self, x_ned, y_ned, heading):

        # Reference position (USBL)
        docking_station_lat = 63.4406991
        docking_station_lon = 10.348936
            
        """
        # 1. GEOD conversion Calculate displacement in GPS
        # Load world model
        geod = Geod(ellps="WGS84")
        r = math.sqrt((x_ned*x_ned) + ((y_ned)*(y_ned)))
        azimuth = (math.atan(y_ned/x_ned))*360/2*math.pi
        new_lon, new_lat, empty = geod.fwd(lons=docking_station_lon, lats=docking_station_lat, az=azimuth, dist=r)
        """
        # 2. Easy conversion
        delta_lat = y_ned / 111320.0
        delta_lon = x_ned / (111320.0 * np.cos(np.radians(docking_station_lat)))
        new_lat = docking_station_lat + delta_lat
        new_lon = docking_station_lon + delta_lon
        
        
        # Convert decimal degrees to NMEA format (ddmm.mmmmm)
        def decimal_to_nmea(degree, is_lat):
            d = int(abs(degree))
            m = (abs(degree) - d) * 60
            if is_lat:
                return f"{d:02d}{m:07.4f}"
            else:
                return f"{d:03d}{m:07.4f}"

        lat_nmea = decimal_to_nmea(new_lat, is_lat=True)
        lon_nmea = decimal_to_nmea(new_lon, is_lat=False)
        lat_dir = 'N' if new_lat >= 0 else 'S'
        lon_dir = 'E' if new_lon >= 0 else 'W'

        # Current time in hhmmss format
        utc_now = time.gmtime()
        time_str = f"{utc_now.tm_hour:02d}{utc_now.tm_min:02d}{utc_now.tm_sec:02d}"

        # Construct GPGLL sentence (without checksum)
        nmea_body = f"GPGLL,{lat_nmea},{lat_dir},{lon_nmea},{lon_dir},{time_str},A"
        nmea_sentence = f"${nmea_body}"

        # Calculate checksum
        checksum = 0
        for c in nmea_body:
            checksum ^= ord(c)
        nmea_sentence += f"*{checksum:02X}\r\n"

        # Send over UDP
        self.udp_socket.sendto(nmea_sentence.encode('ascii'), (self.udp_target_ip, self.udp_target_port))
        #self.get_logger().info(f"Actually sent to {self.udp_target_ip}:{self.udp_target_port}")


        # 9. (Optional) log it
        self.get_logger().info(f"Sent NMEA sentence: {nmea_sentence.strip()}")



    def publish_odometry(self, x, y, heading):
        msg = PoseStamped()
        #msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        
        # Because values are in dm
        x = x/10.0
        y = y/10.0
        # Set the position
        msg.pose.position = Point(x=x, y=y, z=0.0)
        
        # Set the orientation (assuming heading is in degrees)
        quaternion = self.euler_to_quaternion(0, 0, heading*2.0)
        msg.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
        
        self.publisher_.publish(msg)
        #print(msg)
        #self.get_logger().info(f'Published odometry: x={x}, y={y}, heading={heading}')
        return x, y, heading

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to quaternion
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.socket.close()
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
