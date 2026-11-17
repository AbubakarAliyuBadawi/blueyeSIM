import serial
import struct
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# --- Configuration ---
SERIAL_PORT = '/dev/ttySentiboard02'
BAUD_RATE = 921600

DEG_TO_RAD = math.pi / 180.0
G_TO_MS2 = 9.80665

SAMPLE_RATE_HZ = 500

# ARW: 0.15 deg/sqrt(hr) at 500 Hz -> (rad/s)^2
GYRO_COV = [
    9.5e-7, 0.0, 0.0,
    0.0, 9.5e-7, 0.0,
    0.0, 0.0, 9.5e-7
]

# VRW: 0.07 m/s/sqrt(hr) at 500 Hz -> (m/s^2)^2
ACCEL_COV = [
    6.8e-4, 0.0, 0.0,
    0.0, 6.8e-4, 0.0,
    0.0, 0.0, 6.8e-4
]

def decode_24bit(b1, b2, b3):
    """Converts 3 raw bytes into a signed 24-bit integer"""
    val = (b1 << 16) | (b2 << 8) | b3
    if val >= 0x800000:
        val -= 0x1000000
    return val


class Stim300Publisher(Node):
    def __init__(self):
        super().__init__('stim300_imu')
        self.pub = self.create_publisher(Imu, 'imu_stim/data', 10)
        self.get_logger().info(f'Connecting to {SERIAL_PORT}...')
        self.com = serial.Serial(SERIAL_PORT, BAUD_RATE)
        self.data_buffer = b''
        self.get_logger().info('Decoding IMU data and publishing to /imu_stim/data')

    def spin_serial(self):
        while rclpy.ok():
            self.data_buffer += self.com.read(self.com.in_waiting or 1)

            while len(self.data_buffer) >= 10:
                if self.data_buffer[:2] != b'^B':
                    self.data_buffer = self.data_buffer[1:]
                    continue

                data_len = struct.unpack('H', self.data_buffer[2:4])[0]
                total_len = 8 + data_len
                padding_len = (4 - ((total_len + 2) % 4)) % 4
                final_length = total_len + 2 + padding_len

                if len(self.data_buffer) < final_length:
                    break

                payload = self.data_buffer[8:total_len]

                if len(payload) == 50:
                    stim_data = payload[12:]
                    self._publish(stim_data)

                self.data_buffer = self.data_buffer[final_length:]

    def _publish(self, stim_data):
        if stim_data[0] != 0x93:
            return

        # Raw counts / 2^21 = Degrees. Multiply by 500Hz = deg/s. Multiply by pi/180 = rad/s.
        gx = decode_24bit(stim_data[1], stim_data[2], stim_data[3]) / (1 << 21) * SAMPLE_RATE_HZ * DEG_TO_RAD
        gy = decode_24bit(stim_data[4], stim_data[5], stim_data[6]) / (1 << 21) * SAMPLE_RATE_HZ * DEG_TO_RAD
        gz = decode_24bit(stim_data[7], stim_data[8], stim_data[9]) / (1 << 21) * SAMPLE_RATE_HZ * DEG_TO_RAD

        # Raw counts / 2^22 = Velocity change (m/s) per sample. Multiply by 500Hz = Acceleration (m/s^2).
        ax = decode_24bit(stim_data[11], stim_data[12], stim_data[13]) / (1 << 22) * SAMPLE_RATE_HZ
        ay = decode_24bit(stim_data[14], stim_data[15], stim_data[16]) / (1 << 22) * SAMPLE_RATE_HZ
        az = decode_24bit(stim_data[17], stim_data[18], stim_data[19]) / (1 << 22) * SAMPLE_RATE_HZ

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.angular_velocity_covariance = GYRO_COV
        msg.linear_acceleration_covariance = ACCEL_COV
        msg.orientation_covariance[0] = -1.0  # orientation unknown

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Stim300Publisher()
    try:
        node.spin_serial()
    except KeyboardInterrupt:
        pass
    finally:
        node.com.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()