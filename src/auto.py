#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class ModePublisher(Node):
   def __init__(self):
       super().__init__('mode_publisher')
       self.mode_pub = self.create_publisher(String, 'softwareOperationMode', 10)
       self.kill_pub = self.create_publisher(Bool, 'softwareKillSwitch', 10)
       
       # Disable killswitch
       self.kill_pub.publish(Bool(data=False))
       # Set autonomous mode
       self.mode_pub.publish(String(data='autonomous mode'))

def main():
   rclpy.init()
   publisher = ModePublisher()
   rclpy.spin(publisher)
   rclpy.shutdown()

if __name__ == '__main__':
   main()