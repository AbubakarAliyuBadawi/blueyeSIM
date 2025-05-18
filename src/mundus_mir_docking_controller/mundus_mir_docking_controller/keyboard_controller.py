import rclpy
from rclpy.node import Node
from pynput import keyboard
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped


class KeyboardNode(Node):
    def __init__(self):
        super().__init__("keyboard_node")

        # Publisher for velocity commands (matches velocity_controller input)
        self.odom_publisher_ = self.create_publisher(TwistStamped, "/blueye/ref_vel", 10)

        # Start keyboard listener
        self.keyboard = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.keyboard.start()

        # Movement variables
        self.surge = 0.0   # x-direction (forward/backward)
        self.sway = 0.0    # y-direction (sideways)
        self.heave = 0.0   # z-direction (up/down)
        self.yaw = 0.0     # rotation around z-axis

        # Timer to publish messages (every 100ms)
        self.timer = self.create_timer(0.1, self.publish_command)

    def on_press(self, key):
        """ Callback when a key is pressed """
        button = key.char if hasattr(key, 'char') else key.name

        if button in ["up", "down"]:
            self.surge = 1.0 if button == "up" else -1.0  # Adjusted for smooth motion
        elif button in ["left", "right"]:
            self.sway = -1.0 if button == "left" else 1.0
        elif button in ["w", "s"]:
            self.heave = -1.0 if button == "w" else 1.0
        elif button in ["a", "d"]:
            self.yaw = 1.0 if button == "a" else -1.0  # Rotational speed
    
    def publish_command(self):
        twist_msg = TwistStamped()
        twist_msg.header = Header()
        twist_msg.header.frame_id = "body_frame"
        twist_msg.header.stamp = self.get_clock().now().to_msg()

        twist_msg.twist.linear.x = self.surge
        twist_msg.twist.linear.y = self.sway
        twist_msg.twist.linear.z = self.heave
        twist_msg.twist.angular.z = self.yaw  

        self.odom_publisher_.publish(twist_msg)


    def on_release(self, key):
        """ Callback when a key is released """
        button = key.char if hasattr(key, 'char') else key.name

        # Reset movement to zero on key release
        if button in ["up", "down"]:
            self.surge = 0.0
        elif button in ["left", "right"]:
            self.sway = 0.0
        elif button in ["w", "s"]:
            self.heave = 0.0
        elif button in ["a", "d"]:
            self.yaw = 0.0

def main(args=None):
    rclpy.init(args=args)
    keyboard_node = KeyboardNode()
    rclpy.spin(keyboard_node)
    keyboard_node.keyboard.stop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()