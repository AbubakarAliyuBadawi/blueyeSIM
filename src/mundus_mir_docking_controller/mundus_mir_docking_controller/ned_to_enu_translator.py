import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R

class NedToEnuTranslator(Node):
    def __init__(self):
        super().__init__('ned_to_enu_translator')

        self.sub = self.create_subscription(
            Odometry,
            '/blueye/navigation/current_state',
            self.cb,
            10
        )

        self.pub = self.create_publisher(
            Odometry,
            '/blueye/navigation/current_state_enu',
            10
        )

    def cb(self, msg):
        # NED input
        x_n = msg.pose.pose.position.x
        y_e = msg.pose.pose.position.y
        z_d = msg.pose.pose.position.z

        # NED -> ENU mapping
        x_enu = y_e          # east
        y_enu = x_n          # north
        z_enu = -z_d         # up

        # Extract quaternion in NED
        q_ned = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ])

        # Convert quaternion NED -> ENU (swap axes + flip down)
        R_ned = R.from_quat(q_ned)
        R_enu = R.from_matrix(np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1],
        ]) @ R_ned.as_matrix() @ np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1],
        ]).T)

        q_enu = R_enu.as_quat()

        # Build new Odometry message
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = "tag"     # Keep tag frame
        out.child_frame_id = "base_link"

        out.pose.pose.position.x = x_enu
        out.pose.pose.position.y = y_enu
        out.pose.pose.position.z = z_enu

        out.pose.pose.orientation.x = q_enu[0]
        out.pose.pose.orientation.y = q_enu[1]
        out.pose.pose.orientation.z = q_enu[2]
        out.pose.pose.orientation.w = q_enu[3]

        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = NedToEnuTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()