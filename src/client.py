#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from vortex_msgs.action import NavigateWaypoints
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
import time

class GuidanceActionClient(Node):
    def __init__(self):
        super().__init__('guidance_test_client')

        # Create action client
        self.action_client = ActionClient(
            self,
            NavigateWaypoints,
            'navigate_waypoints'
        )

        self.get_logger().info('Guidance Test Client initialized. Waiting for action server...')

    def send_waypoints(self):
        # Wait for action server
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for action server...')

        # Create goal message
        goal_msg = NavigateWaypoints.Goal()
            
        # Create waypoints
        waypoints = [
            (5.0, 3.0, 5.0),   # x, y, z coordinates
            (7.0, 5.0, 7.0),
            (4.0, 3.0, 1.0),
            (4.0, 7.0, 7.0)
        ]

        # Convert waypoints to PoseStamped messages
        for x, y, z in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            # Orientation - using default quaternion (no rotation)
            pose.pose.orientation.w = 1.0
            goal_msg.waypoints.append(pose)

        self.get_logger().info('Sending goal with {} waypoints...'.format(len(waypoints)))

        # Send goal
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal completed with success: {}'.format(result.success))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Current waypoint index: {}'.format(feedback.current_waypoint_index))

def main(args=None):
    rclpy.init(args=args)
    action_client = GuidanceActionClient()
    
    # Send waypoints
    action_client.send_waypoints()
    
    # Spin until complete
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()