from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '3.14159',
                '--frame-id', 'base_link',
                '--child-frame-id', 'sonar_link'
            ]
        ),
        Node(
            package='sonar_3d',
            executable='sonar_3d_point_cloud',
        ),
    ])