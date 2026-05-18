from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    hud = Node(
        package="blueye_visualization",
        executable="operator_hud",
        name="operator_hud",
        output="screen",
    )

    return LaunchDescription([hud])
