from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("update_rate_hz", default_value="2.0"),
            DeclareLaunchArgument("takeover_threshold", default_value="0.65"),
            DeclareLaunchArgument("default_mission_phase", default_value="Transit"),
            Node(
                package="blueye_takeover_bn_real",
                executable="takeover_bn_real",
                name="takeover_bayesian_network_real",
                output="screen",
                parameters=[
                    {
                        "update_rate_hz": LaunchConfiguration("update_rate_hz"),
                        "takeover_threshold": LaunchConfiguration("takeover_threshold"),
                        "default_mission_phase": LaunchConfiguration("default_mission_phase"),
                    }
                ],
            ),
        ]
    )
