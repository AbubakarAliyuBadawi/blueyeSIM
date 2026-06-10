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
            DeclareLaunchArgument("drone_ip", default_value="192.168.1.101"),
            Node(
                package="blueye_takeover_bn_real",
                executable="takeover_bn_real",
                name="takeover_bayesian_network_real",
                output="screen",
                parameters=[
                    {
                        "update_rate_hz": LaunchConfiguration("update_rate_hz"),
                        "takeover_threshold": LaunchConfiguration("takeover_threshold"),
                        "default_mission_phase": LaunchConfiguration(
                            "default_mission_phase"
                        ),
                    }
                ],
            ),
            Node(
                package="blueye_takeover_bn_real",
                executable="usbl_strength_monitor",
                name="usbl_strength_monitor",
                output="screen",
            ),
            Node(
                package="blueye_takeover_bn_real",
                executable="camera_quality_monitor",
                name="camera_quality_monitor",
                output="screen",
                parameters=[
                    {
                        "rtsp_url": ["rtsp://", LaunchConfiguration("drone_ip"), ":8554/test"],
                    }
                ],
            ),
            Node(
                package="blueye_takeover_bn_real",
                executable="aruco_visibility_monitor",
                name="aruco_visibility_monitor",
                output="screen",
            ),
            Node(
                package="blueye_takeover_bn_real",
                executable="inspection_data_quality_monitor",
                name="inspection_data_quality_monitor",
                output="screen",
            ),
            Node(
                package="blueye_takeover_bn_real",
                executable="real_current_publisher",
                name="real_current_publisher",
                output="screen",
            ),
            Node(
                package="blueye_takeover_bn_real",
                executable="real_mission_state_monitors",
                name="real_mission_state_monitors",
                output="screen",
            ),
            Node(
                package="blueye_takeover_bn_real",
                executable="sonar_range_monitor",
                name="sonar_range_monitor",
                output="screen",
            ),
        ]
    )
