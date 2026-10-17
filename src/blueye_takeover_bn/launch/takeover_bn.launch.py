from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("update_rate_hz", default_value="2.0"),
            DeclareLaunchArgument("default_mission_phase", default_value="Undocking"),
            DeclareLaunchArgument("use_default_mission_phase", default_value="true"),
            DeclareLaunchArgument("takeover_threshold", default_value="0.65"),
            Node(
                package="blueye_takeover_bn",
                executable="takeover_bn",
                name="takeover_bayesian_network",
                output="screen",
                parameters=[
                    {
                        "update_rate_hz": LaunchConfiguration("update_rate_hz"),
                        "default_mission_phase": LaunchConfiguration("default_mission_phase"),
                        "use_default_mission_phase": LaunchConfiguration("use_default_mission_phase"),
                        "takeover_threshold": LaunchConfiguration("takeover_threshold"),
                    }
                ],
            ),
            Node(
                package="blueye_takeover_bn",
                executable="camera_rate_monitor",
                name="camera_rate_monitor",
                output="screen",
                parameters=[{
                    "camera_topic": "/blueye/cam/image_color",
                    "camera_quality_topic": "/blueye/camera_quality",
                    "publish_rate_hz": 1.0,
                    "excellent_threshold_hz": 8.0,
                    "good_threshold_hz": 5.0,
                    "poor_threshold_hz": 1.0,
                }],
            ),
            Node(
                package="blueye_takeover_bn",
                executable="human_model",
                name="human_model",
                output="screen",
                parameters=[{
                    "publish_rate_hz":    2.0,
                    "mission_duration_s": 450.0,
                    "fatigue_start":      0.65,
                    "fatigue_end":        0.20,
                    "stress_start":       0.70,
                    "stress_end":         0.25,
                    "noise_std":          0.02,
                }],
            ),
            Node(
                package="gz_battery",
                executable="gz_battery_node",
                name="gz_battery",
                output="screen",
                parameters=[{
                    "battery_topic": "/blueye/battery",
                    "thrusters_topic": "/blueye/thrusters",
                }],
            ),
            Node(
                package="blueye_takeover_bn",
                executable="sim_current",
                name="sim_current",
                output="screen",
                parameters=[{
                    "enabled": False,
                    "enable_after_s": 0.0,
                }],
            ),
            Node(
                package="blueye_takeover_bn",
                executable="usbl_strength_monitor",
                name="usbl_strength_monitor",
                output="screen",
            ),
            Node(
                package="blueye_takeover_bn",
                executable="speed_monitor",
                name="speed_monitor",
                output="screen",
            ),
        ]
    )
