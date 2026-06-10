from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drone_ip_arg = DeclareLaunchArgument(
        "drone_ip",
        default_value="192.168.1.101",
        description="IP address of the Blueye drone",
    )
    rig_camera_url_arg = DeclareLaunchArgument(
        "rig_camera_url",
        default_value="rtsp://192.168.1.10:554/stream1",
        description="RTSP URL of the rig/docking camera",
    )
    sonar_url_arg = DeclareLaunchArgument(
        "sonar_url",
        default_value="http://192.168.1.96",
        description="URL of the WaterLinked sonar web interface",
    )
    usbl_url_arg = DeclareLaunchArgument(
        "usbl_url",
        default_value="http://localhost:10000",
        description="URL of the Sinaps USBL web interface",
    )
    enable_sonar_3d_arg = DeclareLaunchArgument(
        "enable_sonar_3d",
        default_value="true",
        description="Show the Sonar 3D tab (set false when sonar not connected)",
    )
    enable_usbl_arg = DeclareLaunchArgument(
        "enable_usbl",
        default_value="true",
        description="Show the USBL tab (set false when USBL not connected)",
    )

    hud_node = Node(
        package="blueye_visualization_real",
        executable="blueye_real_hud",
        name="blueye_real_operator_hud",
        output="screen",
        parameters=[{
            "drone_ip": LaunchConfiguration("drone_ip"),
            "rig_camera_url": LaunchConfiguration("rig_camera_url"),
            "sonar_url": LaunchConfiguration("sonar_url"),
            "usbl_url": LaunchConfiguration("usbl_url"),
            "enable_sonar_3d": LaunchConfiguration("enable_sonar_3d"),
            "enable_usbl": LaunchConfiguration("enable_usbl"),
        }],
    )

    return LaunchDescription([
        drone_ip_arg, rig_camera_url_arg, sonar_url_arg,
        usbl_url_arg, enable_sonar_3d_arg, enable_usbl_arg,
        hud_node,
    ])
