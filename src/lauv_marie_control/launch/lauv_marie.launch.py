"""
LAUV Marie in Stonefish with its autopilot.

  ros2 launch lauv_marie_control lauv_marie.launch.py                      # with the 3D window
  ros2 launch lauv_marie_control lauv_marie.launch.py rendering:=false     # headless (no sonar or camera)

Then, in a second terminal, the keyboard control:
  ros2 run lauv_marie_control teleop_keyboard

The start position is in stonefish_sim/config/lauv_marie_config.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulation = os.path.join(get_package_share_directory("stonefish_sim"), "launch", "simulation.launch.py")
    rendering = LaunchConfiguration("rendering")
    return LaunchDescription(
        [
            DeclareLaunchArgument("rendering", default_value="true", description="3D window (true/false)"),
            DeclareLaunchArgument("rendering_quality", default_value="med", description="high/med/low"),
            DeclareLaunchArgument("initial_rpm", default_value="0.0",
                                  description="propeller speed the autopilot starts with"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation),
                launch_arguments={"scenario": "lauv_marie", "rendering": "true",
                                  "rendering_quality": LaunchConfiguration("rendering_quality")}.items(),
                condition=IfCondition(rendering),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation),
                launch_arguments={"scenario": "lauv_marie_no_gpu", "rendering": "false"}.items(),
                condition=UnlessCondition(rendering),
            ),
            Node(
                package="lauv_marie_control",
                executable="autopilot",
                name="lauv_marie_autopilot",
                output="screen",
                parameters=[{"initial_rpm": LaunchConfiguration("initial_rpm")}],
            ),
        ]
    )
