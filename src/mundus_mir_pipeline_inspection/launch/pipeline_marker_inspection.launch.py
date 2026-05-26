from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "inspection_z_offset",
                default_value="-0.6",
                description="Offset added to each marker z to place the ROV above the pipeline.",
            ),
            DeclareLaunchArgument(
                "desired_velocity",
                default_value="0.25",
                description="Desired surge velocity used for each inspection waypoint.",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="true",
                description="Immediately send waypoints and start waypoint following.",
            ),
            Node(
                package="mundus_mir_pipeline_inspection",
                executable="pipeline_marker_inspector",
                name="pipeline_marker_inspector",
                output="screen",
                parameters=[
                    {
                        "inspection_z_offset": LaunchConfiguration("inspection_z_offset"),
                        "desired_velocity": LaunchConfiguration("desired_velocity"),
                        "auto_start": LaunchConfiguration("auto_start"),
                    }
                ],
            ),
        ]
    )
