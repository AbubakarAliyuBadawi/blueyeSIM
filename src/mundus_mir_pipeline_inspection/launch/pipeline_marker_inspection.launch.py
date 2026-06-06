from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    waypoint_controller = Node(
        package="mundus_mir_waypoint_controller",
        executable="waypoint_controller",
        name="pipeline_waypoint_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_waypoint_controller")),
        parameters=[
            {
                "odometry_topic": LaunchConfiguration("odometry_topic"),
                "desired_vel_topic": LaunchConfiguration("desired_velocity_topic"),
                "heading_tolerance": LaunchConfiguration("heading_tolerance"),
                "circle_of_acceptance": LaunchConfiguration("circle_of_acceptance"),
                "max_heading_rate": LaunchConfiguration("max_heading_rate"),
                "max_heave_rate": LaunchConfiguration("max_heave_rate"),
                "max_station_keep_rate": LaunchConfiguration("max_station_keep_rate"),
            }
        ],
    )

    velocity_controller = Node(
        package="velocity_controller_4dof_ros2",
        executable="velocity_controller_4dof",
        name="pipeline_velocity_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_velocity_controller")),
        parameters=[
            {
                "topic_subscriber_odometry": LaunchConfiguration("odometry_topic"),
                "topic_subscriber_desired": LaunchConfiguration("desired_velocity_topic"),
                "topic_publisher": LaunchConfiguration("cmd_force_topic"),
                "controller_frequency_hz": 50,
                "rt_surge_kp": 10.0,
                "rt_surge_ki": 0.0,
                "rt_surge_kd": 0.0,
                "rt_surge_satUpper": 10.0,
                "rt_surge_satLower": -10.0,
                "rt_sway_kp": 10.0,
                "rt_sway_ki": 0.0,
                "rt_sway_kd": 0.0,
                "rt_sway_satUpper": 10.0,
                "rt_sway_satLower": -10.0,
                "rt_depth_kp": 10.0,
                "rt_depth_ki": 0.0,
                "rt_depth_kd": 4.0,
                "rt_depth_satUpper": 6.0,
                "rt_depth_satLower": -6.0,
                "rt_yaw_kp": 8.0,
                "rt_yaw_ki": 0.0,
                "rt_yaw_kd": 3.0,
                "rt_yaw_satUpper": 8.0,
                "rt_yaw_satLower": -8.0,
            }
        ],
    )

    thruster_allocator = Node(
        package="mundus_mir_vehicle_interfaces",
        executable="blueye_simulator_interface",
        name="pipeline_blueye_thruster_allocator",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_thruster_allocator")),
        parameters=[
            {
                "cmd_force_topic": LaunchConfiguration("cmd_force_topic"),
                "thrusters_topic": LaunchConfiguration("thrusters_topic"),
                "max_thruster_setpoint": LaunchConfiguration("max_thruster_setpoint"),
            }
        ],
    )

    marker_inspector = Node(
        package="mundus_mir_pipeline_inspection",
        executable="pipeline_marker_inspector",
        name="pipeline_marker_inspector",
        output="screen",
        parameters=[
            {
                "inspection_z_offset": LaunchConfiguration("inspection_z_offset"),
                "desired_velocity": LaunchConfiguration("desired_velocity"),
                "auto_start": LaunchConfiguration("auto_start"),
                "run_waypoint_controller_service": LaunchConfiguration("run_waypoint_controller_service"),
                "clear_waypoints_service": LaunchConfiguration("clear_waypoints_service"),
                "add_waypoint_service": LaunchConfiguration("add_waypoint_service"),
                "go_to_waypoints_service": LaunchConfiguration("go_to_waypoints_service"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_waypoint_controller",
                default_value="true",
                description="Start the waypoint controller node from this launch file.",
            ),
            DeclareLaunchArgument(
                "start_velocity_controller",
                default_value="true",
                description="Start the Blueye 4-DOF velocity controller.",
            ),
            DeclareLaunchArgument(
                "start_thruster_allocator",
                default_value="true",
                description="Start the Stonefish Blueye thruster allocator.",
            ),
            DeclareLaunchArgument(
                "mission_start_delay",
                default_value="2.0",
                description="Seconds to wait before sending inspection waypoints.",
            ),
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
            DeclareLaunchArgument(
                "odometry_topic",
                default_value="/blueye/odom",
                description="Odometry topic used by the waypoint and velocity controllers.",
            ),
            DeclareLaunchArgument(
                "desired_velocity_topic",
                default_value="/blueye/ref_vel",
                description="TwistStamped setpoint topic published by the waypoint controller.",
            ),
            DeclareLaunchArgument(
                "cmd_force_topic",
                default_value="/blueye/cmd_force",
                description="WrenchStamped control output from velocity controller.",
            ),
            DeclareLaunchArgument(
                "thrusters_topic",
                default_value="/blueye/thrusters",
                description="Stonefish thruster setpoint topic.",
            ),
            DeclareLaunchArgument(
                "max_thruster_setpoint",
                default_value="35.0",
                description="Maximum Stonefish thruster setpoint.",
            ),
            DeclareLaunchArgument(
                "heading_tolerance",
                default_value="0.3",
                description="Waypoint heading tolerance in radians.",
            ),
            DeclareLaunchArgument(
                "circle_of_acceptance",
                default_value="0.5",
                description="Waypoint acceptance radius in meters.",
            ),
            DeclareLaunchArgument(
                "max_heading_rate",
                default_value="0.1",
                description="Maximum heading command rate.",
            ),
            DeclareLaunchArgument(
                "max_heave_rate",
                default_value="0.3",
                description="Maximum heave command rate.",
            ),
            DeclareLaunchArgument(
                "max_station_keep_rate",
                default_value="0.3",
                description="Maximum station-keeping command rate.",
            ),
            DeclareLaunchArgument(
                "run_waypoint_controller_service",
                default_value="/blueye/run_waypoint_controller",
                description="Service used to enable the waypoint controller.",
            ),
            DeclareLaunchArgument(
                "clear_waypoints_service",
                default_value="/blueye/clear_waypoints",
                description="Service used to clear old waypoints.",
            ),
            DeclareLaunchArgument(
                "add_waypoint_service",
                default_value="/blueye/add_waypoint",
                description="Service used to add inspection waypoints.",
            ),
            DeclareLaunchArgument(
                "go_to_waypoints_service",
                default_value="/blueye/go_to_waypoints",
                description="Service used to start waypoint following.",
            ),
            waypoint_controller,
            velocity_controller,
            thruster_allocator,
            TimerAction(period=LaunchConfiguration("mission_start_delay"), actions=[marker_inspector]),
        ]
    )
