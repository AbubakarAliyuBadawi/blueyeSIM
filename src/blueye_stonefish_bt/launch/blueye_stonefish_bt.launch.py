from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bt_file_arg = DeclareLaunchArgument(
        "bt_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("blueye_stonefish_bt"),
            "behavior_trees",
            "TestMission.xml",
        ]),
        description="Path to the BehaviorTree XML file to execute.",
    )

    odometry_topic_arg = DeclareLaunchArgument(
        "odometry_topic",
        default_value="/blueye/odom",
        description="Odometry topic used by the waypoint controller.",
    )

    bt_node = Node(
        package="blueye_stonefish_bt",
        executable="blueye_stonefish_bt",
        name="blueye_stonefish_bt",
        output="screen",
        parameters=[{"behavior_tree_path": LaunchConfiguration("bt_file")}],
    )

    waypoint_controller = Node(
        package="mundus_mir_waypoint_controller",
        executable="waypoint_controller",
        name="bt_waypoint_controller",
        output="screen",
        parameters=[{
            "odometry_topic": LaunchConfiguration("odometry_topic"),
            "desired_vel_topic": "/blueye/ref_vel",
            "heading_tolerance": 0.3,
            "circle_of_acceptance": 0.5,
            "max_heading_rate": 0.15,
            "max_heave_rate": 0.3,
            "max_station_keep_rate": 0.3,
        }],
    )

    velocity_controller = Node(
        package="velocity_controller_4dof_ros2",
        executable="velocity_controller_4dof",
        name="bt_velocity_controller",
        output="screen",
        parameters=[{
            "topic_subscriber_odometry": LaunchConfiguration("odometry_topic"),
            "topic_subscriber_desired": "/blueye/ref_vel",
            "topic_publisher": "/blueye/cmd_force",
            "controller_frequency_hz": 50,
            "rt_surge_kp": 18.0, "rt_surge_ki": 0.0, "rt_surge_kd": 4.0,
            "rt_surge_satUpper": 18.0, "rt_surge_satLower": -18.0,
            "rt_sway_kp": 18.0,  "rt_sway_ki": 0.0,  "rt_sway_kd": 5.0,
            "rt_sway_satUpper": 18.0,  "rt_sway_satLower": -18.0,
            "rt_depth_kp": 14.0, "rt_depth_ki": 0.0, "rt_depth_kd": 4.0,
            "rt_depth_satUpper": 12.0,  "rt_depth_satLower": -12.0,
            "rt_yaw_kp": 10.0,   "rt_yaw_ki": 0.0,   "rt_yaw_kd": 3.0,
            "rt_yaw_satUpper": 10.0,    "rt_yaw_satLower": -10.0,
        }],
    )

    thruster_allocator = Node(
        package="mundus_mir_vehicle_interfaces",
        executable="blueye_simulator_interface",
        name="bt_thruster_allocator",
        output="screen",
        parameters=[{
            "cmd_force_topic": "/blueye/cmd_force",
            "thrusters_topic": "/blueye/thrusters",
            "max_thruster_setpoint": 35.0,
        }],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "autorepeat_rate": 0.0,
            "deadzone": 0.08,
        }],
    )

    joystick_controller = Node(
        package="mundus_mir_blueye_joystick_cpp",
        executable="joystick_controller",
        name="joystick_controller",
        output="screen",
        parameters=[{
            "quadratic_mapping": False,
            "max_thrust_surge": 10.0,
            "max_thrust_sway": 10.0,
            "max_thrust_heave": 6.0,
            "max_thrust_yaw": 8.0,
            "cmd_topic": "/blueye/cmd_force",
            "velocity_control": True,
            "max_velocity_x": 0.5,
            "max_velocity_y": 0.5,
            "max_velocity_z": 0.5,
            "max_angular_velocity_yaw": 1.0,
            "velocity_topic": "/blueye/ref_vel",
            "joystick_deadband": 0.08,
            "right_joystick_x_axis": 3,
            "right_joystick_y_axis": 4,
            "left_joystick_x_axis": 0,
            "left_joystick_y_axis": 1,
        }],
    )

    return LaunchDescription([
        bt_file_arg,
        odometry_topic_arg,
        waypoint_controller,
        velocity_controller,
        thruster_allocator,
        joy_node,
        joystick_controller,
        bt_node,
    ])
