from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    debug_view = LaunchConfiguration("debug_view")
    require_aruco_detection = LaunchConfiguration("require_aruco_detection")
    auto_start = LaunchConfiguration("auto_start")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("mundus_mir_stonefish_docking_controller"),
                "config",
                "stonefish_robot_localization.yaml",
            ]
        ),
        description="robot_localization EKF configuration file.",
    )

    debug_view_arg = DeclareLaunchArgument(
        "debug_view",
        default_value="false",
        description="Show OpenCV ArUco debug window.",
    )

    require_aruco_detection_arg = DeclareLaunchArgument(
        "require_aruco_detection",
        default_value="true",
        description="Stop publishing motion commands when fewer than the required ArUco markers are visible.",
    )

    auto_start_arg = DeclareLaunchArgument(
        "auto_start",
        default_value="false",
        description="Start docking motion immediately instead of waiting for /blueye/start_docking.",
    )

    aruco_pose = Node(
        package="mundus_mir_stonefish_docking_controller",
        executable="stonefish_aruco_pose",
        name="stonefish_aruco_pose",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "image_topic": "/blueye/cam/image_color",
                "debug_view": debug_view,
                "fx": 1662.8,
                "fy": 1662.8,
                "cx": 960.0,
                "cy": 540.0,
                "min_markers_for_detection": 3,
            }
        ],
    )

    sensor_republisher = Node(
        package="mundus_mir_stonefish_docking_controller",
        executable="stonefish_sensor_republisher",
        name="stonefish_sensor_republisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "imu_in_topic": "/blueye/imu/data_raw",
                "dvl_in_topic": "/blueye/dvl/sim",
                "imu_out_topic": "/blueye/imu_enu",
                "dvl_out_topic": "/blueye/dvl_enu",
                "base_frame_id": "base_link",
                "imu_frame_id": "base_link",
            }
        ],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[params_file],
    )

    docking_controller = Node(
        package="mundus_mir_stonefish_docking_controller",
        executable="stonefish_docking_controller",
        name="stonefish_docking_controller",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "odometry_topic": "/odometry/filtered",
                "visual_pose_topic": "/blueye/pose_estimated_board_stamped",
                "velocity_odometry_topic": "/blueye/odom",
                "command_topic": "/blueye/ref_vel",
                "require_aruco_detection": require_aruco_detection,
                "use_visual_pose_fallback": False,
                "prefer_visual_pose": True,
                "auto_start": auto_start,
            }
        ],
    )

    velocity_controller = Node(
        package="velocity_controller_4dof_ros2",
        executable="velocity_controller_4dof",
        name="velocity_controller",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "topic_subscriber_odometry": "/blueye/odom",
                "topic_subscriber_desired": "/blueye/ref_vel",
                "topic_publisher": "/blueye/cmd_force",
                "controller_frequency_hz": 50,
                "rt_surge_kp": 18.0,
                "rt_surge_ki": 0.0,
                "rt_surge_kd": 4.0,
                "rt_surge_satUpper": 18.0,
                "rt_surge_satLower": -18.0,
                "rt_sway_kp": 18.0,
                "rt_sway_ki": 0.0,
                "rt_sway_kd": 5.0,
                "rt_sway_satUpper": 18.0,
                "rt_sway_satLower": -18.0,
                "rt_depth_kp": 14.0,
                "rt_depth_ki": 0.0,
                "rt_depth_kd": 4.0,
                "rt_depth_satUpper": 12.0,
                "rt_depth_satLower": -12.0,
                "rt_yaw_kp": 10.0,
                "rt_yaw_ki": 0.0,
                "rt_yaw_kd": 3.0,
                "rt_yaw_satUpper": 10.0,
                "rt_yaw_satLower": -10.0,
            }
        ],
    )

    thruster_allocator = Node(
        package="mundus_mir_vehicle_interfaces",
        executable="blueye_simulator_interface",
        name="stonefish_blueye_thruster_allocator",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "cmd_force_topic": "/blueye/cmd_force",
                "thrusters_topic": "/blueye/thrusters",
                "max_thruster_setpoint": 35.0,
            }
        ],
    )

    return LaunchDescription(
        [
            params_file_arg,
            debug_view_arg,
            require_aruco_detection_arg,
            auto_start_arg,
            aruco_pose,
            sensor_republisher,
            ekf_node,
            docking_controller,
            velocity_controller,
            thruster_allocator,
        ]
    )
