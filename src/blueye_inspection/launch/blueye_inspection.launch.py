from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    debug_view = LaunchConfiguration("debug_view")
    publish_debug_image = LaunchConfiguration("publish_debug_image")
    use_compressed_image = LaunchConfiguration("use_compressed_image")
    compressed_image_topic = LaunchConfiguration("compressed_image_topic")
    require_aruco_detection = LaunchConfiguration("require_aruco_detection")
    auto_start = LaunchConfiguration("auto_start")
    target_selection_mode = LaunchConfiguration("target_selection_mode")
    yaw_control_mode = LaunchConfiguration("yaw_control_mode")
    require_heading_for_completion = LaunchConfiguration("require_heading_for_completion")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("blueye_inspection"),
                "config",
                "blueye_inspection_ekf.yaml",
            ]
        ),
        description="robot_localization EKF configuration file.",
    )
    debug_view_arg = DeclareLaunchArgument(
        "debug_view",
        default_value="false",
        description="Show OpenCV ArUco debug window.",
    )
    publish_debug_image_arg = DeclareLaunchArgument(
        "publish_debug_image",
        default_value="true",
        description="Publish annotated inspection camera feed on /blueye/inspection/debug_image.",
    )
    use_compressed_image_arg = DeclareLaunchArgument(
        "use_compressed_image",
        default_value="false",
        description="Subscribe to sensor_msgs/CompressedImage instead of sensor_msgs/Image.",
    )
    compressed_image_topic_arg = DeclareLaunchArgument(
        "compressed_image_topic",
        default_value="/blueye/cam_down/image_color/compressed",
        description="Compressed camera image topic.",
    )
    require_aruco_detection_arg = DeclareLaunchArgument(
        "require_aruco_detection",
        default_value="true",
        description="Stop inspection motion unless a pipeline ArUco marker is visible.",
    )
    auto_start_arg = DeclareLaunchArgument(
        "auto_start",
        default_value="false",
        description="Start pipeline inspection immediately instead of waiting for /blueye/start_pipeline_inspection.",
    )
    target_selection_mode_arg = DeclareLaunchArgument(
        "target_selection_mode",
        default_value="nearest_visible",
        description="Target selection: nearest_visible or ordered_visible.",
    )
    yaw_control_mode_arg = DeclareLaunchArgument(
        "yaw_control_mode",
        default_value="face_target",
        description="Yaw control: face_target, fixed, or off.",
    )
    require_heading_for_completion_arg = DeclareLaunchArgument(
        "require_heading_for_completion",
        default_value="false",
        description="Require yaw tolerance before a marker is marked inspected.",
    )

    aruco_detector = Node(
        package="blueye_inspection",
        executable="pipeline_aruco_detector",
        name="pipeline_aruco_detector",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "image_topic": "/blueye/cam_down/image_color",
                "compressed_image_topic": compressed_image_topic,
                "use_compressed_image": use_compressed_image,
                "debug_view": debug_view,
                "publish_debug_image": publish_debug_image,
                "debug_image_topic": "/blueye/inspection/debug_image",
                "min_markers_for_detection": 1,
                "accepted_marker_ids": [21, 22, 23],
                "fx": 1662.8,
                "fy": 1662.8,
                "cx": 960.0,
                "cy": 540.0,
                "base_yaw_offset": 0.0,
                "pose_z_min": -10.0,
                "pose_z_max": 10.0,
            }
        ],
    )

    sensor_republisher = Node(
        package="mundus_mir_stonefish_docking_controller",
        executable="stonefish_sensor_republisher",
        name="pipeline_sensor_republisher",
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
        name="pipeline_ekf_filter_node",
        output="screen",
        parameters=[params_file],
    )

    inspection_controller = Node(
        package="blueye_inspection",
        executable="pipeline_inspection_controller",
        name="pipeline_inspection_controller",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "odometry_topic": "/odometry/filtered",
                "visual_pose_topic": "/blueye/pipeline_pose_estimated_stamped",
                "velocity_odometry_topic": "/blueye/odom",
                "command_topic": "/blueye/ref_vel",
                "require_aruco_detection": require_aruco_detection,
                "require_visual_pose": True,
                "auto_start": auto_start,
                "target_selection_mode": target_selection_mode,
                "yaw_control_mode": yaw_control_mode,
                "require_heading_for_completion": require_heading_for_completion,
                "inspection_z_offset": 0.1,
            }
        ],
    )

    velocity_controller = Node(
        package="velocity_controller_4dof_ros2",
        executable="velocity_controller_4dof",
        name="pipeline_velocity_controller",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "heading_hold_enabled": False,
                "topic_subscriber_odometry": "/blueye/odom",
                "topic_subscriber_desired": "/blueye/ref_vel",
                "topic_publisher": "/blueye/cmd_force",
                "controller_frequency_hz": 50,
                "rt_surge_kp": 12.0,
                "rt_surge_ki": 0.0,
                "rt_surge_kd": 2.0,
                "rt_surge_satUpper": 12.0,
                "rt_surge_satLower": -12.0,
                "rt_sway_kp": 12.0,
                "rt_sway_ki": 0.0,
                "rt_sway_kd": 3.0,
                "rt_sway_satUpper": 12.0,
                "rt_sway_satLower": -12.0,
                "rt_depth_kp": 12.0,
                "rt_depth_ki": 0.0,
                "rt_depth_kd": 4.0,
                "rt_depth_satUpper": 8.0,
                "rt_depth_satLower": -8.0,
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
            publish_debug_image_arg,
            use_compressed_image_arg,
            compressed_image_topic_arg,
            require_aruco_detection_arg,
            auto_start_arg,
            target_selection_mode_arg,
            yaw_control_mode_arg,
            require_heading_for_completion_arg,
            aruco_detector,
            sensor_republisher,
            ekf_node,
            inspection_controller,
            velocity_controller,
            thruster_allocator,
        ]
    )
