from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = FindPackageShare(package='mundus_mir_docking_controller').find('mundus_mir_docking_controller')

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([pkg_share, 'urdf', 'base.urdf.xml']),
    ])

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('mundus_mir_docking_controller'), 'config', 'robot_localization.yaml'
        ]),
        description='Path to the EKF configuration file',
    )

    robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        default_value=robot_description_content,
        description='Robot description XML file.',
    )

    # TF, DVL and ArUco detection are started here unless real_sensors.launch.py already runs them for the
    # whole mission (the takeover network needs their evidence before docking); the EKF is always started
    with_sensors_arg = DeclareLaunchArgument(
        'with_sensors',
        default_value='true',
        description='Also start robot_state_publisher, the DVL driver and the ArUco pose estimation',
    )
    sensors = IfCondition(LaunchConfiguration('with_sensors'))

    # ArUco visual detection — estimates drone pose relative to dock marker
    pose_estimation_aruco = Node(
        package='mundus_mir_docking_controller',
        executable='pose_estimation_aruco_real_life',
        name='pose_estimation_aruco_real_life',
        output='screen',
        condition=sensors,
    )

    # EKF — fuses DVL velocity + ArUco pose into /blueye/navigation/current_state
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    # TF publisher — needed by EKF and ArUco for frame transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
        condition=sensors,
    )

    # DVL — provides velocity measurements to the EKF
    waterlinked_dvl = Node(
        package='waterlinked_dvl',
        executable='waterlinked_dvl',
        name='waterlinked_dvl',
        output='screen',
        condition=sensors,
    )

    # Docking controller — follows the docking waypoints in the tag frame (/odometry/filtered) and drives
    # /blueye/commands once /blueye/docking_mode is set true (launch_docking_real.sh does that). It prints every
    # command at 10 Hz, so its output goes to the log file (~/.ros/log) only.
    docking_sequence = Node(
        package='mundus_mir_docking_controller',
        executable='docking_sequence',
        name='docking_node',
        output='log',
    )

    ld = LaunchDescription()
    ld.add_action(params_file_arg)
    ld.add_action(robot_description_arg)
    ld.add_action(with_sensors_arg)
    ld.add_action(robot_state_publisher)
    ld.add_action(ekf_node)
    ld.add_action(docking_sequence)
    ld.add_action(waterlinked_dvl)
    ld.add_action(pose_estimation_aruco)
    return ld
