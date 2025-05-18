from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import TimerAction
import os

def generate_launch_description():
    # Find the package path
    pkg_share = FindPackageShare(package='mundus_mir_docking_controller').find('mundus_mir_docking_controller')

    # Load the URDF file contents directly
    urdf_path = os.path.join(pkg_share, 'urdf', 'base.urdf.xml')
    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    robot_description = {'robot_description': robot_description_content}

    # Declare parameter file path
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('mundus_mir_docking_controller'),
            'config',
            'robot_localization.yaml'
        ]),
        description='Path to the EKF configuration file'
    )

    # EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Pose Estimation Node (Aruco)
    pose_estimation_aruco = Node(
        package='mundus_mir_docking_controller',
        executable='pose_estimation_aruco_node',
        name='pose_estimation_aruco',
        output='screen',
    )

    # Sim DVL and IMU republisher
    dvl_and_imu_republisher_sim_node = Node(
        package='mundus_mir_docking_controller',
        executable='dvl_and_imu_republisher_sim_node',
        name='dvl_and_imu_republisher_sim_node',
        output='screen',
    )
    
    docking_sequence = TimerAction(
    period=20.0,  # delay in seconds
    actions=[
        Node(
            package='mundus_mir_docking_controller',
            executable='docking_sequence',
            name='docking_sequence',
            output='screen',
        )
    ]
)

    return LaunchDescription([
        params_file_arg,
        robot_state_publisher,
        ekf_node,
        pose_estimation_aruco,
        dvl_and_imu_republisher_sim_node,
        docking_sequence
    ])
