from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Paths and arguments
    # Find the package where the URDF file is located
    # pkg_share = FindPackageShare(package='blueye_description').find('blueye_description')
    pkg_share = FindPackageShare(package='mundus_mir_docking_controller').find('mundus_mir_docking_controller')
    # Path to the URDF file
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'base.urdf.xml'])
    # Use xacro to process URDF file if necessary
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        urdf_file
    ])
    
    # Declare the robot description as a parameter
    robot_description = {'robot_description': robot_description_content}
    
    # Declare arguments
    params_file_arg = DeclareLaunchArgument(
    'params_file',
    default_value=PathJoinSubstitution([
    FindPackageShare('mundus_mir_docking_controller'), 'config', 'robot_localization.yaml'
    ]),
    description='Path to the EKF configuration file'
    )
    
    # # Declare a launch argument allowing for changing the robot description dynamically
    robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        default_value=robot_description_content,
        description='Robot description XML file.'
    )
    
    
    pose_estimation_aruco = Node(
        package='mundus_mir_docking_controller',
        executable='pose_estimation_aruco_node',
        name='pose_estimation_aruco',
        output='screen',
    )
    
    ekf_node = Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[LaunchConfiguration('params_file')],
     )
    
    # Node to publish TF transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
        
    dvl_and_imu_republisher_sim_node = Node(
        package='mundus_mir_docking_controller',
        executable='dvl_and_imu_republisher_sim_node',
        name='dvl_and_imu_republisher_sim_node',
    )

    # # Launch description with all nodes and launch arguments
    ld = LaunchDescription()

    # # EKF nodes
    ld.add_action(params_file_arg)
    ld.add_action(robot_description_arg)
    ld.add_action(ekf_node)
    ld.add_action(robot_state_publisher)
        
    ld.add_action(pose_estimation_aruco)
    
    # # Do not run this node when doing real-life testing. Only in simulator
    ld.add_action(dvl_and_imu_republisher_sim_node)
    
    return ld

