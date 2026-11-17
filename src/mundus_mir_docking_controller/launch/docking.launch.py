from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Paths and arguments
    # Find the package where the URDF file is located
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
        default_value=PathJoinSubstitution([FindPackageShare('mundus_mir_docking_controller'), 'config', 'robot_localization.yaml']),
        description='Path to the EKF configuration file'
    )
    
    # Declare a launch argument allowing for changing the robot description dynamically
    robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        default_value=robot_description_content,
        description='Robot description XML file.'
    )
    
    pose_estimation_aruco_real_life = Node(
        package='mundus_mir_docking_controller',
        executable='pose_estimation_aruco_real_life',
        name='pose_estimation_aruco_real_life',
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

    blueye_commands = Node(
        package='mundus_mir_docking_controller',
        executable='blueye_commands',
        name='blueye_commands',
    )

    joystick_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
    )

    blueye_joystick = Node(
        package='blueye_joystick_cpp',
        executable='joystick_controller',
        name='blueye_joystick',
        output='screen',
    )

    blueye_dvl = Node(
        package='waterlinked_dvl',
        executable='waterlinked_dvl',
        name='waterlinked_dvl',
        output='screen',
    )

    blueye_usbl = Node(
        package="evologic_usbl",
        executable="evologics_usbl",
        name="evologics_parser",
        output='screen',
    )

    # Launch description with all nodes and launch arguments
    ld = LaunchDescription()

    # EKF nodes
    ld.add_action(params_file_arg)
    ld.add_action(robot_description_arg)
    ld.add_action(ekf_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(joystick_node)
    ld.add_action(blueye_joystick)
    ld.add_action(blueye_dvl)

    # -----------Run these nodes only when doing real world runs----------
    
    # Run this in real life
    ld.add_action(pose_estimation_aruco_real_life)
    
    # Run this node when doing real-life testing  
    ld.add_action(blueye_commands)
    
    #---------------------------------------------------------------------
    
    # Make sure this node doesnt run simultaneously with the Waypoint_and_control node
    # Start and stop the py-script version of this in the terminal instead
    # ld.add_action(keyboard_controller)
    
    return ld