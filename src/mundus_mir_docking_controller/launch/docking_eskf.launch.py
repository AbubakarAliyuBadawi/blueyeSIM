from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    # ============================================================
    # Launch variables
    # ============================================================
    auto_align_arg = DeclareLaunchArgument(
    'auto_align',
    default_value='false',
    description='Automatically align ESKF to ArUco frame'
    )

    # ============================================================
    # PACKAGE / URDF
    # ============================================================

    pkg_share = FindPackageShare(
        package='mundus_mir_docking_controller'
    ).find('mundus_mir_docking_controller')

    urdf_file = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'base.urdf.xml'
    ])

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        urdf_file
    ])

    robot_description = {
        'robot_description': robot_description_content
    }

    # ============================================================
    # LAUNCH ARGUMENTS
    # ============================================================

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('mundus_mir_docking_controller'),
            'config',
            'robot_localization.yaml'
        ]),
        description='Path to localization configuration file'
    )

    robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        default_value=robot_description_content,
        description='Robot description XML file'
    )

    # ============================================================
    # ARUCO POSE ESTIMATION
    # ============================================================

    pose_estimation_aruco_real_life = Node(
        package='mundus_mir_docking_controller',
        executable='pose_estimation_aruco_real_life',
        name='pose_estimation_aruco_real_life',
        output='screen',
    )

    # ============================================================
    # CUSTOM BLUEYE ESKF
    # ============================================================

    blueye_eskf = Node(
        package='blueye_eskf',
        executable='real_life_eskf',
        name='blueye_eskf',
        output='screen',
        parameters=[
            {
                'auto_align': LaunchConfiguration('auto_align')
            }
        ]
    )

    # ============================================================
    # ROBOT STATE PUBLISHER
    # ============================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # ============================================================
    # BLUEYE COMMAND INTERFACE
    # ============================================================

    blueye_commands = Node(
        package='mundus_mir_docking_controller',
        executable='blueye_commands',
        name='blueye_commands',
        output='screen',
    )

    # ============================================================
    # JOYSTICK
    # ============================================================

    joystick_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    blueye_joystick = Node(
        package='blueye_joystick_cpp',
        executable='joystick_controller',
        name='blueye_joystick',
        output='screen',
    )

    # ============================================================
    # DVL
    # ============================================================

    blueye_dvl = Node(
        package='waterlinked_dvl',
        executable='waterlinked_dvl',
        name='waterlinked_dvl',
        output='screen',
    )

    # ============================================================
    # USBL
    # ============================================================

    blueye_usbl = Node(
        package='evologic_usbl',
        executable='evologics_usbl',
        name='evologics_parser',
        output='screen',
    )
    
    # ============================================================
    # Inspection Leo
    # ============================================================
    inspection_leo = Node(
        package='mundus_mir_docking_controller',
        executable='inspection_leo',
        name='inspection_leo',
        output='screen',
    )

    # ============================================================
    # LAUNCH DESCRIPTION
    # ============================================================

    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(params_file_arg)
    ld.add_action(robot_description_arg)
    ld.add_action(auto_align_arg)

    # Core system
    ld.add_action(robot_state_publisher)

    # Sensors
    ld.add_action(blueye_dvl)
    ld.add_action(blueye_usbl)

    # Localization
    ld.add_action(blueye_eskf)

    # Perception
    ld.add_action(pose_estimation_aruco_real_life)

    # Vehicle interface
    ld.add_action(blueye_commands)

    # Joystick
    ld.add_action(joystick_node)
    ld.add_action(blueye_joystick)

    # Inspection Node
    ld.add_action(inspection_leo)

    return ld