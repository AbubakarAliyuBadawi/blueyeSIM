"""
Sensors of the real Blueye that run for the whole mission, so the takeover network has their evidence from the
start: TF (robot_state_publisher), the Water Linked DVL driver (/blueye/sensor/dvl) and the ArUco pose
estimation (camera image on /blueye/image, dock board pose on /blueye/pose_estimated_board_stamped).

Start it after the handler and before the behaviour tree:
  ros2 launch mundus_mir_docking_controller real_sensors.launch.py

The tree's docking step (launch_docking_real.sh, docking_bt.launch.py) sees these nodes running and then adds
only the EKF. Node names are the same as in docking_bt.launch.py.
"""

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare(package='mundus_mir_docking_controller').find('mundus_mir_docking_controller')
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([pkg_share, 'urdf', 'base.urdf.xml']),
    ])

    return LaunchDescription([
        # TF publisher — needed by the ArUco pose estimation and, at docking, the EKF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
        ),
        # DVL — velocity and bottom lock: DVL status for the network, velocity for the docking EKF
        Node(
            package='waterlinked_dvl',
            executable='waterlinked_dvl',
            name='waterlinked_dvl',
            output='screen',
        ),
        # Camera and ArUco detection — image for the camera-quality monitor, dock board pose for docking
        Node(
            package='mundus_mir_docking_controller',
            executable='pose_estimation_aruco_real_life',
            name='pose_estimation_aruco_real_life',
            output='screen',
        ),
    ])
