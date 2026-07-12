from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    drone_ip_arg = DeclareLaunchArgument(
        'drone_ip',
        default_value='192.168.1.101',
        description='IP address of the Blueye ROV (used by navigation scripts)'
    )

    # Connects to the drone, publishes telemetry, and exposes
    # /blueye/depth_hold, /blueye/heading_hold, /blueye/stationkeep services.
    # Also publishes /blueye/battery and /blueye/speed for the BT.
    blueye_handler_node = Node(
        package='blueye_handler',
        executable='blueye_handler',
        name='blueye_handler',
        output='screen'
    )

    rov_bt_pkg_dir = get_package_share_directory('blueye_real_bt')
    bt_dir = os.path.join(rov_bt_pkg_dir, 'behavior_trees')
    behavior_tree_path = os.path.join(bt_dir, 'FullMission.xml')

    if not os.path.exists(behavior_tree_path):
        raise FileNotFoundError(f"Behavior tree file not found: {behavior_tree_path}")

    blueye_mission_node_real = Node(
        package='blueye_real_bt',
        executable='blueye_real_bt',
        name='blueye_mission_real',
        output='screen',
        parameters=[{
            'behavior_tree_path': behavior_tree_path,
            'bt_directory': bt_dir
        }]
    )

    delayed_bt_node = TimerAction(
        period=5.0,
        actions=[blueye_mission_node_real]
    )

    return LaunchDescription([
        drone_ip_arg,
        blueye_handler_node,
        delayed_bt_node,
    ])