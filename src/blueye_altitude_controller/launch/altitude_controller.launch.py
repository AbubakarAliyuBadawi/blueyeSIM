from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    target_altitude_arg = DeclareLaunchArgument(
        'target_altitude',
        default_value='2.0',
        description='Target altitude in meters'
    )
    
    enabled_arg = DeclareLaunchArgument(
        'enabled',
        default_value='false',
        description='Enable altitude control at startup'
    )
    
    min_depth_arg = DeclareLaunchArgument(
        'min_depth',
        default_value='1.0',
        description='Minimum allowed depth in meters'
    )
    
    max_depth_arg = DeclareLaunchArgument(
        'max_depth',
        default_value='30.0',
        description='Maximum allowed depth in meters'
    )
    
    # Create the node
    altitude_controller_node = Node(
        package='blueye_altitude_controller',
        executable='altitude_controller',
        name='altitude_controller',
        output='screen',
        parameters=[{
            'target_altitude': LaunchConfiguration('target_altitude'),
            'altitude_control_enabled': LaunchConfiguration('enabled'),
            'min_depth': LaunchConfiguration('min_depth'),
            'max_depth': LaunchConfiguration('max_depth'),
            'safety_margin': 0.5,
            'max_z_velocity': 0.3,
            'kp': 0.5,
            'ki': 0.1,
            'kd': 0.2,
            'velocity_topic': '/blueye/ref_vel'
        }]
    )
    
    return LaunchDescription([
        target_altitude_arg,
        enabled_arg,
        min_depth_arg,
        max_depth_arg,
        altitude_controller_node
    ])