from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "autorepeat_rate": 20.0,
            "deadzone": 0.08,
        }],
    )

    # Joystick publishes desired velocities to /blueye/ref_vel (velocity control mode)
    joystick_controller = Node(
        package="mundus_mir_blueye_joystick_cpp",
        executable="joystick_controller",
        name="joystick_controller",
        output="screen",
        parameters=[{
            "quadratic_mapping": False,
            "max_thrust_surge": 10.0,
            "max_thrust_sway": 10.0,
            "max_thrust_heave": 6.0,
            "max_thrust_yaw": 8.0,
            "cmd_topic": "/blueye/cmd_force",
            "velocity_control": True,
            "max_velocity_x": 0.5,
            "max_velocity_y": 0.5,
            "max_velocity_z": 0.5,
            "max_angular_velocity_yaw": 1.0,
            "velocity_topic": "/blueye/ref_vel",
            "joystick_deadband": 0.08,
            "right_joystick_x_axis": 3,
            "right_joystick_y_axis": 4,
            "left_joystick_x_axis": 0,
            "left_joystick_y_axis": 1,
        }],
    )

    # 4-DOF PID velocity controller:
    # /blueye/ref_vel (TwistStamped) + /blueye/odom (Odometry) -> /blueye/cmd_force (WrenchStamped)
    # Yaw PID compensates for any residual sway-yaw coupling.
    # Gains tuned conservatively for Stonefish I_z=3.5 (vs Gazebo I_z=100).
    # Increase yaw_kp/kd if yaw response feels sluggish.
    velocity_controller = Node(
        package="velocity_controller_4dof_ros2",
        executable="velocity_controller_4dof",
        name="blueye_velocity_controller",
        output="screen",
        parameters=[{
            "topic_subscriber_odometry": "/blueye/odom",
            "topic_subscriber_desired":  "/blueye/ref_vel",
            "topic_publisher":           "/blueye/cmd_force",
            "controller_frequency_hz": 50,
            "rt_surge_kp":      10.0,
            "rt_surge_ki":       0.0,
            "rt_surge_kd":       0.0,
            "rt_surge_satUpper": 10.0,
            "rt_surge_satLower":-10.0,
            "rt_sway_kp":       10.0,
            "rt_sway_ki":        0.0,
            "rt_sway_kd":        0.0,
            "rt_sway_satUpper":  10.0,
            "rt_sway_satLower": -10.0,
            "rt_depth_kp":      10.0,
            "rt_depth_ki":       0.0,
            "rt_depth_kd":       4.0,
            "rt_depth_satUpper": 6.0,
            "rt_depth_satLower":-6.0,
            "rt_yaw_kp":        8.0,
            "rt_yaw_ki":         0.0,
            "rt_yaw_kd":         3.0,
            "rt_yaw_satUpper":   8.0,
            "rt_yaw_satLower":  -8.0,
        }],
    )

    stonefish_thruster_allocator = Node(
        package="mundus_mir_vehicle_interfaces",
        executable="blueye_simulator_interface",
        name="stonefish_blueye_thruster_allocator",
        output="screen",
        parameters=[{
            "cmd_force_topic": "/blueye/cmd_force",
            "thrusters_topic": "/blueye/thrusters",
            "max_thruster_setpoint": 35.0,
        }],
    )

    return LaunchDescription([
        joy_node,
        joystick_controller,
        velocity_controller,
        stonefish_thruster_allocator,
    ])
