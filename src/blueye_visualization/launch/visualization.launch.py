#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='blueye_visualization',
            executable='rov_visual_2d_xy',
            name='rov_visual_2d_xy'
        ),
        Node(
            package='blueye_visualization',
            executable='rov_visual_2d_z',
            name='rov_visual_2d_z'
        ),
        Node(
            package='blueye_visualization',
            executable='rov_viz_matplotlib_3d',
            name='rov_viz_matplotlib_3d'
        )
    ])
