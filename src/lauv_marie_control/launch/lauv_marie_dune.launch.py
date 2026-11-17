"""
LAUV Marie in Stonefish, flown by DUNE: the simulator, DUNE (the LAUV's onboard software) and the bridge between them.

  ros2 launch lauv_marie_control lauv_marie_dune.launch.py                     # with the 3D window
  ros2 launch lauv_marie_control lauv_marie_dune.launch.py rendering:=false    # headless

Then plans, from a second terminal or from Neptus (DUNE shows up there as lauv-simulator-1):
  ros2 run lauv_marie_control dune_plan goto 100 50 --depth 5 --popup

DUNE is the copy in dune_dir (default ~/dune-simulator/dune-marie), so the stock simulator in ~/dune-simulator/dune
stays as it is. Do not run both at the same time: they have the same name, lauv-simulator-1. The configuration
dune/lauv-marie-stonefish.ini of this package is copied to dune_dir/etc at every launch; DUNE's logs go to
dune_dir/log as usual.

The autopilot of this package is not started: DUNE's own controllers fly Marie.
"""

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONFIG = "lauv-marie-stonefish"


def setup(context):
    share = get_package_share_directory("lauv_marie_control")
    dune_dir = os.path.expanduser(LaunchConfiguration("dune_dir").perform(context))
    dune = os.path.join(dune_dir, "bin", "dune")
    if not os.path.exists(dune):
        raise RuntimeError(f"DUNE not found at {dune}: set dune_dir:=<folder with bin/dune and etc/>")
    shutil.copy(os.path.join(share, "dune", CONFIG + ".ini"), os.path.join(dune_dir, "etc", CONFIG + ".ini"))
    config = LaunchConfiguration("dune_config").perform(context)
    if not os.path.exists(os.path.join(dune_dir, "etc", config + ".ini")):
        raise RuntimeError(f"DUNE configuration {config}.ini not found in {dune_dir}/etc")

    rendering = LaunchConfiguration("rendering").perform(context).lower() == "true"
    simulation = os.path.join(get_package_share_directory("stonefish_sim"), "launch", "simulation.launch.py")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation),
            launch_arguments={
                "scenario": "lauv_marie" if rendering else "lauv_marie_no_gpu",
                "rendering": "true" if rendering else "false",
                "rendering_quality": LaunchConfiguration("rendering_quality").perform(context),
                "scenario_config_override": os.path.join(share, "dune", "stonefish_start.yaml"),
            }.items(),
        ),
        ExecuteProcess(
            cmd=[dune, "-c", config, "-p", "Simulation"],
            cwd=os.path.join(dune_dir, "bin"),
            name="dune",
            output=LaunchConfiguration("dune_output").perform(context),
        ),
        Node(
            package="lauv_marie_control",
            executable="dune_bridge",
            name="lauv_marie_dune_bridge",
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("rendering", default_value="true", description="3D window (true/false)"),
            DeclareLaunchArgument("rendering_quality", default_value="med", description="high/med/low"),
            DeclareLaunchArgument("dune_dir", default_value="~/dune-simulator/dune-marie",
                                  description="DUNE folder (bin/dune, etc/)"),
            DeclareLaunchArgument("dune_config", default_value=CONFIG,
                                  description="DUNE configuration in dune_dir/etc (one that requires " + CONFIG + ")"),
            DeclareLaunchArgument("dune_output", default_value="screen",
                                  description="screen: DUNE's messages in this terminal; log: only in its log file"),
            OpaqueFunction(function=setup),
        ]
    )
