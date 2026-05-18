import os
from glob import glob

from setuptools import setup

package_name = "mundus_mir_stonefish_docking_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="badawi",
    maintainer_email="badawi@example.com",
    description="Stonefish docking controller for the Blueye drone.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "stonefish_aruco_pose = mundus_mir_stonefish_docking_controller.stonefish_aruco_pose:main",
            "stonefish_sensor_republisher = mundus_mir_stonefish_docking_controller.stonefish_sensor_republisher:main",
            "stonefish_docking_controller = mundus_mir_stonefish_docking_controller.stonefish_docking_controller:main",
        ],
    },
)
