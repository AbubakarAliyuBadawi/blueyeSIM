import os
from glob import glob

from setuptools import setup

package_name = "blueye_inspection"

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
    description="ArUco-triggered Blueye pipeline inspection controller for Stonefish.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pipeline_aruco_detector = blueye_inspection.pipeline_aruco_detector:main",
            "pipeline_inspection_controller = blueye_inspection.pipeline_inspection_controller:main",
        ],
    },
)
