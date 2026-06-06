import os
from glob import glob

from setuptools import setup


package_name = "blueye_takeover_bn_real"

setup(
    name=package_name,
    version="0.1.0",
    packages=[
        package_name,
        package_name + ".scripts",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.xdsl")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Badawi",
    maintainer_email="abubakar.a.badawi@ntnu.no",
    description="Real-mission Bayesian network takeover request inference for Blueye ROV.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "takeover_bn_real = blueye_takeover_bn_real.takeover_bn_real_node:main",
            "aruco_visibility_monitor = blueye_takeover_bn_real.scripts.aruco_visibility_monitor:main",
            "camera_quality_monitor = blueye_takeover_bn_real.scripts.camera_quality_monitor:main",
            "inspection_data_quality_monitor = blueye_takeover_bn_real.scripts.inspection_data_quality_monitor:main",
            "real_current_publisher = blueye_takeover_bn_real.scripts.real_current_publisher:main",
            "real_mission_state_monitors = blueye_takeover_bn_real.scripts.real_mission_state_monitors:main",
            "sonar_range_monitor = blueye_takeover_bn_real.scripts.sonar_range_monitor:main",
            "usbl_strength_monitor = blueye_takeover_bn_real.scripts.usbl_strength_monitor:main",
        ],
    },
)
