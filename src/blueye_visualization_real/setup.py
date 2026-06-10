import os
from glob import glob

from setuptools import find_packages, setup

package_name = "blueye_visualization_real"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="badawi",
    maintainer_email="abubakaraliyubadawi@gmail.com",
    description="Operator HUD for real Blueye drone with takeover support",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "blueye_real_hud = blueye_visualization_real.operator_hud_real:main",
        ],
    },
)
