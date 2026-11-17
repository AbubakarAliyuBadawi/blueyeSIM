import os
from glob import glob

from setuptools import setup


package_name = "lauv_marie_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "dune"), glob("dune/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Badawi",
    maintainer_email="abubakar.a.badawi@ntnu.no",
    description="Heading, depth and speed autopilot and keyboard control for LAUV Marie in Stonefish.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "autopilot = lauv_marie_control.autopilot:main",
            "teleop_keyboard = lauv_marie_control.teleop_keyboard:main",
            "dune_bridge = lauv_marie_control.dune_bridge:main",
            "dune_plan = lauv_marie_control.dune_plan:main",
        ],
    },
)
