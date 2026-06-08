import os
from glob import glob

from setuptools import setup


package_name = "blueye_takeover_bn"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
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
    description="Full-mission Bayesian network takeover request inference for Blueye ROV.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "takeover_bn = blueye_takeover_bn.takeover_bn_node:main",
            "camera_rate_monitor = blueye_takeover_bn.camera_rate_monitor:main",
        ],
    },
)
