import os
from glob import glob

from setuptools import setup

package_name = "blueye_sonar_detection"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="badawi",
    maintainer_email="abubakaraliyubadawi@gmail.com",
    description="FLS sonar-based pipeline detection for the Blueye ROV.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pipeline_sonar_detector = blueye_sonar_detection.pipeline_sonar_detector:main",
        ],
    },
)
