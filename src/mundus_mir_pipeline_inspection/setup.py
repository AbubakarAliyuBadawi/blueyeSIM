import os
from glob import glob

from setuptools import setup

package_name = "mundus_mir_pipeline_inspection"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="badawi",
    maintainer_email="badawi@example.com",
    description="Simple marker-based pipeline inspection mission launcher for Blueye in Stonefish.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pipeline_marker_inspector = mundus_mir_pipeline_inspection.pipeline_marker_inspector:main",
        ],
    },
)
