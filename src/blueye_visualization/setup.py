from setuptools import find_packages, setup
import os
from glob import glob

package_name = "blueye_visualization"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="badawi",
    maintainer_email="abubakaraliyubadawi@gmail.com",
    description="Operator HUD for Blueye X3 ROV",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "operator_hud = blueye_visualization.operator_hud:main",
        ],
    },
)
