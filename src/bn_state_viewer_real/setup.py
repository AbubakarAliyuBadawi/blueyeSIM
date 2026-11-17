from setuptools import setup

package_name = "bn_state_viewer_real"

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
    maintainer="Badawi",
    maintainer_email="abubakaraliyubadawi@gmail.com",
    description="Live view of the takeover Bayesian network on the real Blueye, with the input of every evidence node.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bn_state_viewer_real = bn_state_viewer_real.viewer_node:main",
        ],
    },
)
