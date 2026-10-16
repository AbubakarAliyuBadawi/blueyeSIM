from setuptools import setup

package_name = "bn_evidence_gui"

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
    description="GUI node to inject evidence into the Bayesian Network during simulation.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bn_evidence_gui = bn_evidence_gui.gui_node:main",
        ],
    },
)
