from setuptools import find_packages
from setuptools import setup

setup(
    name='mundus_mir_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('mundus_mir_msgs', 'mundus_mir_msgs.*')),
)
