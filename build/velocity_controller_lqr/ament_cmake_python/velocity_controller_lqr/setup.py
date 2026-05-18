from setuptools import find_packages
from setuptools import setup

setup(
    name='velocity_controller_lqr',
    version='1.0.0',
    packages=find_packages(
        include=('velocity_controller_lqr', 'velocity_controller_lqr.*')),
)
