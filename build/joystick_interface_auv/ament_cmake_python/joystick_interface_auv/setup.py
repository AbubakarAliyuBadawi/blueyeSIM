from setuptools import find_packages
from setuptools import setup

setup(
    name='joystick_interface_auv',
    version='0.0.0',
    packages=find_packages(
        include=('joystick_interface_auv', 'joystick_interface_auv.*')),
)
