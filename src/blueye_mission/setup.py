from setuptools import setup
import os
from glob import glob

package_name = 'blueye_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ROV mission control using YASMIN',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rov_mission = blueye_mission.rov_mission:main',
            'dock_distance_calc = blueye_mission.dock_distance_calc:main',
            'battery_management = blueye_mission.battery_management:main'
        ],
    },
)