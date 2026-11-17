from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sonar_3d'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'wlsonar==0.4.0',
    ],
    zip_safe=True,
    maintainer='laura',
    maintainer_email='lauraest@stud.ntnu.no',
    description='Waterlinked 3D sonar point cloud publisher',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sonar_3d_point_cloud = sonar_3d.point_cloud_with_intensity:main',
        ],
    },
)
