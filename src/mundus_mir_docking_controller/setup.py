from setuptools import setup
import os
from glob import glob

package_name = 'mundus_mir_docking_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Bjorn Magnus',
    maintainer='gabriele',
    maintainer_email='gabriele.kasparaviciute@ntnu.no',
    description='Integrated docking controller package for Blueye AUVs',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'docking_sequence = mundus_mir_docking_controller.docking_control:main',
            'keyboard_control = mundus_mir_docking_controller.keyboard_controller:main',
            'blueye_commands = mundus_mir_docking_controller.blueye_commands:main',
            'blueye_camera = mundus_mir_docking_controller.blueye_camera_node:main',
            'blueye_imu = mundus_mir_docking_controller.blueye_imu_node:main',
            'dvl_and_imu_republisher_sim_node = mundus_mir_docking_controller.dvl_and_imu_republisher_sim:main',
            'aruco_board = mundus_mir_docking_controller.aruco_board_actual_sim:main',
            'pose_estimation_aruco_node = mundus_mir_docking_controller.pose_estimation_aruco:main',
        ],
    },
)

