import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'ros2_rover'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro') + glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ayush',
    maintainer_email='robot@workbench.local',
    description='ROS2 Mecanum Rover Stack: URDF, Static TF, ToF LaserScan/Range Publisher, Nav2 Omnidirectional Config',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tof_laserscan_publisher = ros2_rover.tof_laserscan_publisher:main',
            'ld2402_radar_node = ros2_rover.ld2402_radar_node:main',
            'doa_srp_phat_node = ros2_rover.doa_srp_phat_node:main',
            'voice_follower_node = ros2_rover.voice_follower_node:main',
        ],
    },
)
