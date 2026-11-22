from setuptools import setup
import os
from glob import glob

package_name = 'aruco_handle'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eric Lind',
    maintainer_email='elind1@berkeley.edu',
    description='ArUco-based handle search pose estimator for ROS 2.',
    license='UNLICENSED',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_node = aruco_handle.aruco_node:main',
            'platform_pose_node = aruco_handle.platform_pose_node:main',
        ],
    },
)
