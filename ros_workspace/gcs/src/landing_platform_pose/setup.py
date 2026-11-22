from setuptools import setup

package_name = 'landing_platform_pose'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eric Lind',
    maintainer_email='elind1@berkeley.edu',
    description='ArUco-based landing platform pose estimator for ROS 2.',
    license='UNLICENSED',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'platform_pose_node = '
            'landing_platform_pose.platform_pose_node:main',
        ],
    },
)
