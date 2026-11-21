from setuptools import find_packages, setup

package_name = 'lab6_capture'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kushtimusPrime',
    maintainer_email='kushtimusprime@gmail.com',
    description='Lab 6 nodes: hold-to-drive teleop and image + pose capture.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'hold_teleop = lab6_capture.hold_teleop:main',
            'capture_pose = lab6_capture.capture_pose:main',
        ],
    },
)
