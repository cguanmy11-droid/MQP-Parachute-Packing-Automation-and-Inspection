from glob import glob
import os

from setuptools import setup

package_name = 'side_arm_motor_control_bridge'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Side Arm Team',
    maintainer_email='zheren@example.com',
    description='ROS 2 serial bridge for ESP32 side arm controller.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = side_arm_motor_control_bridge.serial_bridge_node:main'
        ],
    },
)

