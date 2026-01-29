from setuptools import setup
import os
from glob import glob

package_name = 'main_arm_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fiona',
    maintainer_email='fkprendergast@wpi.edu',
    description='Main arm control package for line manipulation and line stow setup',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'main_arm_interface_node = main_arm_control.main_arm_interface_node:main',
        'main_arm_planner_node = main_arm_control.main_arm_planner_node:main',
        'main_arm_teleop_node = main_arm_control.main_arm_teleop_node:main',
        'xbox_arm_controller_node = main_arm_control.xbox_arm_controller_node:main',
        'test_kinematics = main_arm_control.test_kinematics:main'
        ],
    },
)