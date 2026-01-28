import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'side_arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='fprendergast18@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'side_arm_interface_node = side_arm_control.side_arm_interface_node:main',
            'coordinate_node = side_arm_control.coordinate_node:main',
            'manual_jog = side_arm_control.manual_jog:main',
            'side_arm_visualizer = side_arm_control.side_arm_visualizer:main',
        ],
    },
)
