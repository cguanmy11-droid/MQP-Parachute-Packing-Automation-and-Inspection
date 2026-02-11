from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'parachute_coordinator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config', 'motion_patterns'),
            glob('config/motion_patterns/*.json')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
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
		'packing_coordinator_node = parachute_coordinator.packing_coordinator_node:main',
        'packing_demo_node = parachute_coordinator.packing_demo_node:main',
        ],
    },
)
