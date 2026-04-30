import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'parachute_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.widgets'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='fprendergast18@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'operator_console = parachute_gui.main_window:main',
        ],
    },
)