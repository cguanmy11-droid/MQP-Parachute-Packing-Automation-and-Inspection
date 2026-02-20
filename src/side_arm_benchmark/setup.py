from setuptools import find_packages, setup

package_name = 'side_arm_benchmark'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oliver',
    maintainer_email='ovancampen28@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'system_benchmark = side_arm_benchmark.system_benchmark:main',
            'vis_servo_benchmark = side_arm_benchmark.vis_servo_benchmark:main',
            'coord_collect = side_arm_benchmark.coord_collect.py',
        ],
    },
)
