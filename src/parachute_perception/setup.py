from setuptools import find_packages, setup

package_name = 'parachute_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'loop_detector_node = parachute_perception.loop_detector_node:main',
            'target_selector_node = parachute_perception.target_selector_node:main',
            'hook_verification_node = parachute_perception.hook_verification_node:main',
            'loop_visualizer_node = parachute_perception.loop_visualizer_node:main',
            'test_loop_publisher_node = parachute_perception.test_loop_publisher_node:main',
            'loop_ground_truth_node = parachute_perception.loop_ground_truth_node:main',
            'detection_simulator_node = parachute_perception.detection_simulator_node:main',
            'camera_to_3d_node = parachute_perception.camera_to_3d_node:main',
        ],
    },
)
