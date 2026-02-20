from setuptools import setup
from glob import glob

package_name = 'yolo_detect_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        (
            'share/' + package_name + '/runs/yolo26m_hole2_100/weights',
            ['runs/yolo26m_hole2_100/weights/best.pt'],
        ),
    ],
    install_requires=[
        'setuptools',
        'ultralytics>=8.2.0',
        'opencv-python>=4.7.0',
        'numpy>=1.23.0',
    ],
    zip_safe=False,
    maintainer='zheren',
    maintainer_email='zheren@example.com',
    description='ROS 2 wrapper for running YOLOv8 detection on a USB camera.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_detector = yolo_detect_ros.detector_node:main',
        ],
    },
)


