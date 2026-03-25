from setuptools import find_packages, setup
import os
from glob import glob

package_name = "top_cam_loop_state"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MQP Team",
    maintainer_email="todo@wpi.edu",
    description="Top camera loop state detection with YOLO and ROS 2 publishing",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "loop_state_node = top_cam_loop_state.loop_state_node:main",
        ],
    },
)
