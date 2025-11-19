from setuptools import find_packages
from setuptools import setup

setup(
    name='widowx_custom_msgs',
    version='0.0.1',
    packages=find_packages(
        include=('widowx_custom_msgs', 'widowx_custom_msgs.*')),
)
