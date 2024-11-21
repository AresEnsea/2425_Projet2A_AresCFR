from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2_serial_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('ros2_serial_msgs', 'ros2_serial_msgs.*')),
)
