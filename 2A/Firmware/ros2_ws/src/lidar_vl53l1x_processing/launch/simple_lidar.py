import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='lidar_vl53l1x_processing',
            executable='lidar_ensea',
            name='simple_lidar'
        )
    ])