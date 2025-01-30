#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package paths
    serial_package_path = get_package_share_directory('serial_package')
    ydlidar_package_path = get_package_share_directory('ydlidar_ros2_driver')
    bag_path = os.path.expanduser('~/ros2_ws/src/serial_package/bags/rosbag2_2025_01_28-15_48_25/rosbag2_2025_01_28-15_48_25_0.db3')

    # YDLidar Configuration
    parameter_file = LaunchConfiguration('params_file')
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(ydlidar_package_path, 'params', 'X4.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    # YDLidar Node
    ydlidar_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        namespace=''
    )

    # Static Transform Publisher for Lidar
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame'],
        output='screen'
    )

    # Safety Node
    safety_node = Node(
        package='ydlidar_ros2_driver',
        executable='safety_node.py',
        output='screen'
    )

    # Serial Node
    serial_node = Node(
        package='serial_package',
        executable='serial_node_5.py',
        output='screen'
    )

    # ROS 2 Bag Play
    rosbag_play = ExecuteProcess(
	cmd=['ros2', 'bag', 'play', bag_path],
        output='screen'
    )

    return LaunchDescription([
        params_declare,
        ydlidar_node,
        tf2_node,
        safety_node,
        serial_node,
        rosbag_play
    ])
