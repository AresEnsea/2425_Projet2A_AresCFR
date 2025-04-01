from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_package',
            executable='UART_Node',
            output='screen'),
    ])
