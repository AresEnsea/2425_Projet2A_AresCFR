from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_package',
            executable='tirette',
            name='tirette_node',
            output='screen'
        ),
        Node(
            package='serial_package',
            executable='bag_player_node',
            name='bag_player_node',
            output='screen'
        ),
        Node(
            package='serial_package',
            executable='bag_listener',
            name='bag_listener_node',
            output='screen'
        ),
    ])
