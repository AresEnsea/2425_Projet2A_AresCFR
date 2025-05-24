from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the share directory for ydlidar_ros2_driver to locate the params file
    ydlidar_share_dir = get_package_share_directory('ydlidar_ros2_driver')
    params_file = os.path.join(ydlidar_share_dir, 'params', 'X4.yaml')

    # Define the LifecycleNode for ydlidar_ros2_driver_node
    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        namespace='/',
        parameters=[params_file],
        output='screen'
    )

    # Define the static transform publisher for the laser frame
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame'],
        output='screen'
    )

    return LaunchDescription([
        Node(
            package='serial_package',
            executable='tirette',
            name='tirette',
            output='screen'
        ),
        Node(
            package='serial_package',
            executable='bag_player_node',
            name='bag_player_node',
            output='screen'
        ),
        driver_node,  # Directly include the YDLidar node
        tf2_node,     # Directly include the static transform publisher
        Node(
            package='serial_package',
            executable='bag_listener',
            name='bag_listener',
            output='screen'
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=driver_node,
                on_exit=[Shutdown()]  # Shutdown the entire launch when driver_node exits
            )
        )
    ])
