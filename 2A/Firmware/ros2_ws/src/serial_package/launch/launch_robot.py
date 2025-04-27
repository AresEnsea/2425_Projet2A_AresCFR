from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Chemin vers le package ydlidar_ros2_driver
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    
    # Déclaration du fichier de paramètres pour ydlidar
    parameter_file = LaunchConfiguration('params_file')
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'X4.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    # Nœud pour le driver ydlidar
    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        namespace=''
    )

    # Nœud pour la transformation statique
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame'],
        output='screen'
    )

    # Nœud pour safety_node.py
    safety_node = Node(
        package='ydlidar_ros2_driver',
        executable='safety_node.py',
        name='safety_node',
        output='screen',
        emulate_tty=True
    )

    # Nœud pour screen_reader.py
    screen_reader_node = Node(
        package='serial_package',
        executable='screen_reader.py',
        name='screen_reader_node',
        output='screen',
        emulate_tty=True
    )

    # Nœud pour bagfile_stm32.py
    bagfile_stm32_node = Node(
        package='serial_package',
        executable='bagfile_stm32.py',
        name='bagfile_stm32_node',
        output='screen',
        emulate_tty=True
    )

    # Nœud pour tirette_node.py
    tirette_node = Node(
        package='serial_package',
        executable='tirette_node.py',
        name='tirette_node',
        output='screen',
        emulate_tty=True
    )

    # Retourne la description du lancement avec tous les nœuds
    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_node,
        safety_node,
        screen_reader_node,
        bagfile_stm32_node,
        tirette_node
    ])
