from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # === Inclusion du launch file YDLIDAR ===
    ydlidar_launch_dir = get_package_share_directory('ydlidar_ros2_driver')
    ydlidar_launch_file = os.path.join(ydlidar_launch_dir, 'launch', 'ydlidar_launch.py')

    params_file = LaunchConfiguration('params_file')

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(ydlidar_launch_dir, 'params', 'X4.yaml'),
        description='Chemin vers le fichier YAML des paramètres du LIDAR'
    )

    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ydlidar_launch_file),
        launch_arguments={'params_file': params_file}.items()
    )

    # === Noeud UART_Node ===
    uart_node = Node(
        package='serial_package',
        executable='UART_Node',
        name='UART_Node',
        output='screen'
    )

    # === Noeud OdometryNode ===
    odom_node = Node(
        package='serial_package',
        executable='OdometryNode',
        name='OdometryNode',
        output='screen'
    )

    return LaunchDescription([
        declare_params,
        ydlidar_launch,
        uart_node,
        odom_node
    ])
