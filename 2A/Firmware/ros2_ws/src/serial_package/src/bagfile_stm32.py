#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os

class FileHandler(Node):
    def __init__(self):
        super().__init__('file_handler')
        
        # Créer le subscriber pour écouter les commandes STM32
        self.subscription = self.create_subscription(
            String,
            'stm32_commands',
            self.command_callback,
            10
        )
        
        # Chemin vers le répertoire des fichiers
        self.file_path = os.path.expanduser('~/ros2_ws/src/serial_package/bags/')
        
        self.get_logger().info('File handler node started')

    def command_callback(self, msg):
        # Extraire la commande du message reçu
        command = msg.data.strip()
        
        # Vérifier si la commande n'est pas vide et correspond au format attendu
        if command and command.startswith('file') and command[4:].isdigit():
            file_number = int(command[4:])
            if 0 <= file_number <= 5:  # Modifié pour accepter jusqu'à file5
                # Construire le chemin complet du fichier bag
                bag_file = os.path.join(self.file_path, command, f"{command}_0.db3")
                
                # Vérifier si le fichier existe
                if os.path.exists(bag_file):
                    try:
                        # Exécuter le fichier bag avec ros2 bag play
                        subprocess.run(["ros2", "bag", "play", bag_file], check=True)
                        self.get_logger().info(f'Executed ROS bag file: {bag_file}')
                    except subprocess.CalledProcessError as e:
                        self.get_logger().error(f'Error executing ROS bag file: {str(e)}')
                else:
                    self.get_logger().warning(f'ROS bag file not found: {bag_file}')
            else:
                self.get_logger().warning(f'Invalid file number: {file_number}. Must be between 0 and 5.')

def main():
    rclpy.init()
    file_handler = FileHandler()
    
    try:
        rclpy.spin(file_handler)
    except KeyboardInterrupt:
        pass
    finally:
        file_handler.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
