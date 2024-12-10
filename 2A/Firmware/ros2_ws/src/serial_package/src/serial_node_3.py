#!/usr/bin/env python3

#This code allows the robot tho move by receiving orders from the UART in the form of  numbers : "01000400", and at the same time he sends them to the topic "keyboard_commands" so we can register them with a rosbag

import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tty
import sys
import termios

class SimpleNode(Node):
    def __init__(self, node_name='keyboard_controller'):  # Ajout du paramètre node_name avec valeur par défaut
        super().__init__(node_name)  # Passage du node_name au constructeur parent
        
        # Créer le publisher
        self.publisher = self.create_publisher(
            String,
            'keyboard_commands',
            10
        )
        
        # Configuration du port série
        self.uart2_port = '/dev/serial0'
        self.baudrate = 115200
        self.timeout = 1
        
        self.get_logger().info('Keyboard controller node initialized')

    def send_serial_message(self, message):
        try:
            with serial.Serial(self.uart2_port, self.baudrate, timeout=self.timeout) as ser:
                self.get_logger().info(f"Sending serial message: {message.strip()}")
                ser.write(message.encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")

    def publish_command(self, command):
        msg = String()
        msg.data = command
        command = command.rstrip('\n\r\0') 
        self.publisher.publish(msg)
        self.get_logger().info(f'Published command: {command}')

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main(args=None):
    rclpy.init(args=args)
    
    # Création du nœud avec un nom spécifique
    node = SimpleNode('keyboard_controller')
    
    print("Keyboard Teleop:")
    print(" - 'z': forward")
    print(" - 's': backward") 
    print(" - 'q': left")
    print(" - 'd': right")
    print(" - 'x': exit")
    
    try:
        while rclpy.ok():
            key = getch()
            
            command = None
            if key == 'z':  # Avancer
                command = "04000400"
            elif key == 's':  # Reculer
                command = "00000000"
            elif key == 'q':  # Gauche
                command = "04000100"
            elif key == 'd':  # Droite
                command = "01000400"
            elif key == 'x':  # Quitter
                break
                
            if command:
                # Envoyer sur le port série
                node.send_serial_message(f"{command}\n\r\0")
                # Publier sur le topic ROS2
                node.publish_command(command)
            
            rclpy.spin_once(node, timeout_sec=0.1)
    
    except KeyboardInterrupt:
        pass
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
