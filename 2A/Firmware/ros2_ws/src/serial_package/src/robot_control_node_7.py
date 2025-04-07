#!/usr/bin/env python3
import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import tty
import sys
import termios
import select

class SimpleNode(Node):
    def __init__(self, node_name='keyboard_controller'):
        super().__init__(node_name)

        # Publisher pour les commandes clavier
        self.publisher = self.create_publisher(
            String,
            'keyboard_commands',
            10
        )

        # Subscription au topic stop_moteur
        self.stop_subscription = self.create_subscription(
            Bool,
            'stop_moteur',
            self.stop_moteur_callback,
            10
        )

        # Timer pour envoyer des commandes d'arrêt répétées si nécessaire
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.uart2_port = '/dev/serial0'
        self.baudrate = 115200
        self.timeout = 1

        # État d'arrêt des moteurs
        self.motor_stopped = False

        # États des broches
        self.pin_1 = 0  # Première broche toggle
        self.pin_2 = 0  # Deuxième broche toggle
        self.last_pin = 0  # Dernière broche, peut être 0, 1 ou 2

        # Stockage de la dernière commande active
        self.active_command = "0000000000000"  # Par défaut : arrêt

        self.get_logger().info('Keyboard controller node initialized')

    def stop_moteur_callback(self, msg):
        was_stopped = self.motor_stopped
        self.motor_stopped = msg.data
        
        if self.motor_stopped and not was_stopped:
            self.get_logger().warn("Motor stop signal received. Immediately sending stop command.")
            self.send_stop_command()
        elif not self.motor_stopped and was_stopped:
            self.get_logger().info("Motor resume signal received. Sending active command.")
            self.send_serial_message(self.active_command)
            self.publish_command(self.active_command)

    def timer_callback(self):
        # Envoie la commande d'arrêt à 10Hz si les moteurs sont arrêtés
        if self.motor_stopped:
            self.send_stop_command()

    def send_stop_command(self):
        # Envoie la commande d'arrêt (tout à zéro)
        command = "0000000000000"
        self.send_serial_message(command)
        self.publish_command(command)

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
        self.publisher.publish(msg)
        self.get_logger().info(f'Published command: {command}')

def is_data():
    # Vérifie s'il y a une entrée clavier prête
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def getch_non_blocking(node):
    # Version non-bloquante de getch avec logique des commandes
    if is_data():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if key == 'z':  # Avancer (forward)
            return f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
        elif key == 's':  # Reculer (backward)
            return f"0060000600{node.pin_1}{node.pin_2}{node.last_pin}"
        elif key == 'i':  # Toggle première broche
            node.pin_1 = 1 - node.pin_1
            node.get_logger().info(f"Toggled first pin: {node.pin_1}")
            return f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
        elif key == 'r':  # Arrêt
            return f"0000000000{node.pin_1}{node.pin_2}{node.last_pin}"
        elif key == 'a':  # Arrière gauche
            return f"1040010100{node.pin_1}{node.pin_2}{node.last_pin}"
        elif key == 'w':  # Reculer à gauche
            return f"0040000100{node.pin_1}{node.pin_2}{node.last_pin}"
        elif key == 'e':  # Avant droite
            return f"1010010400{node.pin_1}{node.pin_2}{node.last_pin}"
        elif key == 'x':  # Arrière droite
            return f"0010000400{node.pin_1}{node.pin_2}{node.last_pin}"
        elif key == 'q':  # Gauche
            return f"1040000400{node.pin_1}{node.pin_2}{node.last_pin}"
        elif key == 'd':  # Droite
            return f"0040010400{node.pin_1}{node.pin_2}{node.last_pin}"
        elif key == 'o':  # Toggle deuxième broche
            node.pin_2 = 1 - node.pin_2
            node.get_logger().info(f"Toggled second pin: {node.pin_2}")
            return f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
        elif key == 'h':  # Dernière broche à 1
            node.last_pin = 1
            node.get_logger().info(f"Last pin set to: {node.last_pin}")
            return f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
        elif key == 'b':  # Dernière broche à 2
            node.last_pin = 2
            node.get_logger().info(f"Last pin set to: {node.last_pin}")
            return f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
        elif key == 'g':  # Dernière broche à 0
            node.last_pin = 0
            node.get_logger().info(f"Last pin set to: {node.last_pin}")
            return f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
        elif key == 'v':  # Dernière broche à 3
            node.last_pin = 3
            node.get_logger().info(f"Last pin set to: {node.last_pin}")
            return f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
        elif key == 'n':  # Dernière broche à 4
            node.last_pin = 4
            node.get_logger().info(f"Last pin set to: {node.last_pin}")
            return f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
        elif key == 'j':  # Dernière broche à 5
            node.last_pin = 5
            node.get_logger().info(f"Last pin set to: {node.last_pin}")
            return f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
        elif key == 'c':  # Quitter
            return None
    return None

def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleNode('keyboard_controller')
    
    print("Keyboard Teleop:")
    print(" - 'z': forward")
    print(" - 's': backward")
    print(" - 'i': toggle first pin")
    print(" - 'o': toggle second pin")
    print(" - 'h': set last pin to 1")
    print(" - 'b': set last pin to 2")
    print(" - 'g': set last pin to 0")
    print(" - 'c': quit")
    
    # Configuration du mode non-bloquant pour stdin
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(fd)
    
    try:
        while rclpy.ok():
            # Traite les callbacks ROS (timer et subscription stop_moteur)
            rclpy.spin_once(node, timeout_sec=0.01)
            
            # Traite les entrées clavier uniquement si les moteurs ne sont pas arrêtés
            if not node.motor_stopped:
                command = getch_non_blocking(node)
                
                if command:
                    if command == "0000000000000" and node.motor_stopped:
                        node.send_stop_command()
                    else:
                        # Envoie via le port série
                        node.send_serial_message(f"{command}")
                        # Publie sur le topic ROS2
                        node.publish_command(command)
                        # Met à jour la commande active
                        node.active_command = command
            # Évite de surcharger le CPU
            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    
    finally:
        # Restaure les paramètres du terminal
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
