#!/usr/bin/env python3
import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool  # Added Bool message type
import tty
import sys
import termios

class SimpleNode(Node):
    def __init__(self, node_name='keyboard_controller'):
        super().__init__(node_name)
        
        # Keyboard commands publisher (unchanged)
        self.publisher = self.create_publisher(
            String,
            'keyboard_commands',
            10
        )
        
        # New subscription to stop_moteur topic
        self.stop_subscription = self.create_subscription(
            Bool,  # Boolean message type
            'stop_moteur',  # Topic name
            self.stop_moteur_callback,  # Callback function
            10  # Queue size
        )
        
        self.uart2_port = '/dev/serial0'
        self.baudrate = 115200
        self.timeout = 1
        
        # New attribute to track motor stop state
        self.motor_stopped = False
        
        # Toggle state remains the same
        self.toggle_state = 0
        
        self.get_logger().info('Keyboard controller node initialized')
    
    # New callback to handle stop_moteur messages
    def stop_moteur_callback(self, msg):
        # Update motor stop state based on received Boolean
        self.motor_stopped = msg.data
        
        if self.motor_stopped:
            self.get_logger().warn("Motor stop signal received. Disabling keyboard control.")
        else:
            self.get_logger().info("Motor resume signal received. Keyboard control re-enabled.")
    
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
    
    node = SimpleNode('keyboard_controller')
    
    print("Keyboard Teleop:")
    print(" - 'z': forward")
    print(" - 's': backward") 
    print(" - 'q': left")
    print(" - 'd': right")
    print(" - 'p': toggle 9th character")
    print(" - 'x': exit")
    
    try:
        while rclpy.ok():
            # Check if motors are stopped before processing keyboard input
            if node.motor_stopped:
                # Optional: Add a small delay to prevent high CPU usage
                time.sleep(0.1)
                rclpy.spin_once(node, timeout_sec=0.1)
                continue
            
            key = getch()
            
            command = None
            if key == 'z':  # Avancer
                command = f"06000600{node.toggle_state}"
            elif key == 's':  # Reculer
                command = f"00000000{node.toggle_state}"
            elif key == 'q':  # Gauche
                command = f"04000100{node.toggle_state}"
            elif key == 'd':  # Droite
                command = f"01000400{node.toggle_state}"
            elif key == 'p':  # Toggle 9th character
                node.toggle_state = 1 - node.toggle_state
                continue
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
