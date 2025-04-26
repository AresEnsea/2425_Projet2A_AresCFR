#!/usr/bin/env python3
import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import tty
import sys
import termios

class SimpleNode(Node):
    def __init__(self, node_name='keyboard_controller'):
        super().__init__(node_name)

        # Keyboard commands publisher
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

        # Pin states
        self.pin_1 = 0  # First toggle pin
        self.pin_2 = 0  # Second toggle pin
        self.last_pin = 0  # Last pin, can be 0, 1, or 2

        # Store the last full command sent
        self.last_command = "1060010600000"  # Default value, 13 characters long

        self.get_logger().info('Keyboard controller node initialized')

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
    print(" - 'i': toggle first pin")
    print(" - 'o': toggle second pin")
    print(" - 'h': set last pin to 1")
    print(" - 'b': set last pin to 2")
    print(" - 'g': set last pin to 0")
    print(" - 'c': quit")
    
    try:
        while rclpy.ok():
            # Check if motors are stopped before processing keyboard input
            if node.motor_stopped:
                # Optional: Add a small delay to prevent high CPU usage
                time.sleep(0.1)
                rclpy.spin_once(node, timeout_sec=0.1)
            key = getch()

            command = None
            if key == 'z':  # Avancer (forward)
                command = f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
            elif key == 's':  # Reculer (backward)
                command = f"0060000600{node.pin_1}{node.pin_2}{node.last_pin}"
            elif key == 'i':  # Toggle first pin
                node.pin_1 = 1 - node.pin_1
                node.get_logger().info(f"Toggled first pin: {node.pin_1}")
                command = f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"

            elif key == 'r':  # Stop command
                command = f"0000000000{node.pin_1}{node.pin_2}{node.last_pin}"
            elif key == 'a':  # Backward left
                command = f"1040010100{node.pin_1}{node.pin_2}{node.last_pin}"
            elif key == 'w':  # Back left
                command = f"0040000100{node.pin_1}{node.pin_2}{node.last_pin}"
            elif key == 'e':  # Forward right
                command = f"1010010400{node.pin_1}{node.pin_2}{node.last_pin}"
            elif key == 'x':  # Back right
                command = f"0010000400{node.pin_1}{node.pin_2}{node.last_pin}"
            elif key == 'q':  # Left
                command = f"1040000400{node.pin_1}{node.pin_2}{node.last_pin}"
            elif key == 'd':  # Right
                command = f"0040010400{node.pin_1}{node.pin_2}{node.last_pin}"

            elif key == 'o':  # Toggle second pin
                node.pin_2 = 1 - node.pin_2
                node.get_logger().info(f"Toggled second pin: {node.pin_2}")
                command = f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"

            elif key == 'h':  # Set last pin to 1
                node.last_pin = 1
                node.get_logger().info(f"Last pin set to: {node.last_pin}")
                command = f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"

            elif key == 'b':  # Set last pin to 2
                node.last_pin = 2
                node.get_logger().info(f"Last pin set to: {node.last_pin}")
                command = f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"

            elif key == 'g':  # Set last pin to 0
                node.last_pin = 0
                node.get_logger().info(f"Last pin set to: {node.last_pin}")
                command = f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
            
            elif key == 'v':  # Set last pin to 0
                node.last_pin = 3
                node.get_logger().info(f"Last pin set to: {node.last_pin}")
                command = f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"

            elif key == 'n':  # Set last pin to 0
                node.last_pin = 4
                node.get_logger().info(f"Last pin set to: {node.last_pin}")
                command = f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"

            elif key == 'j':  # Set last pin to 0
                node.last_pin = 5
                node.get_logger().info(f"Last pin set to: {node.last_pin}")
                command = f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"


            elif key == 'c':  # Quitter (exit)
                break

            if command:
                # Send over the serial port
                node.send_serial_message(f"{command}")
                # Publish on the ROS2 topic
                node.publish_command(command)
                node.last_command = command

            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
