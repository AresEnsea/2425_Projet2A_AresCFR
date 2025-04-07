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

        # Three toggle states bound to keys "i", "o", and "p"
        self.toggle_state_1 = 0  # Controlled by "i"
        self.toggle_state_2 = 0  # Controlled by "o"
        self.toggle_state_3 = 0  # Controlled by "p"

        # Store the last full command sent
        self.last_command = "00000000000"  # Default value, adjusted for 3 toggles

        self.get_logger().info('Keyboard controller node initialized')
    
    def stop_moteur_callback(self, msg):
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
    print(" - 'e': forward right")
    print(" - 'a': backward left")
    print(" - 'q': left")
    print(" - 'd': right")
    print(" - 'w': back left")
    print(" - 'x': back right")
    print(" - 'i': toggle pin 1")
    print(" - 'o': toggle pin 2")
    print(" - 'p': toggle pin 3")
    print(" - 'c': exit")
    
    try:
        while rclpy.ok():
            if node.motor_stopped:
                time.sleep(0.2)
                rclpy.spin_once(node, timeout_sec=0.1)
            key = getch()

            command = None
            if key == 'z':
                command = f"1060010600{node.toggle_state_1}{node.toggle_state_2}{node.toggle_state_3}"
            elif key == 's':
                command = f"0060000600{node.toggle_state_1}{node.toggle_state_2}{node.toggle_state_3}"
            elif key == 'r':
                command = f"0000000000{node.toggle_state_1}{node.toggle_state_2}{node.toggle_state_3}"
            elif key == 'a':
                command = f"1040010100{node.toggle_state_1}{node.toggle_state_2}{node.toggle_state_3}"
            elif key == 'w':
                command = f"0040000100{node.toggle_state_1}{node.toggle_state_2}{node.toggle_state_3}"
            elif key == 'e':
                command = f"1010010400{node.toggle_state_1}{node.toggle_state_2}{node.toggle_state_3}"
            elif key == 'x':
                command = f"0010000400{node.toggle_state_1}{node.toggle_state_2}{node.toggle_state_3}"
            elif key == 'q':
                command = f"1040000400{node.toggle_state_1}{node.toggle_state_2}{node.toggle_state_3}"
            elif key == 'd':
                command = f"0040010400{node.toggle_state_1}{node.toggle_state_2}{node.toggle_state_3}"
            
            elif key == 'i':
                node.toggle_state_1 = 1 - node.toggle_state_1
                node.last_command = node.last_command[:-3] + str(node.toggle_state_1) + node.last_command[-2:]
                command = node.last_command
            elif key == 'o':
                node.toggle_state_2 = 1 - node.toggle_state_2
                node.last_command = node.last_command[:-2] + str(node.toggle_state_2) + node.last_command[-1:]
                command = node.last_command
            elif key == 'p':
                node.toggle_state_3 = 1 - node.toggle_state_3
                node.last_command = node.last_command[:-1] + str(node.toggle_state_3)
                command = node.last_command
            elif key == 'c':
                break
            
            if command:
                node.send_serial_message(f"{command}")
                node.publish_command(command)
                node.last_command = command

            rclpy.spin_once(node, timeout_sec=0.2)
    
    except KeyboardInterrupt:
        pass
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
