#!/usr/bin/env python3

import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tty
import sys
import termios
import select

class SimpleNode(Node):
    def __init__(self, node_name='keyboard_controller'):
        super().__init__(node_name)

        self.publisher = self.create_publisher(String, 'keyboard_commands', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.uart2_port = '/dev/serial0'
        self.baudrate = 115200
        self.timeout = 1

        self.toggle_states = {
            't': 0, 'y': 0, 'u': 0, 'i': 0, 'o': 0, 'p': 0,
            'f': 0, 'g': 0, 'h': 0, 'j': 0, 'k': 0, 'l': 0
        }
        self.cycle_states = {'c': 0, 'v': 0, 'b': 0, 'n': 0}
        self.active_command = "00000000000000000000000000"

        self.get_logger().info('Keyboard controller node initialized')

    def timer_callback(self):
        pass

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
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def getch_non_blocking():
    if is_data():
        return sys.stdin.read(1)
    return None

def interpret_key_command(key, node):
    # Toggle keys and their corresponding positions (11-22)
    toggle_keys = {'t': 10, 'y': 11, 'u': 12, 'i': 13, 'o': 14, 'p': 15,
                   'f': 16, 'g': 17, 'h': 18, 'j': 19, 'k': 20, 'l': 21}
    # Cycle keys and their corresponding positions (23-26)
    cycle_keys = {'c': 22, 'v': 23, 'b': 24, 'n': 25}
    
    # Initialize command with current state
    command = list(node.active_command)

    # Handle original commands (only affecting first 10 characters)
    if key == 'z':
        command[0:10] = list("1060010600")
    elif key == 's':
        command[0:10] = list("0060000600")
    elif key == 'a':
        command[0:10] = list("1040010100")
    elif key == 'w':
        command[0:10] = list("0040000100")
    elif key == 'e':
        command[0:10] = list("1010010400")
    elif key == 'x':
        command[0:10] = list("0010000400")
    elif key == 'q':
        command[0:10] = list("1040000400")
    elif key == 'd':
        command[0:10] = list("0040010400")
    elif key == 'r':
        node.toggle_states = {k: 0 for k in node.toggle_states}
        node.cycle_states = {k: 0 for k in node.cycle_states}
        return "00000000000000000000000000"

    # Handle toggle keys (bits 11-22)
    if key in toggle_keys:
        node.toggle_states[key] = 1 - node.toggle_states[key]
        node.get_logger().info(f"Toggled {key} to: {node.toggle_states[key]}")
        command[toggle_keys[key]] = str(node.toggle_states[key])

    # Handle cycle keys (bits 23-26)
    if key in cycle_keys:
        node.cycle_states[key] = (node.cycle_states[key] + 1) % 3
        node.get_logger().info(f"Cycled {key} to: {node.cycle_states[key]}")
        command[cycle_keys[key]] = str(node.cycle_states[key])

    return ''.join(command)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode('keyboard_controller')

    print("Keyboard Teleop:")
    print(" - 'z': forward")
    print(" - 's': backward")
    print(" - 'a': left")
    print(" - 'w': right")
    print(" - 'e': diagonal forward-left")
    print(" - 'x': diagonal forward-right")
    print(" - 'q': diagonal backward-left")
    print(" - 'd': diagonal backward-right")
    print(" - 't,y,u,i,o,p,f,g,h,j,k,l': toggle bits 11-22")
    print(" - 'c,v,b,n': cycle bits 23-26 (0,1,2)")
    print(" - 'r': reset/stop")
    print(" - 'm': quit")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            key = getch_non_blocking()
            if key is not None:
                if key == 'm':
                    break

                command = interpret_key_command(key, node)

                if command:
                    node.send_serial_message(command)
                    node.publish_command(command)
                    node.active_command = command

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
