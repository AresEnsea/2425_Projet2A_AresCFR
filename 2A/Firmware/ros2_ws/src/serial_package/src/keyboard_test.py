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

        self.publisher = self.create_publisher(String, 'keyboard_commands', 10)

        self.stop_subscription = self.create_subscription(
            Bool,
            'stop_moteur',
            self.stop_moteur_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.timer_callback)  # Changed the timer to 1 second

        self.uart2_port = '/dev/serial0'
        self.baudrate = 115200
        self.timeout = 1

        self.motor_stopped = False
        self.toggle_states = {
            't': 0, 'y': 0, 'u': 0, 'i': 0, 'o': 0, 'p': 0,
            'f': 0, 'g': 0, 'h': 0, 'j': 0, 'k': 0, 'l': 0
        }
        self.cycle_states = {'c': 0, 'v': 0, 'b': 0, 'n': 0}
        self.active_command = "00000000000000000000000000"

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
        if self.motor_stopped:
            self.send_stop_command()
        else:
            # Toggle bits 11-22 every second
            for key in list(self.toggle_states.keys())[0:12]:  # Toggling 't' to 'l' (11th to 22nd)
                self.toggle_states[key] = 1 - self.toggle_states[key]  # Toggle between 0 and 1
                self.get_logger().info(f"Toggled {key} to: {self.toggle_states[key]}")
            
            # Update the active command with the new toggled states
            command = list(self.active_command)
            for idx, key in enumerate(self.toggle_states.keys()):
                command[10 + idx] = str(self.toggle_states[key])  # Position 11 to 22

            self.active_command = ''.join(command)

            # Send the updated command
            self.send_serial_message(self.active_command)
            self.publish_command(self.active_command)

    def send_stop_command(self):
        command = "00000000000000000000000000"
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
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def getch_non_blocking():
    if is_data():
        return sys.stdin.read(1)
    return None

def interpret_key_command(key, node):
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

    return ''.join(command)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode('keyboard_controller')

    print("Keyboard Teleop:")
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

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
