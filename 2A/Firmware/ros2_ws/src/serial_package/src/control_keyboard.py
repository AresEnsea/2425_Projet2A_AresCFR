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

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.uart2_port = '/dev/serial0'
        self.baudrate = 115200
        self.timeout = 1

        self.motor_stopped = False
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

    def send_stop_command(self):
        command = "00000000000000000000000000"
        self.send_serial_message(command)
        self.publish_command(command)

    def send_serial_message(self, message):
        try:
            with serial.Serial(self.uart2_port, self.baudrate, timeout=self.timeout) as ser:
                prefixed_message = f">{message}"
                self.get_logger().info(f"Sending serial message: {prefixed_message.strip()}")
                ser.write(prefixed_message.encode())
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
    command = list(node.active_command)
    if key == 'z':
        command[0:10] = list("1060010600")
    elif key == 's':
        command[0:10] = list("0060000600")
    elif key == 'q':
        command[0:10] = list("1040000400")
    elif key == 'd':
        command[0:10] = list("0040010400")
    elif key == 'r':
        return "00000000000000000000000000"
    elif key == 'u':
        command[10] = '1' if command[10] == '0' else '0'
    elif key == 'i':
        command[11] = '1' if command[11] == '0' else '0'
    elif key == 'o':
        command[12] = '1' if command[12] == '0' else '0'
    elif key == 'j':
        command[13] = '1' if command[13] == '0' else '0'
    elif key == 'k':
        command[14] = '1' if command[14] == '0' else '0'
    elif key == 'l':
        command[15] = '1' if command[15] == '0' else '0'
    elif key == 'f':
        for i in range(16, 22):
            command[i] = '1' if command[i] == '0' else '0'
    elif key == 'g':
        command[22] = '0'
    elif key == 'b':
        command[22] = '1'
    elif key == 't':
        command[22] = '2'
    elif key == 'h':
        command[23] = '0'
    elif key == 'n':
        command[23] = '1'
    elif key == 'y':
        command[23] = '2'
    else:
        return node.active_command
    return ''.join(command)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode('keyboard_controller')

    print("Keyboard Teleop:")
    print(" - 'z': forward")
    print(" - 's': backward")
    print(" - 'q': diagonal backward-left")
    print(" - 'd': diagonal backward-right")
    print(" - 'r': reset/stop")
    print(" - 'u': toggle character 11")
    print(" - 'i': toggle character 12")
    print(" - 'o': toggle character 13")
    print(" - 'j': toggle character 14")
    print(" - 'k': toggle character 15")
    print(" - 'l': toggle character 16")
    print(" - 'f': toggle characters 17 to 22")
    print(" - 'g': set character 23 to 0")
    print(" - 't': set character 23 to 1")
    print(" - 'b': set character 23 to 2")
    print(" - 'h': set character 24 to 0")
    print(" - 'y': set character 24 to 1")
    print(" - 'n': set character 24 to 2")
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
                    if not node.motor_stopped:
                        node.send_serial_message(command)
                        node.publish_command(command)
                        node.active_command = command
                    else:
                        node.get_logger().info("Motor stopped: command ignored.")

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
