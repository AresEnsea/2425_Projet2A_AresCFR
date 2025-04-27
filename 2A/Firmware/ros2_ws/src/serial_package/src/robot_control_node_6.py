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

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.uart2_port = '/dev/serial0'
        self.baudrate = 115200
        self.timeout = 1

        self.motor_stopped = False
        self.pin_1 = 0
        self.pin_2 = 0
        self.last_pin = 0

        self.active_command = "0000000000000"

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
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def getch_non_blocking():
    if is_data():
        return sys.stdin.read(1)
    return None


def interpret_key_command(key, node):
    if key == 'z':
        return f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
    elif key == 's':
        return f"0060000600{node.pin_1}{node.pin_2}{node.last_pin}"
    elif key == 'i':
        node.pin_1 = 1 - node.pin_1
        node.get_logger().info(f"Toggled first pin: {node.pin_1}")
        return f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
    elif key == 'o':
        node.pin_2 = 1 - node.pin_2
        node.get_logger().info(f"Toggled second pin: {node.pin_2}")
        return f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
    elif key == 'r':
        return f"0000000000{node.pin_1}{node.pin_2}{node.last_pin}"
    elif key == 'a':
        return f"1040010100{node.pin_1}{node.pin_2}{node.last_pin}"
    elif key == 'w':
        return f"0040000100{node.pin_1}{node.pin_2}{node.last_pin}"
    elif key == 'e':
        return f"1010010400{node.pin_1}{node.pin_2}{node.last_pin}"
    elif key == 'x':
        return f"0010000400{node.pin_1}{node.pin_2}{node.last_pin}"
    elif key == 'q':
        return f"1040000400{node.pin_1}{node.pin_2}{node.last_pin}"
    elif key == 'd':
        return f"0040010400{node.pin_1}{node.pin_2}{node.last_pin}"
    elif key == 'h':
        node.last_pin = 1
        node.get_logger().info(f"Last pin set to: {node.last_pin}")
        return f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
    elif key == 'b':
        node.last_pin = 2
        node.get_logger().info(f"Last pin set to: {node.last_pin}")
        return f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
    elif key == 'g':
        node.last_pin = 0
        node.get_logger().info(f"Last pin set to: {node.last_pin}")
        return f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
    elif key == 'v':
        node.last_pin = 3
        node.get_logger().info(f"Last pin set to: {node.last_pin}")
        return f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
    elif key == 'n':
        node.last_pin = 4
        node.get_logger().info(f"Last pin set to: {node.last_pin}")
        return f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
    elif key == 'j':
        node.last_pin = 5
        node.get_logger().info(f"Last pin set to: {node.last_pin}")
        return f"1060010600{node.pin_1}{node.pin_2}{node.last_pin}"
    else:
        return None


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode('keyboard_controller')

    print("Keyboard Teleop:")
    print(" - 'z': forward")
    print(" - 's': backward")
    print(" - 'i': toggle first pin")
    print(" - 'o': toggle second pin")
    print(" - 'h', 'b', 'g', 'v', 'n', 'j': set last pin")
    print(" - 'r': stop")
    print(" - 'c': quit")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            key = getch_non_blocking()
            if key is not None:
                if key == 'c':
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
