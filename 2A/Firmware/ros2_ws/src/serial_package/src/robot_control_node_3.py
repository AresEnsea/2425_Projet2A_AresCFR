#!/usr/bin/env python3
import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import tty
import sys
import termios
import threading

class SimpleNode(Node):
    def __init__(self, node_name='keyboard_controller'):
        super().__init__(node_name)

        self.publisher = self.create_publisher(
            String,
            'keyboard_commands',
            10
        )

        self.stop_subscription = self.create_subscription(
            Bool,
            'stop_moteur',
            self.stop_moteur_callback,
            10
        )

        self.uart2_port = '/dev/serial0'
        self.baudrate = 115200
        self.timeout = 1

        self.motor_stopped = False
        self.toggle_state = 0
        self.last_command = "00000000"

        self.current_command = None
        self.command_lock = threading.Lock()

        self.get_logger().info('Keyboard controller node initialized')

    def stop_moteur_callback(self, msg):
        self.motor_stopped = msg.data
        if self.motor_stopped:
            self.get_logger().warn("Motor stop signal received. Sending stop command.")
            self.send_stop_command()
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

    def send_stop_command(self):
        stop_command = f"00000000{self.toggle_state}"
        self.send_serial_message(f"{stop_command}\n\r\0")
        self.publish_command(stop_command)

    def keyboard_listener(self):
        while rclpy.ok():
            key = getch()
            command = None
            if key == 'z':
                command = f"06000600{self.toggle_state}"
            elif key == 's':
                command = f"00000000{self.toggle_state}"
            elif key == 'q':
                command = f"04000100{self.toggle_state}"
            elif key == 'd':
                command = f"01000400{self.toggle_state}"
            elif key == 'p':
                self.toggle_state = 1 - self.toggle_state
                self.last_command = self.last_command[:-1] + str(self.toggle_state)
                command = self.last_command
            elif key == 'x':
                rclpy.shutdown()
                break

            if command:
                with self.command_lock:
                    self.current_command = command

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

    keyboard_thread = threading.Thread(target=node.keyboard_listener)
    keyboard_thread.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.motor_stopped:
                node.send_stop_command()
            else:
                with node.command_lock:
                    if node.current_command:
                        node.send_serial_message(f"{node.current_command}\n\r\0")
                        node.publish_command(node.current_command)
                        node.last_command = node.current_command
                        node.current_command = None
            time.sleep(0.2)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()
        keyboard_thread.join()

if __name__ == '__main__':
    main()
