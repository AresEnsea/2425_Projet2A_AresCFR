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
            # Toggle the 23rd and 24th characters (indices 22 and 23) between 0, 1, and 2
            command = list(self.active_command)
            current_v = int(command[22])  # 23rd character (v)
            current_b = int(command[23])  # 24th character (b)
            next_v = (current_v + 1) % 3
            next_b = (current_b + 1) % 3
            command[22] = str(next_v)
            command[23] = str(next_b)
            new_command = ''.join(command)
            self.send_serial_message(new_command)
            self.publish_command(new_command)
            self.active_command = new_command

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
                    if not node.motor_stopped:
                        node.send_serial_message(command)
                        node.publish_command(command)
                        node.active_command = command
                    else:
                        node.get_logger().info("Motor stopped: command ignored.")

            time.sleep(1)

    except KeyboardInterrupt:
        pass

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
