#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class SerialBagListener(Node):
    def __init__(self, node_name='serial_bag_listener'):
        super().__init__(node_name)

        # Create subscription to keyboard_commands topic
        self.subscription = self.create_subscription(
            String,
            'keyboard_commands',
            self.command_callback,
            10
        )

        # Create subscription to stop_moteur topic
        self.stop_subscription = self.create_subscription(
            Bool,
            'stop_moteur',
            self.stop_moteur_callback,
            10
        )

        # Serial port configuration
        self.uart2_port = '/dev/serial0'
        self.baudrate = 115200
        self.timeout = 1

        # Motor stop state
        self.motor_stopped = False

        self.get_logger().info('Serial bag listener node initialized')

    def stop_moteur_callback(self, msg):
        """Handle stop_moteur signal"""
        was_stopped = self.motor_stopped
        self.motor_stopped = msg.data

        if self.motor_stopped and not was_stopped:
            self.get_logger().warn("Motor stop signal received. Subsequent commands will be modified.")
        elif not self.motor_stopped and was_stopped:
            self.get_logger().info("Motor resume signal received.")

    def command_callback(self, msg):
        """Callback for receiving commands from keyboard_commands topic"""
        command = msg.data
        if self.motor_stopped:
            if len(command) >= 10:
                modified_command = "0000000000" + command[10:]
            else:
                modified_command = "0000000000"
            self.get_logger().info(f'Motor stopped: modifying command from "{command}" to "{modified_command}"')
            self.send_serial_message(modified_command)
        else:
            self.get_logger().info(f'Received command: {command}')
            self.send_serial_message(command)

    def send_serial_message(self, message):
        """Send the received command to the serial port"""
        try:
            with serial.Serial(self.uart2_port, self.baudrate, timeout=self.timeout) as ser:
                prefixed_message = f">{message}"
                self.get_logger().info(f"Sending serial message: {prefixed_message.strip()}")
                ser.write(prefixed_message.encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialBagListener('serial_bag_listener')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
