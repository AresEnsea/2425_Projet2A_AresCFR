#!/usr/bin/env python3

#This code is supposed to receive the command from the topic "keyboard_commands" and resend them to the UART (while adding the necessary character)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial  # For UART communication

class KeyboardCommandSubscriber(Node):
    def __init__(self):
        super().__init__('keyboard_command_subscriber')
        
        # Create a subscriber to the 'keyboard_commands' topic
        self.subscription = self.create_subscription(
            String,
            'keyboard_commands',
            self.keyboard_command_callback,
            10
        )
        
        # Set up UART communication (ensure the port and baud rate are correct)
        self.ser = serial.Serial('/dev/serial0', 115200)  # Modify with your correct port and baud rate
        self.get_logger().info("UART communication established")

    def keyboard_command_callback(self, msg):
        # Send the received message (keyboard command) to the UART
        command = msg.data  # This is the string received from the topic
        command_for_UART = command
        self.get_logger().info(f"Received keyboard command: {command}")
        
        # Send the command to UART (make sure it's encoded as bytes)
        self.ser.write(command_for_UART.encode())  # Sending as bytes

def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    node = KeyboardCommandSubscriber()

    # Spin the node to keep it active and processing incoming messages
    rclpy.spin(node)

    # Close UART connection on shutdown
    node.ser.close()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
