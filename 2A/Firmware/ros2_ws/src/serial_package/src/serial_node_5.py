#!/usr/bin/env python3

#version  rosbag qui s'arrete avec le lidar 

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import serial  # For UART communication


class KeyboardCommandSubscriber(Node):
    def __init__(self):
        super().__init__('keyboard_command_subscriber')
        
        # Create a subscriber to the 'keyboard_commands' topic
        self.keyboard_subscription = self.create_subscription(
            String,
            'keyboard_commands',
            self.keyboard_command_callback,
            10
        )

        # Create a subscriber to the 'stop_moteur' topic
        self.stop_subscription = self.create_subscription(
            Bool,
            'stop_moteur',
            self.stop_moteur_callback,
            10
        )
        
        # Set up UART communication (ensure the port and baud rate are correct)
        self.ser = serial.Serial('/dev/serial0', 115200)  # Modify with your correct port and baud rate
        self.get_logger().info("UART communication established")

        # Track the stop state
        self.stop_motor = False

    def keyboard_command_callback(self, msg):
        # Check if stop_moteur is active
        if self.stop_motor:
            self.get_logger().warn("Motor stop is active. Ignoring keyboard command.")
            return

        # Send the received message (keyboard command) to the UART
        command = msg.data  # This is the string received from the topic
        self.get_logger().info(f"Received keyboard command: {command}")
        
        # Send the command to UART (make sure it's encoded as bytes)
        self.ser.write(command.encode())  # Sending as bytes

    def stop_moteur_callback(self, msg):
        # Update the motor stop state
        self.stop_motor = msg.data
        if self.stop_motor:
            self.get_logger().info("Received stop signal from stop_moteur topic. Stopping motors.")
            # Optionally send a specific stop command to the UART
            self.ser.write("STOP".encode())  # Replace "STOP" with your actual stop command
        else:
            self.get_logger().info("Received resume signal from stop_moteur topic. Motors enabled.")

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
f
