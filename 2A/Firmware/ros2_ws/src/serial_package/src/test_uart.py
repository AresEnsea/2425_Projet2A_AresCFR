#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial

class UARTNode(Node):
    def __init__(self):
        super().__init__('uart_node')
        self.serial_port = serial.Serial(
            port='/dev/ttyAMA0',  # Adjust based on your Pi's UART port
            baudrate=115200,      # Match STM32's configured baudrate
            timeout=1
        )
        
        # Create publisher and subscriber for testing
        self.publisher = self.create_publisher(std_msgs.msg.String, 'uart_tx', 10)
        self.subscription = self.create_subscription(
            std_msgs.msg.String, 
            'uart_rx', 
            self.listener_callback, 
            10
        )

    def listener_callback(self, msg):
        # Send data over UART
        self.serial_port.write(msg.data.encode())

    def uart_read(self):
        # Read data from UART
        if self.serial_port.in_waiting > 0:
            data = self.serial_port.readline().decode().strip()
            msg = std_msgs.msg.String()
            msg.data = data
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    uart_node = UARTNode()
    
    # Periodic read
    while rclpy.ok():
        uart_node.uart_read()
        rclpy.spin_once(uart_node)

    uart_node.destroy_node()
    rclpy.shutdown()
