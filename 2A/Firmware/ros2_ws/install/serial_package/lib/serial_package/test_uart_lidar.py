#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial

class UartReaderNode(Node):
    def __init__(self):
        super().__init__('uart_reader_node')
        self.get_logger().info("UART Reader Node has started")
        
        # Open the UART serial port (adjust to the correct one for your setup)
        self.ser = serial.Serial('/dev/serial0', 115200, timeout=1)  # Set your serial port and baud rate
        
        # Set a timer to read data from UART every 0.1 seconds
        self.timer = self.create_timer(0.1, self.read_uart)

    def read_uart(self):
        if self.ser.in_waiting > 0:
            data = self.ser.readline().strip()  # decode('utf-8').
            if data:
                self.get_logger().info(f"Received UART Data: {data}")
    
    def destroy_node(self):
        self.ser.close()  # Close the UART connection when shutting down
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UartReaderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
