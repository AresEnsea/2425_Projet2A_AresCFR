#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

class SerialListener(Node):
    def __init__(self):
        super().__init__('serial_listener')
        
        # Configure the serial port
        self.serial_port = serial.Serial(
            port='/dev/serial0',  # Adjust port as needed
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        
        # Create a publisher for the commands
        self.publisher_ = self.create_publisher(String, 'stm32_commands', 10)
        
        # Start the serial reading thread
        self.thread = threading.Thread(target=self.read_serial)
        self.thread.daemon = True
        self.thread.start()
        
        self.get_logger().info('Serial listener node started')

    def read_serial(self):
        while rclpy.ok():
            if self.serial_port.in_waiting:
                try:
                    # Read line from serial
                    command = self.serial_port.readline().decode('utf-8').strip()
                    
                    # Create and publish the message
                    msg = String()
                    msg.data = command
                    self.publisher_.publish(msg)
                    
                    self.get_logger().info(f'Received command: {command}')
                    
                except Exception as e:
                    self.get_logger().error(f'Error reading serial: {str(e)}')

def main():
    rclpy.init()
    serial_listener = SerialListener()
    
    try:
        rclpy.spin(serial_listener)
    except KeyboardInterrupt:
        pass
    finally:
        serial_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
