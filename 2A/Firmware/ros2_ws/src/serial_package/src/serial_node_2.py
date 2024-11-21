#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time

class SimpleListener(Node):
    def __init__(self, port='/dev/ttyS0', baudrate=115200):
        super().__init__('simple_listener')
        self.serial_port = serial.Serial(port, baudrate, timeout=1)
        self.get_logger().info(f"Listening on {port} at {baudrate} baud")

    def run(self):
        while rclpy.ok():
            if self.serial_port.in_waiting > 0:
                received_data = self.serial_port.readline().decode('utf-8').strip()
                self.get_logger().info(f"Received: {received_data}")
            rclpy.spin_once(self)  # Process callbacks
            time.sleep(0.1)  # Sleep for a brief period

    def destroy_node(self):
        self.serial_port.close()  # Close the serial connection
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    listener_node = SimpleListener()

    try:
        listener_node.run()
    except Exception as e:
        listener_node.get_logger().error(f"Exception: {e}")
    finally:
        listener_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
