#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import threading
import time
import signal
import sys
from std_msgs.msg import String  # Importer le type de message pour le topic

class UartReaderNode(Node):
    def __init__(self):
        super().__init__('uart_reader_node')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.ser = None
        self.shutdown = False

        # Créer un éditeur pour le topic 'file_stm_choisi'
        self.publisher_ = self.create_publisher(String, 'file_stm_choisi', 10)
        self.get_logger().info("Publisher created for topic 'file_stm_choisi'")

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f"Opened serial port {self.port} at {self.baudrate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        self.thread = threading.Thread(target=self.read_serial, daemon=True)
        self.thread.start()

    def read_serial(self):
        while rclpy.ok() and not self.shutdown:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    data = self.ser.readline().decode('utf-8').strip()
                    self.get_logger().info(f"Received: {data}")
                    # Publier les données sur le topic 'file_stm_choisi'
                    msg = String()
                    msg.data = data
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published to 'file_stm_choisi': {data}")
            except (OSError, serial.SerialException) as e:
                self.get_logger().error(f"Serial error: {e}")
                time.sleep(1)
            time.sleep(0.01)

    def __del__(self):
        self.shutdown = True
        if self.ser and self.ser.is_open:
            if rclpy.ok():
                self.get_logger().info("Closing serial port")
            self.ser.close()

def signal_handler(sig, frame, node):
    node.get_logger().info("Received shutdown signal")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)

def main():
    rclpy.init()
    node = UartReaderNode()
    signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, node))
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
