#!/usr/bin/env python3

import os
import subprocess
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from ament_index_python.packages import get_package_share_directory

class BagPlayerNode(Node):
    def __init__(self):
        super().__init__('bag_player_node')
        self.tirette_sub = self.create_subscription(Bool, '/tirette_value', self.tirette_callback, 10)
        self.file_stm_sub = self.create_subscription(String, '/file_stm_choisi', self.file_stm_callback, 10)
        self.tirette_false_received = False
        self.file_stm_received = False
        self.bag_file_name = None
        self.bag_process = None
        self.ser = None
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Bag Player Node has started')

    def tirette_callback(self, msg):
        if not msg.data:
            self.tirette_false_received = True
            self.get_logger().info('Received tirette_value = false')

    def file_stm_callback(self, msg):
        if not self.file_stm_received:
            self.bag_file_name = msg.data
            self.file_stm_received = True
            self.get_logger().info(f'Received bag file name: {self.bag_file_name}')

    def timer_callback(self):
        if self.tirette_false_received and self.file_stm_received and self.bag_process is None:
            # Get bags directory using get_package_share_directory
            bags_dir = os.path.join(get_package_share_directory('serial_package'), 'bags')
            bag_path = os.path.join(bags_dir, self.bag_file_name)
            if not os.path.exists(bag_path):
                self.get_logger().error(f'Bag file not found: {bag_path}')
                return
            self.get_logger().info(f'Starting bag playback: {bag_path}')
            try:
                self.bag_process = subprocess.Popen(['ros2', 'bag', 'play', bag_path])
            except subprocess.SubprocessError as e:
                self.get_logger().error(f'Failed to start bag playback: {e}')
                return
            # Open serial port
            try:
                self.ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=1)
                self.get_logger().info('Serial port /dev/serial0 opened')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to open serial port: {e}')
                if self.bag_process:
                    self.bag_process.terminate()
                    self.bag_process.wait()
                    self.bag_process = None
                return
            # Create subscribers
            self.serial_out_sub = self.create_subscription(String, '/keyboard_commands', self.serial_out_callback, 10)
            self.stop_moteur_sub = self.create_subscription(Bool, '/stop_moteur', self.stop_moteur_callback, 10)

    def serial_out_callback(self, msg):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(msg.data.encode())
                self.get_logger().debug(f'Sent to serial: {msg.data}')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to write to serial port: {e}')

    def stop_moteur_callback(self, msg):
        if msg.data and self.bag_process:
            self.bag_process.terminate()
            self.bag_process.wait()
            self.get_logger().info('Bag playback stopped')
            self.bag_process = None
            if self.ser:
                self.ser.close()
                self.ser = None
                self.get_logger().info('Serial port closed')

    def destroy_node(self):
        if self.bag_process:
            self.bag_process.terminate()
            self.bag_process.wait()
            self.get_logger().info('Bag playback terminated')
        if self.ser:
            self.ser.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BagPlayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
