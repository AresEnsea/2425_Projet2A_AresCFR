#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        
        self.declare_parameter('serial_port', '/dev/serial0')
        self.declare_parameter('baudrate', 115200)
        
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        self.motor_stopped = False
        
        self.subscription_cmd = self.create_subscription(
            String, 'keyboard_commands', self.keyboard_callback, 10)
        self.subscription_stop = self.create_subscription(
            Bool, 'stop_moteur', self.stop_moteur_callback, 10)
        
        self.get_logger().info('Serial bridge node initialized')

    def stop_moteur_callback(self, msg):
        self.motor_stopped = msg.data
        status = 'stopped' if self.motor_stopped else 'resumed'
        self.get_logger().info(f'Motor {status}')

    def keyboard_callback(self, msg):
        if self.motor_stopped:
            self.get_logger().warn('Command ignored: Motor is stopped')
            return
        
        self.send_serial_message(msg.data)

    def send_serial_message(self, message):
        try:
            with serial.Serial(self.serial_port, self.baudrate, timeout=1) as ser:
                ser.write(message.encode())
                self.get_logger().info(f'Sent: {message}')
        except serial.SerialException as e:
            self.get_logger().error(f'Error opening serial port: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
