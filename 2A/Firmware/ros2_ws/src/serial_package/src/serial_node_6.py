#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import serial
import subprocess

class KeyboardCommandSubscriber(Node):
    def __init__(self):
        super().__init__('keyboard_command_subscriber')
        
        self.keyboard_subscription = self.create_subscription(
            String,
            'keyboard_commands',
            self.keyboard_command_callback,
            10
        )
        
        self.stop_subscription = self.create_subscription(
            Bool,
            'stop_moteur',
            self.stop_moteur_callback,
            10
        )
        
        self.ser = serial.Serial('/dev/serial0', 115200)
        self.get_logger().info("UART communication established")
        self.stop_motor = False

    def keyboard_command_callback(self, msg):
        if self.stop_motor:
            self.get_logger().warn("Motor stop is active. Ignoring keyboard command.")
            return
        
        command = msg.data
        self.get_logger().info(f"Received keyboard command: {command}")
        self.ser.write(command.encode())

    def stop_moteur_callback(self, msg):
        self.stop_motor = msg.data
        if self.stop_motor:
            self.get_logger().info("Received stop signal from stop_moteur topic. Stopping motors and pausing rosbag.")
            self.ser.write("STOP".encode())
            try:
                # Pause rosbag
                subprocess.run(['ros2', 'bag', 'pause'])
            except Exception as e:
                self.get_logger().error(f"Failed to pause rosbag: {e}")
        else:
            self.get_logger().info("Received resume signal from stop_moteur topic. Motors enabled and resuming rosbag.")
            try:
                # Resume rosbag
                subprocess.run(['ros2', 'bag', 'resume'])
            except Exception as e:
                self.get_logger().error(f"Failed to resume rosbag: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCommandSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
