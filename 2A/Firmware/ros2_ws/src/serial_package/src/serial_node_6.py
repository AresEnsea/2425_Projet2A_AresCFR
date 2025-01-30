#version finale avec un fichier rosbag qui s'arrete avec le lidar et qui reprend derriere
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import serial
import subprocess
import signal
import os

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
        self.rosbag_process = None
        self.rosbag_path = os.path.expanduser('~/ros2_ws/src/serial_package/bags/rosbag2_2025_01_28-14_10_13/')
        
    def start_rosbag_play(self):
        if not self.rosbag_process:
            try:
                if not os.path.exists(self.rosbag_path):
                    self.get_logger().error(f"Rosbag path does not exist: {self.rosbag_path}")
                    return
                
                self.rosbag_process = subprocess.Popen(
                    ['ros2', 'bag', 'play', self.rosbag_path],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                self.get_logger().info(f"Started rosbag playback from {self.rosbag_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to start rosbag: {e}")

    def stop_rosbag_play(self):
        if self.rosbag_process:
            try:
                self.rosbag_process.send_signal(signal.SIGINT)
                try:
                    self.rosbag_process.wait(timeout=5)  # Attendre 5 secondes max
                except subprocess.TimeoutExpired:
                    self.rosbag_process.kill()  # Force kill si SIGINT ne fonctionne pas
                    self.rosbag_process.wait()
                
                self.rosbag_process = None
                self.get_logger().info("Stopped rosbag playback")
            except Exception as e:
                self.get_logger().error(f"Failed to stop rosbag: {e}")

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
            self.stop_rosbag_play()
        else:
            self.get_logger().info("Received resume signal from stop_moteur topic. Motors enabled and resuming rosbag.")
            self.start_rosbag_play()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCommandSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        if node.rosbag_process:
            node.stop_rosbag_play()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
