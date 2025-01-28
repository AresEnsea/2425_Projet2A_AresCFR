#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # Define a QoS profile with RELIABLE reliability policy
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Ensure reliability
            depth=10  # Depth specifies the buffer size for messages
        )
        
        # Create the subscription with the defined QoS profile
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile
        )
        
        # Publisher for stop_moteur
        self.stop_publisher = self.create_publisher(Bool, 'stop_moteur', 10)

    def scan_callback(self, msg):
        # Filter valid ranges (greater than 0 and not infinity)
        valid_ranges = [r for r in msg.ranges if r > 0 and r != float('inf')]
        
        # Count obstacles that are closer than 0.4m
        close_obstacles = [r for r in valid_ranges if r < 0.4]
        
        # Publish True if there are 10 or more obstacles close, False otherwise
        stop_msg = Bool()
        stop_msg.data = len(close_obstacles) >= 10
        self.stop_publisher.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
