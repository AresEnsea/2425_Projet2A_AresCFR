#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

class LidarVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_visualizer')

        # Set up QoS profile with Best Effort policy
        qos_profile = QoSProfile(
            depth=10,  # Buffer size for the subscriber
            reliability=ReliabilityPolicy.BEST_EFFORT  # Set the QoS reliability policy to best effort
        )

        # Subscribe to the /scan topic with the defined QoS
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Ensure this matches your published topic
            self.listener_callback,
            qos_profile  # Use the defined QoS profile here
        )
        self.subscription  # prevent unused variable warning

        # Create a plot for visualizing lidar data
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.ax.set_rticks([1, 2, 3, 4, 5, 10])  # Set radius ticks
        self.ax.set_rmax(10)  # Adjust to your lidar range max

        # Set up the plot
        self.scatter = self.ax.scatter([], [], s=5)

    def listener_callback(self, msg):
        # Convert the LaserScan data into polar coordinates for plotting
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)

        # Filter out invalid readings (e.g., inf or NaN)
        valid_indices = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        angles = angles[valid_indices]
        ranges = ranges[valid_indices]

        # Update the scatter plot
        self.scatter.set_offsets(np.column_stack((angles, ranges)))
        self.ax.set_rticks([1, 2, 3, 4, 5, 10])  # Update radius ticks based on data
        self.ax.set_rmax(np.max(ranges))  # Update max radius to reflect data range

        # Redraw the plot
        plt.draw()
        plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = LidarVisualizer()

    # Start matplotlib plotting loop
    plt.ion()  # Turn on interactive mode for real-time updates
    plt.show()

    rclpy.spin(node)

    # Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
