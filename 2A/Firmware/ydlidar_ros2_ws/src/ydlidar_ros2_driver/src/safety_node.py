import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class SafetyNode(Node):
   def __init__(self):
       super().__init__('safety_node')
       self.subscription = self.create_subscription(
           LaserScan, '/scan', self.scan_callback, 10)
       self.stop_publisher = self.create_publisher(Bool, 'stop_moteur', 10)

   def scan_callback(self, msg):
       # Filtrer les valeurs valides 
       valid_ranges = [r for r in msg.ranges if r > 0 and r != float('inf')]
       
       # Compter les valeurs inférieures à 0.4
       close_obstacles = [r for r in valid_ranges if r < 0.4]
       
       # Publier True si 10 valeurs ou plus, False sinon
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
