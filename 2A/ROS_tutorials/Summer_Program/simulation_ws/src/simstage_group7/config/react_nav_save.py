#!/bin/python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ReactiveNavigation():
    def __init__(self):
        self.cmd_vel = Twist()
        
        self.laser_msg_0 = LaserScan()
        self.laser_msg_1 = LaserScan()
        self.laser_msg_2 = LaserScan()
        self.laser_msg_3 = LaserScan()

        self.robot_stopped = False
        self.obstacle_distance = 100

        self.active_robot = str(rospy.get_param("reactive_controller_py/active_robot"))
        self.available_robots = list(rospy.get_param("reactive_controller_py/available_robots"))

        self._cmd_topic = "/cmd_vel"                 
        self._laser_topic = "/base_scan"             

        self.rate = rospy.Rate(5)

    def laser_bc_0(self, callback):
        self.laser_msg_0 = callback
    def laser_bc_1(self, callback):
        self.laser_msg_1 = callback
    def laser_bc_2(self, callback):
        self.laser_msg_2 = callback
    def laser_bc_3(self, callback):
        self.laser_msg_3 = callback
    
    def calculate_command(self):
        for robot in self.available_robots:
            if robot == "robot_0":
                self.publisher_robot(robot, self.laser_msg_0)
            elif robot == "robot_1":
                self.publisher_robot(robot, self.laser_msg_1)
            elif robot == "robot_2":
                self.publisher_robot(robot, self.laser_msg_2)
            elif robot == "robot_3":
                self.publisher_robot(robot, self.laser_msg_3)

    def run(self):
        while not rospy.is_shutdown():
            self.calculate_command()
            self.rate.sleep()

    def publisher_robot(self, robot, laser_msg):
        cmd_vel = Twist()
        _cmd_topic ="/cmd_vel"     # robot +
        _laser_topic = "/base_scan"  # robot +

        if robot == "robot_0":
            rospy.Subscriber(_laser_topic, LaserScan, self.laser_bc_0, queue_size=1)
        elif robot == "robot_1":
            rospy.Subscriber(_laser_topic, LaserScan, self.laser_bc_1, queue_size=1)
        elif robot == "robot_2":
            rospy.Subscriber(_laser_topic, LaserScan, self.laser_bc_2, queue_size=1)
        elif robot == "robot_3":
            rospy.Subscriber(_laser_topic, LaserScan, self.laser_bc_3, queue_size=1)

        pub_CMD = rospy.Publisher(_cmd_topic, Twist, queue_size=1)

        if type(laser_msg.ranges) == tuple:
            # Calcul des indices pour 30 et 90 degrés
            angle_min = laser_msg.angle_min
            angle_increment = laser_msg.angle_increment
            
            index_30_deg = int((30 * (3.14159 / 180) - angle_min) / angle_increment)
            index_90_deg = int((90 * (3.14159 / 180) - angle_min) / angle_increment)
            
            distance_30_deg = laser_msg.ranges[index_30_deg]
            distance_90_deg = laser_msg.ranges[index_90_deg]

            rospy.loginfo(f"Distance à 30°: {distance_30_deg:.2f} m, à 90°: {distance_90_deg:.2f} m")

            obstacle_distance = min(laser_msg.ranges)

            if obstacle_distance > 0.25:
                cmd_vel.linear.x = 1.0
                cmd_vel.angular.z = 0.0
            else:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 1.0

        pub_CMD.publish(cmd_vel)
    
    
if __name__ == '__main__':
    rospy.init_node('reactive_navigation_py')
    controller = ReactiveNavigation()
    controller.run()

