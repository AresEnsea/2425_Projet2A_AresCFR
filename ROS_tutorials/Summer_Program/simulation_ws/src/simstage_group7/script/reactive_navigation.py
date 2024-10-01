#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ReactiveNavigation:
    def __init__(self):
        self.mur = 0  
        self.cmd_vel = Twist()
        self.laser_msg_0 = LaserScan()

        self.robot_stopped = False
        self.obstacle_distance = 100

        self.active_robot = str(rospy.get_param("reactive_controller_py/active_robot"))
        self.available_robots = list(rospy.get_param("reactive_controller_py/available_robots"))

        self._cmd_topic = "cmd_vel"
        self._laser_topic = "base_scan"

        self.rate = rospy.Rate(5)
        
    def laser_bc_0(self, callback):
        self.laser_msg_0 = callback

    def calculate_command(self):
        for robot in self.available_robots:
            if robot == "robot_0":
                self.publisher_robot(robot, self.laser_msg_0)

    def run(self):
        while not rospy.is_shutdown():
            self.calculate_command()
            self.rate.sleep()

    def publisher_robot(self, robot, laser_msg):
        cmd_vel = Twist()
        _cmd_topic = "cmd_vel"
        _laser_topic = "base_scan"
        
        if robot == "robot_0":
            rospy.Subscriber(_laser_topic, LaserScan, self.laser_bc_0, queue_size=1)

        pub_CMD = rospy.Publisher(_cmd_topic, Twist, queue_size=1)

        if laser_msg.ranges:
            obstacle_distance = min(laser_msg.ranges)
            
            # Log pour déboguer l'état et la distance
            rospy.loginfo(f"mur: {self.mur}, obstacle_distance: {obstacle_distance}")
            
            if self.mur == 0:#on va chercher à se rapprocher d'un mur
                cmd_vel.linear.x = 0.3
                cmd_vel.angular.z = 0.0
                if 0.01 < obstacle_distance < 0.1:  # recul
                    cmd_vel.linear.x = -0.2
                    cmd_vel.angular.z = 0.0
                    self.mur = 1  # Change l'état de mur

            elif self.mur == 1: #le robot sera collé au mur et le suivra
                if 0.18 < obstacle_distance:  # Tourne à droite
                    cmd_vel.linear.x = 0.1
                    cmd_vel.angular.z = -0.7
                elif 0.01 < obstacle_distance < 0.1:  # recul
                    cmd_vel.linear.x = -0.2
                    cmd_vel.angular.z = 0.0
                else:  # gauche
                    cmd_vel.linear.x = 0.1
                    cmd_vel.angular.z = 0.5
            
            pub_CMD.publish(cmd_vel)

if __name__ == '__main__':
    rospy.init_node('reactive_navigation_py')
    controller = ReactiveNavigation()
    controller.run()
