#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from math import pi

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.run()


    def run(self):
        #set twist velocities for use
        forward_vel = Twist()
        turn_vel = Twist()
        forward_vel.linear.x = 0.2
        forward_vel.linear.y = 0.0
        forward_vel.linear.z = 0.0
        forward_vel.angular.x = 0.0
        forward_vel.angular.y = 0.0
        forward_vel.angular.z = 0.0
        turn_vel.linear.x = 0.0
        turn_vel.linear.y = 0.0
        turn_vel.linear.z = 0.0
        turn_vel.angular.x = 0.0
        turn_vel.angular.y = 0.0
        turn_vel.angular.z = pi / 32 #smaller rot speed was more consistent
        
        #it counts
        for_its = 200 #200 its at 0.2m/s to move 4m
        rot_its = 180 #160 its at pi/32 rad/s to rot 90 degrees additional 20 its for error correction micro shifts for each turn based on how much it drifts
        #while not rospy.is_shutdown():  # uncomment to use while loop
        for k in range(for_its):
            self.vel_pub.publish(forward_vel)
            self.rate.sleep()
        for k in range(rot_its+2):
            self.vel_pub.publish(turn_vel)
            self.rate.sleep()
        for k in range(for_its+1):
            self.vel_pub.publish(forward_vel)
            self.rate.sleep()
        for k in range(rot_its-5):
            self.vel_pub.publish(turn_vel)
            self.rate.sleep()
        for k in range(for_its):
            self.vel_pub.publish(forward_vel)
            self.rate.sleep()
        for k in range(rot_its+40):
            self.vel_pub.publish(turn_vel)
            self.rate.sleep()
        for k in range(for_its+18):
            self.vel_pub.publish(forward_vel)
            self.rate.sleep()
        #stop
        vel = Twist()
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = 0.0
        self.vel_pub.publish(vel)
        self.rate.sleep()
        """
        vel.linear.x = 0
        vel.angular.z = 0.05
        #while not rospy.is_shutdown():  # uncomment to use while loop
        for i in range(50):
            self.vel_pub.publish(vel)
            self.rate.sleep()
        """

if __name__ == '__main__':
    try:
        tb = Turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")