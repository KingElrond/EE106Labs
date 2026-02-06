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
        turn_vel = Twist()
        vel = Twist() #stop vel
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = 0.0
        turn_vel.linear.x = 0.0
        turn_vel.linear.y = 0.0
        turn_vel.linear.z = 0.0
        turn_vel.angular.x = 0.0
        turn_vel.angular.y = 0.0
        turn_vel.angular.z = 0.1 #smaller rot speed was more consistent used 0.1 instead of pi/32 because it had less variance when turning.
        
        #it counts
        #rot_its = 180 #160 its at pi/32 rad/s to rot 90 degrees additional 20 its for error correction micro shifts for each turn based on how much it drifts
        rot_its = 170 #trying this again using 0.1 since that is close to pi / 32 initially tried 157, but too small, upped by 5 until it did the 90 degree turn.
        #while not rospy.is_shutdown():  # uncomment to use while loop
        #forward
        self.GoForward4m()
        #turn
        for k in range(rot_its):
            self.vel_pub.publish(turn_vel)
            self.rate.sleep()
        #stop
        self.vel_pub.publish(vel)
        self.rate.sleep()
        #forward
        self.GoForward4m()
        #turn
        for k in range(rot_its):
            self.vel_pub.publish(turn_vel)
            self.rate.sleep()
        #stop
        self.vel_pub.publish(vel)
        self.rate.sleep()
        #forward
        self.GoForward4m()
        #turn
        for k in range(rot_its):
            self.vel_pub.publish(turn_vel)
            self.rate.sleep()
        #stop
        self.vel_pub.publish(vel)
        self.rate.sleep()
        #forward
        self.GoForward4m()
        #stop
        self.vel_pub.publish(vel)
        self.rate.sleep()
        
    def GoForward4m(self):
        vel = Twist() #set all vels to 0.0
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = 0.0
        #ok, we need to smoothly accelerate from 0.00 to 0.2 with step size of 0.01 for no jerkiness. also Decell too
        #math time, did the math, if I go each speed for 1 full second, I will travel 4m
        #accelerate loop
        for i in range(20):
            vel.linear.x = 0.01 + (0.01 * i)
            for k in range(10):
                self.vel_pub.publish(vel)
                self.rate.sleep()
        #deccelerate loop
        for i in range(20):
            vel.linear.x = 0.2 - (0.01 * (i+1))
            for k in range(10):
                self.vel_pub.publish(vel)
                self.rate.sleep()


if __name__ == '__main__':
    try:
        tb = Turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")