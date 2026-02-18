#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin
import numpy as np
import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

class Controller:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point # reference (desired value)
        self.previous_error = 0

    def update(self, current_value, theta_error=False):
        # calculate P_term and D_term
        error = self.set_point - current_value
        if theta_error:
            while error > pi:
                error -= 2*pi
            while error < -pi:
                error += 2*pi
        P_term = self.Kp * error #Kp e(t)
        D_term = self.Kd * (error - self.previous_error) #Kd(e(t)-e(t-dt))
        self.previous_error = error
        return P_term + D_term

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0

    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D

class Turtlebot():

    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        # reset odometry to zero
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()
        # subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')

    def run(self):
        # add your code here to adjust your movement based on 2D pose feedback
        # Use the Controller
        #set goal positions
        goalpos = [(4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)]
        cont = Controller(0.9,0.1)
        for gx, gy in goalpos:
            while not rospy.is_shutdown():
                #get pose from Pose2D
                x, y, th = self.pose.x, self.pose.y, self.pose.theta
                #get dist
                disX = gx-x
                disY = gy-y
                dist = sqrt(disX**2 + disY**2)
                #check if at goal
                dist_threshold = 0.05
                #adding thing for last waypoint stopping within 0.1 of it.
                if (gx == 0.0) and (gy == 0.0):
                    dist_threshold = 0.01
                if dist < dist_threshold:
                    self.vel_pub.publish(Twist()) #stop
                    self.rate.sleep()
                    break
                #Now we do PID stuff
                #calculate desired theta towards goal point
                gth = atan2(disY, disX)
                cont.setPoint(gth)
                #PID output
                w = cont.update(th, True)
                
                #prevent glitches with max angular vel
                if w > 2.0:
                    w = 2.0
                elif w < -2.0:
                    w = -2.0
                #set linear vel
                if (gth - th) > 0.5:
                    v = 0.0
                elif (gth - th) > 0.2 and dist > 0.2:
                    v = 0.1 #half max speed
                elif dist <= 0.2:
                    v = 0.01 #min speed
                else:
                    v = 0.22 #max linear speed
                
                vel = Twist() #vel msg
                vel.linear.x = v
                vel.linear.y = 0.0
                vel.linear.z = 0.0
                vel.angular.x = 0.0
                vel.angular.y = 0.0
                vel.angular.z = w
                self.vel_pub.publish(vel)
                self.rate.sleep()
        self.vel_pub.publish(Twist())
        
        

    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            # display (x, y, theta) on the terminal
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))

if __name__ == '__main__':
    whatever = Turtlebot()