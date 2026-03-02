#!/usr/bin/env python3

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

def euclid_dist(candidate, goal):
    #manhattan Distance
    return sqrt((abs(candidate[0] - goal[0]))**2 + (abs(candidate[1] - goal[1]))**2)

class Controller: #PID controller from lab 4
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
        
        self.prev_vx, self.prev_vy = 0.0, 0.0
        self.prev_wp = [0,0]

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')


    def run(self):
        waypoints = [[0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5],\
                      [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0],\
                      [1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0], [0, 0]]
        for i in range(len(waypoints)-1):
            self.move_to_point(waypoints[i], waypoints[i+1])


    def move_to_point(self, current_waypoint, next_waypoint):
        # generate polynomial trajectory and move to current_waypoint
        # next_waypoint is to help determine the velocity to pass current_waypoint
        lv_max = 0.22 #max linear speed
        av_max = 2.0 #max angular speed
        cont = Controller(0.9,0.1)
        gx, gy = current_waypoint
        nx, ny = next_waypoint
        #get pose from Pose2D
        x, y, th = self.pose.x, self.pose.y, self.pose.theta
        #get dist
        disX = gx-x
        disY = gy-y
        disNX = nx - gx
        disNY = ny - gy
        dist = euclid_dist((x,y),current_waypoint)
        distN = euclid_dist(current_waypoint, next_waypoint)
        
        gth = atan2(disY, disX)
        ngth = atan2((ny-gy), (nx-gx))
        nturn = ngth - gth
        while nturn > pi:
            nturn -= 2*pi
        while nturn < -pi:
            nturn += 2*pi
        
        turn_abs = abs(nturn)

        # map [0 .. pi/2] -> [1 .. 0] for adjusting turn end speeds.
        if turn_abs >= (pi/2):
            turn_scale = 0.0
        else:
            turn_scale = ((pi/2) - turn_abs) / (pi/2)
            
        #set end v for turn to next waypoint
        vx1 = 0.0
        vy1 = 0.0
        if distN > 0.01: #final waypoint edge case
            vx1 = (lv_max * turn_scale) * (disNX / distN)
            vy1 = (lv_max * turn_scale) * (disNY / distN)
        gEndVel = sqrt((vx1**2) + (vy1**2))

        #3rd order stuff guessing hard here.
        T = max(1.5, dist / 0.20)   # 0.20 m/s target speed
        dt = 0.1
        vx0, vy0 = self.prev_vx, self.prev_vy
        x0 = x
        y0 = y
        xT = gx
        yT = gy
        
        #poly time scale
        ax0, ax1, ax2, ax3 = self.polynomial_time_scaling_3rd_order(x0, vx0, xT, vx1, T)
        ay0, ay1, ay2, ay3 = self.polynomial_time_scaling_3rd_order(y0, vy0, yT, vy1, T)
        
        t = 0.0
        while not rospy.is_shutdown() and t <= T:
            #get pose from Pose2D
            x, y, th = self.pose.x, self.pose.y, self.pose.theta
            
            #use poly time scale thingy
            # polynomial moving target at time t
            x_des = ax0 + ax1*t + ax2*(t**2) + ax3*(t**3)
            y_des = ay0 + ay1*t + ay2*(t**2) + ay3*(t**3)

            disX = x_des - x
            disY = y_des - y
            dist = sqrt(disX**2 + disY**2)
            
            #Now we do PID stuff
            #calculate desired theta towards goal point
            gth = atan2(disY, disX)
            ang_err = gth - th
            while ang_err > pi:
                ang_err -= 2*pi
            while ang_err < -pi:
                ang_err += 2*pi
                
                
            cont.setPoint(gth)
            #PID output
            w = cont.update(th, True)
            
            #prevent glitches with max angular vel
            gdist = euclid_dist((x,y),current_waypoint)
            if w > av_max:
                w = av_max
            elif w < -av_max:
                w = -av_max
            #set linear vel
            if abs(ang_err) > 0.5:
                v = 0.0
            elif abs(ang_err) > 0.2 and gdist > 0.2:
                v = 0.1 #half max speed
            elif gdist <= 0.2 or (T - t) <= 0.3:
                v = gEndVel #the desired end speed.
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
            t += dt
        self.vel_pub.publish(Twist())
        self.prev_vx = vx1
        self.prev_vy = vy1


    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end, T):
        # input: p,v: position and velocity of start/end point
        #        T: the desired time to complete this segment of trajectory (in second)
        # output: the coefficients of this polynomial
        #find a0, a1, a2, a3 x0-xt a,b,c,d
        matrix_x = np.array([p_start, p_end, v_start, v_end], dtype=float)
        matrix_t = np.array([
            [0,    0,   0, 1],
            [T**3, T**2, T, 1],
            [0,    0,   1, 0],
            [3*T**2, 2*T, 1, 0]
        ], dtype=float)
        a3, a2, a1, a0 = np.linalg.solve(matrix_t, matrix_x)
        return a0, a1, a2, a3


    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
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
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


if __name__ == '__main__':
    whatever = Turtlebot()