#!/usr/bin/env python3

import rospy
import roslib
import sys
import numpy as np
import tf
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Twist


class turtlebot_behavior:

    def __init__(self):
        # rospy.init_node("left_wall_follwer")
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber("/scan", LaserScan, self.callback)
        rospy.loginfo("Left wall follower initialized")

    def callback(self, data):
        pts = []
        for i, r in enumerate(data.ranges):
            if r == float('inf') or r == 0.0:
                continue

            angle = data.angle_min + i * data.angle_increment
            if 1.52 <= angle <= 1.62:
                x_s = r * math.cos(angle)
                y_s = r * math.sin(angle)

                p = PointStamped()
                p.header = data.header
                p.header.frame_id = data.header.frame_id
                p.point.x = x_s
                p.point.y = y_s
                p.point.z = 0.0
                pts.append(p)

        if not pts:
            return

        dists = []
        for p in pts:
            try:
                self.tf_listener.waitForTransform('left_limit', p.header.frame_id, p.header.stamp, rospy.Duration(0.1))
                p_out = self.tf_listener.transformPoint('left_limit', p)
                dx = p_out.point.x
                dy = p_out.point.y
                dists.append(math.hypot(dx, dy))
            except (tf.LookupException, tf.ExtrapolationException):
                rospy.logwarn_throttle(5, "TF to left limit not available")
                continue

        cmd = Twist()

        if not dists or not pts:
            rospy.loginfo("No points in the left window")
            cmd.linear.x = 0.07
            cmd.angular.z = 0.07
            self.cmd_pub.publish(cmd)
            return

        rospy.loginfo("Min left distance: %.2f", min(dists))

        front_angles = [
            r for i, r in enumerate(data.ranges)
            if r != float('inf') and r != 0.0 and -0.1 <= (data.angle_min + i * data.angle_increment) <= 0.1
        ]
        min_front_dist = min(front_angles) if front_angles else float('inf')

        if min_front_dist < 0.3:
            rospy.logwarn("Obstacle ahead: stopping")
            cmd.linear.x = 0.0
            cmd.angular.z = -0.4
            self.cmd_pub.publish(cmd)
            return

        if 0.3 < min_front_dist < 0.5:
            rospy.loginfo("Preparing right turn to avoid front obstacle")
            cmd.linear.x = 0.0
            cmd.angular.z = -0.4
            self.cmd_pub.publish(cmd)
            return

        if min(dists) > 0.4:
            rospy.loginfo("Sharp left turn")
            cmd.linear.x = 0.2
            cmd.angular.z = 0.4
            self.cmd_pub.publish(cmd)
            return

        if min(dists) < 0.1:
            rospy.loginfo("Left obstacle: steer right")
            cmd.linear.x = 0.1
            cmd.angular.z = -0.05
        elif min(dists) > 0.2:
            rospy.loginfo("Too far from left wall: steer left")
            cmd.linear.x = 0.1
            cmd.angular.z = 0.05
        else:
            rospy.loginfo("keep going on left wall")
            cmd.linear.x = 0.1
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node("left_wall_follower")
        node = turtlebot_behavior()
        node.run()
    except rospy.ROSInterruptException:
        pass


