#!/usr/bin/env python3
import roslib
roslib.load_manifest('ee106w26')
import rospy
import math
import tf
import geometry_msgs.msg
import numpy as np
rospy.init_node('tf_listener_node')

# initialization of the ROS tf listener
listener = tf.TransformListener()

rate = rospy.Rate(10.0)
# the goal of this node is to continously listen to the transformation relation between the base_link and front_laser ROS frames and print the Translation and Rotation of the captured transformation matrix.
while not rospy.is_shutdown():
   try:
       # capture the tf of the two frames the exact moment of the command execution (rospy.Time(0))
       (trans,rot) = listener.lookupTransform('/base_link', '/front_laser', rospy.Time(0))
   except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
       continue

   # print of the Translation and Rotation information, by demonstrating the Quaternion, Euler, and Rotation Matrix representation of the latter.
   print("The translation is (x,y,z) = " + str(trans))
   print("The rotation (quaternion) is (x,y,z,w) = " + str(rot))
   print("The rotation (euler) is (r,p,y) = " + str(tf.transformations.euler_from_quaternion(rot)))
   rot_mat = tf.transformations.quaternion_matrix(rot)
   print("The rotation (rotation matrix) is = " + str(tf.transformations.quaternion_matrix(rot)))

   # we assume that a Lidar point is detected, w.r.t the Lidar's frame
   laser_point_detected = [1, 0, 0, 1]

   # initialization of the tf matrix to describe it in the /base_link frame
   rot_mat[0,3] = trans[0]
   rot_mat[1,3] = trans[1]
   rot_mat[2,3] = trans[2]
   print(np.dot(rot_mat , laser_point_detected))

   rate.sleep()