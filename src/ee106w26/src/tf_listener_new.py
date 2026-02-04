#!/usr/bin/env python3
import roslib
roslib.load_manifest('ee106w26')
import rospy
import math
import tf
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
#import geometry_msgs.msg
import numpy as np
#----------------------------------------------------------------------------------------------------------------------

def callback(scan: LaserScan):
  # Get the transformations and rotations
  try:
    (trans,rot) = listener.lookupTransform('front_bumper','front_laser',rospy.Time(0))
  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    rospy.logwarn("No TF available yet")
    return
  # Build a 4x4 homogeneous transform matrix T
  T = tf.transformations.quaternion_matrix(rot)
  T[0:3,3] = trans
  worst = 'minor'
  # Loop over all ranges, skip the inf ranges
  for idx, r in enumerate(scan.ranges):
    if not np.isfinite(r):
      continue
    # transform each point
    theta = scan.angle_min + idx * scan.angle_increment
    x_Laser = r * math.cos(theta)
    y_Laser = r * math.sin(theta)
    p_Laser = np.array([x_Laser, y_Laser, 0.0, 1.0])
    # transofrm into base_link frame
    p_Bumper = T.dot(p_Laser)
    # compute distance (in meters)
    distance = np.hypot(p_Bumper[0], p_Bumper[1])
    print(distance)
    #COMPLETE CODE….
    if(distance < 0.2):
       worst = 'critical'
    elif (distance < 0.5):
       worst = 'major'
    # classify critical, major, minor
    #PUBLISH worst
  pub.publish(worst)
  
if __name__ == '__main__':
  rospy.init_node('tf_listener_new')
  listener = tf.TransformListener()
  # Create a publisher to view the output of
  pub = rospy.Publisher('jackal_robot_status', String, queue_size=1)
  # Create a subscriber for the /front/scan topic that will send LiDAR data to callback()
  rospy.Subscriber('/front/scan', LaserScan, callback)
  rospy.loginfo("TF with LiDAR")
  rospy.spin()