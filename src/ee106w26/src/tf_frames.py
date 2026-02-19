#!/usr/bin/env python3
import roslib
roslib.load_manifest('ee106w26')
import rospy
import math
import tf
import tf.transformations
import geometry_msgs.msg
import numpy as np
import sys



class turtlebot_tf_frames:

  def __init__(self):

    #using ROS tf Tutorial for TF Broadcaster
    #initialize TF Broadcaster
    self.br = tf.TransformBroadcaster()
    #set Rate
    self.rate = rospy.Rate(50) 
    
    rospy.loginfo("Publishing static TF frames: base_scan, left_limit, right_limit")
    
    while not rospy.is_shutdown():
      now = rospy.Time.now()

      # identity quaternion (no rotation)
      q = (0.0, 0.0, 0.0, 1.0)

      # 1) base_footprint -> base_scan (0,0,0.20)
      self.br.sendTransform(
        (0.0, 0.0, 0.20),
        q,
        now,
        "base_scan",
        "base_footprint"
      )
      
      # 2) base_scan -> left_limit (0,+0.07,0)
      self.br.sendTransform(
        (0.0, 0.07, 0.0),
        q,
        now,
        "left_limit",
        "base_scan"
      )

      # 3) base_scan -> right_limit (0,-0.07,0)
      self.br.sendTransform(
        (0.0, -0.07, 0.0),
        q,
        now,
        "right_limit",
        "base_scan"
      )
      self.rate.sleep()


def main(args):
  rospy.init_node('lab5_tf_frames', anonymous=True)
  tf_node = turtlebot_tf_frames()

if __name__ == '__main__':
  main(sys.argv)
