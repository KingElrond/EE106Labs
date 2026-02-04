#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Header
#
# Add code here to import the new ROS message...
#
from ee106w26.msg import EE106lab_custom_new
import random

def talker():
    rospy.init_node('talker')
    pub = rospy.Publisher('EE106lab_topic', EE106lab_custom_new, queue_size = 10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
      msg = EE106lab_custom_new()
      msg.header = Header()
      msg.header.stamp = rospy.Time.now()

      #
      # Add code here to create a new object of the new ROS message, to assign the random integers, and publish through the ROS topic...
      #
      msg.int1 = random.randint(0, 100)
      msg.int2 = random.randint(0, 100)
      pub.publish(msg)
      
      rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass