#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
#
# Add code here to import the new ROS message...
#
from ee106w26.msg import EE106lab_custom_new

def callback(data):
    #
    # Add code here to perform the addition of the two integer field of the variable data
    #
    result = data.int1 + data.int2
    rospy.loginfo("int1=%d int2=%d sum=%d stamp=%s", data.int1, data.int2, result, str(data.header.stamp))



def listener():
    rospy.init_node('listener')
    #
    # Initialize the ROS subscriber to capture the new  message-type ROS topic. The function "callback" will be the callback of the ROS subscriber.
    #
    rospy.Subscriber('EE106lab_topic', EE106lab_custom_new, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()