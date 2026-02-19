#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from rospy_tutorials.srv import AddTwoInts

def callback(data):
   rospy.loginfo(data.data)

# ROS Service function to be executed when the service is called. The return will provide the response of the service to the caller.
def add_two_ints(req):
   print(req)
   return (req.a + req.b)

def listener():
   rospy.init_node('node_b')
   rospy.Subscriber('chatter', String, callback)
   # Initialization of the ROS Service
   rospy.Service('add_two_ints', AddTwoInts, add_two_ints)
   rospy.spin()

if __name__ == '__main__':
   listener()