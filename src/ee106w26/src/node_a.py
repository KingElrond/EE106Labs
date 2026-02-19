#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Header
from rospy_tutorials.srv import AddTwoInts

def talker():

   pub = rospy.Publisher('chatter', String, queue_size = 10)
   rate = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
   header = Header()
   header.stamp = rospy.Time.now()

   content = "welcome to the Robotics Lab " + str(header.stamp)
   pub.publish(content)

   # Call of the ROS Service 'add_two_ints'
   rospy.wait_for_service('add_two_ints')

   add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
   try:
       response_msg = add_two_ints(2, 1)
       print(response_msg)
   except rospy.ServiceException as exc:
       print("Service did not process request: " + str(exc))

   rate.sleep()


if __name__ == '__main__':
   try:
       rospy.init_node('node_a')
       talker()
   except rospy.ROSInterruptException:
       pass