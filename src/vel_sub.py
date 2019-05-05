#!/usr/bin/env python
import rospy
from VelocityMsg.msg import *

def callback(data):
     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("vel_cmd", VelocityMsg, callback)

     # spin() simply keeps python from exiting until this node is stopped
     rospy.spin()

if __name__ == '__main__':
    listener()