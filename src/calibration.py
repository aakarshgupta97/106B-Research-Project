#!/usr/bin/env python
"""
Author: Amay Saxena 
"""
import os, sys
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
import numpy as np
import tf2_ros
import tf
import roslaunch
import math
from constants import turtlebot_colors

def quaternion_matrix(quaternion):
    _EPS = np.finfo(float).eps * 4.0
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return numpy.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])


def all_active_bots():
    bots = []
    for i, color in enumerate(turtlebot_colors):
        topics = rospy.get_published_topics(namespace=color)
        if len(topics) > 0:
            bots.append(i)
    return bots

def calibrate():
    rospy.init_node('calibrate', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    bots = all_active_bots()
    print "ALL BOTS FOUND - "
    print [turtlebot_colors[i] for i in bots]
    print '-------------------------'
    rospy.sleep(2)

    locs = []
    for color in [turtlebot_colors[i] for i in bots]:
        file = './launch/calibration_' + color + '.launch'
        launch = roslaunch.parent.ROSLaunchParent(uuid, [file])
        launch.start()

        rospy.loginfo("started")

        found = False
        while not found:
            if '/ar_pose_marker' in [x[0] for x in rospy.get_published_topics()]:
                found = True
        found = False
        print "Waiting"
        rospy.sleep(15)
        print "Done Waiting"
        while not found:
            msg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
            markers = msg.markers
            if len(markers) > 0:
                # t = np.array([eval('markers[0].pose.pose.position.' + att) for att in ['x', 'y', 'z']])
                t = np.array([markers[0].pose.pose.position.x, markers[0].pose.pose.position.x, markers[0].pose.pose.position.x])
                print "Translation seen:", t

                x, y, z, w = [eval('markers[0].pose.pose.orientation.' + att) for att in ['x', 'y', 'z', 'w']]
                g = quaternion_matrix([x, y, z, w])
                print "Rotation Seen:", g[:3, :3]
                g[:3, 3] = -np.dot(g[:3, :3].T, t)
                g[:3, :3] = g[:3, :3].T
                locs.append(g)
                found = True

        raw_input("Proceed?")
        launch.shutdown()
    return bots, locs

if __name__ == '__main__':
    bots, locs = calibrate()
    np.save('serialized/bots', np.array(bots))
    np.save('serialized/transforms', np.array(locs))    
