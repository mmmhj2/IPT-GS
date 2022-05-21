#!/usr/bin/env python3
'''
A Python script publishing fake mavros/local_position/pose topic
'''

import roslib
roslib.load_manifest("gcserver")
import rospy as rp
import sys

from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    rp.init_node('fake_pose')
    rp.myargv(argv = sys.argv)
    
    pub = rp.Publisher("mavros/local_position/pose", PoseStamped)
    rate = rp.Rate(30)
    
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 1
    pose.pose.orientation.w = 1
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    
    while not rp.is_shutdown():
        pub.publish(pose)
        rate.sleep()
        
