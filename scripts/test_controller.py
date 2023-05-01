#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 09:16:20 2022

@author: jonathan
"""

import rospy
import time
import json
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, Twist

twist_pub = None

s = 10
k = 0.0
r = 0.5
w = 1
k_dir = 1

def poseCallback(pose):

    twist = Twist()
    twist.linear.x = 1
    twist.angular.z = 0.5

    twist_pub.publish(twist)


def main():
    global twist_pub

    rospy.init_node("test_controller", anonymous=True)
    twist_pub = rospy.Publisher("twist", Twist, queue_size=100)
    rospy.Subscriber("pose", Pose, poseCallback)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    main()
