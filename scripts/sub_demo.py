#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Author: GreatAlexander
# @Date:   2016-11-15
# @Last Modified by:   GreatAlexander
# @Last Modified time: 2016-11-15

# System
import time
import math
import martyPython as marty
# ROS
import rospy
from marty_msgs.msg import GPIOs

demoing = False
count = 0
limit = 200

def callback(data):
    global demoing
    global count
    if data.gpio[0] == 1.0:
        if demoing == False:
            rospy.loginfo("DEMOING!")
            demoing = True
            count = 0
            marty.demo()
            marty.stop()
        else:
            rospy.loginfo("WAITING! Count: %d", count)
    count = count + 1
    if demoing == True:
        if count > limit:
            count = 0
            demoing = False

def listener():

    rospy.init_node('sub_demo', anonymous=True)

    rospy.Subscriber("/gpios", GPIOs, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
