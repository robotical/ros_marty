#!/usr/bin/env python
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('test_node')
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            raise

        print("translation: " + str(round(trans[0], 3)) + ", " + str(round(trans[1], 3)) + ", " + str(round(trans[2], 3)))
        print("rotation: " + str(round(rot[0], 3)) + ", " + str(round(rot[1], 3)) + ", " + str(round(rot[2], 3)) + ", " + str(round(rot[3], 3)))

        rate.sleep()
