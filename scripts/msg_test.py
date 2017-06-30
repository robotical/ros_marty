#!/usr/bin/env python
# Robotical Ltd. 2016 - Apache License
import rospy
# from std_msgs.msg import Int8
from marty_msgs.msg import ServoMsg, ServoMsgArray

def talker():
    pub = rospy.Publisher('/servo_array', ServoMsgArray, queue_size=10)
    rospy.init_node('servo_test', anonymous=True)
    rate = rospy.Rate(10)
    servo_cmd_array = ServoMsgArray()
    servo_cmd_1 = ServoMsg()
    servo_cmd_2 = ServoMsg()
    servo_cmd_1.servo_id = 0
    servo_cmd_1.servo_cmd = 60
    servo_cmd_2.servo_id = 8
    servo_cmd_2.servo_cmd = 0
    servo_cmd_array.servo_msg.append(servo_cmd_1)
    servo_cmd_array.servo_msg.append(servo_cmd_2)
    done = False

    while not rospy.is_shutdown():
        if not done:
            pub.publish(servo_cmd_array)
            done = True
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
