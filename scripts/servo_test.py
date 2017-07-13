#!/usr/bin/env python
# license removed for brevity
import rospy
#from std_msgs.msg import Int8
from marty_msgs.msg import ServoMsg, ServoMsgArray

def talker():
    pub = rospy.Publisher('/marty/servo_array', ServoMsgArray, queue_size=10)
    rospy.init_node('servo_test', anonymous=True)
    rate = rospy.Rate(50)
    servo_cmd_array = ServoMsgArray()
    servo_cmd_1 = ServoMsg()
    servo_cmd_2 = ServoMsg()
    servo_cmd_1.servo_id = 7
    servo_cmd_1.servo_cmd = 0
    servo_cmd_2.servo_id = 8
    servo_cmd_2.servo_cmd = 0
    servo_cmd_array.servo_msg.append(servo_cmd_1)
    servo_cmd_array.servo_msg.append(servo_cmd_2)
    servo_cmd = 0

    up = True
    inc = 5
    lim = 120
    while not rospy.is_shutdown():
        if (up == True):
            if (servo_cmd < lim):
                servo_cmd = servo_cmd + inc
            else:
                up = False
        else:
            if (servo_cmd > -lim):
                servo_cmd = servo_cmd - inc
            else:
                up = True

        servo_cmd_array.servo_msg[0].servo_cmd = servo_cmd
        servo_cmd_array.servo_msg[1].servo_cmd = servo_cmd
        pub.publish(servo_cmd_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
