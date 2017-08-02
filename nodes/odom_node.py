#!/usr/bin/env python
import rospy, math, tf, time
import geometry_msgs.msg, nav_msgs.msg

class MartyOdom(object):

    def __init__(self):
        super(MartyOdom, self).__init__()
        rospy.init_node('marty_odom')
        self.rate = rospy.Rate(10)
        self.listener = tf.TransformListener()
        self.odom_pub_ = rospy.Publisher(
            '/marty/odom', nav_msgs.msg.Odometry, queue_size=10)

    def return_rate(self):
        return self.rate

    def return_transform(self):
        trans = [0.0, 0.0, 0.0, 0.0]
        rot = [0.0, 0.0, 0.0, 0.0]
        try:
            (trans, rot) = self.listener.lookupTransform(
                '/base_link', '/odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        return (trans, rot)

    def create_odom_msg(self, trans, rot, accuracy = 3):
        # time_start = rospy.get_time()
        # print("start: " + str(time_start))

        odom_msg = nav_msgs.msg.Odometry()

        odom_pose = geometry_msgs.msg.PoseWithCovariance()
        odom_twist = geometry_msgs.msg.TwistWithCovariance()
        odom_quaternion = geometry_msgs.msg.Quaternion()

        odom_quaternion.x = round(rot[0], accuracy)
        odom_quaternion.y = round(rot[1], accuracy)
        odom_quaternion.z = round(rot[2], accuracy)
        odom_quaternion.w = round(rot[3], accuracy)

        odom_pose.pose.position.x = round(trans[0], accuracy);
        odom_pose.pose.position.y = round(trans[1], accuracy);
        odom_pose.pose.position.z = round(trans[2], accuracy);
        odom_pose.pose.orientation = odom_quaternion;


        # odom_twist.twist.linear.x

        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom";
        odom_msg.pose = odom_pose

        return odom_msg

    def publish_odom(self, odom):
        self.odom_pub_.publish(odom)

def main():
    odom_node = MartyOdom()
    rate = odom_node.return_rate()
    rospy.loginfo("Initialised odom_node with rate %s", rate)
    while rospy.is_shutdown() is not True:
        (trans, rot) = odom_node.return_transform()
        odom_msg = odom_node.create_odom_msg(trans, rot)
        odom_node.publish_odom(odom_msg)
        rate.sleep()

if __name__ == '__main__':
    main()
