#!/usr/bin/python3

import queue
import rospy
from geometry_msgs.msg import Twist


class TestRotate:
    def __init__(self, angular_speed, rate, topic, invert_speed_rate):
        self.rate = rate
        self.topic = topic
        self.angular_speed = angular_speed

        self.cmd_vel_pub = rospy.Publisher("/"+topic, Twist, queue_size=1)

        self.twist = Twist()
        self.twist.angular.z = self.angular_speed

    def twist_callback(self, timerEvent=None):
        self.cmd_vel_pub.publish(self.twist)
        rospy.Timer(rospy.Duration(1.0/rate), self.twist_callback, oneshot=True)

    def invert_angular(self, timerEvent=None):
        self.twist.angular.z = -self.twist.angular.z
        rospy.Timer(rospy.Duration(1.0/invert_speed_rate), self.invert_angular, oneshot=True)



if __name__ == "__main__":

    rospy.init_node("test_rotate_in_place")
    rate = rospy.get_param("~rate", 20)
    invert_speed_rate = rospy.get_param("~invert_speed_rate", 0.1)
    topic = rospy.get_param("~topic", "cmd_vel")
    angular_speed = rospy.get_param("~angular_speed", 1.0)

    test_rotate = TestRotate(angular_speed, rate, topic, invert_speed_rate)
    test_rotate.twist_callback()
    test_rotate.invert_angular()

    while not rospy.is_shutdown():
        rospy.sleep(1)


