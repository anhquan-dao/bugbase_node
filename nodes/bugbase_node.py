#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf as ros_tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np
import serial
import time

from bugbase_driver import ESP32BugBase

WRITETEST = False
READTEST = False

SELFTEST = False

# =========================================================================


class BugBaseEncoder:
    def __init__(self, base_width, ticks_per_meter, left_enc_inverted, right_enc_inverted):
        self.BASE_WIDTH = base_width
        self.TICKS_PER_METER = ticks_per_meter

        self.odom = Odometry()
        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"

        self.odom.pose.covariance[0] = 0.01
        self.odom.pose.covariance[7] = 0.01
        self.odom.pose.covariance[14] = 9999
        self.odom.pose.covariance[21] = 9999
        self.odom.pose.covariance[28] = 9999
        self.odom.pose.covariance[35] = 0.01

        self.odom.twist.covariance = self.odom.pose.covariance

        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0
        self.last_enc_time = rospy.Time.now().to_sec()

        self.motor_encoder_ratio = 1.3333333
        self.left_enc_inverted = left_enc_inverted
        self.right_enc_inverted = right_enc_inverted

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def calculate_odom(self, vl_enc, vr_enc):
        if self.right_enc_inverted:
            vr_enc = -vr_enc
        if self.left_enc_inverted:
            vl_enc = -vl_enc

        vr_enc *= self.motor_encoder_ratio
        vl_enc *= self.motor_encoder_ratio

        current_time = rospy.Time.now().to_sec()
        dt = current_time - self.last_enc_time

        linear_vel = (vr_enc + vl_enc)/(self.TICKS_PER_METER * 2.0)
        angular_vel = (-vr_enc + vl_enc) / \
            (self.TICKS_PER_METER * self.BASE_WIDTH)

        if angular_vel == 0:
            self.current_x += linear_vel*np.cos(self.current_theta)*dt
            self.current_y += linear_vel*np.sin(self.current_theta)*dt
        else:
            r = linear_vel/angular_vel

            self.current_theta += angular_vel * dt
            self.current_theta = self.normalize_angle(self.current_theta)

            self.current_x += r*(np.sin(self.current_theta) -
                                 np.sin(self.current_theta - angular_vel*dt))
            self.current_y -= r*(np.cos(self.current_theta) -
                                 np.cos(self.current_theta - angular_vel*dt))

        odom_quat_orientation = ros_tf.transformations.quaternion_from_euler(
            0, 0, self.current_theta)

        self.odom.header.stamp = rospy.Time.now()
        self.odom.pose.pose.position.x = self.current_x
        self.odom.pose.pose.position.y = self.current_y
        self.odom.pose.pose.orientation.z = odom_quat_orientation[2]
        self.odom.pose.pose.orientation.w = odom_quat_orientation[3]

        self.odom.twist.twist.linear.x = linear_vel
        self.odom.twist.twist.angular.z = angular_vel

        self.last_enc_time = current_time

        return self.odom


class Node:

    def __init__(self):
        rospy.init_node("bugbase_node")
        rospy.on_shutdown(self.shutdown)

        port_name = rospy.get_param("~port", "/dev/ttyUSB0")
        baud_rate = rospy.get_param("~baudrate", 115200)
        timeout = rospy.get_param("~timeout", 0.1)
        rospy.loginfo("Connecting to ESP board on port {} at baudrate {}".format(
            port_name, baud_rate))

        base_width = rospy.get_param("~base_width", 0.4948)
        ticks_per_meter = rospy.get_param("~ticks_per_meter", 4904.7)
        left_inverted = rospy.get_param("~left_inverted", True)
        right_inverted = rospy.get_param("~right_inverted", True)
        turn_direction = rospy.get_param("~turn_direction", True)

        use_dynamic_acceleration = rospy.get_param(
            "~use_dynamic_acceleration", True)
        acceleration = rospy.get_param("~acceleration", 7000)

        update_rate = rospy.get_param("~update_rate", 20)

        left_enc_inverted = rospy.get_param("~left_enc_inverted", False)
        right_enc_inverted = rospy.get_param("~right_enc_inverted", True)

        yaw_offset = rospy.get_param("~yaw_offset", 0)

        self.bugbase = ESP32BugBase(port_name, baud_rate, timeout,
                                    base_width, ticks_per_meter, left_inverted, right_inverted,
                                    turn_direction, use_dynamic_acceleration, acceleration)

        if not self.bugbase.connect():
            rospy.logfatal("Could not connect to ESP32 board")
            rospy.signal_shutdown("Could not connect to ESP32 board")

        self.bugbase.setAcceleration(acceleration)
        rospy.loginfo("Setting motor's acceleration to " +
                      str(acceleration) + " ticks/second^2")
        # time.sleep(1)

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback, queue_size=1)

        odom_topic = rospy.get_param("~odom_topic", "/odometry/wheel")
        self.odom_publisher = rospy.Publisher(
            odom_topic, Odometry, queue_size=1)

        self.encoderOdom = BugBaseEncoder(
            base_width, ticks_per_meter, left_enc_inverted, right_enc_inverted)

        self.vr_ticks, self.vl_ticks = 0, 0
        self.accel_mode = 0

    def run(self):
        rospy.loginfo("Starting motor driver")

        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            speed1, speed2 = 0, 0

            if not WRITETEST:
                read_what = self.bugbase.readDriver()
                if read_what == 0:
                    speed1, speed2 = self.bugbase.readSpeed()

                    # rospy.loginfo("Encoder Speed: " + str(speed2) + "  " + str(speed1))

                    if(speed1 == 0x10000):
                        rospy.logwarn("Encoder Message Counter Fail")
                        speed1, speed2 = 0x0000, 0x0000

                    self.odom_publisher.publish(
                        self.encoderOdom.calculate_odom(speed1, speed2))

                    if SELFTEST:
                        self.vr_ticks, self.vl_ticks = np.random.randint(
                            -7500, 7500), np.random.randint(-7500, 7500)
                        self.bugbase.setSpeed(self.vr_ticks, self.vl_ticks)

                elif read_what == 9:

                    print(self.bugbase.readString())

                elif read_what == 10:
                    print("error")

            rate.sleep()

    def cmd_callback(self, data):

        try:
            if not READTEST:
                self.vr_ticks, self.vl_ticks = self.bugbase.inv_kinematics(
                    data.linear.x, data.angular.z)
                # rospy.loginfo("Set Stepper Speed: " + str(self.vr_ticks) + "  " + str(self.vl_ticks))
                if not SELFTEST:
                    self.bugbase.setSpeed(self.vr_ticks, self.vl_ticks)

            if WRITETEST:
                # self.bugbase.setAcceleration(7500)
                pass

        except OSError as e:
            rospy.logwarn(e.errno)

    def shutdown(self):
        rospy.loginfo("Shutting down")

        try:
            self.bugbase.setSpeed(0, 0)
        except Exception as e:
            rospy.logfatal("Cound not shutdown motors!!!")
            rospy.logfatal(e)

        del self.bugbase


if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass

    rospy.loginfo("Exiting")
