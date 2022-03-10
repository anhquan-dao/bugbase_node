#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf as ros_tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np
import serial
import time
import threading

from bugbase_driver import ESP32BugBase

#=========================================================================
class BugBaseEncoder:
	def __init__(self, base_width, ticks_per_meter, left_enc_inverted, right_enc_inverted):
		self.BASE_WIDTH = base_width
		self.TICKS_PER_METER = ticks_per_meter

		self.odom = Odometry()
		self.odom.header.stamp 	 = rospy.Time.now()
		self.odom.header.frame_id = "odom"
		self.odom.child_frame_id  = "base_link"
		
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
		angular_vel = (vr_enc - vl_enc)/(self.TICKS_PER_METER * self.BASE_WIDTH)

		self.current_x += (linear_vel * np.cos(angular_vel)) * dt
		self.current_y += (linear_vel * np.sin(angular_vel)) * dt
		self.current_theta += angular_vel * dt
		odom_quat_orientation = ros_tf.transformations.quaternion_from_euler(\
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
		rospy.loginfo("Connecting to ESP board on port {} at baudrate {}".format(port_name, baud_rate))

		base_width = rospy.get_param("~base_width", 0.4948)
		ticks_per_meter = rospy.get_param("~ticks_per_meter", 4904.7)
		left_inverted = rospy.get_param("~left_inverted", True)
		right_inverted = rospy.get_param("~right_inverted", True)
		
		left_enc_inverted = rospy.get_param("~left_enc_inverted", False)
		right_enc_inverted = rospy.get_param("~right_enc_inverted", True)

		acceleration = rospy.get_param("~acceleration",7000)

		yaw_offset = rospy.get_param("~yaw_offset", 0)

		# TODO
		# Add left_inverted and right_inverted configuration

		self.bugbase = ESP32BugBase(port_name, baud_rate, timeout,\
								  base_width, ticks_per_meter, left_inverted, right_inverted)

		if not self.bugbase.connect():
			rospy.logfatal("Could not connect to ESP32 board")
			rospy.signal_shutdown("Could not connect to ESP32 board")

		rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback)

		odom_topic = rospy.get_param("~odom_topic", "/odometry/wheel")
		self.odom_publisher = rospy.Publisher(odom_topic, Odometry, queue_size=1)

		self.encoderOdom = BugBaseEncoder(base_width, ticks_per_meter, left_enc_inverted, right_enc_inverted)

	def run(self):
		rospy.loginfo("Starting motor driver")
		rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			speed1, speed2 = 0, 0

			speed1, speed2 = self.bugbase.readSpeed()

			# Convert 2bytes signed to 4bytes signed
			if(speed1 & 0x8000): 
				speed1 = -0x10000 + speed1
			if(speed2 & 0x8000): 
				speed2 = -0x10000 + speed2

			
			self.odom_publisher.publish(self.encoderOdom.calculate_odom(speed1, speed2))
			
			# rospy.loginfo(str(speed1) + " " + str(-speed2))

			# TODO
			# add inverted wheel speed configuration

			rate.sleep()

		
	def cmd_callback(self, data):

		try:
			vr_ticks, vl_ticks = self.bugbase.inv_kinematics(data.linear.x, data.angular.z)
			# rospy.loginfo("{} {}".format(hex(vr_ticks), hex(vl_ticks)))
			self.bugbase.setSpeed(vr_ticks, vl_ticks)
		except OSError as e:
			rospy.logwarn(e.errno)
		

	def shutdown(self):
		rospy.loginfo("Shutting down")

		try:
			self.bugbase.setSpeed(0,0)
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