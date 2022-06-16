#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import rosparam
import tf as ros_tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import diagnostic_updater
import diagnostic_msgs

import numpy as np
import serial
import time

from bugbase_node.bugbase_driver import ESP32BugBase
# from bugbase_node.speed_graph import SpeedVisualizer

from bugbase_node.msg import BugbaseMotor, BugbaseMotorArray

WRITETEST = False
READTEST = False

SELFTEST = False

# =========================================================================


class BugBaseEncoder:
    def __init__(self, params):
        self.BASE_WIDTH = params["base_width"]
        self.TICKS_PER_METER = params["ticks_per_meter"]

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

        self.motor_encoder_ratio = params["motor_enc_ratio"]
        self.wheel_encoder_ratio = params["wheel_enc_ratio"]
        self.left_enc_inverted = params["left_enc_inverted"]
        self.right_enc_inverted = params["right_enc_inverted"]

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

        vr_enc *= self.wheel_encoder_ratio
        vl_enc *= self.wheel_encoder_ratio

        # rospy.loginfo("Encoder tick speed: " + str(vr_enc) + "  " + str(vl_enc))

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

            self.current_x += r*(-np.sin(self.current_theta) +
                                 np.sin(self.current_theta + angular_vel*dt))
            self.current_y += r*(np.cos(self.current_theta) -
                                 np.cos(self.current_theta + angular_vel*dt))


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

    def __init__(self, params):
        rospy.on_shutdown(self.shutdown)
        
        self.update_period = params["update_period"]
        self.no_encoder_operation = params["no_encoder_operation"]

        self.bugbase = ESP32BugBase(params)

        if not self.bugbase.connect():
            rospy.logfatal("Could not connect to ESP32 board")
            rospy.signal_shutdown("Could not connect to ESP32 board")

        self.cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback, queue_size=1)

        self.odom_publisher = rospy.Publisher(
            params["odom_topic"], Odometry, queue_size=1)

        self.encoderOdom = BugBaseEncoder(params)

        self.vr_ticks, self.vl_ticks = 0, 0
        self.accel_mode = 0
        
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("none")

        self.updater.add("Function updater", self.dummy_diagnostic)

        self.debug_publisher = rospy.Publisher("motor_state", BugbaseMotorArray, queue_size=1)
        self.debug_msg = BugbaseMotorArray()
        self.debug_msg.motor = [BugbaseMotor(), BugbaseMotor()]

        self.diag_accel = [0, 0]
        self.diag_speed = [0, 0]

    def run(self):
        rospy.loginfo("Starting motor driver")
        
        while not rospy.is_shutdown():
            speed1, speed2 = 0, 0

            if not WRITETEST:
                read_what = self.bugbase.waitForHeader()
                if read_what <= 1:
                    if(read_what == 0):
                        speed1, speed2 = self.bugbase.readSpeed()

                    if(read_what == 1):
                        data = self.bugbase.readSpeedProfile()
                        speed1, speed2 = data[-2:]
                        self.diag_accel[0], self.diag_accel[1], self.diag_speed[0], self.diag_speed[1] = data[:4]
                        # print(data)

                    # rospy.loginfo("Encoder Speed: " + str(speed2) + "  " + str(speed1))

                    if(speed1 == 0x10000):
                        rospy.logwarn("Encoder Message Counter Fail")
                        speed1, speed2 = 0x0000, 0x0000

                    self.odom_publisher.publish(
                        self.encoderOdom.calculate_odom(speed1, speed2))
                    
                elif read_what == 3:
                    data = self.bugbase.readbyte()
                    # print(data)

                elif read_what == 4:
                    self.diag_accel[0], self.diag_accel[1], self.diag_speed[0], self.diag_speed[1] = self.bugbase.readSpeedDebug()

                elif read_what == 9:
                    print(self.bugbase.readString())

                elif read_what == 10:
                    print("error")

                elif read_what == 11:
                    # print("timeout")
                    pass

            self.updater.force_update()

            self.debug_msg.motor[0].acceleration = self.diag_accel[0]
            self.debug_msg.motor[0].velocity = speed1
            self.debug_msg.motor[0].est_velocity = self.diag_speed[0]
            self.debug_msg.motor[0].set_velocity = self.vr_ticks

            self.debug_msg.motor[1].acceleration = self.diag_accel[1]
            self.debug_msg.motor[1].velocity = speed2
            self.debug_msg.motor[1].est_velocity = self.diag_speed[1]
            self.debug_msg.motor[1].set_velocity = self.vl_ticks

            self.debug_publisher.publish(self.debug_msg)

            # rate.sleep()

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

    def dummy_diagnostic(self, stat):

        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Nothing to report")
        stat.add("Acceleration 0", self.diag_accel[0])
        stat.add("Acceleration 1", self.diag_accel[1])
        stat.add("Est Speed 0", self.diag_speed[0])
        stat.add("Est Speed 1", self.diag_speed[1])

        stat.add("Actual Speed 0", self.vl_ticks)
        stat.add("Actual Speed 1", self.vr_ticks)

        return stat

    def shutdown(self):
        self.cmd_sub.unregister()
        self.odom_publisher.unregister()
        rospy.loginfo("Shutting down")

        try:
            self.bugbase.setSpeed(0, 0)
            if(not self.bugbase.sendShutdownRequest()):
                rospy.logerr("Could not clear buffer!!!")
        except Exception as e:
            rospy.logfatal("Cound not shutdown motors!!!")
            rospy.logfatal(e)

if __name__ == "__main__":
    rospy.init_node("bugbase_node")

    port_name = rospy.get_param("~port", "/dev/ttyUSB0")
    baud_rate = rospy.get_param("~baudrate", 115200)
    timeout = rospy.get_param("~timeout", 0.1) # Unit: s
    rospy.loginfo("Connecting to ESP board on port {} at baudrate {}".format(
        port_name, baud_rate))

    # base_width = rospy.get_param("~base_width", 0.4948) # Unit: m
    # ticks_per_meter = rospy.get_param("~ticks_per_meter", 4904.7) # Unit: ticks/m
    # left_inverted = rospy.get_param("~left_inverted", True)
    # right_inverted = rospy.get_param("~right_inverted", True)
    # turn_direction = rospy.get_param("~turn_direction", True)\
    
    # acceleration = rospy.get_param("~acceleration", 2.0) # Unit: m/s^2
    # weak_acceleration = rospy.get_param("~weak_acceleration", 1.0) # Unit: m/s^2
    # deceleration = rospy.get_param("~deceleration", 3.0) # Unit: m/s^2
    # brake_accel = rospy.get_param("~brake_accel", 7.0) # Unit: m/s^2
    # accel_profile = [weak_acceleration, acceleration, deceleration, brake_accel]

    # max_acceleration = rospy.get_param("~max_acceleration", 10.0) # Unit: m/s^2
    # decel_time_limit = rospy.get_param("~deceleration_time_limit", 200) # Unit: ms
    # accel_time_limit = rospy.get_param("~acceleration_time_limit", 500) # Unit: ms

    # accel_boundary_speed = rospy.get_param("~accel_boundary_speed", 0.2)
    # decel_boundary_speed = rospy.get_param("~decel_boundary_speed", 0.2)

    # update_rate = rospy.get_param("~update_rate", 50) # Unit: Hz
    # update_period = 1000.0/update_rate
    
    # no_encoder_operation = rospy.get_param("~no_encoder_operation", False)
    # left_enc_inverted = rospy.get_param("~left_enc_inverted", False)
    # right_enc_inverted = rospy.get_param("~right_enc_inverted", True)
    # wheel_enc_ratio = rospy.get_param("~wheel_enc_ratio", 1.3333)
    # motor_enc_ratio = rospy.get_param("~motor_enc_ratio", 9.62)

    # odom_topic = rospy.get_param("~odom_topic", "/odometry/wheel")

    # visualizer = rospy.get_param("~visualizer", False)

    # playground_enable = rospy.get_param("~playground_enable", False)
    # angular_acceleration = rospy.get_param("~angular_acceleration", 0.5)
    
    # params = {
    #         'port' : port_name,
    #         'baud' : baud_rate,
    #         'timeout' : timeout,
    #         'base_width' : base_width,
    #         'ticks_per_meter' : ticks_per_meter,
    #         'left_inverted' : left_inverted,
    #         'right_inverted' : right_inverted,
    #         'turn_direction' : turn_direction,
    #         'accel_profile' : accel_profile,
    #         'accel_boundary_speed' : accel_boundary_speed,
    #         'decel_boundary_speed' : decel_boundary_speed,
    #         'max_acceleration' : max_acceleration,
    #         'decel_time_limit' : decel_time_limit,
    #         'accel_time_limit' : accel_time_limit,
    #         'update_period' : update_period,
    #         'update_rate' : update_rate,
    #         'no_encoder_operation' : no_encoder_operation,
    #         'left_enc_inverted' : left_enc_inverted,
    #         'right_enc_inverted' : right_enc_inverted,
    #         'wheel_enc_ratio' : wheel_enc_ratio,
    #         'motor_enc_ratio' : motor_enc_ratio,
    #         'odom_topic' : odom_topic,
    #         'visualizer' : visualizer,
    #         'playground_enable': playground_enable,
    #         "angular_acceleration": angular_acceleration
    #     }

    param_list = rosparam.list_params(rospy.get_name())
    params = {}
    for i in range(len(param_list)):
        param_list[i] = param_list[i].replace(rospy.get_name()+"/", "", 1)
        params[param_list[i]] = rospy.get_param("~"+param_list[i])

    params["update_period"] = 1000.0/params["update_rate"]
    params["accel_profile"] = [params["weak_acceleration"], \
                               params["acceleration"], \
                               params["deceleration"], \
                               params["brake_accel"]]

    try:
        node = Node(params)
        node.run()
    except rospy.ROSInterruptException:
        pass

    rospy.loginfo("Exiting")
