#!/usr/bin/python3
# -*- coding: utf-8 -*-

import time
import numpy as np
from multiprocessing import Lock

import rospy
import diagnostic_updater
import diagnostic_msgs
from rosgraph import roslogging
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

from bugbase_node.test_odrive import ODriveInterface


class ODriveNode:

    def __init__(self, params = dict()):

        self.__default_params = {"wheel_direction": False,
                                "turn_direction": False,
                                "base_width": 0.5,
                                "rounds_per_meter": 1.0}

        for default_key in self.__default_params.keys():
            if(default_key not in params.keys()):
                params[default_key] = self.__default_params[default_key]

        self.odrive = ODriveInterface(params)

        self.error_diag_updater = diagnostic_updater.Updater()
        self.error_diag_updater.setHardwareID("none")
        self.error_diag_updater.add("ODrive Error", self.error_diagnostic_callback)
        
        self.battery_diag_updater = diagnostic_updater.Updater()
        self.battery_diag_updater.setHardwareID("none")
        self.battery_diag_updater.add("ODrive Battery", self.battery_diagnostic_callback)

        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback, buff_size=1)
        self.odom_pub    = rospy.Publisher("/odom", Odometry, queue_size=1)

        self.prev_tick = [0, 0]
        self.current_pose = Pose()
        self.odom = Odometry()

        self.odom_calculate_Hz = 60

        self.update_diagnostics_timer_active = False
        self.calculate_odom_timer_active     = False

        self.battery = None
        self.axis_error = 2*[ODriveInterface.ODriveError()]

        self.mutex = Lock()

    def error_diagnostic_callback(self, stat):
        if not self.update_diagnostics_timer_active:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "ODrive is disconnected!")
            return stat

        self.mutex.acquire()
        try:
            self.axis_error = self.odrive.get_error()
        finally:
            self.mutex.release()

        if not self.axis_error:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "ODrive is disconnected!")
            return stat

        summary_error = 0

        for i, error in enumerate(self.axis_error):
            if error != 0:
                summary_error += 1

            stat.add("Axis %d error" %(i), str(error.axis_error_name()))
            stat.add("Encoder %d error" %(i), str(error.encoder_error_name()))
            stat.add("Motor %d error" %(i), str(error.motor_error_name()))
            stat.add("Controller %d error" %(i), str(error.controller_error_name()))

        if summary_error != 0:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Error!")

        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "No error.")

        return stat

    def battery_diagnostic_callback(self, stat):
        if not self.update_diagnostics_timer_active:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "ODrive is disconnected!")
            return stat

        self.mutex.acquire()
        try:
            self.battery = self.odrive.get_battery()
        finally:
            self.mutex.release()

        if (not self.battery):
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "ODrive is disconnected!")
            return stat
        
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "OK")
        stat.add("Battery voltage", self.battery[0])
        stat.add("Battery current", self.battery[1])

        return stat

    def cmd_callback(self, cmd):   
        self.mutex.acquire()
        try:         
            self.odrive.velocity_command_callback(cmd.linear.x, cmd.angular.z)
        finally:
            self.mutex.release()
    
    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def calculate_odom(self, timerEvent):
        if not self.calculate_odom_timer_active:
            return

        self.mutex.acquire()
        try:
            axis_vel = self.odrive.get_estimate_vel()

            if(axis_vel):
                axis0_vel, axis1_vel = axis_vel
                
                if(self.odrive.wheel_direction):
                    axis1_vel = -axis1_vel
                else:
                    axis0_vel = -axis0_vel

                current_time = rospy.Time.now().to_sec()
                dt = current_time - self.last_odom_time
                
                linear_vel = (axis0_vel + axis1_vel)/(self.odrive.rounds_per_meter * 2.0)
                angular_vel = (axis0_vel - axis1_vel)/(self.odrive.rounds_per_meter * self.odrive.base_width)

                if not self.odrive.turn_direction:
                    angular_vel = -angular_vel
                    
                current_theta = 2*np.arctan2(self.current_pose.orientation.z, \
                                        self.current_pose.orientation.w)
                if angular_vel == 0:
                    self.current_pose.position.x += linear_vel*np.cos(current_theta)*dt
                    self.current_pose.position.y += linear_vel*np.sin(current_theta)*dt
                else:
                    r = linear_vel/angular_vel

                    self.current_pose.position.x += r*(-np.sin(current_theta) + \
                                                np.sin(current_theta + angular_vel*dt))

                    self.current_pose.position.y += r*(np.cos(current_theta) - \
                                                np.cos(current_theta + angular_vel*dt))

                    current_theta += angular_vel * dt
                    current_theta = self.normalize_angle(current_theta)

                    self.current_pose.orientation.z = np.sin(current_theta/2.0)
                    self.current_pose.orientation.w = np.cos(current_theta/2.0)

                self.last_odom_time = current_time

                self.odom.header.stamp = rospy.Time.now()
                self.odom.pose.pose    = self.current_pose

                self.odom.twist.twist.linear.x = linear_vel
                self.odom.twist.twist.angular.z = angular_vel

                self.odom_pub.publish(self.odom)

        finally:
            self.mutex.release()

    def update_diagnostics(self, timeEvent):
        self.error_diag_updater.update()
        self.battery_diag_updater.update()
        
    def main(self):

        self.update_diagnostics_timer = rospy.Timer(rospy.Duration(1), self.update_diagnostics)
        self.calculate_odom_timer     = rospy.Timer(rospy.Duration(1.0/self.odom_calculate_Hz), self.calculate_odom)

        while not rospy.is_shutdown():
            time.sleep(0.5)

            if self.odrive.driver:
                if self.axis_error[0] != 0 or self.axis_error[1] != 0:
                    self.mutex.acquire()
                    try:
                        self.odrive.reboot()
                    finally:
                        self.mutex.release()

            else:
                if self.odrive.connect():
                    if self.odrive.engage():
                        self.last_odom_time = rospy.Time.now().to_sec()

                        self.update_diagnostics_timer_active = True
                        self.calculate_odom_timer_active     = True
                        
                        self.error_diag_updater.force_update()
                        self.battery_diag_updater.force_update()

if __name__ == "__main__":

    rospy.init_node("test")
    odrive_node = ODriveNode()
    odrive_node.main()

