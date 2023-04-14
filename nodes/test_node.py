#!/usr/bin/python3
# -*- coding: utf-8 -*-

import multiprocessing
import time
import numpy as np
from multiprocessing import Lock, Process

import rospy
import rosparam
import diagnostic_updater
import diagnostic_msgs
from rosgraph import roslogging
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

from bugbase_node.test_odrive import ODriveInterface, ODriveRampControl, ODriveLivePlotter

class ROSLogger(object):
    """Imitate a standard Python logger, but pass the messages to rospy logging.
    """
    def debug(self, msg):    rospy.logdebug(msg) 
    def info(self, msg):     rospy.loginfo(msg)
    def warn(self, msg):     rospy.logwarn(msg)
    def error(self, msg):    rospy.logerr(msg)
    def critical(self, msg): rospy.logfatal(msg)

ros_logger = ROSLogger()

class ODriveNode:

    def __init__(self, params = dict()):

        self.__default_params = {"wheel_direction": False,
                                "turn_direction": False,
                                "base_width": 0.5,
                                "rounds_per_meter": 1.0,
                                "odom_calculation_rate_Hz": 60,
                                "control_rate_Hz": 100}

        for default_key in self.__default_params.keys():
            if(default_key not in params.keys()):
                params[default_key] = self.__default_params[default_key]

        self.odrive = ODriveInterface(params, ros_logger)
        # self.ramp_control = ODriveRampControl(params)

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

        self.odom_calculation_rate_Hz = params["odom_calculation_rate_Hz"]
        self.control_rate_Hz = params["control_rate_Hz"]

        self.axis_vel = [0,0]

        self.update_diagnostics_timer_active = False
        self.calculate_odom_timer_active     = False
        self.cmd_vel_callback_active         = False

        self.battery = None
        self.axis_error = 2*[ODriveInterface.ODriveError()]

        self.vl_rpm, self.vr_rpm = 0, 0
        self.vl_rpm_input, self.vr_rpm_input = 0, 0
        self.vl_ramp_rpm, self.vr_ramp_rpm = 0, 0

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

    def vel_ramp_control(self, vl_rpm, vr_rpm):
        self.ramp_control.set_set_vel(vl_rpm, vr_rpm)

        axis_vel = self.odrive.get_estimate_vel()
        self.ramp_control.set_current_vel(axis_vel[0], axis_vel[1])

        wheel_ramp    = [0, 0]
        _, wheel_ramp[0] = self.ramp_control.get_ramp(0)
        # rospy.loginfo("Ramp state of Axis0: %d %f" %(_, wheel_ramp[0]))
        _, wheel_ramp[1] = self.ramp_control.get_ramp(1)
        # rospy.loginfo("Ramp state of Axis0: %d %f" %(_, wheel_ramp[1]))

        return wheel_ramp[0], wheel_ramp[1]

    def convert_to_wheel_vel(self, linear_vel, angular_vel):

        base_width       = self.odrive.base_width
        rounds_per_meter = self.odrive.rounds_per_meter
        turn_direction   = self.odrive.turn_direction
        wheel_direction  = self.odrive.wheel_direction

        if turn_direction:
            vl = linear_vel + angular_vel * base_width / 2.0
            vr = linear_vel - angular_vel * base_width / 2.0
        else:
            vr = linear_vel + angular_vel * base_width / 2.0
            vl = linear_vel - angular_vel * base_width / 2.0

        vr_rpm = float(vr * rounds_per_meter)
        vl_rpm = float(vl * rounds_per_meter)
        ##########################################

        if wheel_direction:
            vr_rpm = -vr_rpm
        else:
            vl_rpm = -vl_rpm  
     
        return vl_rpm, vr_rpm

    def cmd_callback(self, cmd): 
        if not self.cmd_vel_callback_active:
            return

        self.mutex.acquire()
        try:
            # Convert velocity command to axis velocity rpm
            self.vl_rpm_input, self.vr_rpm_input = self.convert_to_wheel_vel(cmd.linear.x, cmd.angular.z)

            self.odrive.drive(left_vel_rpm  = self.vl_rpm_input, \
                            right_vel_rpm = self.vr_rpm_input)
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
            
            if(not axis_vel):
                return

            self.axis_vel[0], self.axis_vel[1] = axis_vel
        finally:
            self.mutex.release()


        axis0_vel, axis1_vel = self.axis_vel
        
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
    
    def update_diagnostics(self, timeEvent):
        self.error_diag_updater.update()
        self.battery_diag_updater.update()

    def control(self, timeEvenr):

        # Calculate the velocity ramp rate
        vl_ramp_rpm, vr_ramp_rpm = self.vel_ramp_control(self.vl_rpm_input, self.vr_rpm_input)

        if False:
            pass
        else:

            vl_ramp_rpm = min(vl_ramp_rpm, vr_ramp_rpm)
            vr_ramp_rpm = vl_ramp_rpm
        
        self.vl_ramp_rpm, self.vr_ramp_rpm = vl_ramp_rpm, vr_ramp_rpm
        
        self.mutex.acquire()
        try:
            pass

        finally:
            self.mutex.release()
        
    def main(self):
        self.update_diagnostics_timer = rospy.Timer(rospy.Duration(1), self.update_diagnostics)
        self.calculate_odom_timer     = rospy.Timer(rospy.Duration(1.0/self.odom_calculation_rate_Hz), self.calculate_odom)

        # liveplotter = ODriveLivePlotter(lambda: [-odrive_node.odrive.axes[0].encoder.vel_estimate, \
        #                             -odrive_node.odrive.axes[0].controller.vel_setpoint, \
        #                             odrive_node.odrive.axes[0].controller.config.vel_ramp_rate, \
        #                             -odrive_node.odrive.axes[0].motor.current_control.Iq_measured, \
        #                             -odrive_node.odrive.axes[0].motor.current_control.Iq_setpoint])

        # rospy.Timer(rospy.Duration(0.01), liveplotter.draw_data)

        while not rospy.is_shutdown():
            time.sleep(0.02)

            # liveplotter.fetch_data(self.mutex)

            self.mutex.acquire()
            try:
                if self.odrive.driver:
                    if self.axis_error[0] != 0 or self.axis_error[1] != 0:
                        self.cmd_vel_callback_active         = False
                        self.update_diagnostics_timer_active = False
                        self.calculate_odom_timer_active     = False

                        self.axis_error = 2*[ODriveInterface.ODriveError()]
                        self.odrive.reboot()

                elif self.odrive.connect() and self.odrive.engage():

                    self.last_odom_time = rospy.Time.now().to_sec()

                    self.update_diagnostics_timer_active = True
                    self.calculate_odom_timer_active     = True
                    self.cmd_vel_callback_active         = True

                else:
                    self.odrive.reboot()

            finally:
                self.mutex.release()

if __name__ == "__main__":
    rospy.init_node("bugbase_odrive_node")
    
    param_list = rosparam.list_params(rospy.get_name())
    params = {}
    for i in range(len(param_list)):
        param_list[i] = param_list[i].replace(rospy.get_name()+"/", "", 1)
        params[param_list[i]] = rospy.get_param("~"+param_list[i])
    
    odrive_node = ODriveNode(params)

    rospy.on_shutdown(odrive_node.odrive.idle)
    
    try:
        odrive_node.main()
    except rospy.ROSInterruptException:
        pass

    rospy.loginfo("Exiting")

