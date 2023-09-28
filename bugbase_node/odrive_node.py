import multiprocessing
import time
import numpy as np
from multiprocessing import Lock, Process

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

import diagnostic_updater
import diagnostic_msgs

import nav_msgs.msg

from .odrive_interface import ODriveError, ODriveInterface, ODriveRampControl, ODriveLivePlotter


def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle
    
class ROSLogger():
    """Imitate a standard Python logger, but pass the messages to rospy logging.
    """
    def __init__(self, ros_node):
        self.ros_node = ros_node
    def debug(self, msg):    self.ros_node.get_logger().debug(msg) 
    def info(self, msg):     self.ros_node.get_logger().info(msg)
    def warning(self, msg):     self.ros_node.get_logger().warning(msg)
    def error(self, msg):    self.ros_node.get_logger().error(msg)
    def critical(self, msg): self.ros_node.get_logger().fatal(msg)

class ODriveNode(Node):

    CONNECTION_ATTEMPT = 5
    def __init__(self):

        super().__init__('odrive_node')

        self.__default_params = {"odom_calculation_rate_Hz": 60,
                                "control_rate_Hz": 100,
                                "odrive_watchdog_enable": True,
                                "odrive_watchdog_timeout": 5.0,
                                "antislip_enable": True,
                                "wheel_direction": False,
                                "turn_direction": False,
                                "base_width": 0.5,
                                "rounds_per_meter": 1.0}

        self.declare_parameters(
            namespace='',
            parameters=[(key, default_value) for key, default_value in self.__default_params.items()]
        )

        params = dict()

        for default_key in self.__default_params.keys():
            params[default_key] = self.get_parameter(default_key).value
        
        ros_logger = ROSLogger(self)

        self.odrive = ODriveInterface(params, ros_logger)

        # ------------ Diagnostics resources ------------
        self.diag_updater = diagnostic_updater.Updater(self)
        self.diag_updater.setHardwareID("none")
        self.diag_updater.add("ODrive Error", self.error_diagnostic_callback)
        self.diag_updater.add("ODrive Battery", self.battery_diagnostic_callback)

        self.battery = None
        self.axis_error = 2*[ODriveError()]
        # ------------------------------------------------

        # ------------ Velocity control and Odom calculation resources ------------
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 1, callback_group=self.callback_group)
        self.odom_pub    = self.create_publisher(Odometry, "/odom", 1, callback_group=self.callback_group)

        self.prev_tick = [0, 0]
        self.current_pose = Pose()
        self.odom = Odometry()
        self.odom.twist.covariance[0] = 0.01
        self.odom.twist.covariance[7] = 0.01
        self.odom.twist.covariance[14] = 9999
        self.odom.twist.covariance[21] = 9999
        self.odom.twist.covariance[28] = 9999
        self.odom.twist.covariance[35] = 0.01
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"

        self.odom_calculation_rate_Hz = params["odom_calculation_rate_Hz"]
        self.control_rate_Hz = params["control_rate_Hz"]

        self.axis_vel = [0,0]

        self.vl_rpm, self.vr_rpm = 0, 0
        self.vl_rpm_input, self.vr_rpm_input = 0, 0
        self.vl_ramp_rpm, self.vr_ramp_rpm = 0, 0

        self.last_odom_time = self.get_clock().now().to_msg()
        # ------------------------------------------------

        # ------------ Async process resource ------------
        self.mutex = Lock()
        self.odrive_active          = False
        self.cmd_vel_alive          = False

        self.first = 0

        # Create a timer to constantly retrigger the watchdog timeout
        # while configuring the ODrive and engaging.
        # Afterwards, it will be destroyed.
        self.preconfig_watchdog_feed_timer = self.create_timer(params["odrive_watchdog_timeout"]/2, self.preconfig_watchdog_feed)
        
        # Create a one-shot timer to send zero velocity command upon impending
        # watchdog timeout caused by timeout in self.cmd_callback.
        self.antislip_enable = params["antislip_enable"]
        if(self.antislip_enable):
            self.timeout_antislip_timer = self.create_timer(params["odrive_watchdog_timeout"]/2, self.timeout_antislip)
            self.timeout_antislip_timer.cancel()
        
        self.check_odrive_routine_timer = self.create_timer(1.0, self.check_odrive_routine)

        self.calculate_odom_timer = self.create_timer(1.0/self.odom_calculation_rate_Hz, self.calculate_odom)

        self.update_diagnostics_timer = self.create_timer(1.0, self.update_diagnostics)
        # ------------------------------------------------
        

    def connect(self, attempt=None) -> bool:
        if attempt == None:
            attempt = self.CONNECTION_ATTEMPT

        self.preconfig_watchdog_feed_timer.reset()
        for i in range(attempt):
            if not self.odrive.connect():
                break

            if not self.odrive.engage():
                self.odrive.reboot()
                break

            self.odrive_active  = True
            self.preconfig_watchdog_feed_timer.cancel()
            return True

        return False

    def timeout_antislip(self, period_ms=2000):
        
        self.mutex.acquire()
        try:
            self.cmd_vel_alive         = False

            rate = self.create_rate(2.0/self.odrive.odrive_watchdog_timeout)
            cycle = period_ms/(self.odrive.odrive_watchdog_timeout*1000)
            
            self.get_logger().warn("%s timeout detected! Start antislip..."%(self.cmd_vel_sub.topic_name))
            for i in range(int(cycle)):
                self.odrive.drive(0, 0)
                rate.sleep()

            self.timeout_antislip_timer.cancel()

        finally:
            self.mutex.release()
        

    def preconfig_watchdog_feed(self):
        if self.odrive_active:
            return

        self.mutex.acquire()
        try:
            self.odrive.watchdog_feed()
        finally:
            self.mutex.release()

    def error_diagnostic_callback(self, stat):
        if not self.odrive_active:
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
        if not self.odrive_active:
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
        stat.add("Battery voltage", str(self.battery[0]))
        stat.add("Battery current", str(self.battery[1]))

        return stat

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

        if wheel_direction:
            vr_rpm = -vr_rpm
        else:
            vl_rpm = -vl_rpm  
     
        return vl_rpm, vr_rpm

    def cmd_callback(self, cmd) -> None:
        self.cmd_vel_alive         = True 
        if not self.odrive_active:
            return
        
        
        self.mutex.acquire()
        try:
            if(self.antislip_enable):
                self.timeout_antislip_timer.reset()
                
            self.first = self.get_clock().now().to_msg().nanosec
            # Convert velocity command to axis velocity rpm
            self.vl_rpm_input, self.vr_rpm_input = self.convert_to_wheel_vel(cmd.linear.x, cmd.angular.z)

            self.odrive.drive(left_vel_rpm  = self.vl_rpm_input, \
                            right_vel_rpm = self.vr_rpm_input)
        finally:
            self.mutex.release()

    def calculate_odom(self) -> None:

        axis0_vel, axis1_vel = 0, 0

        if(self.mutex.acquire(False)):
        # try:
            axis_vel = self.odrive.get_estimate_vel()
            
            if axis_vel:
                self.axis_vel[0], self.axis_vel[1] = axis_vel
                axis0_vel, axis1_vel = self.axis_vel
            
        # finally:
            self.mutex.release()

        if(self.odrive.wheel_direction):
            axis1_vel = -axis1_vel
        else:
            axis0_vel = -axis0_vel

        current_time = self.get_clock().now().to_msg()
        dt = current_time.sec - self.last_odom_time.sec
        
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
            current_theta = normalize_angle(current_theta)

            self.current_pose.orientation.z = np.sin(current_theta/2.0)
            self.current_pose.orientation.w = np.cos(current_theta/2.0)

        self.last_odom_time = current_time

        self.odom.header.stamp = current_time
        self.odom.pose.pose    = self.current_pose

        self.odom.twist.twist.linear.x = linear_vel
        self.odom.twist.twist.angular.z = angular_vel

        self.odom_pub.publish(self.odom)
    
    def update_diagnostics(self):
        self.diag_updater.update()
        
    def check_odrive_routine(self) -> None:
        self.mutex.acquire()
        try:
            if (not self.odrive.driver) and self.cmd_vel_alive:
                self.connect(attempt=1)

            elif self.axis_error[0] != 0 or self.axis_error[1] != 0:
                self.odrive_active  = False

                self.odrive.reboot()

        finally:
            self.mutex.release()

    def destroy_node(self):
        self.preconfig_watchdog_feed_timer.destroy()
        self.check_odrive_routine_timer.destroy()
        self.update_diagnostics_timer.destroy()
        self.calculate_odom_timer.destroy()
        self.odrive.idle()

        super().destroy_node()
    

def main(args=None):

    rclpy.init(args=args)
    
    odrive_node = ODriveNode()

    # ros_executor = MultiThreadedExecutor(num_threads=2)
    # ros_executor.add_node(odrive_node)

    # try:
    #     ros_executor.spin()
    # except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    #     ros_executor.shutdown()
    #     odrive_node.destroy_node()

    rclpy.spin(odrive_node)
    odrive_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

