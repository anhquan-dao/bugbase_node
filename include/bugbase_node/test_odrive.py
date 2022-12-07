#!/usr/bin/python3
import serial
from serial.serialutil import SerialException

import sys
import time
import logging
import traceback

import odrive
from odrive.enums import *

import fibre
# from fibre import ChannelBrokenException, ChannelDamagedException

default_logger = logging.getLogger(__name__)
default_logger.setLevel(logging.DEBUG)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)

default_logger.addHandler(ch)


class ODriveInterface:

    class ODriveError:
        def __init__(self):
            self.axis_error = 0
            self.encoder_error = 0
            self.motor_error = 0
            self.controller_error = 0

        def __eq__(self, other):
            if type(other) != int:
                return False

            return self.axis_error \
                + self.encoder_error \
                + self.motor_error \
                + self.controller_error == other

        def __ne__(self, other):
            if type(other) != int:
                return False

            return self.axis_error \
                + self.encoder_error \
                + self.motor_error \
                + self.controller_error != other

        def axis_error_name(self, axis_error=None):
            if axis_error:
                return AxisError(axis_error)

            return AxisError(self.axis_error)

        def encoder_error_name(self, encoder_error=None):
            if encoder_error:
                return EncoderError(encoder_error)
                
            return EncoderError(self.encoder_error)

        def motor_error_name(self, motor_error=None):
            if motor_error:
                return MotorError(motor_error)
                
            return MotorError(self.motor_error)

        def controller_error_name(self, controller_error=None):
            if controller_error:
                return ControllerError(controller_error)
                
            return ControllerError(self.controller_error)



    def __init__(self, params = dict(), logger = None):

        self.__default_params = {"enable_watchdog": True,
                                "odrive_watchdog_timeout": 1.0,
                                "wheel_direction": False,
                                "turn_direction": False,
                                "base_width": 0.5,
                                "rounds_per_meter": 1.0}

        for default_key in self.__default_params.keys():
            if(default_key not in params.keys()):
                params[default_key] = self.__default_params[default_key]


        self.driver = None
        self.axes = None

        if logger:
            self.logger = logging.setLoggerClass(logger)
        else:
            self.logger = default_logger

        self.odrive_watchdog_timeout = params["odrive_watchdog_timeout"]

        self.odrive_axis_error = (self.ODriveError(), self.ODriveError())

        self.wheel_direction = params["wheel_direction"]
        self.turn_direction = params["turn_direction"]

        self.base_width = float(params["base_width"])
        self.rounds_per_meter = float(params["rounds_per_meter"])

        self.ticks_per_round = None
        pass

    def __del__(self):
        self.idle()

    def connect(self):

        self.logger.info("Try connecting to ODrive...")
        if self.driver:
            self.logger.info("Already connected. Disconnecting and reconnecting.")
            return True
        try:
            self.driver = odrive.find_any(timeout=30)
        except KeyboardInterrupt:
            sys.exit()
        except:
            self.logger.error("Error connecting to ODrive!")
            return False

        self.axes = (self.driver.axis0, self.driver.axis1)
        if(self.axes[0].error != 0 or self.axes[1].error != 0):
            self.logger.error("Axis0 error: %d. Axis1 error: %d" %(self.axes[0].error, self.axes[1].error))
            self.reboot()
            return False

        if not self.idle():
            self.driver = None
            return False

        if(self.axes[0].config.enable_watchdog == False \
        or self.axes[1].config.enable_watchdog == False):
        # or abs(self.axes[0].config.watchdog_timeout - self.odrive_watchdog_timeout) > 1e-3 \
        # or abs(self.axes[1].config.watchdog_timeout - self.odrive_watchdog_timeout) > 1e-3):

            self.logger.info("Enable watchdog timeout")
            for axis in self.axes:
                axis.config.enable_watchdog = True
                axis.config.watchdog_timeout = self.odrive_watchdog_timeout

            self.driver.save_configuration()
            self.driver = None
            return False

        else:
            self.logger.info("Watchdog has already been enabled")

        self.ticks_per_round = (self.axes[0].encoder.config.cpr, \
                                self.axes[1].encoder.config.cpr)
        
        self.watchdog_feed()

        return True
       

    def engage(self):
        if(self.driver and self.axes):
            self.logger.info("Try to engage ODrive")
            for axis in self.axes:
                axis.controller.input_vel = 0.0
                axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

            self.watchdog_feed()  

            if(self.axes[0].current_state != AXIS_STATE_CLOSED_LOOP_CONTROL \
            or self.axes[1].current_state != AXIS_STATE_CLOSED_LOOP_CONTROL):
                
                self.logger.error("Cannot set state of axis")
                self.reboot()
                
                return False

            self.logger.info("Successfully engaging ODrive")
            return True
        
        return False

    def idle(self):
        if self.driver and self.axes:
            for axis in self.axes:
                axis.requested_state = AXIS_STATE_IDLE
                axis.controller.input_vel = 0.0

            if self.axes[0].current_state != AXIS_STATE_IDLE \
            or self.axes[1].current_state != AXIS_STATE_IDLE:
                self.logger.error("Cannot set ODrive to IDLE state")
                self.logger.error("State of axis0: %d. State of axis1: %d" %(self.axes[0].current_state, self.axes[1].current_state))
                return False

            self.logger.info("Set axes to IDLE state")
            return True

        return False

    def reboot(self):
        if not self.driver:
            self.logger.error("Not connected")
            return False

        try:
            self.driver.reboot()
        except fibre.libfibre.ObjectLostError:
            self.logger.error("Rebooted ODrive")
            self.driver = None
        except Exception as ex:
            self.logger.error("Failed to reboot due to excpetion of %s. Argument: \n%s" %(type(ex).__name__, ex))
        finally:
            self.driver = None
            self.axes   = None
        
        return True

    def watchdog_feed(self):
        if (not self.driver) or (not self.axes):
            return False
        
        self.axes[0].watchdog_feed()
        self.axes[1].watchdog_feed()

        return True

    def drive(self, left_vel_m_s, right_vel_m_s):
        if (not self.driver) or (not self.axes):
            return False

        self.axes[0].controller.input_vel = left_vel_m_s
        self.axes[1].controller.input_vel = right_vel_m_s
    
        return self.watchdog_feed()

    def velocity_command_callback(self, linear_vel, angular_vel):
        if (not self.driver) or (not self.axes):
            return False

        if self.turn_direction:
            vr = linear_vel - angular_vel * self.base_width / 2.0
            vl = linear_vel + angular_vel * self.base_width / 2.0
        else:
            vr = linear_vel + angular_vel * self.base_width / 2.0
            vl = linear_vel - angular_vel * self.base_width / 2.0

        self.vr_ticks = float(vr * self.rounds_per_meter)
        self.vl_ticks = float(vl * self.rounds_per_meter)
        ##########################################

        if self.wheel_direction:
            self.vr_ticks = -self.vr_ticks
        else:
            self.vl_ticks = -self.vl_ticks  

        self.logger.info("Left wheel speed: %f. Right wheel speed: %f" % (self.vl_ticks, self.vr_ticks))       
            
        return self.drive(self.vl_ticks, self.vr_ticks)

    def get_ticks(self):
        if (not self.driver) or (not self.axes):
            return False

        axis0_tick = self.axes[0].encoder.shadow_count
        axis1_tick = self.axes[1].encoder.shadow_count

        return (axis0_tick, axis1_tick)

    def get_estimate_vel(self):
        if (not self.driver) or (not self.axes):
            return False

        axis0_estimate_vel = self.axes[0].encoder.vel_estimate
        axis1_estimate_vel = self.axes[1].encoder.vel_estimate

        return (axis0_estimate_vel, axis1_estimate_vel)

    def get_battery(self):
        if self.driver and self.axes:
            vbus_voltage = self.driver.vbus_voltage
            ibus         = self.driver.ibus

            return (vbus_voltage, ibus)

        return False


    def get_error(self):
        if (not self.driver) or (not self.axes):
            return False
        
        for i, axis in enumerate(self.axes):
            self.odrive_axis_error[i].axis_error       = axis.error
            self.odrive_axis_error[i].encoder_error    = axis.encoder.error
            self.odrive_axis_error[i].motor_error      = axis.motor.error
            self.odrive_axis_error[i].controller_error = axis.controller.error

        return self.odrive_axis_error

 

if __name__ == "__main__":

    driver = ODriveInterface()

    while True:
        if driver.driver:
            print("OK")
            driver.velocity_command_callback(1.0, 0.5)
            time.sleep(0.5)
            continue

        else:
            if driver.connect():
                driver.engage()

            else:
                time.sleep(0.5)




