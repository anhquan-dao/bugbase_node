#!/usr/bin/python3
import serial
from serial.serialutil import SerialException

import sys
import time
import logging
import traceback
from enum import IntEnum
import numpy as np

import odrive
from odrive.enums import *

import fibre
# from fibre import ChannelBrokenException, ChannelDamagedException
import matplotlib
import matplotlib.pyplot as plt

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
                                "odrive_watchdog_timeout": 2.0,
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
            self.watchdog_feed()
        except KeyboardInterrupt:
            sys.exit()
        except Exception as ex:
            self.logger.error("Failed to connect due to excpetion of %s. Argument: \n%s" %(type(ex).__name__, ex))
            return False

        self.axes = (self.driver.axis0, self.driver.axis1)
        if(self.axes[0].error != 0 or self.axes[1].error != 0):
            self.logger.error("Axis0 error: %d. Axis1 error: %d" %(self.axes[0].error, self.axes[1].error))
            self.reboot()
            return False

        for i in range(5):
            self.watchdog_feed()
            if self.idle():
                break
            time.sleep(0.2)
        else:
            self.driver = None
            return False

        need_reset = False

        for i, axis in enumerate(self.axes):
            self.logger.info("Try to configure axis %d" %(i))

            # Enable watchdog timer
            if(axis.config.enable_watchdog == False \
            or abs(axis.config.watchdog_timeout - self.odrive_watchdog_timeout) > 1e-3):

                self.logger.info("\tTry to enable watchdog timeout")
                axis.config.enable_watchdog = True
                axis.config.watchdog_timeout = self.odrive_watchdog_timeout

                need_reset |= True

            else:
                self.logger.info("\tWatchdog has already been enabled")

            self.logger.info("\t  Watchdog enable: %s" %(axis.config.enable_watchdog))
            self.logger.info("\t  Watchdog timeout: %s" %(axis.config.watchdog_timeout))

            # Enable ramped velocity control
            if(axis.controller.config.input_mode != INPUT_MODE_VEL_RAMP \
            or axis.controller.config.control_mode != CONTROL_MODE_VELOCITY_CONTROL):

                self.logger.info("\tTry to set controller to Ramped Velocity Control")
                axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
                axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL 

                need_reset |= True

            else:
                self.logger.info("\tController has already been set to Ramped Velocity Control")

            self.logger.info("\t  Control Mode: %s" %(ControlMode(axis.controller.config.control_mode)))
            self.logger.info("\t  Input Mode: %s" %(InputMode(axis.controller.config.input_mode)))

            # Set current control parameter
            if(abs(axis.motor.config.current_lim - 15) >  1e-3 \
            or abs(axis.motor.config.current_control_bandwidth - 100) > 1e-3 \
            or abs(axis.controller.config.torque_ramp_rate - 0.05) > 1e-3):

                self.logger.info("\tTry to configure current control")
                
                axis.motor.config.current_lim = 15
                axis.motor.config.current_control_bandwidth = 100
                axis.controller.config.torque_ramp_rate = 0.05

                need_reset |= True

            else:
                self.logger.info("\tCurrent control has already been configured") 

            self.logger.info("\t  %f" %(axis.motor.config.current_lim))
            self.logger.info("\t  %f" %(axis.motor.config.current_control_bandwidth))
            self.logger.info("\t  %f" %(axis.controller.config.torque_ramp_rate))

            self.watchdog_feed()

        if need_reset:
            self.save_configuration()
            self.logger.info("Reset to save configuration")
            return False

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
                
                self.logger.error("Cannot set axes to AXIS_STATE_CLOSED_LOOP_CONTROL")
                # self.logger.error("State of axis0: %d. State of axis1: %d" %(self.axes[0].current_state, self.axes[1].current_state))
                
                return False

            self.logger.info("Axes have been set to AXIS_STATE_CLOSED_LOOP_CONTROL")
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
                self.logger.error("Cannot set axes to AXIS_STATE_IDLE")
                # self.logger.error("State of axis0: %d. State of axis1: %d" %(self.axes[0].current_state, self.axes[1].current_state))
                return False

            self.logger.info("Axes have been set to AXIS_STATE_IDLE")
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

    def save_configuration(self):
        if not self.driver:
            self.logger.error("Not connected")
            return False

        try:
            self.driver.save_configuration()
        except fibre.libfibre.ObjectLostError:
            self.logger.error("Save ODrive configuration")
            self.driver = None
        except Exception as ex:
            self.logger.error("Failed to save configuration due to excpetion of %s. Argument: \n%s" %(type(ex).__name__, ex))
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

    def drive(self, left_vel_rpm, right_vel_rpm, left_vel_ramp_rpm=None, right_vel_ramp_rpm=None):
        if (not self.driver) or (not self.axes):
            return False

        if(left_vel_ramp_rpm):
            self.axes[0].controller.config.vel_ramp_rate = left_vel_ramp_rpm

        if(right_vel_ramp_rpm):
            self.axes[1].controller.config.vel_ramp_rate = right_vel_ramp_rpm

        self.axes[0].controller.input_vel = left_vel_rpm
        self.axes[1].controller.input_vel = right_vel_rpm
    
        return self.watchdog_feed()

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

    @staticmethod
    def start_anticogging_calibration(dev=None):
        if dev == None:
            try:
                dev = odrive.find_any(timeout=30)
            except KeyboardInterrupt:
                sys.exit()
            except Exception as ex:
                print("Failed to connect due to excpetion of %s. Argument: \n%s" %(type(ex).__name__, ex))
                return False

        axes = [dev.axis0, dev.axis1]

        print("Setting before anticogging calibration")
        print("\tDisable watchdog timer")

        dev.axis0.config.enable_watchdog = False
        dev.axis1.config.enable_watchdog = False

        try:
            dev.save_configuration()
        except fibre.libfibre.ObjectLostError:
            print("Save ODrive configuration")
        except Exception as ex:
            print("Failed to save configuration due to excpetion of %s. Argument: \n%s" %(type(ex).__name__, ex))

        dev = odrive.find_any(timeout=30)
        axes = [dev.axis0, dev.axis1]

        pos_gain = [0]*2
        vel_integrator_gain = [0]*2

        for i in range(2):
            axes[i].encoder.config.use_index = True
            axes[i].controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            axes[i].controller.config.input_mode   = INPUT_MODE_PASSTHROUGH

            pos_gain[i] = axes[i].controller.config.pos_gain
            vel_integrator_gain[i] = axes[i].controller.config.vel_integrator_gain 

            print("\tPosition gain of axis %d: %f" %(i, pos_gain[i]))
            print("\tVelocity I-gain of axis %d: %f" %(i, vel_integrator_gain[i]))

            axes[i].requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

            while axes[i].current_state != AXIS_STATE_IDLE:
                time.sleep(1)

            axes[i].controller.config.pos_gain = 1
            axes[i].controller.config.vel_integrator_gain = 0.5

            # axes[i].controller.config.anticogging.calib_pos_threshold = 2
            # axes[i].controller.config.anticogging.calib_vel_threshold = 2

        print("Start anticogging calibration")

        time.sleep(1)

        axes[0].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        axes[1].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        axes[0].controller.start_anticogging_calibration()
        axes[1].controller.start_anticogging_calibration()

        print("\tState of axis 0: %d" %(AxisState(axes[0].current_state)))
        print("\tState of axis 1: %d" %(AxisState(axes[1].current_state)))

        while axes[0].controller.config.anticogging.calib_anticogging \
        or axes[1].controller.config.anticogging.calib_anticogging:
            print("Axis 0 anticogging calibration state: %d" %(axes[0].controller.config.anticogging.calib_anticogging))
            print("\tAxis 0 at index: %d" %(axes[0].controller.config.anticogging.index))
            print("Axis 1 anticogging calibration state: %d" %(axes[1].controller.config.anticogging.calib_anticogging))
            print("\tAxis 1 at index: %d" %(axes[1].controller.config.anticogging.index))
            print("----------------------------")
            time.sleep(1)

        for i in range(2):
            axes[i].requested_state = AXIS_STATE_IDLE

            axes[i].controller.config.pos_gain = pos_gain[i]
            axes[i].controller.config.vel_integrator_gain = vel_integrator_gain[i]

    def get_error(self):
        if (not self.driver) or (not self.axes):
            return False
        
        for i, axis in enumerate(self.axes):
            self.odrive_axis_error[i].axis_error       = axis.error
            self.odrive_axis_error[i].encoder_error    = axis.encoder.error
            self.odrive_axis_error[i].motor_error      = axis.motor.error
            self.odrive_axis_error[i].controller_error = axis.controller.error

        return self.odrive_axis_error        

class ODriveRampControl:
    class RampState(IntEnum):
        WEAK_ACCELERATION   = 0
        ACCELERATION        = 1
        DECELERATION        = 3
        STRONG_DECELERATION = 2
        BRAKE               = 4
        COAST               = 5

    def __init__(self, params):
        self.__default_params = {"rounds_per_meter"        : 1.0, \
                                "weak_acceleration"        : 0.5, \
                                "acceleration"             : 0.4, \
                                "deceleration"             : 0.8, \
                                "strong_deceleration"      : 1.2, \
                                "brake"                    : 2.0, \
                                "decelration_time_limit_ms": 1000}

        for default_key in self.__default_params.keys():
            if(default_key not in params.keys()):
                params[default_key] = self.__default_params[default_key]
        
        self.current_wheel_vel = [0, 0]
        self.set_wheel_vel     = [0, 0]

        self.ramp_value = 5*[0]
        self.ramp_value[self.RampState.WEAK_ACCELERATION]   = params["weak_acceleration"] * params["rounds_per_meter"]
        self.ramp_value[self.RampState.ACCELERATION]        = params["acceleration"] * params["rounds_per_meter"]
        self.ramp_value[self.RampState.DECELERATION]        = params["deceleration"] * params["rounds_per_meter"]
        self.ramp_value[self.RampState.STRONG_DECELERATION] = params["strong_deceleration"] * params["rounds_per_meter"]
        self.ramp_value[self.RampState.BRAKE]               = params["brake"] * params["rounds_per_meter"]

        self.deceleration_timer = 2*[time.time()]
        self.deceleration_flag  = 2*[False]
        self.deceleration_duration = 2*[0]
        self.decelration_time_limit_ms = params["deceleration_time_limit_ms"]
        self.done_deceleration_flag = 2*[False]
    
    def set_current_vel(self, axis0_vel, axis1_vel):
        self.current_wheel_vel[0] = axis0_vel
        self.current_wheel_vel[1] = axis1_vel

    def set_set_vel(self, axis0_vel, axis1_vel):
        self.set_wheel_vel[0] = axis0_vel
        self.set_wheel_vel[1] = axis1_vel

    def get_ramp_state(self, axis):
        '''
        Get velocity ramp state by comparing set velocity and current velocity
        
        '''

        # If the axis is already decelerating, check if the deceleration time has exceeded
        # If yes, switch to BRAKE 
        self.deceleration_duration[axis]= time.time() - self.deceleration_timer[axis]
        if(self.deceleration_flag[axis] \
        and self.deceleration_duration[axis] > self.decelration_time_limit_ms):

            self.deceleration_flag[axis] = False
            return self.RampState.BRAKE

        if(self.current_wheel_vel[axis] == 0):
            return self.RampState.ACCELERATION

        if(self.set_wheel_vel[axis] == 0):
            return self.RampState.BRAKE

        if(self.current_wheel_vel[axis] * self.set_wheel_vel[axis] > 0):
            return self.RampState.ACCELERATION

    def get_ramp(self, axis):
        
        if(self.current_wheel_vel[axis] * self.set_wheel_vel[axis] >= 0):
            # Reset deceleration parameter
            self.deceleration_timer[axis] = time.time()

            if(abs(self.current_wheel_vel[axis] - self.set_wheel_vel[axis]) < 0.1):
                return self.RampState.COAST, self.ramp_value[self.RampState.ACCELERATION]

            if(abs(self.set_wheel_vel[axis]) > abs(self.current_wheel_vel[axis])):
                return self.RampState.ACCELERATION, self.ramp_value[self.RampState.ACCELERATION]

            if(self.set_wheel_vel[axis] == 0):
                return self.RampState.BRAKE, self.ramp_value[self.RampState.BRAKE]

            return self.RampState.DECELERATION, self.ramp_value[self.RampState.DECELERATION]
                      
        # Use sigmoid function to switch between deceleration and braking value
        deceleration_duration = time.time() - self.deceleration_timer[axis]

        theta_1 = 1.0
        z_1 = (deceleration_duration - self.decelration_time_limit_ms)/theta_1

        return self.RampState.DECELERATION, self.ramp_value[self.RampState.DECELERATION] \
            + self.ramp_value[self.RampState.BRAKE]/(1.0 + np.exp(-z_1)) 

class ODriveLivePlotter:
    def __init__(self, var_callback):
        self.var_callback = var_callback
        self.vals = []
        self.num_samples = 1000
        self.data_rate = 100
        self.plot_rate = 10

        plt.ion()
        self.fig = plt.figure()

        self.fail_timer = time.time()
        self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.pause = False

    def fetch_data(self, lock):
        if(time.time() - self.fail_timer < 1):
            return

        lock.acquire()
        try:
            data = self.var_callback()
            self.vals.append(data)
            if len(self.vals) > self.num_samples:
                self.vals = self.vals[-self.num_samples:]
            
        except Exception as ex:
            print(str(ex))  
            self.fail_timer = time.time()

        finally:
            lock.release()

    def onclick(self, event):
        self.pause = not self.pause

    def draw_data(self, event):
        if(not self.pause):
            plt.clf()
            plt.plot(self.vals)
            plt.legend(list(range(len(self.vals))))
            
        self.fig.canvas.draw()
        self.fig.canvas.start_event_loop(1/self.plot_rate)

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




