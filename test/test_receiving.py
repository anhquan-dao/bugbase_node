#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import unittest
import time

from bugbase_node.bugbase_driver import ESP32BugBase


default_params = {
            'port' : '/dev/ttyUSB0',
            'baud' : 115200,
            'timeout' : 0.1,
            'base_width' : 0.52,
            'ticks_per_meter' : 30000,
            'left_inverted' : False,
            'right_inverted' : True,
            'turn_direction' : True,
            'accel_profile' : [0.4, 0.8, 1.0, 2.0],
            'max_acceleration' : 3.0,
            'decel_time_limit' : 200,
            'accel_time_limit' : 500,
            'update_period' : 25.0,
            'update_rate' : 40,
            'no_encoder_operation' : False,
            'left_enc_inverted' : False,
            'right_enc_inverted' : False,
            'wheel_enc_ratio' : 1.3333,
            'odom_topic' : '/odometry/wheel',
            'visualizer' : False
        }

class TestConnectionEstablish(unittest.TestCase):

    def setUp(self):
        self.stepper = ESP32BugBase(default_params)

    def test_connection(self):
        self.assertTrue(self.stepper.connect())

    def tearDown(self):
        self.assertTrue(self.stepper.sendShutdownRequest())
        del self.stepper


class TestMessageTransimission(unittest.TestCase):
    
    def setUp(self):
        self.stepper = ESP32BugBase(default_params)
        self.assertTrue(self.stepper.connect())

    def test_params_read(self):
        self.assertEqual(self.stepper.UPDATE_PERIOD, 1000.0/default_params['update_rate'])
        
        acceleration_profile_ticks = default_params['accel_profile'][:]
        for i in range(len(acceleration_profile_ticks)):
            acceleration_profile_ticks[i] *= default_params['ticks_per_meter']
            acceleration_profile_ticks[i] = int(acceleration_profile_ticks[i])
        self.assertEqual(self.stepper.ACCELERATION, acceleration_profile_ticks)

        self.assertEqual(self.stepper.port, default_params["port"])

        self.assertEqual(self.stepper.baud, default_params["baud"])

        self.assertEqual(self.stepper.timeout, default_params["timeout"])

        self.assertEqual(self.stepper.LEFT_INVERTED, default_params["left_inverted"])
        self.assertEqual(self.stepper.RIGHT_INVERTED, default_params["right_inverted"])
        self.assertEqual(self.stepper.BASE_WIDTH, default_params["base_width"])
        self.assertEqual(self.stepper.TICKS_PER_METER, default_params["ticks_per_meter"])
        self.assertEqual(self.stepper.MAX_ACCELERATION, default_params["max_acceleration"]* default_params['ticks_per_meter'])
        self.assertEqual(self.stepper.ACCEL_TIME, default_params["accel_time_limit"])
        self.assertEqual(self.stepper.DECEL_TIME, default_params["decel_time_limit"])
    
    def test_params_loading(self):
        self.stepper.send_command(self.stepper.MSG_HEADER.READ_PARAMS)

        while True:
            read_what = self.stepper.waitForHeader()
            if read_what == 2:
                expected_data = str(self.stepper.ACCELERATION[0]) + " " +\
                    str(self.stepper.ACCELERATION[1]) + " " +\
                    str(self.stepper.ACCELERATION[2]) + " " +\
                    str(self.stepper.ACCELERATION[3]) + " " +\
                    str("{:.2f}".format(self.stepper.UPDATE_PERIOD)) + "\r"
                # print(default_params['accel_profile'])
                self.assertEqual(self.stepper.readString(), expected_data)
                break

    def tearDown(self):
        self.assertTrue(self.stepper.sendShutdownRequest())
        self.assertEqual(self.stepper.ser.in_waiting, 0)
        
        del self.stepper

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('bugbase_node', 'test_bare_bones', TestConnectionEstablish)
    rosunit.unitrun('bugbase_node', 'test_bare_bones', TestMessageTransimission)


