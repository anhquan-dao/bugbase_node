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

class TestSpeedProfile(unittest.TestCase):

    def setUp(self):
        self.stepper = ESP32BugBase(default_params)
        self.assertTrue(self.stepper.connect())       

    def test_speed_profile(self):
        test_time = 2.0

        accel0, accel1 = 0.0, 0.0
        est_speed0, est_speed1 = 0.0, 0.0
        timeout_error_cnt = 0

        start = time.time()
        dt_timer = time.time()

        while time.time()-start < test_time:
            self.stepper.setSpeed(10000, 10000)
            read_what = self.stepper.waitForHeader()
            if read_what == 1:
                data = self.stepper.readSpeedProfile()

                dt = time.time() - dt_timer
                accel0, accel1 = data[:2]

                print("Loop time: " + str(dt))
                print(data)
                # print(str(data[2] - est_speed0) + " " + str(accel0 * dt))
                # print(str(data[1] - est_speed0) + " " + str(accel0 * dt))

                # self.assertTrue(abs(data[2] - est_speed0 - accel0 * dt) < 100)
                # self.assertTrue(abs(data[3] - est_speed1 - accel1 * dt) < 100)
                
                est_speed0, est_speed1 = data[2:4]
                accel0, accel1 = data[:2]

                dt_timer = time.time()

            


    def tearDown(self):
        self.assertTrue(self.stepper.sendShutdownRequest())
        self.assertEqual(self.stepper.ser.in_waiting, 0)
        del self.stepper

if __name__ == "__main__":
    import rosunit
    # rosunit.unitrun("bugbase_node", "test_speed_profile", TestSpeedProfile)