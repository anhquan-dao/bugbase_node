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

class TestWeirdFloat(unittest.TestCase):

    def setUp(self):
        self.stepper = ESP32BugBase(default_params)
        self.assertTrue(self.stepper.connect())

    def test_weird_float(self):

        test_time = 5.0

        timeout_error_cnt = 0

        time.sleep(0.08)
        start = time.time()

        while time.time()-start < test_time:
            read_what = self.stepper.waitForHeader()
            if read_what == 1:
                data = self.stepper.readSpeedProfile()

                self.assertEqual(data[-1], 0)
                self.assertEqual(data[-2], 0)

            if read_what >= 10:
                timeout_error_cnt += 1
                if timeout_error_cnt > 5:
                    self.fail("Timeout error")

    def test_no_comm(self):

        test_time = 5.0

        timeout_error_cnt = 0

        time.sleep(0.08)
        start = time.time()

        while time.time()-start < test_time:
            read_what = self.stepper.waitForHeader()
            if read_what == 3:
                data = self.stepper.readbyte()

                if(data[0]):
                    # print(data)
                    self.assertLessEqual(data[1], 5)

            if read_what >= 10:
                timeout_error_cnt += 1
                if timeout_error_cnt > 5:
                    self.fail("Timeout error")

    def tearDown(self):
        self.assertTrue(self.stepper.sendShutdownRequest())
        self.assertEqual(self.stepper.ser.in_waiting, 0)
        del self.stepper

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun("bugbase_node", "test_weird_float", TestWeirdFloat)