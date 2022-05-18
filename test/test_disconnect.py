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

class TestDisconnect(unittest.TestCase):

    def test_disconnect(self):
        for i in range(10):
            with self.subTest(i=i):

                self.stepper = ESP32BugBase(default_params)
                self.assertTrue(self.stepper.connect())
                
                time.sleep(0.01)

                self.assertTrue(self.stepper.sendShutdownRequest())
                self.assertEqual(self.stepper.ser.in_waiting, 0)

                print("---------------------------")


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun("bugbase_node", "test_disconnect", TestDisconnect)