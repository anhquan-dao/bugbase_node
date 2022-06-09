#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import unittest
import time

from threading import Thread
import numpy as np


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
            'update_period' : 16.6667,
            'update_rate' : 60,
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
        start_timer = time.time()
        self.assertTrue(self.stepper.connect())
        init_duration = time.time() - start_timer
        print(init_duration)

    def tearDown(self):
        self.assertTrue(self.stepper.sendShutdownRequest())
        del self.stepper

class TestDisconnect(unittest.TestCase):

    def test_disconnect(self):
        for i in range(10):

            self.stepper = ESP32BugBase(default_params)
            self.assertTrue(self.stepper.connect())
            
            time.sleep(0.01)

            self.assertTrue(self.stepper.sendShutdownRequest())
            self.assertEqual(self.stepper.ser.in_waiting, 0)

            print("---------------------------")


class TestMessageTransimission(unittest.TestCase):
    
    def setUp(self):
        self.stepper = ESP32BugBase(default_params)
        self.assertTrue(self.stepper.connect())
 
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
    
    def test_message_rate(self):
        
        msg_cnt = 0
        dt_timer = time.time()
        avg_dt = 0
        for i in range(100):
            read_what = self.stepper.waitForHeader()
            if(read_what == 1):
                dt = time.time() - dt_timer
                msg_cnt += 1
                avg_dt += dt
                print(dt)
                dt_timer = time.time()
        
        self.assertNotEqual(msg_cnt, 0)
        msg_cycle_period = 1000*(avg_dt)/msg_cnt
        print("Average message cycle period: " + str(msg_cycle_period))
        self.assertLessEqual(msg_cycle_period, self.stepper.UPDATE_PERIOD * 1.2)
        self.assertGreaterEqual(msg_cycle_period, self.stepper.UPDATE_PERIOD * 0.8)

    def test_speed_assignment(self):
        test_time = 5.0

        start = time.time()
        while True:
            self.stepper.setSpeed(10000, 10000)
            read_what = self.stepper.waitForHeader()
            if read_what == 1:
                data = self.stepper.readSpeedProfile()

                print(data)
                if(data[2:4] == (10000, 10000)):
                    print(time.time() - start)
                    break

            if(time.time()-start > test_time):
                self.fail("Timeout no result")

    def tearDown(self):
        self.assertTrue(self.stepper.sendShutdownRequest())
        self.assertEqual(self.stepper.ser.in_waiting, 0)
        
        time.sleep(1.0)
        del self.stepper

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('bugbase_node', 'test_bare_bones', TestConnectionEstablish)
    rosunit.unitrun('bugbase_node', 'test_bare_bones', TestMessageTransimission)


