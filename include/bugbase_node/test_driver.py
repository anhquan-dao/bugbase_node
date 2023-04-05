#!/usr/bin/python
# -*- coding: utf-8 -*-
import serial
import time
import struct
import numpy as np

import bugbase_node.esp32_message_header as ESP32Header
from bugbase_node.bugbase_driver import ESP32BugBase
from base_mcu_interface.base_driver import BaseDriver

class TestDriver(BaseDriver):
	def __init__(self, params=dict()):
		BaseDriver.__init__(self, params)

		self.__default_params = {"accel": 1.0,
								"decel": 1.5,
								"brake": 3.0,
								"soft_accel": 0.5,
								"soft_decel": 1.2,
								"accel_boundary_speed": 2.0,
								"decel_boundary_speed": 3.0,
								"base_width": 0.55,
								"ticks_per_meter": 3965,
								"motor_enc_ratio": 1.3333,
								"left_inverted": False,
								"right_inverted": False,
								"turn_direction": True,
								"update_rate": 30,
								"no_encoder_operation": False}

		for default_key in self.__default_params.keys():
			if(default_key not in params.keys()):
				params[default_key] = self.__default_params[default_key]

		self.BASE_WIDTH = params["base_width"]
		self.TICKS_PER_METER = params["ticks_per_meter"]
		self.MOTOR_ENC_RATIO = params["motor_enc_ratio"]

		self.LEFT_INVERTED = params["left_inverted"]
		self.RIGHT_INVERTED = params["right_inverted"]
		self.TURN_DIRECTION = params["turn_direction"]

		self.ACCEL = params["accel"]
		self.DECEL = params["decel"]
		self.BRAKE = params["brake"]
		self.SOFT_ACCEL = params["soft_accel"]
		self.SOFT_DECEL = params["soft_decel"]
		self.ACCEL_BOUNDARY_SPEED = params["accel_boundary_speed"]
		self.DECEL_BOUNDARY_SPEED = params["decel_boundary_speed"]
		
		self.UPDATE_RATE = float(params["update_rate"])

		self.MSG_HEADER = ESP32Header.ROSInterfaceMessageHeader()
	
		self.float_data = 0.0

		try:
			while True:
				self.readString()
		except UnicodeDecodeError:
			pass  

		self.writeMessage(self.MSG_HEADER.SOFT_RESET & 0xff, \
			write_method=self.writeNone)    
		while True:
			error = self.readMessage(data_cb=self.data_callback)

			if(error == -5):
				return -1

			if(error == 1):
				# Send Acceleration config
				self.writeMessage(0x56, self.ACCELERATION[0])
				self.writeMessage(0x57, self.ACCELERATION[1])
				self.writeMessage(0x58, self.ACCELERATION[2])

				# Send Update period
				self.writeMessage(0x59, self.UPDATE_PERIOD)


	def setAcceleration(self, accel=None, decel=None, brake=None):
		if accel == None:
			accel = self.ACCEL
		if decel == None:
			decel = self.DECEL
		if brake == None:
			brake =  self.BRAKE

		return self.writeMessage(self.MSG_HEADER.WRITE_ACCEL_PROFILE_CFG, \
			accel, decel, brake)

	def setDynamicAcceleration(self, soft_accel=None, soft_decel=None, \
		accel_boundary_speed=None, decel_boundary_speed=None):

		if(soft_accel == None):
			soft_accel = self.SOFT_ACCEL

		if(soft_decel == None):
			soft_decel = self.SOFT_DECEL

		if(accel_boundary_speed == None):
			accel_boundary_speed = self.ACCEL_BOUNDARY_SPEED

		if(decel_boundary_speed == None):
			decel_boundary_speed = self.DECEL_BOUNDARY_SPEED

		return self.writeMessage(self.MSG_HEADER.WRITE_DYNAMIC_ACCEL_CFG, \
			 soft_accel, soft_decel, accel_boundary_speed, decel_boundary_speed)
	
	def setUpdateRate(self, update_rate = None):
		if(update_rate == None):
			update_rate = self.UPDATE_RATE

		return self.writeMessage(self.MSG_HEADER.WRITE_UPDATE_RATE_CFG, \
			update_rate)

	def data_callback(self, header, data):
		self.float_data = self.parseFloat()
		if header == 0x95:
			print(str(time.time()) + " Message " + str(hex(header)) + ": " + str(self.float_data))
		
	def error_callback(self, error):
		if(error != 0):
			print("Test pass method as argument")
			pass

