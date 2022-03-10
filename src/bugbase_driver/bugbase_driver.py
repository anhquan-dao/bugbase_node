#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import serial
import time

class ESP32BugBase:

	def __init__(self, port, baud, timeout, base_width, ticks_per_meter):
		self.port = port
		self.baud = baud
		self.timeout = timeout

		self.BASE_WIDTH = base_width
		self.TICKS_PER_METER = ticks_per_meter
	
	class Cmd():
		GETSPEED = 0
		SETSPEED = 1

	def connect(self):
		try:
			self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
		except:
			return False

		return True

	def send_command(self, cmd):
		self.ser.write(chr(cmd))

	def readbyte(self):
		data = self.ser.read(1)
		if len(data):
			val = ord(data)
			return (1,val)
		return (0,0)

	def read4(self):
		val1 = self.readbyte()
		if val1[0]:
			val2 = self.readbyte()
			if val2[0]:
				val3 = self.readbyte()
				if val3[0]:
					val4 = self.readbyte()
					if val4[0]:
						return (1,val1[1]<<24|val2[1]<<16|val3[1]<<8|val4[1])
		return (0,0)

	def writebyte(self, val):
		self.ser.write(chr(val&0xff))

	def writelong(self, cmd, val1, val2, val3, val4):
		try:
			self.send_command(cmd)
			self.writebyte(val1)
			self.writebyte(val2)
			self.writebyte(val3)
			self.writebyte(val4)
		except:
			return False

		return True

	def setSpeed(self, vr_ticks, vl_ticks):
		self.writelong(self.Cmd.SETSPEED, (vl_ticks>>8)&0xff,\
										  (vl_ticks&0xff),\
										  (vr_ticks>>24)&0xff,\
										  (vr_ticks>>16)&0xff)

	def readSpeed(self):
		self.ser.flushInput()
		self.send_command()

		data = self.read4()
		speed_1 = ((data>>8)&0xff) | (data&0xff)
		speed_2 = ((data>>24)&0xff) | ((data>>16)&0xff)

		return speed_1, speed_2

	def send_cmd(self, vr_ticks, vl_ticks):
		# cmd = "<" + "{:0.2f}".format(vl_ticks).zfill(9)  + "|" + \
	    #   "{:0.2f}".format(vr_ticks).zfill(9) + ">"
		# self.ser.write(cmd.encode())

		pass

	def inv_kinematics(self, linear_x, angular_z):
		vr = linear_x + angular_z * self.BASE_WIDTH/2.0
		vl = linear_x - angular_z * self.BASE_WIDTH/2.0

		vr_ticks = int(vr * self.TICKS_PER_METER)
		vl_ticks = int(vl * self.TICKS_PER_METER)

		return vr_ticks, vl_ticks
	
	def __del__(self):
		self.ser.close()
