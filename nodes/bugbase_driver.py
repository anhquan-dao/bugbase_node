#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import serial
import time

class ESP32BugBase:

	def __init__(self, port, baud, timeout, base_width, ticks_per_meter, left_inverted, right_inverted):
		self.port = port
		self.baud = baud
		self.timeout = timeout

		self.LEFT_INVERTED = left_inverted
		self.RIGHT_INVERTED = right_inverted

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
		self.ser.write(chr(cmd&0xff))

	def readbyte(self):
		data = self.ser.read(1)
		if len(data):
			val = ord(data)
			return (1,val)
		return (0,0)

	def read4(self):
		zeroByte = (0,0)
		val1 = self.readbyte()
		zeroByte = self.readbyte()
		if val1[0] and zeroByte[1] == 0:
			val2 = self.readbyte()
			zeroByte = self.readbyte()
			if val2[0] and zeroByte[1] == 0:
				val3 = self.readbyte()
				zeroByte = self.readbyte()
				if val3[0] and zeroByte[1] == 0:
					val4 = self.readbyte()
					zeroByte = self.readbyte()
					if val4[0] and zeroByte[1] == 0:
						self.readbyte()
						return (1,val1[1]<<24|val2[1]<<16|val3[1]<<8|val4[1])
		return (0,0)

	def writebyte(self, val):
		self.ser.write(chr(val&0xff))

	def writelong(self, cmd, val1, val2, val3, val4):
		try:
			self.writebyte(77);
			self.send_command(cmd)
			self.writebyte(0)
			self.writebyte(val1)
			self.writebyte(0)
			self.writebyte(val2)
			self.writebyte(0)
			self.writebyte(val3)
			self.writebyte(0)
			self.writebyte(val4)
		except:
			return False

		return True

	def setSpeed(self, vr_ticks, vl_ticks):
		# print("{} {}".format(hex((vr_ticks>>8)&0xff), \
		# 					 hex(vr_ticks&0xff)))

		# print("{} {}".format(hex((vl_ticks>>8)&0xff), \
		# 					 hex(vl_ticks&0xff)))

		self.writelong(self.Cmd.SETSPEED, (vl_ticks>>8)&0xff,\
										  (vl_ticks&0xff),\
										  (vr_ticks>>8)&0xff,\
										  (vr_ticks&0xff))

	def waitForHeader(self):
		header1, header2 = (0,0), (0,0)

		while True:
			header2 = self.readbyte()
			if(header1[1] == 0x6f and header2[1] == 0x57):
				break
			header1 = header2

	def readSpeed(self):
		self.waitForHeader()
		data = self.read4()
		if(data[0]):
			speed_1 = (data[1]>>16)&0xffff
			speed_2 = data[1]&0xffff
			# print(hex(speed_1))
			# print(hex(speed_2))

			return speed_1, speed_2

		return 0xffff, 0xffff

	def inv_kinematics(self, linear_x, angular_z):
		vr = linear_x - angular_z * self.BASE_WIDTH/2.0
		vl = linear_x + angular_z * self.BASE_WIDTH/2.0

		vr_ticks = int(vr * self.TICKS_PER_METER)
		vl_ticks = int(vl * self.TICKS_PER_METER)
		
		if self.RIGHT_INVERTED:
			vr_ticks = -vr_ticks
		if self.LEFT_INVERTED:
			vl_ticks = -vl_ticks
			
		return vr_ticks, vl_ticks
	
	def __del__(self):
		self.ser.close()
