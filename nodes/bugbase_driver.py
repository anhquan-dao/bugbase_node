#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import serial
import time
from datetime import datetime


NORMAL_MODE = 0
DECEL_MODE = 1
BRAKE_MODE = 2

class ESP32BugBase:

	def __init__(self, port, baud, timeout, base_width, ticks_per_meter, left_inverted, right_inverted, acceleration):
		self.port = port
		self.baud = baud
		self.timeout = timeout

		self.LEFT_INVERTED = left_inverted
		self.RIGHT_INVERTED = right_inverted

		self.BASE_WIDTH = base_width
		self.TICKS_PER_METER = ticks_per_meter

		self.ACCELERATION = acceleration

		self.encoder_msg_cnt = 0
		self.speed_msg_cnt = 0
		self.accel_msg_cnt = 0
	
		self.first_encoder_msg = True

		self.error_count = 0
		
		# Differentiate acceleration and deceleration following Khue's suggestion
		self.prev_vr_ticks = 0
		self.prev_vl_ticks = 0

	class Cmd():
		SETSPEED     = 0x4D01
		SETACCEL     = 0x4D67

		READSPEED    = 0x6f57

	def connect(self):
		try:
			self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
		except:
			return False

		return True

	def send_command(self, cmd):
		self.ser.write(chr((cmd>>8)&0xff))
		self.ser.write(chr(cmd&0xff))

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
		# time.sleep(0.01)
		# print(hex(val&0xff))

	def writelong(self, cmd, msg_cnt, val1, val2, val3, val4):
		# print(hex(val1) + " " + hex(val2) + " " + hex(val3) + " " + hex(val4))
		try:
			self.send_command(cmd)
			
			self.writebyte(msg_cnt)
			self.writebyte(val1)
			self.writebyte(val2)
			self.writebyte(val3)
			self.writebyte(val4)
		except:
			return False

		return True

	def write6bytes(self, cmd, msg_cnt, val1, val2, val3, val4, val5, val6):
		# print(hex(val1) + " " + hex(val2) + " " + hex(val3) + " " + hex(val4))
		try:
			self.send_command(cmd)
			
			self.writebyte(msg_cnt)
			self.writebyte(val1)
			self.writebyte(val2)
			self.writebyte(val3)
			self.writebyte(val4)
			self.writebyte(val5)
			self.writebyte(val6)
		except:
			return False

		return True

	def setSpeed(self, vr_ticks, vl_ticks):
		# print("{} {}".format(hex((vr_ticks>>8)&0xff), \
		# 					 hex(vr_ticks&0xff)))

		# print("{} {}".format(hex((vl_ticks>>8)&0xff), \
		# 					 hex(vl_ticks&0xff)))

		
		# print(self.speed_msg_cnt&0xff)
		self.writelong(self.Cmd.SETSPEED, self.speed_msg_cnt,\
										  (vr_ticks>>8)&0xff,\
										  (vr_ticks&0xff),\
										  (vl_ticks>>8)&0xff,\
										  (vl_ticks&0xff))
		self.speed_msg_cnt += 1
	
	def setAcceleration(self, acceleration):
		
		self.writelong(self.Cmd.SETACCEL, self.accel_msg_cnt,\
										  (acceleration>>8)&0xff,\
			                              (acceleration&0xff),\
										  (acceleration>>8)&0xff,\
										  (acceleration&0xff))
		self.accel_msg_cnt += 1

	def waitForHeader(self):
		header1, header2 = (0,0), (0,0)
		while True:
			header2 = self.readbyte()
			
			if(header1[1] == 0x6f and header2[1] == 0x57):
				return True

			header1 = header2

	def readSpeed(self):
		global SELFTEST

		while self.ser.in_waiting <= 7:
			pass

		if not self.waitForHeader():
			print("No encoder header found!!!")
			return 0x10000, 0x10000
		# print(datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])
		msg_cnt = self.readbyte()
		# print(datetime.utcnow().strftime('%H:%M:%S.%f')[:-3] + " Encoder message counter: " + str(msg_cnt[1]))
		if(msg_cnt[0]):
			if self.first_encoder_msg:
				self.encoder_msg_cnt = msg_cnt[1]
				self.first_encoder_msg = False
			elif(msg_cnt[1] == (self.encoder_msg_cnt+1)&0xff):
				self.encoder_msg_cnt += 1
			else:
				self.error_count += 1
				print("Encoder message counter fail!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
				# input()

				self.first_encoder_msg = True
				return 0x10000, 0x10000

			data = self.read4()
			if(data[0]):
				speed_1 = (data[1]>>16)&0xffff
				speed_2 = data[1]&0xffff

				# Convert 2bytes signed to 4bytes signed
				if(speed_1 & 0x8000):
					speed_1 = -0x10000 + speed_1
				if(speed_2 & 0x8000): 
					speed_2 = -0x10000 + speed_2

				return speed_1, speed_2

			# else:
			# 	return 0x10000, 0x10000
				
	
		return 0x0000, 0x0000
	
	def inv_kinematics(self, linear_x, angular_z):
		vr = linear_x + angular_z * self.BASE_WIDTH / 2.0
		vl = linear_x - angular_z * self.BASE_WIDTH / 2.0

		vr_ticks = int(vr * self.TICKS_PER_METER)
		vl_ticks = int(vl * self.TICKS_PER_METER)
		##########################################
		
		if self.RIGHT_INVERTED:
			vr_ticks = -vr_ticks
		if self.LEFT_INVERTED:
			vl_ticks = -vl_ticks
			
		return vr_ticks, vl_ticks
	
	def __del__(self):
		self.ser.close()
