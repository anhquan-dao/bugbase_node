#!/usr/bin/python
# -*- coding: utf-8 -*-

import serial
import time
import struct

import bugbase_node.esp32_message_header as ESP32Header

class ESP32BugBase:

    def __init__(self, params):
        self.port = params["port"]
        self.baud = params["baud"]
        self.timeout = params["timeout"]

        self.is_initialized = False
 
        #######################################################
        # User-defined parameters through rosparam
        self.LEFT_INVERTED = params["left_inverted"]
        self.RIGHT_INVERTED = params["right_inverted"]
        self.TURN_DIRECTION = params["turn_direction"]

        self.BASE_WIDTH = params["base_width"]
        self.TICKS_PER_METER = params["ticks_per_meter"]
        self.MOTOR_ENC_RATIO = params["motor_enc_ratio"]

        self.ACCELERATION = params["accel_profile"][:]
        for i in range(len(self.ACCELERATION)):
            self.ACCELERATION[i] *= self.TICKS_PER_METER * self.MOTOR_ENC_RATIO
            self.ACCELERATION[i] = int(self.ACCELERATION[i])

        self.ACCEL_BOUNDARY_SPEED = int(params["accel_boundary_speed"]*self.TICKS_PER_METER*self.MOTOR_ENC_RATIO)
        self.DECEL_BOUNDARY_SPEED = int(params["decel_boundary_speed"]*self.TICKS_PER_METER*self.MOTOR_ENC_RATIO)
        
        self.MAX_ACCELERATION = int(params["max_acceleration"]*self.TICKS_PER_METER*self.MOTOR_ENC_RATIO)
        self.ACCEL_TIME = params["accel_time_limit"]
        self.DECEL_TIME = params["decel_time_limit"]
        self.UPDATE_PERIOD = params["update_period"]

        self.NO_ENCODER_OPERATION = params["no_encoder_operation"]
        ######################################################
        self.encoder_msg_cnt = 0
        self.speed_msg_cnt = 0
        self.accel_msg_cnt = 0

        self.first_encoder_msg = True

        self.error_count = 0
        self.max_header_try = 20

        self.vr_ticks, self.vl_ticks = 0, 0

        self.header1, self.header2 = (0, 0), (0, 0)
        self.MSG_HEADER = ESP32Header.ROSInterfaceMessageHeader()

        #================================
        # PLAYGROUND SECTION

        self.PLAYGROUND_ENABLE = params["playground_enable"]

        # Differentiate acceleration and deceleration following Khue's suggestion
        self.vel_timer = time.time()
        self.ANGULAR_ACCELERATION = params["angular_acceleration"]
        
        self.angular_z = 0
        self.prev_linear_x, self.prev_angular_z = 0, 0
        self.prev_vr_ticks = 0
        self.prev_vl_ticks = 0
        #================================

    class Cmd():
        SETSPEED = 0x4D01

        READSPEED = 0x6f57

        INIT = 0x7954
        INIT_ACCEL = 0x7956
        INIT_DYNAMIC_ACCEL = 0x7957
        INIT_UPDATE_RATE = 0x7958

    def connect(self):
        '''
        Establish connection with the ESP32 driver
        Update new params as per roslaunch
        '''
        
        try:
            self.ser = serial.Serial(
                self.port, self.baud, timeout=self.timeout)
            
            # time.sleep(0.1)

            # Get buffer length before sending INIT command
            # Flush all the data from the the buffer length before
            # resetting procedure
            # buffer_len = self.ser.in_waiting
            # self.ser.read_all()
            self.send_command(self.MSG_HEADER.SOFT_RESET)

            header_wait_timer = time.time()
            while True:
                read_what = self.waitForHeader()

                if read_what == 8:
                    print("Alright")
                    self.writeInitParam()
                    self.is_initialized = True
                    break
                if read_what == 9:
                    # print(self.readString())
                    pass
                if read_what == 10:
                    print("error")
                    return False
                if time.time() - header_wait_timer > 2.0:
                    print("Timeout")
                    return False

        except OSError as e:
            return False

        return True

    def readbyte(self):
        data = self.ser.read(1)
        if len(data):
            val = ord(data)
            return (1, val)
        return (0, 0)

    def readString(self):
        data = self.ser.readline().decode("utf-8")
        return data[:-1]

    def read4(self):
        val1 = self.readbyte()
        if val1[0]:
            val2 = self.readbyte()
            if val2[0]:
                val3 = self.readbyte()
                if val3[0]:
                    val4 = self.readbyte()
                    if val4[0]:
                        return (1, val1[1] << 24 | val2[1] << 16 | val3[1] << 8 | val4[1])
        return (0, 0)

    def read8(self):
        val1 = self.read4()
        if val1[0]:
            val2 = self.read4()
            if val2[0]:
                return (1, val1[1] << 32 | val2[1])
        
        return (0, 0)

    def writebyte(self, val):
        val_bytearray = bytes(bytearray((val&0xff,)))
        self.ser.write(val_bytearray)
        # time.sleep(0.01)
        # print(hex(val&0xff))
    
    def send_command(self, cmd):
        cmd_bytearray = bytes(bytearray(((cmd>>8)&0xff, cmd&0xff)))
        self.ser.write(cmd_bytearray)

    def writeshort(self, val1, val2):
        try:
            self.writebyte(val1)
            self.writebyte(val2)
        except:
            return False

        return True


    def writelong(self, val1, val2, val3, val4):
        # print(hex(val1) + " " + hex(val2) + " " + hex(val3) + " " + hex(val4))
        try:
            self.writebyte(val1)
            self.writebyte(val2)
            self.writebyte(val3)
            self.writebyte(val4)
        except:
            return False

        return True

    def writelonglong(self, val1, val2):
        try:
            self.writebyte((val1 >> 24) & 0xff)
            self.writebyte((val1 >> 16) & 0xff)
            self.writebyte((val1 >> 8) & 0xff)
            self.writebyte(val1 & 0xff)

            self.writebyte((val2 >> 24) & 0xff)
            self.writebyte((val2 >> 16) & 0xff)
            self.writebyte((val2 >> 8) & 0xff)
            self.writebyte(val2 & 0xff)
        except:
            return False

        return True

    def setSpeed(self, vr_ticks, vl_ticks):
        # print("{} {}".format(hex((vr_ticks>>8)&0xff), \
        # 					 hex(vr_ticks&0xff)))

        # print("{} {}".format(hex((vl_ticks>>8)&0xff), \
        # 					 hex(vl_ticks&0xff)))

        # print(self.speed_msg_cnt&0xff)
        self.send_command(self.MSG_HEADER.WRITE_SPEED_CMD)
        self.writebyte(self.speed_msg_cnt)
        self.writelonglong(vr_ticks, vl_ticks)
        self.speed_msg_cnt += 1

    def setAcceleration(self, acceleration = None):

        if acceleration == None:
            acceleration = self.ACCELERATION
        
        weak_accel, accel, decel, brake = acceleration

        self.send_command(self.MSG_HEADER.WRITE_ACCEL_PROFILE_CFG)
        self.writebyte(0x00)
        self.writelong((accel >> 24) & 0xff, (accel >> 16) & 0xff,
                       (accel >> 8) & 0xff, accel & 0xff)
        self.writelong((decel >> 24) & 0xff, (decel >> 16) & 0xff,
                       (decel >> 8) & 0xff, decel & 0xff)
        self.writelong((brake >> 24) & 0xff, (brake >> 16) & 0xff,
                       (brake >> 8) & 0xff, brake & 0xff)
        self.writelong((weak_accel >> 24) & 0xff, (weak_accel >> 16) & 0xff,
                       (weak_accel >> 8) & 0xff, weak_accel & 0xff)

    def setDynamicAcceleration(self, dynamic_accel_cfg = None):
        if dynamic_accel_cfg == None:
            dynamic_accel_cfg = [self.ACCEL_BOUNDARY_SPEED,
                                 self.DECEL_BOUNDARY_SPEED]

        accel_bound, decel_bound = dynamic_accel_cfg

        self.send_command(self.MSG_HEADER.WRITE_DYNAMIC_ACCEL_CFG)
        self.writebyte(0x00)
        self.writelong((accel_bound >> 24) & 0xff, (accel_bound >> 16 & 0xff),
                       (accel_bound >> 8) & 0xff, (accel_bound & 0xff))
        self.writelong((decel_bound >> 24) & 0xff, (decel_bound >> 16 & 0xff),
                       (decel_bound >> 8) & 0xff, (decel_bound & 0xff))

    def setUpdatePeriod(self, update_rate = None):
        
        if update_rate == None:
            update_rate = self.UPDATE_PERIOD

        # Convert float to bytearray with length 4
        # Then convert the bytearray to uint64_t to perform
        # bitwise operation
        update_rate_b = bytearray(struct.pack(">f", update_rate))
        update_rate_i = struct.unpack(">I", update_rate_b)[0]

        self.send_command(self.MSG_HEADER.WRITE_UPDATE_PERIOD_CFG)
        self.writebyte(0x00)
        self.writelong((update_rate_i>>24) & 0xff, (update_rate_i >> 16) & 0xff,
                       (update_rate_i>>8 ) & 0xff,  update_rate_i & 0xff)
    
    def sendShutdownRequest(self):
        
        self.setSpeed(0,0)
        # Send shutdown request
        self.send_command(self.MSG_HEADER.SHUTDOWN)

        self.is_initialized = False

 
        while self.ser.in_waiting:
            self.ser.read_all()
            time.sleep(0.5)

        if self.ser.in_waiting == 0:
            return True

        print("Buffer: " + str(self.ser.in_waiting))

        return False

    def writeInitParam(self):

        self.setAcceleration()
        self.setUpdatePeriod()
        self.setDynamicAcceleration()

    def waitForHeader(self):
        """ 
        Read ESP32 incoming message

        Return value:
        
        0:  Encoder speeds
        1:  Speed Profile 
        2:  Params (for debugging)
        3:  Tx Message Counter
        4:  Speed debug message
        8:  Init Ready
        9:  Custom message
        10: Error
        11: Timeout
        """

        header_2byte = 0
        header_try_timer = time.time()
        empty_buffer_timer = time.time()
        while True:
            while self.ser.in_waiting < 2:
                if(time.time() - empty_buffer_timer > (self.UPDATE_PERIOD*2.0)/1000.0):
                    return 11
                # Wait for the set update period, tolerance 10%
                time.sleep((self.UPDATE_PERIOD*1.1)/1000.0) 

            self.header2 = self.readbyte()
            # print("{} {}".format(hex(self.header1[1]), hex(self.header2[1])))

            header_2byte = (self.header1[1]<<8) | self.header2[1]
            
            if(header_2byte == self.MSG_HEADER.READ_HUMAN_MESSAGE):
                return 9
            elif(header_2byte == self.MSG_HEADER.READ_INIT_READY):
                return 8
            elif(header_2byte == self.MSG_HEADER.READ_ENCODER):
                return 0
            elif(header_2byte == self.MSG_HEADER.READ_FULL_SPEED_PROFILE):
                return 1
            elif(header_2byte == self.MSG_HEADER.READ_PARAMS):
                return 2
            elif(header_2byte == self.MSG_HEADER.READ_RX_MSG_CNT):
                return 3
            elif(header_2byte == self.MSG_HEADER.READ_DEBUG_MSG):
                return 4
            elif(header_2byte == self.MSG_HEADER.READ_ERROR):
                return 10

            self.header1 = self.header2
            if(time.time() -  header_try_timer > 0.1):
                return 11

    def readSpeed(self):
        msg_cnt = self.readbyte()
        # print(datetime.utcnow().strftime('%H:%M:%S.%f')[:-3] + " Encoder message counter: " + str(msg_cnt[1]))
        if(msg_cnt[0]):
            if self.first_encoder_msg:
                self.encoder_msg_cnt = msg_cnt[1]
                self.first_encoder_msg = False
            elif(msg_cnt[1] == (self.encoder_msg_cnt+1) & 0xff):
                self.encoder_msg_cnt += 1
            else:
                self.error_count += 1

                self.first_encoder_msg = True
                # return 0x10000, 0x10000

            data = self.read4()
            if(data[0]):
                speed_1 = (data[1] >> 16) & 0xffff
                speed_2 = data[1] & 0xffff

                # Convert 2bytes signed to 4bytes signed
                if(speed_1 & 0x8000):
                    speed_1 = -0x10000 + speed_1
                if(speed_2 & 0x8000):
                    speed_2 = -0x10000 + speed_2

                return speed_1, speed_2

            # else:
            # 	return 0x10000, 0x10000

        return 0x0000, 0x0000

    def readSpeedProfile(self):
        data = self.read8()
        if(data[0]):
            accel_0 = (data[1]>>32) & 0xffffffff
            accel_1 = data[1] & 0xffffffff
        
            data = self.read8()
            if(data[0]):
                est_speed_0 = (data[1] >> 32) & 0xffffffff
                est_speed_1 = data[1] & 0xffffffff

                # Convert 4 bytes unsigned to 4 bytes signed
                if(est_speed_0 & 0x80000000):
                    est_speed_0 = -0x100000000 + est_speed_0
                if(est_speed_1 & 0x80000000):
                    est_speed_1 = -0x100000000 + est_speed_1

                data = self.read8()
                if(data[0]):
                    speed_0 = (data[1] >> 32) & 0xffffffff
                    speed_1 = data[1] & 0xffffffff

                    # Convert 4 bytes unsigned to 4 bytes signed
                    if(speed_0 & 0x80000000):
                        speed_0 = -0x100000000 + speed_0
                    if(speed_1 & 0x80000000):
                        speed_1 = -0x100000000 + speed_1

                    return accel_0, accel_1, est_speed_0, est_speed_1, speed_0, speed_1

    def readSpeedDebug(self):
        data = self.read8()
        if(data[0]):
            accel_0 = (data[1]>>32) & 0xffffffff
            accel_1 = data[1] & 0xffffffff
        
            data = self.read8()
            if(data[0]):
                est_speed_0 = (data[1] >> 32) & 0xffffffff
                est_speed_1 = data[1] & 0xffffffff

                # Convert 4 bytes unsigned to 4 bytes signed
                if(est_speed_0 & 0x80000000):
                    est_speed_0 = -0x100000000 + est_speed_0
                if(est_speed_1 & 0x80000000):
                    est_speed_1 = -0x100000000 + est_speed_1

                return accel_0, accel_1, est_speed_0, est_speed_1

    def inv_kinematics(self, linear_x, angular_z):
        
        self.angular_z, self.linear_x = angular_z, linear_x
        if self.TURN_DIRECTION:
            vr = linear_x - angular_z * self.BASE_WIDTH / 2.0
            vl = linear_x + angular_z * self.BASE_WIDTH / 2.0
        else:
            vr = linear_x + angular_z * self.BASE_WIDTH / 2.0
            vl = linear_x - angular_z * self.BASE_WIDTH / 2.0

        self.vr_ticks = int(vr * self.TICKS_PER_METER * self.MOTOR_ENC_RATIO)
        self.vl_ticks = int(vl * self.TICKS_PER_METER * self.MOTOR_ENC_RATIO)
        ##########################################

        if self.RIGHT_INVERTED:
            self.vr_ticks = -self.vr_ticks
        if self.LEFT_INVERTED:
            self.vl_ticks = -self.vl_ticks            
            
        return self.vr_ticks, self.vl_ticks

    def test_acceleration_setting(self):
        if self.PLAYGROUND_ENABLE:
            dt = time.time() - self.vel_timer
            d_angular = angular_z - self.prev_angular_z
            if(abs(d_angular)/dt > self.ANGULAR_ACCELERATION):
                if(d_angular > 0):
                    angular_z = self.prev_angular_z + self.ANGULAR_ACCELERATION * dt
                else:
                    angular_z = self.prev_angular_z - self.ANGULAR_ACCELERATION * dt


    def __del__(self):
        self.ser.close()
