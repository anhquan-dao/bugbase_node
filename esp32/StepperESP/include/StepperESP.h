#ifndef _STEPPER_ESP_H
#define _STEPPER_ESP_H

#include "Arduino.h"
#include <HardwareSerial.h>

#include <FastAccelStepper.h>
#include <EncoderRTOS.h>
#include <TMCStepper.h>

#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f 

class StepperESP{
    private:
        FastAccelStepperEngine engine = FastAccelStepperEngine();
        FastAccelStepper *stepper[2] = {NULL, NULL};

        HardwareSerial serial_1{1};
        TMC2209Stepper *driver = NULL;

        EncoderRTOS encoder[2];

        boolean initialized = true;


        char  received_arr[19];
        char  left_received_arr[8];
        char  right_received_arr[8];

        float motors_vel_float[2] = {0.0, 0.0}; 
        int32_t motors_vel[2] = {0, 0}; // ticks per microseconds
        boolean motors_fw[2] = {true, true};
        boolean new_value = false;

        static const byte numChars = 19;
        char receivedChars[numChars];

        boolean newData = false;

        uint8_t enable_pin;
        uint8_t clk_pin[2];
        uint8_t dir_pin[2];

        uint8_t encoderA[2], encoderB[2];
        

    public:
        StepperESP();
        ~StepperESP();
        void setEnablePin(uint8_t enable_pin_);
        void setClockPin(uint8_t clk_pin_1, uint8_t clk_pin_2);
        void setDirectionPin(uint8_t dir_pin_1, uint8_t dir_pin_2);

        void setEncoderPin(const uint8_t encoderA_[2], const uint8_t encoderB_[2]);

        void initialize();

        void getEncoderSpeed();
        void setSpeed(int16_t speed0, int16_t speed1);
        void setAcceleration(int16_t accel0, int16_t accel1);
        void setDirection(boolean dir0, boolean dir1);
        void motorControl();
        void recvWithStartEndMarkers();

        void getEncoderCount(){
            tick_count[0] = encoder[0].getCount();
            tick_count[1] = encoder[1].getCount();
        }

        volatile int16_t tick_speed[2] = {0,0};
        volatile int64_t tick_count[2] = {0,0};
        float dt = 10; //microseconds

};

#endif