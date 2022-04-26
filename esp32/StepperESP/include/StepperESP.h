// -*- lsst-c++ -*-
#ifndef _STEPPER_ESP_H
#define _STEPPER_ESP_H

#include "Arduino.h"
#include <HardwareSerial.h>

#include <FastAccelStepper.h>
#include <EncoderRTOS.h>
#include <TMCStepper.h>
#include <MessageHeader.h>

#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f

class StepperESP
{
public:
    FastAccelStepperEngine engine = FastAccelStepperEngine();
    FastAccelStepper *stepper[2] = {NULL, NULL};

    EncoderRTOS encoder[2];
    uint8_t enable_pin;
    uint8_t clk_pin[2];
    uint8_t dir_pin[2];

    uint8_t encoderA[2], encoderB[2];

    ESP32MessageHeader header;

    //----------------------------------------------------------
    // Parameters used in old communication method
    char received_arr[19];
    char left_received_arr[8];
    char right_received_arr[8];

    float motors_vel_float[2] = {0.0, 0.0};
    int32_t motors_vel[2] = {0, 0}; // ticks per microseconds
    boolean motors_fw[2] = {true, true};
    boolean new_value = false;

    static const byte numChars = 19;
    char receivedChars[numChars];

    boolean newData = false;
    //----------------------------------------------------------  


    //----------------------------------------------------------
    // Initialization variables

    /// Disable message TX during reset
    boolean disableTx = true;
    /// Reinitialization flag
    boolean re_init = false;
    /// Number of retries when reading initialization parameters before failing
    uint8_t retry_limit = 10;
    /** 
     * Initialization update header for update messages
     * 
     * 0x56: acceleration profile
     * 
     * 0x57: dynamic acceleration configuration
     * 
     * 0x58: udpate rate
     */
    const int8_t init_updatelist[3] = {0x56, 0x57, 0x58};
    uint8_t init_checklist = 0x00;

    //----------------------------------------------------------

    uint8_t encoder_msg_cnt = 0;

    /**
     * Acceleration values for Accelerating, Decelerating and Braking.
     * Unit: Ticks/s^2
     */ 
    uint16_t acceleration_val[3] = {3000, 5000, 7000}; 
    
    /**
     * Use dynamic acceleration to ensure both wheels stop
     * at the same time when braking.
     */
    boolean use_dynamic_accel = false;
    bool brake_flag = false;

    /**
     * Used to determine the deceleration required so that the wheels.
     * Unit: ms
     * 
     * Formula: 
     * 
     * deceleration = stepper.getCurrentSpeedInMilliHz() / decel_divisor
     */
    int16_t decel_divisor = 500;

    //-----------------------------------------------------------
    // If deceleration takes too long, the acceleration value
    // will be increased to ensure faster deceleration

    uint16_t deceleration_timer[2] = {0, 0};
    uint16_t deceleration_duration[2] = {0, 0};
    boolean deceleration_flag[2] = {false, false};
    boolean done_deceleration_flag[2] = {false, false};
    uint16_t deceleration_time_limit = 100;
    //-----------------------------------------------------------

    enum SPEED_SCENARIO
    {
        ACCEL,
        DECEL,
        BRAKE
    };

    enum QUEUE_STATE{
        STEPPER0_FULL,
        STEPPER1_FULL,
        FULL,
        OK
    };

    StepperESP();
    ~StepperESP();
    void setEnablePin(uint8_t enable_pin_);
    void setClockPin(uint8_t clk_pin_1, uint8_t clk_pin_2);
    void setDirectionPin(uint8_t dir_pin_1, uint8_t dir_pin_2);

    void setEncoderPin(const uint8_t encoderA_[2], const uint8_t encoderB_[2]);

    void initializeStepperEncoder();

    void reset();
    void sendInitReady();
    /**
     * Read Initialization parameters including: acceleration, 
     * encoder update rate, max speed
     * 
     * TODO: update valid and invalid value 
     */
    void readInitParam();
    
    int8_t getQueueState();

    void getEncoderSpeed();
    void setSpeed(int16_t speed0, int16_t speed1);

    /**
     * Determine "Speed Scenario" based on set speed.
     * This method is deprecated and is kept until the set acceleration method is finalized
     * @param speed   Stepper's speed
     * @param stepper Pointer to the stepper
     * @retval 2 Braking
     * @retval 1 Deceleration
     * @retval 0 Acceleration
     */
    uint8_t SetAccelerationMode(int16_t speed, FastAccelStepper *stepper);

    /**
     * @brief Determine "Speed Scenario" based on set speed. Set extra flags
     * to limit deceleration time
     * @param speed    Stepper's speed
     * @param speed_no Stepper's number
     * @retval 2 Braking
     * @retval 1 Deceleration
     * @retval 0 Acceleration
     * @overload
     */
    uint8_t SetAccelerationMode(int16_t speed, int stepper_no);

    /**
     * @brief Set the Acceleration of the output steps
     *
     * @param accel0
     * @param accel1
     */
    void setAcceleration(int16_t accel0, int16_t accel1);

    /**
     * Set the Acceleration profile of the driver for different scenario:
     * Accelerating, Decelerating and Braking
     * @param accel Acceleration
     * @param decel Deceleration
     * @param brake Braking Deceleration
     */
    void setAccelerationProfile(int16_t accel, int16_t decel, int16_t brake){
        acceleration_val[0] = accel;
        acceleration_val[1] = decel;
        acceleration_val[2] = brake;
    }

    void setDirection(boolean dir0, boolean dir1);
    void motorControl();
    void recvWithStartEndMarkers();

    void SetAccelerationDivisor(uint8_t accel_div);

    void getEncoderCount()
    {
        tick_count[0] = encoder[0].getCount();
        tick_count[1] = encoder[1].getCount();
    }

    void sendEncoderSpeed()
    {
        if(!disableTx)
        {
            Serial.write((header.SEND_ENCODER >> 8) & 0xff);
            Serial.write(header.SEND_ENCODER & 0xff);
            Serial.write(encoder_msg_cnt);
            Serial.write((tick_speed[0] >> 8) & 0xff);
            Serial.write(tick_speed[0] & 0xff);
            Serial.write((tick_speed[1] >> 8) & 0xff);
            Serial.write(tick_speed[1] & 0xff);
            encoder_msg_cnt += 1;
        }
    }

    /**
     * @brief Send human-friendly readable message
     * @param msg Pointer to the char array
     */
    void sendCustomMessage(const char *msg);
    void sendError()
    {
        Serial.write((header.SEND_ERROR>>8) & 0xff);
        Serial.write(header.SEND_ENCODER & 0xff);
        Serial.write(0x00);
        Serial.write(0x00);
    }
    volatile int16_t tick_speed[2] = {0, 0};
    volatile int64_t tick_count[2] = {0, 0};

    uint8_t accel_mode;

    float dt = 50; // microseconds
};

#endif