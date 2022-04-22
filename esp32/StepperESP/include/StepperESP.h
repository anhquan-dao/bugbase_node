// -*- lsst-c++ -*-
#ifndef _STEPPER_ESP_H
#define _STEPPER_ESP_H

#include "Arduino.h"
#include <HardwareSerial.h>

#include <FastAccelStepper.h>
#include <EncoderRTOS.h>
#include <TMCStepper.h>

#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f

class StepperESP
{
public:
    FastAccelStepperEngine engine = FastAccelStepperEngine();
    FastAccelStepper *stepper[2] = {NULL, NULL};

    HardwareSerial serial_1{1};
    TMC2209Stepper *driver = NULL;

    EncoderRTOS encoder[2];

    boolean initialized = true;

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

    uint8_t enable_pin;
    uint8_t clk_pin[2];
    uint8_t dir_pin[2];

    uint8_t encoderA[2], encoderB[2];

    // ACCEL, DECEL, REVERSE, BRAKE
    uint16_t acceleration_val[4] = {3000, 5000, 5000, 7000};
    int8_t acceleration_divisor = 80;

    uint16_t deceleration_timer[2] = {0, 0};
    uint16_t deceleration_duration[2] = {0, 0};
    boolean deceleration_flag[2] = {false, false};
    boolean done_deceleration_flag[2] = {false, false};
    uint16_t deceleration_time_limit = 100;

    bool brake_flag = false;
    int8_t accel_flag = 0;

    uint8_t encoder_msg_cnt = 0;

    enum SPEED_SCENARIO
    {
        ACCEL,
        DECEL,
        REVERSE,
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
    
    int8_t getQueueState();

    void getEncoderSpeed();
    void setSpeed(int16_t speed0, int16_t speed1);

    /**
     * @brief Determine "Speed Scenario" based on set speed
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
     */
    uint8_t SetAccelerationMode(int16_t speed, int stepper_no);

    /**
     * @brief Set the Acceleration of the output steps
     *
     * @param accel0
     * @param accel1
     */
    void setAcceleration(int16_t accel0, int16_t accel1);

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
        Serial.write(0x6f);
		Serial.write(0x57);
		Serial.write(encoder_msg_cnt);
		Serial.write((tick_speed[0] >> 8) & 0xff);
		Serial.write(tick_speed[0] & 0xff);
		Serial.write((tick_speed[1] >> 8) & 0xff);
		Serial.write(tick_speed[1] & 0xff);
        encoder_msg_cnt += 1;
    }

    void sendCustomMessage(const char *start, uint8_t len);
    volatile int16_t tick_speed[2] = {0, 0};
    volatile int64_t tick_count[2] = {0, 0};

    uint8_t accel_mode;

    float dt = 50; // microseconds
};

#endif