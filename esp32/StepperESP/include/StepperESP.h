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
    boolean re_init = true;
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

    boolean send_full;

    //----------------------------------------------------------

    uint8_t encoder_msg_cnt = 0;

    /**
     * Acceleration values for Accelerating, Decelerating and Braking.
     * Unit: Ticks/s^2
     */ 
    uint32_t acceleration_val[5] = {2500, 3000, 5000, 5000, 7000};
    uint32_t max_acceleration = 20000; 
    
    /**
     * Use dynamic acceleration to ensure both wheels stop
     * at the same time when braking.
     */
    boolean use_dynamic_accel = true;
    bool brake_flag = false;
    volatile bool new_speed_flag = true;

    /**
     * Used to determine the deceleration required so that the wheels.
     * Unit: ms
     * 
     * Formula: 
     * 
     * deceleration = stepper.getCurrentSpeedInMilliHz() / decel_divisor
     */
    int16_t decel_divisor = 200;
    int16_t accel_divisor = 500;


    //-----------------------------------------------------------
    // If deceleration takes too long, the acceleration value
    // will be increased to ensure faster deceleration

    uint16_t deceleration_timer[2] = {0, 0};
    uint16_t deceleration_duration[2] = {0, 0};
    boolean deceleration_flag[2] = {false, false};
    boolean done_deceleration_flag[2] = {false, false};
    uint16_t deceleration_time_limit = 100;
    //-----------------------------------------------------------

    //-----------------------------------------------------------
    // During low speed, the acceleration will be lower to induce more torque
    // While during high speed, the acceleration will be higher.
    // The boundary value between low and high speed range is defined here

    // The speed is measured at the encoder, unit: tick/second
    uint16_t boundary_speed = 10000;
    uint16_t boundary_speed2 = 30000;
    // The gear ratio between the motor shaft and encoder shaft
    float motor_enc_ratio = 5;
    // The gear ratio between the wheel shaft and encoder shaft
    float wheel_enc_ratio = 1.333;
    //-----------------------------------------------------------

    enum SPEED_SCENARIO
    {   
        WEAK_ACCEL,
        ACCEL,
        DECEL,
        STRONG_DECEL,
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

    void readShutdownRequest()
    {   
        disableTx = true;
        re_init = true;
        delay(50);
        // Serial.flush();
        char buffer[100] = {};
        while(Serial.available())
        {   
            if(Serial.available() < 100)
            {
                Serial.readBytes(buffer, Serial.available());
            }
            else
            {
                Serial.readBytes(buffer, 100);
            }
        }
    }
    
    int8_t getQueueState();

    void getEncoderSpeed();

    /**
     * Set speed and determine the acceleration based on the set speed. 
     * This function is subject to change.
     */
    void setSpeedAccel(int16_t speed0, int16_t speed1);

    /**
     * Set speed 
     */
    void setSpeed(int32_t speed0, int32_t speed1);


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
     * Continuously acquire ramp state from the FastAccelStepper libray and determine
     * the required acceleration. Similar to the SetAccelerationMode function, but used
     * in a task loop instead of when setting speed. Currently, the strong deceleration is
     * equal to Braking Acceleration
     * @param stepper_no Stepper's number
     * @retval 4 Braking
     * @retval 3 Strong Deceleration
     * @retval 2 Deceleration
     * @retval 1 Acceleration
     * @retval 0 Weak Acceleration
     */
    uint8_t GetAccelerationMode(int stepper_no);

    void SetDynamicAcceleration();

    int8_t GetMotorState(int stepper_no, int32_t &tick_diff, int8_t &direction);
    void UpdateVelocityRamp();
    int32_t projected_tick_velocity[2] = {0, 0};


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
     * @param weak_accel Weak Acceleration
     */
    void setAccelerationProfile(int32_t accel, int32_t decel, int32_t brake, int32_t weak_accel){
        acceleration_val[0] = weak_accel;
        acceleration_val[1] = accel;
        acceleration_val[2] = decel;
        acceleration_val[3] = brake;
        acceleration_val[4] = brake;
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

    /**
     * Send human-friendly readable message header. To send the actual
     * message, use Serial.print and send the "\n" symbol
     * signal the end of the message right after calling this function. 
     * Or use Serial.println.
     */
    void sendCustomMessageHeader()
    {
        Serial.write((header.SEND_HUMAN_MESSAGE >> 8) & 0xff);
	    Serial.write(header.SEND_HUMAN_MESSAGE & 0xff);
    }
    void sendError()
    {   
        if(!disableTx)
        {
            Serial.write((header.SEND_ERROR>>8) & 0xff);
            Serial.write(header.SEND_ERROR & 0xff);
            Serial.write(0x00);
            Serial.write(0x00);
        }
        
    }
    void sendSpeedProfile()
    {   
        if(!disableTx)
        {
            Serial.write((header.SEND_FULL_SPEED_PROFILE>>8) & 0xff);
            Serial.write(header.SEND_FULL_SPEED_PROFILE & 0xff);

            Serial.write((tick_accel[0] >> 24) & 0xff);
            Serial.write((tick_accel[0] >> 16) & 0xff);
            Serial.write((tick_accel[0] >> 8) & 0xff);
            Serial.write(tick_accel[0] & 0xff);

            Serial.write((tick_accel[1] >> 24) & 0xff);
            Serial.write((tick_accel[1] >> 16) & 0xff);
            Serial.write((tick_accel[1] >> 8) & 0xff);
            Serial.write(tick_accel[1] & 0xff);

            Serial.write((tick_speed_est[0] >> 24) & 0xff);
            Serial.write((tick_speed_est[0] >> 16) & 0xff);
            Serial.write((tick_speed_est[0] >> 8) & 0xff);
            Serial.write(tick_speed_est[0] & 0xff);

            Serial.write((tick_speed_est[1] >> 24) & 0xff);
            Serial.write((tick_speed_est[1] >> 16) & 0xff);
            Serial.write((tick_speed_est[1] >> 8) & 0xff);
            Serial.write(tick_speed_est[1] & 0xff);

            // set_tick_speed[0] = stepper[0]->getSpeedInMilliHz()/1000;
            // set_tick_speed[1] = stepper[1]->getSpeedInMilliHz()/1000;

            Serial.write((tick_speed[0] >> 24) & 0xff);
            Serial.write((tick_speed[0] >> 16) & 0xff);
            Serial.write((tick_speed[0] >> 8) & 0xff);
            Serial.write(tick_speed[0] & 0xff);

            Serial.write((tick_speed[1] >> 24) & 0xff);
            Serial.write((tick_speed[1] >> 16) & 0xff);
            Serial.write((tick_speed[1] >> 8) & 0xff);
            Serial.write(tick_speed[1] & 0xff);
        }
    }
    void sendParams()
    {   
        if(!disableTx)
        {
            Serial.write((header.SEND_PARAMS >> 8) & 0xff);
            Serial.write(header.SEND_PARAMS & 0xff);

            Serial.print(acceleration_val[0]); Serial.print(" ");
            Serial.print(acceleration_val[1]); Serial.print(" ");
            Serial.print(acceleration_val[2]); Serial.print(" ");
            Serial.print(acceleration_val[3]); Serial.print(" ");
            
            Serial.println(dt);
        }
    }

    volatile uint8_t rx_msg_cnt = 0; // Reset every Tx cycle
    void sendTotalRxMsg()
    {
        if(!disableTx)
        {
            Serial.write((header.SEND_RX_MSG_CNT >> 8) & 0xff);
            Serial.write(header.SEND_RX_MSG_CNT & 0xff);

            Serial.write(rx_msg_cnt);
            rx_msg_cnt = 0;
        }
    }
    volatile int32_t tick_accel[2] = {0, 0}; // Tick acceleration to set. Unit: Ticks/s^2
    volatile int32_t tick_speed[2] = {0, 0}; // Tick speed recorded by the encoder. Unit: Unit: Ticks/s
    volatile int32_t tick_speed_est[2] = {0, 0}; // Tick speed created by the FastAccelStepper library. Unit: Ticks/s
    volatile int32_t set_tick_speed[2] = {0, 0}; // Tick speed set by the Python node. Unit: Ticks/s
    volatile boolean dir[2] = {true, true};
    volatile int64_t tick_count[2] = {0, 0}; // Tick count recorded by the encoder. Unit: Ticks

    uint8_t accel_mode;

    float dt = 50; // microseconds
};

#endif