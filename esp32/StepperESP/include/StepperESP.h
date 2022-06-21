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

typedef ESP32MessageHeader MSG_HEADER;

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

    MSG_HEADER header;

    
    // Parameters used in deprecated communication method
    //================================
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
    //================================


    // Initialization variables
    //================================

    // Disable message TX during reset. Set to true during resetting
    boolean disableTx = true;
    // Reinitialization flag. Set to true during resetting
    boolean re_init = true;
    // Number of retries when reading initialization parameters before failing
    uint8_t retry_limit = 10;
    // Bit field for checking configuration message 
    uint8_t init_checklist = 0x00;

    enum QUEUE_STATE{
        STEPPER0_FULL,
        STEPPER1_FULL,
        FULL,
        OK
    };

    //================================

    uint8_t encoder_msg_cnt = 0;

    /**
     * Acceleration values for Accelerating, Decelerating and Braking.
     * Unit: Ticks/s^2
     */ 
    uint32_t acceleration_val[5] = {2500, 3000, 5000, 5000, 7000};
    uint32_t max_acceleration = 20000; 


    //-----------------------------------------------------------
    // During low speed, the acceleration will be lower to induce more torque
    // While during high speed, the acceleration will be higher.
    // The boundary value between low and high speed range is defined here
    // The speed is measured at the encoder, unit: tick/second

    // Boundary Speed when accelerating from 0
    uint32_t boundary_speed = 2000;
    // Boundary Speed when decelerating to 0
    uint32_t boundary_speed2 = 6000;
    //-----------------------------------------------------------

    enum SPEED_SCENARIO
    {   
        WEAK_ACCEL,
        ACCEL,
        DECEL,
        STRONG_DECEL,
        BRAKE        
    };


    // Variables used in deprecated methods of determining acceleration
    //===================================

        
    /**
     * Use dynamic acceleration to ensure both wheels stop
     * at the same time when braking.
     */
    boolean use_dynamic_accel = true;
    bool brake_flag = false;
    volatile bool new_speed_flag = true;

    /**
     * This variable is used in deprecated method. 
     * Used to determine the deceleration required so that the wheels 
     * stops within specified interval.
     *
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

    //===================================


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

    void readShutdownRequest();
    
    int8_t getQueueState();

    void getEncoderSpeed();

    /**
     * This method is deprecated. Set speed and determine the acceleration based on the set speed. 
     */
    void setSpeedAccel(int16_t speed0, int16_t speed1);

    /**
     * Set speed 
     */
    void setSpeed(int32_t speed0, int32_t speed1);


    /**
     * This method is deprecated.. Determine "Speed Scenario" based on set speed.
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
     * This method is deprecated. Continuously acquire ramp state from the FastAccelStepper libray and determine
     * the required acceleration. Similar to the SetAccelerationMode function, but used
     * in a task loop instead of when setting speed. Currently, the strong deceleration is
     * equal to Braking Acceleration. 
     * @param stepper_no Stepper's number
     * @retval 4 Braking
     * @retval 3 Strong Deceleration
     * @retval 2 Deceleration
     * @retval 1 Acceleration
     * @retval 0 Weak Acceleration
     */
    uint8_t GetAccelerationMode(int stepper_no);

    //---------------------------------------------
    /**
     * This method is deprecated. Instead of determining the acceleration,
     * the velocity ramp is controlled directly.
     */

    // Update the velocity ramp 
    void UpdateVelocityRamp();
    // Get the state of the FastAccelStepper library
    int8_t GetMotorState(int stepper_no, int32_t &tick_diff, int8_t &direction);
    
    int32_t projected_tick_velocity[2] = {0, 0};

    //---------------------------------------------


    /**
     * Continuously acquire ramp state from the FastAccelStepper libray and determine
     * the required acceleration. Similar to the SetAccelerationMode function, but used
     * in a task loop instead of when setting speed.
     */
    void SetDynamicAcceleration();

    /**
     * Continuously acquire ramp state from the FastAccelStepper libray and determine
     * the required acceleration. Similar to the SetAccelerationMode function, but used
     * in a task loop instead of when setting speed.
     */
    void SetDynamicAcceleration(boolean playground_enable);

    #define MOTOR_1_MINOR_STALL       0x0001
    #define MOTOR_1_STALL             0x0002
    #define MOTOR_1_SEVERE_STALL      0x0003
    #define MOTOR_1_STALL_STATUS_MASK 0x00FF

    #define MOTOR_2_MINOR_STALL       0x0100
    #define MOTOR_2_STALL             0x0200
    #define MOTOR_2_SEVERE_STALL      0x0300
    #define MOTOR_2_STALL_STATUS_MASK 0xFF00

    boolean speed_overwrite = false;
    uint32_t speed_overwrite_timer = 0;
    uint32_t speed_overwrite_timer_threshold = 400;

    /**
     * Detect stall in stepper motors by comparing estimated tick velocity and
     * the actual tick velocity.
     */
    int StallDetection();

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

    void sendEncoderSpeed();

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
    void sendCustomMessageHeader();

    void sendError();
    void sendSpeedProfile();
    void sendSpeedProfile(int32_t est_tick_vel0, int32_t est_tick_vel1, int32_t set_tick_accel0, int32_t set_tick_accel1);

    volatile uint8_t rx_msg_cnt = 0; // Reset every Tx cycle

    volatile int32_t tick_accel[2] = {0, 0}; // Tick acceleration to set. Unit: Ticks/s^2
    volatile int32_t tick_speed[2] = {0, 0}; // Tick speed recorded by the encoder. Unit: Unit: Ticks/s
    volatile int32_t tick_speed_est[2] = {0, 0}; // Tick speed created by the FastAccelStepper library. Unit: Ticks/s
    volatile int32_t set_tick_speed[2] = {0, 0}; // Tick speed set by the Python node. Unit: Ticks/s
    volatile boolean dir[2] = {true, true};
    volatile int64_t tick_count[2] = {0, 0}; // Tick count recorded by the encoder. Unit: Ticks

    float dt = 50; // microseconds
};

#endif