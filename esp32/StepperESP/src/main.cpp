#include <Arduino.h>
#include <FastAccelStepper.h>
#include <EncoderRTOS.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "StepperESP.h"

#define INCLUDE_xTaskDelayUntil 1

// #define READ_TEST 1
// #define WRITE_TEST 1

#define STEP_A_PIN 25
#define DIR_A_PIN 26
#define STEP_B_PIN 16
#define DIR_B_PIN 17
#define STEP_EN 27
const uint8_t ENCODER_A[2] = {39, 36};
const uint8_t ENCODER_B[2] = {34, 35};

StepperESP stepper;
TaskHandle_t SerialTask;
TaskHandle_t EncoderTask;

uint8_t speed_msg_cnt = 0;
uint8_t accel_msg_cnt = 0;
uint8_t speed_msg_error = 0;
uint8_t encoder_msg_cnt = 0;

//----------------------------------------------------------------------
void serial_read_task(void *pvParameters);
void encoder_read_task(void *pvParameters);
void setSpeed();
void setAcceleration();

/// Deprecated method
void setAccelerationMode();

void setup()
{
	Serial.begin(115200);

	pinMode(ENCODER_A[0], INPUT_PULLUP); pinMode(ENCODER_A[1], INPUT_PULLUP);
	pinMode(ENCODER_B[0], INPUT_PULLUP); pinMode(ENCODER_B[1], INPUT_PULLUP);

	stepper.setClockPin(STEP_A_PIN, STEP_B_PIN);
	stepper.setDirectionPin(DIR_A_PIN, DIR_B_PIN);
	stepper.setEnablePin(STEP_EN);

	stepper.setEncoderPin(ENCODER_A, ENCODER_B);

	stepper.initializeStepperEncoder();

	xTaskCreatePinnedToCore(
		encoder_read_task,
		"encoder_read",
		2000,
		NULL,
		1,
		&EncoderTask,
		1);
	// xTaskCreatePinnedToCore(
	// 	serial_read_task,
	// 	"serial_read",
	// 	200000,
	// 	NULL,
	// 	1,
	// 	&SerialTask,
	// 	0);
	delay(200);
}

//----------------------------------------------------------------------

int16_t cmd = 0;
uint64_t timeout, dt;
boolean timeout_flag;
uint64_t loop_dt, loop_start_timer;
boolean full_loop_flag = false;

boolean test_flag = false;

void loop()
{	
	if(Serial.available() < 1)
	{	
		
		dt = millis() - timeout;
		if (dt >= 200)
		{	
			if (!timeout_flag)
			{	
				stepper.set_tick_speed[0] = 0;
				stepper.set_tick_speed[1] = 0;
			}
			timeout_flag = true;
		}
		else
		{
			timeout_flag = false;
		}
	}
	else
	{	
		cmd = (cmd << 8) | Serial.read();

		if((cmd | stepper.header.READ_HEADER))
		{
			timeout = millis();
			if (cmd == stepper.header.READ_SPEED_CMD)
			{
				setSpeed();
				stepper.rx_msg_cnt += 1;
			}
			else if (cmd == stepper.header.SOFT_RESET)
			{	
				stepper.sendCustomMessageHeader();
				vTaskSuspend(EncoderTask);
				stepper.reset();
				vTaskResume(EncoderTask);
				stepper.sendCustomMessageHeader();
				Serial.println(stepper.disableTx);
			}
			else if (cmd == stepper.header.SHUTDOWN)
			{
				stepper.readShutdownRequest();
			}
			else if (cmd == stepper.header.SEND_PARAMS)
			{
				stepper.sendParams();
			}
		}
	}

	stepper.setSpeed(stepper.set_tick_speed[0], stepper.set_tick_speed[1]);
	stepper.SetDynamicAcceleration();
	delay(10);
}

void setSpeed()
{
	uint8_t new_msg_cnt = Serial.read();
	if (new_msg_cnt == (speed_msg_cnt + 1))
	{
		speed_msg_cnt += 1;
	}
	else
	{
		speed_msg_error += 1;
		// stepper.sendError();
	}
	int8_t data[8];
	for (int i = 7; i >= 0; i--)
	{
		data[i] = Serial.read();
	}

	int32_t *speed_1, *speed_2;
	speed_1 = (int32_t *)(&data[4]);
	speed_2 = (int32_t *)(&data[0]);

	stepper.set_tick_speed[0] = *speed_1;
	stepper.set_tick_speed[1] = *speed_2;

#ifdef READ_TEST
	stepper.sendCustomMessageHeader();
	Serial.print("Motor Speed: ");
	Serial.print(stepper.set_tick_speed[0]);
	Serial.print(" ");
	Serial.print(stepper.set_tick_speed[1]);
	Serial.println();
#endif
}

void encoder_read_task(void *pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	TickType_t xFrequency = stepper.dt / portTICK_PERIOD_MS;

	for (;;)
	{	
		xFrequency = 20 / portTICK_PERIOD_MS;
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		// if(!stepper.disableTx)
		// {
		// 	stepper.sendCustomMessageHeader();
		// 	Serial.println(millis());
		// }
		stepper.getEncoderSpeed();
		stepper.sendEncoderSpeed();	

	}
}