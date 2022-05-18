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
TaskHandle_t motor_control;
TaskHandle_t encoder_read;

uint8_t speed_msg_cnt = 0;
uint8_t accel_msg_cnt = 0;
uint8_t speed_msg_error = 0;
uint8_t encoder_msg_cnt = 0;

//----------------------------------------------------------------------
void motor_control_task(void *pvParameters);
void encoder_read_task(void *pvParameters);
void setSpeed();
void setAcceleration();

/// Deprecated method
void setAccelerationMode();

void setup()
{
	Serial.begin(115200);

	pinMode(2, INPUT_PULLUP);
	pinMode(4, INPUT_PULLUP);

	stepper.setClockPin(STEP_A_PIN, STEP_B_PIN);
	stepper.setDirectionPin(DIR_A_PIN, DIR_B_PIN);
	stepper.setEnablePin(STEP_EN);

	stepper.setEncoderPin(ENCODER_A, ENCODER_B);

	stepper.initializeStepperEncoder();

	xTaskCreatePinnedToCore(
		encoder_read_task,
		"encoder_read",
		20000,
		NULL,
		1,
		&encoder_read,
		1);
	delay(200);
}

//----------------------------------------------------------------------

float i = 0;
bool decreasing = false;

int16_t cmd = 0;
char buffer[2] = {};

void loop()
{
	static uint64_t timeout;
	static uint64_t dt;
	static boolean timeout_flag;
	static uint64_t loop_dt;
	static uint64_t loop_start_timer;

	loop_start_timer = millis();
	
	while(Serial.available() < 2){
		dt = millis() - timeout;
		if (dt >= 200)
		{	
			if (!timeout_flag)
			{	
				stepper.set_tick_speed[0] = 0;
				stepper.set_tick_speed[1] = 0;
			}
			timeout_flag = true;
			break;
		}
		else
		{
			timeout_flag = false;
		}
		// stepper.sendCustomMessage("Serial buffer empty");
		delay(5);
	}
	cmd = (cmd << 8) | Serial.read();
	if (cmd == stepper.header.READ_SPEED_CMD)
	{
		setSpeed();
		stepper.rx_msg_cnt += 1;
		timeout = millis();
	}
	else if (cmd == stepper.header.SOFT_RESET) // 0x7954: Reset
	{	
		vTaskSuspend(encoder_read);
		delay(200);
		stepper.reset();
		stepper.rx_msg_cnt += 1;
		vTaskResume(encoder_read);
		timeout = millis();
	}
	else if (cmd == stepper.header.SHUTDOWN)
	{
		stepper.readShutdownRequest();
		stepper.rx_msg_cnt += 1;
	}
	else if (cmd == stepper.header.SEND_PARAMS)
	{
		stepper.sendParams();
		stepper.rx_msg_cnt += 1;
	}


	stepper.setSpeed(stepper.set_tick_speed[0], stepper.set_tick_speed[1]);
	stepper.SetDynamicAcceleration();
	if(!stepper.disableTx)
		// stepper.sendSpeedProfile();
		;
	loop_dt = millis() - loop_start_timer;
	// stepper.sendCustomMessageHeader();
	// Serial.print("Main Loop runtime: "); Serial.println(loop_dt);	
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
		// Serial.read();
		data[i] = Serial.read();
	}

	int32_t *speed_1, *speed_2;
	speed_1 = (int32_t *)(&data[4]);
	speed_2 = (int32_t *)(&data[0]);

	stepper.set_tick_speed[0] = *speed_1;
	stepper.set_tick_speed[1] = *speed_2;

#ifdef READ_TEST
	Serial.write((stepper.header.SEND_HUMAN_MESSAGE >> 8) & 0xff);
	Serial.write(stepper.header.SEND_HUMAN_MESSAGE & 0xff);
	Serial.print("Motor Speed: ");
	Serial.print(*speed_1);
	Serial.print(" ");
	Serial.print(*speed_2);
	Serial.println();
#endif
}

void encoder_read_task(void *pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	TickType_t xFrequency = stepper.dt / portTICK_PERIOD_MS;

	for (;;)
	{
		xFrequency = stepper.dt / portTICK_PERIOD_MS;
		
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		stepper.getEncoderSpeed();
		
		// stepper.SetDynamicAcceleration();
		stepper.sendSpeedProfile();
		stepper.sendTotalRxMsg();		
	}
}
void motor_control_task(void *pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	TickType_t xFrequency = 50 / portTICK_PERIOD_MS;

	for (;;)
	{	
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		if(!stepper.disableTx)
		{
			
		}
		
		
	}
}
