#include <Arduino.h>
#include <FastAccelStepper.h>
#include <EncoderRTOS.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "StepperESP.h"

#define INCLUDE_xTaskDelayUntil 1

#define STEP_A_PIN 17
#define DIR_A_PIN 25
#define STEP_B_PIN 16
#define DIR_B_PIN 27
#define STEP_EN 26
const uint8_t ENCODER_A[2] = {36,34};
const uint8_t ENCODER_B[2] = {35,4};

StepperESP stepper;
TaskHandle_t motor_control;
TaskHandle_t encoder_read;

//----------------------------------------------------------------------
void motor_control_task(void* pvParameters);
void encoder_read_task(void* pvParameters);
void setSpeed();

void setup() {
  Serial.begin(115200);

  stepper.setClockPin(STEP_A_PIN, STEP_B_PIN);
  stepper.setDirectionPin(DIR_A_PIN, DIR_B_PIN);
  stepper.setEnablePin(STEP_EN);

  stepper.setEncoderPin(ENCODER_A, ENCODER_B);

  stepper.initialize();

  xTaskCreatePinnedToCore(
    encoder_read_task,
    "encoder_read",
    10000,
    NULL,
    1,
    &encoder_read,
    0);
  delay(200);

}

//----------------------------------------------------------------------

float i = 0;
bool decreasing = false;

int8_t cmd = 0;

void loop() {
	int haha = Serial.available();
	if(haha>0){
		cmd = Serial.read();
		if(cmd == 0x4D)
			cmd = Serial.read();
			if(cmd == 0x01)
				setSpeed();		
	}
	delay(100);
}

void setSpeed(){
	int8_t data[4];
	for(int i = 3; i >= 0; i--){
		Serial.read();
		data[i] = Serial.read();
	}
	static int16_t *speed_1, *speed_2;
	speed_1 = (int16_t *)(&data[0]);
	speed_2 = (int16_t *)(&data[2]);
	stepper.setSpeed(*speed_1, *speed_2);

	/* Selftest section */
	// Serial.print(*speed_1, HEX);
	// Serial.print(" ");
	// Serial.print(*speed_2, HEX);
	// Serial.println();
}

void encoder_read_task(void *pvParameters){
	TickType_t xLastWakeTime= xTaskGetTickCount();
	const TickType_t xFrequency = stepper.dt / portTICK_PERIOD_MS;

	for(;;){
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		stepper.getEncoderSpeed();
		Serial.write(0x6f);Serial.write(0x57);
		Serial.write((stepper.tick_speed[0]>>8)&0xff);Serial.write(0);
		Serial.write(stepper.tick_speed[0]&0xff);     Serial.write(0);
		Serial.write((stepper.tick_speed[1]>>8)&0xff);Serial.write(0);
		Serial.write(stepper.tick_speed[1]&0xff);     Serial.write(0);
	}
  
}

