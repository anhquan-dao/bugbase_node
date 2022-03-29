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
const uint8_t ENCODER_A[2] = {39,36};
const uint8_t ENCODER_B[2] = {34,35};

StepperESP stepper;
TaskHandle_t motor_control;
TaskHandle_t encoder_read;

uint8_t speed_msg_cnt = 0;
uint8_t accel_msg_cnt = 0;
uint8_t speed_msg_error = 0;
uint8_t encoder_msg_cnt = 0;

//----------------------------------------------------------------------
void motor_control_task(void* pvParameters);
void encoder_read_task(void* pvParameters);
void setSpeed();
void setAcceleration();
void setAccelerationMode();

void setup(){
  Serial.begin(115200);

  pinMode(2, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);

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
	static uint64_t timeout;
	static uint64_t dt;

	dt = millis() - timeout;
	if(dt >= 500){
		stepper.setSpeed(0,0);
	}
	int haha = Serial.available();
	if(haha>=9){
		// Serial.print("Serial buffer length: ");
		// Serial.println(haha);
		cmd = Serial.read();
		if(cmd == 0x4D){
			cmd = Serial.read();
			if(cmd == 0x01){
				setSpeed();	
				timeout = millis();
			}
			else if(cmd == 0x67){
				setAcceleration();
				timeout = millis();
			}	
		}	
	}
	delay(5);
}

void setSpeed(){
	uint8_t new_msg_cnt = Serial.read();
	if(new_msg_cnt == (speed_msg_cnt+1)){
		speed_msg_cnt += 1;
	}
	else{
		speed_msg_error += 1;
		// Serial.println("ERROR!!");
	}
	int8_t data[4];
	for(int i = 3; i >= 0; i--){
		// Serial.read();
		data[i] = Serial.read();
	}

	int16_t *speed_1, *speed_2;
	speed_1 = (int16_t *)(&data[2]);
	speed_2 = (int16_t *)(&data[0]);

	stepper.setSpeed(*speed_1, *speed_2);

	// if(*accel_mode == 2){
	// 	stepper.setAcceleration(10000,10000);
	// }
	// else if(*accel_mode == 1){
	// 	stepper.setAcceleration(7500,7500);
	// }
	// else{
	// 	stepper.setAcceleration(5000,5000);
	// }

	/* Selftest section */

	#ifdef READ_TEST
		// Serial.print("Motor Speed: ");
		// Serial.print(*speed_1);
		// Serial.print(" ");
		// Serial.print(*speed_2);
		// Serial.print(" Accel mode: ");
		// Serial.println(stepper.accel_mode);
	#endif
}

// void setAccelerationMode(){
// 	Serial.read();

// 	int8_t data[4];
// 	for(int i = 3; i >= 0; i--){
// 		data[i] = Serial.read();
// 	}
// 	int16_t *accel_mode;
// 	accel_mode = (int16_t *)(&data[2]);
// 	if(*accel_mode == 2){
// 		stepper.setAcceleration(10000,10000);
// 	}
// 	else if(*accel_mode == 1){
// 		stepper.setAcceleration(7500,7500);
// 	}
// 	else{
// 		stepper.setAcceleration(5000,5000);
// 	}

// 	#ifdef READ_TEST
// 		Serial.print("Acceleration mode: ");
// 		Serial.println(*accel_mode);
// 	#endif

// }
void setAcceleration(){
	uint8_t new_msg_cnt = Serial.read();
	if(new_msg_cnt == (accel_msg_cnt+1)){
		accel_msg_cnt += 1;
	}
	else{
		accel_msg_cnt += 1;
	}

	int8_t data[4];
	for(int i = 3; i >= 0; i--){
		data[i] = Serial.read();
	}
	int16_t *accel0, *accel1;
	accel0 = (int16_t *)(&data[0]);
	accel1 = (int16_t *)(&data[2]);
	stepper.setAcceleration(*accel0, *accel1);

	/* Selftest section */

	#ifdef READ_TEST
		Serial.print("Acceleration: ");
		Serial.print(*accel0);
		Serial.print(" ");
		Serial.print(*accel1);
		Serial.println();
	#endif
}

void encoder_read_task(void *pvParameters){
	TickType_t xLastWakeTime= xTaskGetTickCount();
	const TickType_t xFrequency = stepper.dt / portTICK_PERIOD_MS;

	for(;;){
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		stepper.getEncoderSpeed();
		
		#ifndef READ_TEST		
			Serial.write(0x6f);Serial.write(0x57);
			Serial.write(encoder_msg_cnt);
			Serial.write((stepper.tick_speed[0]>>8)&0xff);
			Serial.write(stepper.tick_speed[0]&0xff);
			Serial.write((stepper.tick_speed[1]>>8)&0xff);
			Serial.write(stepper.tick_speed[1]&0xff);
			encoder_msg_cnt += 1;
		#endif
		
	}
  
}

