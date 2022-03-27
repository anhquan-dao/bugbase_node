#include "StepperESP.h"

StepperESP::StepperESP(){

}
StepperESP::~StepperESP(){

}
void StepperESP::setEnablePin(uint8_t enable_pin_){
		enable_pin = enable_pin_;
}
void StepperESP::setClockPin(uint8_t clk_pin_1, uint8_t clk_pin_2){
		clk_pin[0] = clk_pin_1;
		clk_pin[1] = clk_pin_2;
}
void StepperESP::setDirectionPin(uint8_t dir_pin_1, uint8_t dir_pin_2){
		dir_pin[0] = dir_pin_1;
		dir_pin[1] = dir_pin_2;
}
void StepperESP::setEncoderPin(const uint8_t encoderA_[2], const uint8_t encoderB_[2]){
	encoderA[0] = encoderA_[0];
	encoderA[1] = encoderA_[1];

	encoderB[0] = encoderB_[0];
	encoderB[1] = encoderB_[1];
}
void StepperESP::initialize(){
		/* Setup stepper driver  */
		// serial_1.begin(115200, SERIAL_8N1, 33, 25);
		// driver = new TMC2209Stepper(&serial_1, R_SENSE, DRIVER_ADDRESS);
		// driver -> begin();                 
		// driver -> toff(5);                 // Enables driver in software
		// driver -> rms_current(600);        // Set motor RMS current
		// driver -> microsteps(8);           // Set microsteps to 1/8th
		// driver -> en_spreadCycle(false);   // Toggle spreadCycle off TMC2208/2209/2224
		//                                 // hence we use StealthChop
		// driver -> pwm_autoscale(true);     // Needed for stealthChop

		/* Attach pins to stepper controllers */

	engine.init();

	stepper[0] = engine.stepperConnectToPin(clk_pin[0]);
	stepper[0]->setDirectionPin(dir_pin[0]);
	stepper[0]->setEnablePin(enable_pin);
	// stepper[0]->setAutoEnable(true);
	// stepper[0]->setDelayToDisable(10000);
	stepper[0]->setAcceleration(7500);
	stepper[0]->setSpeedInHz(0);
	stepper[0]->runForward();

	stepper[1] = engine.stepperConnectToPin(clk_pin[1]);
	stepper[1]->setDirectionPin(dir_pin[1]);
	stepper[1]->setEnablePin(enable_pin);
	// stepper[1]->setAutoEnable(true);
	// stepper[1]->setDelayToDisable(10000);
	stepper[1]->setAcceleration(7500);
	stepper[1]->setSpeedInHz(0);
	stepper[1]->runForward();

	digitalWrite(enable_pin, LOW);

	/* Attach pins to encoders */
	EncoderRTOS::useInternalWeakPullResistors=UP;
	encoder[0].setUnitPCNT(6);
	encoder[0].attachHalfQuad(encoderA[0], encoderA[1]);

	encoder[1].setUnitPCNT(7);
	encoder[1].attachHalfQuad(encoderB[0], encoderB[1]);

}
void StepperESP::getEncoderSpeed(){
	tick_speed[0] = (int16_t)((float)encoder[0].getCount()/(dt*0.001));
	tick_speed[1] = (int16_t)((float)encoder[1].getCount()/(dt*0.001));
	encoder[0].clearCount();
	encoder[1].clearCount();
}
void StepperESP::setSpeed(int16_t speed0, int16_t speed1){

	static long accel0, accel1;
	// Avoid step commands that are too small
	setDirection(speed0>0, speed1>0);
	if(abs(speed0) < 20) stepper[0]->stopMove();
	else stepper[0]->setSpeedInHz(abs(speed0));
	
	if(abs(speed1) < 20) stepper[1]->stopMove();
	else stepper[1]->setSpeedInHz(abs(speed1));

	
	uint8_t accel_mode0 = SetAccelerationMode(speed0, stepper[0]);
	uint8_t accel_mode1 = SetAccelerationMode(speed1, stepper[1]);

	accel_mode = min(accel_mode0, accel_mode1);
 


	if(accel_mode == 2){
		if(brake_flag == false){
			accel0 = stepper[0]->getCurrentSpeedInMilliHz()/80;
			accel1 = stepper[1]->getCurrentSpeedInMilliHz()/80;
			brake_flag = true;
		}
		Serial.print(accel0);
		Serial.print(" ");
		Serial.println(accel1);
		setAcceleration(abs(accel0), abs(accel1));
	}
	else{
		setAcceleration(acceleration_val[accel_mode], acceleration_val[accel_mode]);
		brake_flag = false;
	}
	
}
uint8_t StepperESP::SetAccelerationMode(int16_t speed, FastAccelStepper *stepper){
	// Determine "Speed Scenario" and appropriate accel/decel value
	if(stepper->getCurrentSpeedInUs() == 0){
		return 0;
	}
	if(speed != 0){
		bool positive = speed/stepper->getCurrentSpeedInUs() >= 0;
		double gain = (speed*1000.0)/stepper->getCurrentSpeedInMilliHz();
		if(!positive){
			return 1;
		}
		else if(gain >= 1){
			return 0;
		}
		else{
			return 1;
		}
	}
	else{
		return 2;
	}
}
void StepperESP::setAcceleration(int16_t accel0, int16_t accel1){
	stepper[0]->setAcceleration(accel0);
	stepper[1]->setAcceleration(accel1);
}
void StepperESP::setDirection(boolean dir0, boolean dir1){
	if(dir0 == true) stepper[0]->runForward();
	else stepper[0]->runBackward();

	if(dir1 == true) stepper[1]->runForward();
	else stepper[1]->runBackward();
}
void StepperESP::motorControl(){
	recvWithStartEndMarkers();
	if (newData == true) {
		if (abs(motors_vel[0]) < 20) {
		stepper[0]->stopMove();
		}
		else {
		stepper[0]->setSpeedInHz(motors_vel[0]);
		if (motors_fw[0] == true) {
			stepper[0]->runForward();
		}
		else {
			stepper[0]->runBackward();
		}
		}
		newData = false;
		
		if (abs(motors_vel[1]) < 20) {
		stepper[1]->stopMove();
		}
		else {
		stepper[1]->setSpeedInHz(motors_vel[1]);
		if (motors_fw[1] == true) {
			stepper[1]->runForward();
		}
		else {
			stepper[1]->runBackward();
		}
		}
		newData = false;
	}
}
void StepperESP::recvWithStartEndMarkers(){
	static boolean recvInProgress = false;
	static byte ndx = 0;
	char startMarker = '<';
	char endMarker = '>';
	char rc;

	// if (Serial.available() > 0) {
	while (Serial.available() > 0 && newData == false) {
		rc = Serial.read();

		if (recvInProgress == true) {
			if (rc != endMarker) {
				receivedChars[ndx] = rc;
				ndx++;
				if (ndx >= numChars) {
					ndx = numChars - 1;
				}
			}
			else {
				receivedChars[ndx] = '\0'; // terminate the string
				recvInProgress = false;
				ndx = 0;
				newData = true;

				memcpy(left_received_arr,  receivedChars,     8);
				memcpy(right_received_arr, receivedChars + 9, 8);
				
				motors_vel_float[0] = atof(left_received_arr);
				motors_vel[0] = abs(motors_vel_float[0]);
				if (motors_vel_float[0] > -0.001) {
					motors_fw[0]  = true;
				}
				else {
					motors_fw[0]  = false;
				}

				motors_vel_float[1] = atof(right_received_arr);
				motors_vel[1] = abs(motors_vel_float[1]);
				if (motors_vel_float[1] > -0.001) {
					motors_fw[1]  = true;
				}
				else {
					motors_fw[1]  = false;
				}
			}
		}

		else if (rc == startMarker) {
			recvInProgress = true;
		}
	}
}