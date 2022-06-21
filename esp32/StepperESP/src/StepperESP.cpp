#include "StepperESP.h"

StepperESP::StepperESP()
{
}
StepperESP::~StepperESP()
{
}

//   BASIC SETTINGS
//===================================
void StepperESP::setEnablePin(uint8_t enable_pin_)
{
	enable_pin = enable_pin_;
}
void StepperESP::setClockPin(uint8_t clk_pin_1, uint8_t clk_pin_2)
{
	clk_pin[0] = clk_pin_1;
	clk_pin[1] = clk_pin_2;
}
void StepperESP::setDirectionPin(uint8_t dir_pin_1, uint8_t dir_pin_2)
{
	dir_pin[0] = dir_pin_1;
	dir_pin[1] = dir_pin_2;
}
void StepperESP::setEncoderPin(const uint8_t encoderA_[2], const uint8_t encoderB_[2])
{
	encoderA[0] = encoderA_[0];
	encoderA[1] = encoderA_[1];

	encoderB[0] = encoderB_[0];
	encoderB[1] = encoderB_[1];
}

//   (RE)INITIALIZATION SECTION
//===================================
void StepperESP::reset()
{	
	disableTx = true;
	re_init = true;
	init_checklist = 0x00;

	sendInitReady();
	readInitParam();
}
void StepperESP::sendInitReady()
{
	Serial.write(MSG_HEADER::SEND_INIT_READY >> 8);
	Serial.write(MSG_HEADER::SEND_INIT_READY & 0xff);
	Serial.write(0x00);
	Serial.write(0x00);
}
void StepperESP::readInitParam(){
	uint32_t wait_time = millis();
	uint32_t max_wait_time = 200;
	int16_t cmd = 0;

	for(int i=0; i<retry_limit && init_checklist != 0x07; i++)
	{	
		wait_time = millis();
		
		while(Serial.available() < 5)
		{	
			if((millis() - wait_time) > max_wait_time)
			{	
				break;
			}
			delay(10);
		}

		while(Serial.available() > 5)
		{
			if((millis() - wait_time) > max_wait_time)
			{	
				break;
			}
			cmd = (cmd << 8) | Serial.read();
			if((cmd&0xff00) == MSG_HEADER::READ_HEADER)
			{
				break;
			}
			delay(10);
		}
		

		if((cmd&0xff00) == MSG_HEADER::READ_HEADER)
		{	
			if((cmd == MSG_HEADER::READ_ACCEL_PROFILE_CFG) 
			    && !init_checklist&0x01)
			{	
				Serial.read();
				int8_t data[16];
				for(int i = 15; i >= 0; i--){
					data[i] = Serial.read();
				}
				int32_t *accel_ = (int32_t *)(&data[12]);
				int32_t *decel_ = (int32_t *)(&data[8]);
				int32_t *brake_ = (int32_t *)(&data[4]);
				int32_t *weak_accel = (int32_t *)(&data[0]);
				
				setAccelerationProfile(*accel_, *decel_, *brake_, *weak_accel);
				init_checklist = init_checklist|0x01;
			}
			else if((cmd == MSG_HEADER::READ_DYNAMIC_ACCEL_CFG)
			          && !(init_checklist&0x02))
			{	
				Serial.read();
				int8_t data[8];
				for(int i = 7; i >= 0; i--){
					data[i] = Serial.read();
				}
				boundary_speed = *(int32_t*)(&data[4]);
				boundary_speed2 = *(int32_t*)(&data[0]);

				init_checklist = init_checklist|0x02;
			}
			else if((cmd == MSG_HEADER::READ_UPDATE_PERIOD_CFG) 
			        && !(init_checklist&0x04))
			{	
				Serial.read();
				int8_t data[4];
				for(int i = 3; i >= 0; i--){
					data[i] = Serial.read();
				}
				dt = *(float *)(&data[0]);
				init_checklist = init_checklist|0x04;
			}
		}
	}
	if(init_checklist != 0x07)
	{	
		sendError();
		char msg[] = "Failed Initialization";
		Serial.write((MSG_HEADER::SEND_HUMAN_MESSAGE >> 8) & 0xff);
		Serial.write(MSG_HEADER::SEND_HUMAN_MESSAGE & 0xff);
		Serial.print("Init flag: ");
		Serial.print(init_checklist, HEX);
		Serial.println();
		sendCustomMessage(msg);
	}
	else
	{
		char msg[] = "Successful Initialization";
		sendCustomMessage(msg);
		sendCustomMessageHeader();
		Serial.print(acceleration_val[0]); Serial.print(" ");
		Serial.print(acceleration_val[1]); Serial.print(" ");
		Serial.print(acceleration_val[2]); Serial.print(" ");
		Serial.print(acceleration_val[3]); Serial.print(" ");
		Serial.print(boundary_speed); Serial.print(" ");
		Serial.print(boundary_speed2); Serial.print(" ");
		Serial.println(dt);
		disableTx = false;
		re_init = false;
	}
}
int8_t StepperESP::getQueueState(){
	boolean queue0 = stepper[0]->isQueueFull();
	boolean queue1 = stepper[1]->isQueueFull();
	if(queue0 == queue1)
	{
		if(queue0 == false)
			return QUEUE_STATE::OK;
		else
			return QUEUE_STATE::FULL;
	}
	else if(queue0)
	{
		return QUEUE_STATE::STEPPER0_FULL;
	}
	else if(queue1)
	{
		return QUEUE_STATE::STEPPER1_FULL;
	}
}
void StepperESP::initializeStepperEncoder()
{
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
	stepper[0]->setAutoEnable(true);
	// stepper[0]->setDelayToDisable(10000);
	stepper[0]->setAcceleration(48000);
	stepper[0]->stopMove();
	// stepper[0]->applySpeedAcceleration();
	stepper[0]->runForward();

	stepper[1] = engine.stepperConnectToPin(clk_pin[1]);
	stepper[1]->setDirectionPin(dir_pin[1]);
	stepper[1]->setEnablePin(enable_pin);
	stepper[1]->setAutoEnable(true);
	// stepper[1]->setDelayToDisable(10000);
	stepper[1]->setAcceleration(48000);
	stepper[1]->stopMove();
	// stepper[1]->applySpeedAcceleration();
	stepper[1]->runForward();

	// digitalWrite(enable_pin, LOW);
	// Hello

	/* Attach pins to encoders */
	EncoderRTOS::useInternalWeakPullResistors = UP;
	encoder[0].setUnitPCNT(6);
	encoder[0].attachHalfQuad(encoderA[0], encoderA[1]);

	encoder[1].setUnitPCNT(7);
	encoder[1].attachHalfQuad(encoderB[0], encoderB[1]);
}
void StepperESP::readShutdownRequest()
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


//   VELOCITY CONTROL
//===================================

uint8_t StepperESP::GetAccelerationMode(int stepper_no)
{
	if (abs(set_tick_speed[stepper_no]) <= 100)
	{	
		if (abs(tick_speed_est[stepper_no]) > boundary_speed2)
		{
			return SPEED_SCENARIO::DECEL;
		}
		return SPEED_SCENARIO::BRAKE;
	}
	
	uint8_t ramp_state = stepper[stepper_no]->rampState() & RAMP_STATE_MASK;

	if (ramp_state & RAMP_STATE_REVERSE)
	{
		if (abs(tick_speed_est[stepper_no]) > boundary_speed2)
		{
			return SPEED_SCENARIO::DECEL;
		}
		return SPEED_SCENARIO::STRONG_DECEL;
	}
	else
	{
		done_deceleration_flag[stepper_no] = false;
		if (abs(tick_speed_est[stepper_no]) < boundary_speed)
		{
			return SPEED_SCENARIO::WEAK_ACCEL;
		}
		return SPEED_SCENARIO::ACCEL;
	}
}
void StepperESP::SetDynamicAcceleration()
{	
	static uint8_t prev_accel_mode0;
	static uint8_t prev_accel_mode1;
	static boolean new_accel_flag;

	uint8_t accel_mode0 = GetAccelerationMode(0);
	uint8_t accel_mode1 = GetAccelerationMode(1);

	tick_speed_est[0] = stepper[0]->getCurrentSpeedInMilliHz()/1000;
    tick_speed_est[1] = stepper[1]->getCurrentSpeedInMilliHz()/1000;

	if(accel_mode0 != prev_accel_mode0 || accel_mode1 != prev_accel_mode1)
	{
		new_accel_flag = true;
	}
	if(true)
	{
		new_speed_flag = false;
		new_accel_flag = false;
		prev_accel_mode0 = accel_mode0;
		prev_accel_mode1 = accel_mode1;

		int8_t accel_flag;
		if (accel_mode0 == accel_mode1 && accel_mode0 == SPEED_SCENARIO::BRAKE)
		{
			accel_flag = accel_mode0;
		}
		else
		{
			accel_flag = min(accel_mode0, accel_mode1);
		}
		uint32_t max_accel = acceleration_val[accel_flag];
		int32_t tick_accel_temp0 = abs(set_tick_speed[0] - tick_speed_est[0])*1000;
		int32_t tick_accel_temp1 = abs(set_tick_speed[1] - tick_speed_est[1])*1000;
		
		float divisor = 1.0;
		if(tick_accel_temp0 >= tick_accel_temp1)
		{	
			if(tick_accel_temp0 == 0)
			{
				divisor = 1.0;
			}
			else
			{
				divisor = (float)max_accel/tick_accel_temp0;
			}
		}
		else
		{
			divisor = (float)max_accel/tick_accel_temp1;
		}
		
		
		tick_accel[0] = tick_accel_temp0*divisor;
		tick_accel[1] = tick_accel_temp1*divisor;
		
		setAcceleration(tick_accel[0],tick_accel[1]);

	}
	
}
void StepperESP::SetDynamicAcceleration(boolean playground_enable)
{	
	if(playground_enable == false)
	{
		SetDynamicAcceleration();
	}
	else
	{
		
	}
	
}
int StepperESP::StallDetection()
{	
	float motor_encoder_ratio = 5.34;
	int32_t threshold[3] = {500, 1000, 2000};

	if(millis() - speed_overwrite_timer > speed_overwrite_timer_threshold)
	{
		speed_overwrite = false;
	}

	int32_t tick_diff_0 = tick_speed_est[0] - tick_speed[0] * motor_encoder_ratio;
	int32_t tick_diff_1 = tick_speed_est[1] - tick_speed[1] * motor_encoder_ratio;

	if(abs(tick_diff_0) > threshold[0] || abs(tick_diff_1) > threshold[0])
	{
		sendCustomMessage("Stall Debugging: Motor stall detected");
		sendCustomMessageHeader();
		if(!disableTx)
		{
			Serial.print("Stall difference: "); 
			Serial.println(tick_diff_0);
		}
		Serial.print("Stall difference: "); Serial.println(tick_diff_0);
		setSpeed(0, 0);
		speed_overwrite = true;
		speed_overwrite_timer = millis();

		if(abs(tick_diff_0) > threshold[0])
		{
			return MOTOR_1_STALL_STATUS_MASK;
		}
		if(abs(tick_diff_1) > threshold[0])
		{
			return MOTOR_2_STALL_STATUS_MASK;
		}
	}
	return 0;
}
void StepperESP::setAcceleration(int16_t accel0, int16_t accel1)
{
	stepper[0]->setAcceleration(accel0);
	stepper[1]->setAcceleration(accel1);
}
void StepperESP::setDirection(boolean dir0, boolean dir1)
{
	if (dir0 == true){
		stepper[0]->runForward();
	}
	else{
		stepper[0]->runBackward();
	}
	stepper[0]->applySpeedAcceleration();


	if (dir1 == true){
		stepper[1]->runForward();
	}
	else
	{
		stepper[1]->runBackward();
	}
	stepper[1]->applySpeedAcceleration();
}
void StepperESP::setSpeed(int32_t speed0, int32_t speed1)
{
	if(!re_init)
	{
		// if(abs(set_tick_speed[0] - speed0) > 20)
		// {
		// 	set_tick_speed[0] = speed0;
		// 	new_speed_flag = true;
		// }
		// if(abs(set_tick_speed[1] - speed1) > 20)
		// {
		// 	set_tick_speed[1] = speed1;
		// 	new_speed_flag = true;
		// }

		uint32_t ab_speed0 = abs(speed0);
		uint32_t ab_speed1 = abs(speed1);
		// Avoid step commands that are too small
		dir[0] = speed0 > 0;
		dir[1] = speed1 > 0;
		setDirection(dir[0] > 0, dir[1] > 0);

		if (ab_speed0 < 20)
		{
			stepper[0]->stopMove();
		}
		else
		{
			stepper[0]->setSpeedInHz(ab_speed0);
		}
		if (ab_speed1 < 20)
		{
			stepper[1]->stopMove();
		}
		else
		{
			stepper[1]->setSpeedInHz(ab_speed1);
		}
	}
}
void StepperESP::getEncoderSpeed()
{
	tick_speed[0] = (int16_t)((float)encoder[0].getCount() / (dt * 0.001));
	tick_speed[1] = (int16_t)((float)encoder[1].getCount() / (dt * 0.001));
	encoder[0].clearCount();
	encoder[1].clearCount();
}

//   MESSAGE CONTROL
//===================================

void StepperESP::sendEncoderSpeed()
{
	if(!disableTx)
	{
		Serial.write(MSG_HEADER::SEND_ENCODER >> 8);
		Serial.write(MSG_HEADER::SEND_ENCODER & 0xff);
		Serial.write(encoder_msg_cnt);
		Serial.write(tick_speed[0] >> 8);
		Serial.write(tick_speed[0] & 0xff);
		Serial.write(tick_speed[1] >> 8);
		Serial.write(tick_speed[1] & 0xff);
		encoder_msg_cnt += 1;
	}
}
void StepperESP::sendSpeedProfile()
{   
	if(!disableTx)
	{
		Serial.write((MSG_HEADER::SEND_FULL_SPEED_PROFILE>>8) & 0xff);
		Serial.write(MSG_HEADER::SEND_FULL_SPEED_PROFILE & 0xff);

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
void StepperESP::sendSpeedProfile(int32_t est_tick_vel0, int32_t est_tick_vel1, int32_t set_tick_accel0, int32_t set_tick_accel1)
{
	if(!disableTx)
	{
		Serial.write((MSG_HEADER::SEND_FULL_SPEED_PROFILE>>8) & 0xff);
		Serial.write(MSG_HEADER::SEND_FULL_SPEED_PROFILE & 0xff);

		Serial.write((set_tick_accel0 >> 24) & 0xff);
		Serial.write((set_tick_accel0 >> 16) & 0xff);
		Serial.write((set_tick_accel0 >> 8) & 0xff);
		Serial.write(set_tick_accel0 & 0xff);

		Serial.write((set_tick_accel1 >> 24) & 0xff);
		Serial.write((set_tick_accel1 >> 16) & 0xff);
		Serial.write((set_tick_accel1 >> 8) & 0xff);
		Serial.write(set_tick_accel1 & 0xff);

		Serial.write((est_tick_vel0 >> 24) & 0xff);
		Serial.write((est_tick_vel0 >> 16) & 0xff);
		Serial.write((est_tick_vel0 >> 8) & 0xff);
		Serial.write(est_tick_vel0 & 0xff);

		Serial.write((est_tick_vel1 >> 24) & 0xff);
		Serial.write((est_tick_vel1 >> 16) & 0xff);
		Serial.write((est_tick_vel1 >> 8) & 0xff);
		Serial.write(est_tick_vel1 & 0xff);

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
void StepperESP::sendCustomMessage(const char *msg)
{	
	if(!disableTx)
	{	
		Serial.write((MSG_HEADER::SEND_HUMAN_MESSAGE >> 8) & 0xff);
		Serial.write(MSG_HEADER::SEND_HUMAN_MESSAGE & 0xff);
		Serial.println(msg);
	}
}
void StepperESP::sendCustomMessageHeader()
{
	Serial.write((MSG_HEADER::SEND_HUMAN_MESSAGE >> 8) & 0xff);
	Serial.write(MSG_HEADER::SEND_HUMAN_MESSAGE & 0xff);
}
void StepperESP::sendError()
{   
	if(!disableTx)
	{
		Serial.write((MSG_HEADER::SEND_ERROR>>8) & 0xff);
		Serial.write(MSG_HEADER::SEND_ERROR & 0xff);
		Serial.write(0x00);
		Serial.write(0x00);
	}
	
}

//   DEPRECATED SECTION
//===================================

void StepperESP::setSpeedAccel(int16_t speed0, int16_t speed1)
{
	if(!re_init)
	{	
		uint16_t ab_speed0 = abs(speed0);
		uint16_t ab_speed1 = abs(speed1);
		int32_t accel0 = 0;
		int32_t accel1 = 0;
		// Avoid step commands that are too small
		boolean dir0 = speed0 > 0;
		boolean dir1 = speed1 > 0;

		setDirection(dir0 > 0, dir1 > 0);
		if (ab_speed0 < 20)
			stepper[0]->stopMove();
		else
			stepper[0]->setSpeedInHz(ab_speed0);

		if (ab_speed1 < 20)
			stepper[1]->stopMove();
		else
			stepper[1]->setSpeedInHz(ab_speed1);

		uint8_t accel_mode0 = SetAccelerationMode(speed0, 0);
		uint8_t accel_mode1 = SetAccelerationMode(speed1, 1);

#ifdef READ_TEST
		Serial.println("Acceleration mode: ");
		Serial.print(accel_mode0);
		Serial.print(" ");
		Serial.println(accel_mode1);

#endif
		if (accel_mode0 == accel_mode1 && accel_mode0 == SPEED_SCENARIO::BRAKE)
		{	
			if(use_dynamic_accel)
			{
				if (brake_flag == false)
				{
					accel0 = stepper[0]->getCurrentSpeedInMilliHz() / decel_divisor; //Todo: a configurable variable
					accel1 = stepper[1]->getCurrentSpeedInMilliHz() / decel_divisor; //Todo: a configurable variable
					brake_flag = true;
				}
				setAcceleration(accel0, accel1);
			}
			
			setAcceleration(acceleration_val[SPEED_SCENARIO::BRAKE],
							acceleration_val[SPEED_SCENARIO::BRAKE]);
		}
		else
		{
			brake_flag = false;
			int8_t accel_flag = min(accel_mode0, accel_mode1);
			setAcceleration(acceleration_val[accel_flag], 
							acceleration_val[accel_flag]);
		}

	}
}
uint8_t StepperESP::SetAccelerationMode(int16_t speed, FastAccelStepper *stepper)
{
	if (stepper->getCurrentSpeedInUs() == 0)
	{
		return 0;
	}
	if (speed != 0)
	{
		bool positive = speed / stepper->getCurrentSpeedInUs() >= 0;
		double gain = (speed * 1000.0) / stepper->getCurrentSpeedInMilliHz();
		if (!positive)
		{
			return 1;
		}
		else if (gain >= 1)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		return 2;
	}
}
uint8_t StepperESP::SetAccelerationMode(int16_t speed, int stepper_no)
{
#ifdef READ_TEST
	Serial.print("Internal speed: ");
	Serial.print(stepper_no);
	Serial.print(" ");
	Serial.println(stepper[stepper_no]->getCurrentSpeedInMilliHz() / 1000.0);
#endif
	deceleration_duration[stepper_no] = millis() - deceleration_timer[stepper_no];
	if (deceleration_flag[stepper_no] &&
		deceleration_duration[stepper_no] > deceleration_time_limit)
	{
		deceleration_flag[stepper_no] = false;
		return SPEED_SCENARIO::BRAKE;
	}
	if (stepper[stepper_no]->getCurrentSpeedInUs() == 0)
	{
		return SPEED_SCENARIO::ACCEL;
	}
	if (speed != 0)
	{
		// bool positive = speed/stepper[stepper_no]->getCurrentSpeedInUs() >= 0;
		// double gain = (speed*1000.0)/stepper[stepper_no]->getCurrentSpeedInMilliHz();

		uint8_t ramp_state = stepper[stepper_no]->rampState() & RAMP_STATE_MASK;
		if (ramp_state & RAMP_STATE_REVERSE)
		{
			if (!deceleration_flag[stepper_no] &&
				!done_deceleration_flag[stepper_no])
			{
				deceleration_flag[stepper_no] = true;
				done_deceleration_flag[stepper_no] = true;
				deceleration_timer[stepper_no] = millis();
			}
			if (!deceleration_flag[stepper_no] &&
				done_deceleration_flag[stepper_no])
			{
				return SPEED_SCENARIO::BRAKE;
			}
			return SPEED_SCENARIO::DECEL;
		}
		else
		{
			done_deceleration_flag[stepper_no] = false;
			return SPEED_SCENARIO::ACCEL;
		}
	}
	else
	{
		return SPEED_SCENARIO::BRAKE;
	}
}
void StepperESP::motorControl()
{
	recvWithStartEndMarkers();
	if (newData == true)
	{
		if (abs(motors_vel[0]) < 20)
		{
			stepper[0]->stopMove();
		}
		else
		{
			stepper[0]->setSpeedInHz(motors_vel[0]);
			if (motors_fw[0] == true)
			{
				stepper[0]->applySpeedAcceleration();
				stepper[0]->runForward();
			}
			else
			{
				stepper[0]->applySpeedAcceleration();
				stepper[0]->runBackward();
			}
		}
		newData = false;

		if (abs(motors_vel[1]) < 20)
		{
			stepper[1]->stopMove();
		}
		else
		{
			stepper[1]->setSpeedInHz(motors_vel[1]);
			if (motors_fw[1] == true)
			{
			
				stepper[1]->applySpeedAcceleration();
				stepper[1]->runForward();
			}
			else
			{
				stepper[1]->applySpeedAcceleration();
				stepper[1]->runBackward();
			}
		}
		newData = false;
	}
}
void StepperESP::recvWithStartEndMarkers()
{
	static boolean recvInProgress = false;
	static byte ndx = 0;
	char startMarker = '<';
	char endMarker = '>';
	char rc;

	// if (Serial.available() > 0) {
	while (Serial.available() > 0 && newData == false)
	{
		rc = Serial.read();

		if (recvInProgress == true)
		{
			if (rc != endMarker)
			{
				receivedChars[ndx] = rc;
				ndx++;
				if (ndx >= numChars)
				{
					ndx = numChars - 1;
				}
			}
			else
			{
				receivedChars[ndx] = '\0'; // terminate the string
				recvInProgress = false;
				ndx = 0;
				newData = true;

				memcpy(left_received_arr, receivedChars, 8);
				memcpy(right_received_arr, receivedChars + 9, 8);

				motors_vel_float[0] = atof(left_received_arr);
				motors_vel[0] = abs(motors_vel_float[0]);
				if (motors_vel_float[0] > -0.001)
				{
					motors_fw[0] = true;
				}
				else
				{
					motors_fw[0] = false;
				}

				motors_vel_float[1] = atof(right_received_arr);
				motors_vel[1] = abs(motors_vel_float[1]);
				if (motors_vel_float[1] > -0.001)
				{
					motors_fw[1] = true;
				}
				else
				{
					motors_fw[1] = false;
				}
			}
		}

		else if (rc == startMarker)
		{
			recvInProgress = true;
		}
	}
}
void StepperESP::UpdateVelocityRamp()
{	
	static float update_period = 0.02;
	static int32_t tick_diff[2];
	static int8_t direction[2];
	static int32_t accel[2];
	static int32_t ramp_step[2];

	tick_speed_est[0] = stepper[0]->getCurrentSpeedInMilliHz()/1000;
    tick_speed_est[1] = stepper[1]->getCurrentSpeedInMilliHz()/1000;
       
	if(abs(tick_speed_est[0] - projected_tick_velocity[0]) < 200 &&
	   abs(tick_speed_est[1] - projected_tick_velocity[1]) < 200)
	{
		// Determine whether the motor needs to decelerate, accelerate, or brake
		// Update the tick difference between the current and set velocity
		// and the move direction.
		
		
		uint8_t accel_mode0 = GetMotorState(0, tick_diff[0], direction[0]);
		uint8_t accel_mode1 = GetMotorState(1, tick_diff[1], direction[1]);
		sendCustomMessageHeader();
		Serial.print("Current Acceleration Mode: ");
		Serial.print(accel_mode0); Serial.print(" ");
		Serial.println(accel_mode1);
		uint8_t which_mode = min(accel_mode0, accel_mode1);
		if(accel_mode0 == SPEED_SCENARIO::BRAKE &&
		accel_mode1 == SPEED_SCENARIO::BRAKE)
		{
			which_mode = SPEED_SCENARIO::BRAKE;
		}
		stepper[0]->setAcceleration(acceleration_val[which_mode]);
		stepper[1]->setAcceleration(acceleration_val[which_mode]);

		accel[0] = acceleration_val[which_mode];
		accel[1] = acceleration_val[which_mode];

		sendCustomMessageHeader();
		Serial.print("Apply new tick velocity ");
		Serial.print(accel[0]); Serial.print(" ");
		Serial.print(accel[1]); Serial.print(" ");


		// Determine the appropriate step so that both motors
		// can reach the set velocities at the same time.
		
		float divisor;

		if(abs(tick_diff[1]) < abs(tick_diff[0]))
		{
			divisor = (float)abs(tick_diff[1])/abs(tick_diff[0]);
			accel[1] *= divisor;
		}
		else
		{
			divisor = (float)abs(tick_diff[0])/abs(tick_diff[1]);
			accel[0] *= divisor;
		}

		if(tick_diff[0] == 0){
			accel[0] = 0;
		}
		if(tick_diff[1] == 0){
			accel[1] = 0;
		}

		ramp_step[0] = direction[0] * accel[0] * update_period;
		ramp_step[1] = direction[1] * accel[1] * update_period;

		if(abs(ramp_step[0]) > abs(tick_diff[0]))
		{
			ramp_step[0] = tick_diff[0];
		}
		if(abs(ramp_step[1]) > abs(tick_diff[1]))
		{
			ramp_step[1] = tick_diff[1];
		}

		
		Serial.print(ramp_step[0]); Serial.print(" ");
		Serial.print(ramp_step[1]); Serial.print(" ");
		Serial.print(projected_tick_velocity[0]); Serial.print(" ");
		Serial.print(projected_tick_velocity[1]); Serial.print(" ");
		Serial.print(tick_speed_est[0]); Serial.print(" ");
		Serial.print(tick_speed_est[1]); Serial.print(" ");
		Serial.print(set_tick_speed[0]); Serial.print(" ");
		Serial.println(set_tick_speed[1]);

		projected_tick_velocity[0] = projected_tick_velocity[0] + ramp_step[0];
		projected_tick_velocity[1] = projected_tick_velocity[1] + ramp_step[1];

		setSpeed(projected_tick_velocity[0], projected_tick_velocity[1]);
	}
	else
	{	
		sendCustomMessageHeader();
		Serial.print("Re-set tick velocity    ");
		Serial.print(accel[0]); Serial.print(" ");
		Serial.print(accel[1]); Serial.print(" ");
		Serial.print(ramp_step[0]); Serial.print(" ");
		Serial.print(ramp_step[1]); Serial.print(" ");
		Serial.print(projected_tick_velocity[0]); Serial.print(" ");
		Serial.print(projected_tick_velocity[1]); Serial.print(" ");
		Serial.print(tick_speed_est[0]); Serial.print(" ");
		Serial.print(tick_speed_est[1]); Serial.print(" ");
		Serial.print(set_tick_speed[0]); Serial.print(" ");
		Serial.println(set_tick_speed[1]);
		setSpeed(projected_tick_velocity[0], projected_tick_velocity[1]);
	}
	
}
int8_t StepperESP::GetMotorState(int stepper_no, int32_t &tick_diff, int8_t &direction)
{
	tick_diff = abs(set_tick_speed[stepper_no] - tick_speed_est[stepper_no]);
	bool positive = set_tick_speed[stepper_no] * tick_speed_est[stepper_no] >= 0;
	if(abs(set_tick_speed[stepper_no]) < 100)
	{	
		if(tick_speed_est[stepper_no] > 0)
		{
			direction = -1;
		}
		else
		{
			direction = 1;
		}
		return SPEED_SCENARIO::BRAKE;
	}
	
	if(tick_speed_est[stepper_no] == 0)
	{
		if(set_tick_speed[stepper_no] > 0)
			direction = 1;
		else
			direction = -1;
		return SPEED_SCENARIO::WEAK_ACCEL;
	}
	if(!positive)
	{
		if(tick_speed_est[stepper_no] > 0)
			direction = -1;
		else
			direction = 1;
		return SPEED_SCENARIO::STRONG_DECEL;
	}

	if (tick_speed_est[stepper_no] < set_tick_speed[stepper_no])
	{	
		if(tick_speed_est[stepper_no] > 0)
		{
			direction = 1;
		}
		else
		{
			direction = -1;
		}
		if(abs(tick_speed_est[stepper_no]) < boundary_speed)
		{
			return SPEED_SCENARIO::WEAK_ACCEL;
		}
		return SPEED_SCENARIO::ACCEL;
	}
	
	if(tick_speed_est[stepper_no] > 0)
	{
		direction = -1;
	}
	else
	{
		direction = 1;
	}
	return SPEED_SCENARIO::DECEL;
}
