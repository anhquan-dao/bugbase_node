/*
 * EncoderRTOS.cpp
 *
 *  Created on: Oct 15, 2018
 *      Author: hephaestus
 */

#include <EncoderRTOS.h>



//static EncoderRTOS *gpio2enc[48];
//
//
enum puType EncoderRTOS::useInternalWeakPullResistors=DOWN;
EncoderRTOS *EncoderRTOS::encoders[MAX_ESP32_ENCODERS] = {NULL, NULL, NULL, NULL};

bool EncoderRTOS::attachedInterrupt=false;
pcnt_isr_handle_t EncoderRTOS::user_isr_handle = NULL;

EncoderRTOS::EncoderRTOS() {
	attached = false;
	aPinNumber = (gpio_num_t) 0;
	bPinNumber = (gpio_num_t) 0;
	working = false;
	direction = false;
	unit = (pcnt_unit_t) -1;
}

EncoderRTOS::~EncoderRTOS() {
}

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */
static void IRAM_ATTR pcnt_isr_service(void *arg) {
	EncoderRTOS *enc_ptr = (EncoderRTOS*) arg;
	const pcnt_unit_t i = enc_ptr->unit;

	int64_t status = PCNT.int_st.val;
	// Serial.println(enc_ptr->count);
	if(PCNT.status_unit[i].h_lim_lat){
		status=enc_ptr->r_enc_config.counter_h_lim;
	}
	if(PCNT.status_unit[i].l_lim_lat){
		status=enc_ptr->r_enc_config.counter_l_lim;
	}
	enc_ptr->count = status + enc_ptr->count;
	
	// for (i = 0; i < PCNT_UNIT_MAX; i++) {
	// 	if (intr_status & (BIT(i))) {
	// 		enc_ptr = EncoderRTOS::encoders[i];
	// 		/* Save the PCNT event type that caused an interrupt
	// 		 to pass it to the main program */

	// 		/* int64_t status=0;
	// 		if(PCNT.status_unit[i].h_lim_lat){
	// 			status=ptr->r_enc_config.counter_h_lim;
	// 		}
	// 		if(PCNT.status_unit[i].l_lim_lat){
	// 			status=ptr->r_enc_config.counter_l_lim;
	// 		}
	// 		//pcnt_counter_clear(ptr->unit);
	// 		PCNT.int_clr.val = BIT(i); // clear the interrupt
	// 		ptr->count = status + ptr->count; */
    //         ptr->overflow_count += 1;
	// 	}   
	// }
    
}

void EncoderRTOS::attach(int a, int b, enum encType et) {
	if (attached) {
		Serial.println("Already attached, FAIL!");
		return;
	}

    int index = 0;
	for (; index < MAX_ESP32_ENCODERS; index++) {
		if (EncoderRTOS::encoders[index] == NULL) {
			encoders[index] = this;
			break;
		}
	}
	if (index == MAX_ESP32_ENCODERS) {
		Serial.println("Too many encoders, FAIL!");
		return;
	}
	// Set data now that pin attach checks are done
	fullQuad = et != single;
	this->aPinNumber = (gpio_num_t) a;
	this->bPinNumber = (gpio_num_t) b;

	//Set up the IO state of hte pin
	gpio_pad_select_gpio(aPinNumber);
	gpio_pad_select_gpio(bPinNumber);
	gpio_set_direction(aPinNumber, GPIO_MODE_INPUT);
	gpio_set_direction(bPinNumber, GPIO_MODE_INPUT);
	if(useInternalWeakPullResistors==DOWN){
		gpio_pulldown_en(aPinNumber);
		gpio_pulldown_en(bPinNumber);
	}
	if(useInternalWeakPullResistors==UP){
		gpio_pullup_en(aPinNumber);
		gpio_pullup_en(bPinNumber);
	}
	// Set up encoder PCNT configuration
	r_enc_config.pulse_gpio_num = aPinNumber; //Rotary Encoder Chan A
	r_enc_config.ctrl_gpio_num = bPinNumber;    //Rotary Encoder Chan B

	r_enc_config.unit = unit;
	r_enc_config.channel = PCNT_CHANNEL_0;

	r_enc_config.pos_mode = fullQuad ? PCNT_COUNT_DEC : PCNT_COUNT_DIS; //Count Only On Rising-Edges
	r_enc_config.neg_mode = PCNT_COUNT_INC;   // Discard Falling-Edge

	r_enc_config.lctrl_mode = PCNT_MODE_KEEP;    // Rising A on HIGH B = CW Step
	r_enc_config.hctrl_mode = PCNT_MODE_REVERSE; // Rising A on LOW B = CCW Step

	r_enc_config		.counter_h_lim = _INT16_MAX;
	r_enc_config		.counter_l_lim = _INT16_MIN;

	pcnt_unit_config(&r_enc_config);

	if (et == full) {
		// set up second channel for full quad
		r_enc_config.pulse_gpio_num = bPinNumber; //make prior control into signal
		r_enc_config.ctrl_gpio_num = aPinNumber;    //and prior signal into control

		r_enc_config.unit = unit;
		r_enc_config.channel = PCNT_CHANNEL_1; // channel 1

		r_enc_config.pos_mode = PCNT_COUNT_DEC; //Count Only On Rising-Edges
		r_enc_config.neg_mode = PCNT_COUNT_INC;   // Discard Falling-Edge

		r_enc_config.lctrl_mode = PCNT_MODE_REVERSE;    // prior high mode is now low
		r_enc_config.hctrl_mode = PCNT_MODE_KEEP; // prior low mode is now high

		r_enc_config		.counter_h_lim = _INT16_MAX;
		r_enc_config		.counter_l_lim = _INT16_MIN;

		pcnt_unit_config(&r_enc_config);
	} else { // make sure channel 1 is not set when not full quad
		r_enc_config.pulse_gpio_num = bPinNumber; //make prior control into signal
		r_enc_config.ctrl_gpio_num = aPinNumber;    //and prior signal into control

		r_enc_config.unit = unit;
		r_enc_config.channel = PCNT_CHANNEL_1; // channel 1

		r_enc_config.pos_mode = PCNT_COUNT_DIS; //disabling channel 1
		r_enc_config.neg_mode = PCNT_COUNT_DIS;   // disabling channel 1

		r_enc_config.lctrl_mode = PCNT_MODE_DISABLE;    // disabling channel 1
		r_enc_config.hctrl_mode = PCNT_MODE_DISABLE; // disabling channel 1

		r_enc_config		.counter_h_lim = _INT16_MAX;
		r_enc_config		.counter_l_lim = _INT16_MIN;

		pcnt_unit_config(&r_enc_config);	
	}


	// Filter out bounces and noise
	setFilter(0); // Filter Runt Pulses

	/* Enable events on maximum and minimum limit values */
	pcnt_event_enable(unit, PCNT_EVT_H_LIM);
	pcnt_event_enable(unit, PCNT_EVT_L_LIM);

	pcnt_counter_pause(unit); // Initial PCNT init
	pcnt_counter_clear(unit);
	/* Register ISR handler and enable interrupts for PCNT unit */
	// if(attachedInterrupt==false){
	// 	attachedInterrupt=true;
	// 	esp_err_t er = pcnt_isr_register(pcnt_example_intr_handler,(void *) NULL, (int)0,
	// 			(pcnt_isr_handle_t *)&EncoderRTOS::user_isr_handle);
	// 	if (er != ESP_OK){
	// 		Serial.println("Encoder wrap interrupt failed");
	// 	}
	// }

    if(attachedInterrupt==false){
		attachedInterrupt=true;
		Serial.println("PCNT ISR installation");
        esp_err_t er = pcnt_isr_service_install(ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM);
		er = pcnt_isr_handler_add(unit, pcnt_isr_service, (void *) this);
		if (er != ESP_OK){
			Serial.println("Encoder wrap interrupt failed");
		}
	}
	pcnt_intr_enable(unit);
	pcnt_counter_resume(unit);

}

void EncoderRTOS::setUnitPCNT(int _unit){
    unit = (pcnt_unit_t) _unit;
}
void EncoderRTOS::attachHalfQuad(int aPintNumber, int bPinNumber) {
	attach(aPintNumber, bPinNumber, half);

}
void EncoderRTOS::attachSingleEdge(int aPintNumber, int bPinNumber) {
	attach(aPintNumber, bPinNumber, single);
}
void EncoderRTOS::attachFullQuad(int aPintNumber, int bPinNumber) {
	attach(aPintNumber, bPinNumber, full);
}

void EncoderRTOS::setCount(int64_t value) {
	count = value - getCountRaw();
}
int64_t EncoderRTOS::getCountRaw() {
	int16_t c;
	pcnt_get_counter_value(unit, &c);
	return c;
}
int64_t EncoderRTOS::getCount() {
	return getCountRaw() + count;
}

int64_t EncoderRTOS::clearCount() {
	count = 0;
	return pcnt_counter_clear(unit);
}

int64_t EncoderRTOS::pauseCount() {
	return pcnt_counter_pause(unit);
}

int64_t EncoderRTOS::resumeCount() {
	return pcnt_counter_resume(unit);
}

void EncoderRTOS::setFilter(uint16_t value) {
	if(value>1023)value=1023;
	if(value==0) {
		pcnt_filter_disable(unit);	
	} else {
		pcnt_set_filter_value(unit, value);
		pcnt_filter_enable(unit);	
	}
	
}
