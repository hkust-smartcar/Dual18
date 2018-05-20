/*
 * config.h
 *
 * configure all the peripherals here
 *
 *  Created on: Dec 23, 2017
 *      Author: dipsy
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include <functional>

#include "libsc/led.h"
#include "libsc/joystick.h"
#include "libsc/button.h"
#include "libsc/st7735r.h"
#include "libsc/lcd_typewriter.h"
#include "libsc/lcd_console.h"
#include "libsc/battery_meter.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libbase/k60/pit.h"
#include "libbase/k60/uart.h"
#include "libsc/dir_encoder.h"
#include "libbase/k60/adc.h"
#include <libsc/futaba_s3010.h>
#include <libbase/k60/uart.h>
#include <libsc/alternate_motor.h>
#include "libsc/dir_encoder.h"
#include <libsc/k60/ov7725.h>

using libsc::Led;
using libsc::Lcd;
using libsc::Joystick;
using libsc::St7735r;
using libsc::LcdTypewriter;
using libsc::LcdConsole;
using libsc::BatteryMeter;
using libsc::k60::JyMcuBt106;
using libbase::k60::Pit;
using libbase::k60::Uart;
using libsc::DirEncoder;
using libbase::k60::Adc;
using libsc::FutabaS3010;
using libsc::AlternateMotor;
using libsc::Button;
using libsc::k60::Ov7725;

class myConfig {
public:

	static Led::Config GetLedConfig(uint8_t id) {
		Led::Config config;
		config.id = id;
		return config;
	}
	static LcdTypewriter::Config GetWriterConfig(St7735r *lcd) {
		LcdTypewriter::Config config;
		config.lcd = lcd;
		return config;
	}
	static LcdConsole::Config GetConsoleConfig(St7735r *lcd) {
		LcdConsole::Config config;
		config.lcd = lcd;
		config.region = Lcd::Rect(0, 0, 128, 160);
		return config;
	}
	static Joystick::Config GetJoystickConfig(Joystick::Listener isr) {
		//TODO: finish it
		Joystick::Config config;
		config.id = 0;
		config.is_active_low = true;

		Joystick::Config::Trigger trigger = Joystick::Config::Trigger::kUp;

		config.listener_triggers[0] = trigger;
		config.listener_triggers[1] = trigger;
		config.listener_triggers[2] = trigger;
		config.listener_triggers[3] = trigger;
		config.listener_triggers[4] = trigger;

		config.handlers[0] = isr;
		config.handlers[1] = isr;
		config.handlers[2] = isr;
		config.handlers[3] = isr;
		config.handlers[4] = isr;

		return config;
	}

	static St7735r::Config GetLcdConfig() {
		//TODO: finish it
		St7735r::Config config;
		config.fps = 20;
		return config;
	}

	static Button::Config GetButtonConfig(uint8_t id, Button::Listener isr) {
		Button::Config config;
		config.id = id;
		config.is_active_low = true;
		config.listener_trigger = Button::Config::Trigger::kBoth;
		config.listener = isr;
		return config;
	}

	static DirEncoder::Config GetEncoderConfig(int id) {
		DirEncoder::Config config;
		config.id = id;
		return config;
	}

	static AlternateMotor::Config GetMotorConfig(int id) {
		AlternateMotor::Config config;
		config.id = id;
		config.multiplier = 100;
		return config;
	}

	    static Adc::Config GetAdcConfig(int id){
	    	Adc::Config config;
	    	if (id == 0){
	    		config.adc = Adc::Name::kAdc0Ad8;
	    	}
	    	else if (id == 1){
	    		config.adc = Adc::Name::kAdc0Ad9;
	    	}
	    	else if (id == 2){
	    		config.adc = Adc::Name::kAdc1Ad6B;
	    	}
	    	else {
	    		config.adc = Adc::Name::kAdc1Ad7B;
	    	}

//	    	if (id == 0){ // ptb3
//	    		config.adc = Adc::Name::kAdc0Ad13;
//	    	}
//	    	else if (id == 1){ // ptb2
//	    		config.adc = Adc::Name::kAdc0Ad12;
//	    	}
//	    	else if (id == 2){ //ptb1
//	    		config.adc = Adc::Name::kAdc1Ad9;
//	    	}
//	    	else { //ptb0
//	    		config.adc = Adc::Name::kAdc1Ad8;
//	    	}

	    	config.resolution = Adc::Config::Resolution::k8Bit;
	    	config.speed = Adc::Config::SpeedMode::kExSlow;
	    	config.is_continuous_mode =true;
	    	config.avg_pass =Adc::Config::AveragePass::k32;
	    	return config;
	    }

	    static Ov7725::Config getCameraConfig(uint8_t id) {
			Ov7725::Config config;
			config.id = id;
			config.w = 80;
			config.h = 60;
			config.fps = Ov7725::Config::Fps::kHigh;
			return config;
		}
};

#endif /* INC_CONFIG_H_ */
