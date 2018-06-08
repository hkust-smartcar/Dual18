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
#include "libsc/st7735r.h"
#include "libsc/lcd_typewriter.h"
#include "libsc/lcd_console.h"
#include "libsc/battery_meter.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libbase/k60/pit.h"
#include "libbase/k60/uart.h"
#include "libsc/dir_encoder.h"
#include "libbase/k60/adc.h"
#include "libbase/k60/pin.h"
#include "libsc/servo.h"
#include <libsc/alternate_motor.h>

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
using libbase::k60::Pin;
using libsc::AlternateMotor;
using libsc::Servo;

class myConfig{
public:

	static Led::Config GetLedConfig(int id){
		Led::Config config;
		config.id = id;
		return config;
	}
	static LcdTypewriter::Config GetWriterConfig(St7735r *lcd){
		LcdTypewriter::Config config;
		config.lcd = lcd;
		return config;
	}
	static LcdConsole::Config GetConsoleConfig(St7735r *lcd){
		LcdConsole::Config config;
		config.lcd = lcd;
		config.region = Lcd::Rect(0,0,128,160);
		return config;
	}
    static Joystick::Config GetJoystickConfig(Joystick::Listener isr) {
    	Joystick::Config config;
    	config.id = 0;
    	config.is_active_low = true;
    	config.dispatcher = isr;
    	return config;
    }

    static St7735r::Config GetLcdConfig() {
    	St7735r::Config config;
    	config.fps = 20;
    	return config;
    }

    static JyMcuBt106::Config GetBluetoothConfig(std::function<bool(const Byte *data, const size_t size)> isr) {
    	JyMcuBt106::Config config;
    	config.id = 1;
    	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k9600;
    	config.rx_isr = isr;
    	return config;
    }

    static Pit::Config GetBluetoothPitConfig(std::function<void(Pit*)> isr){
    	Pit::Config pitConfig;
    	pitConfig.channel = 0;
    	pitConfig.count = 75000*10; //job executed once per 10ms
    	pitConfig.isr = isr;
    	return pitConfig;
    }

    static DirEncoder::Config GetEncoderConfig(uint8_t id){
    	DirEncoder::Config config;
    	config.id = id;
    	return config;
    }

    static Servo::Config GetServoConfig(){
       	Servo::Config config;
       	config.id = 0;
       	config.period = 16667; //60hz
       	config.min_pos_width = 1000;
       	config.max_pos_width = 2000;
   		return config;
       }

    static Adc::Config GetAdcConfig(int id){
    	Adc::Config config;
    	if (id == 0){
    		config.pin = Pin::Name::kPtc11;
    	}
    	else if (id == 1){
    		config.pin = Pin::Name::kPtc10;
    	}
    	else if (id == 2){
    		config.pin = Pin::Name::kPtb1;
    	}
    	else if (id == 3) {
    		config.pin = Pin::Name::kPtb0;
    	}
    	else if (id == 4) {
    		config.pin = Pin::Name::kPtb11;
    	}
    	else if (id == 5) {
    		config.pin = Pin::Name::kPtc0;
    	}
//    	if (id == 0){
//    		config.adc = Adc::Name::kAdc0Ad13;
//    	}
//    	else if (id == 1){
//    		config.adc = Adc::Name::kAdc0Ad12;
//    	}
//    	else if (id == 2){
//    		config.adc = Adc::Name::kAdc1Ad9;
//    	}
//    	else if (id == 3) {
//    		config.adc = Adc::Name::kAdc1Ad8;
//    	}
//    	else if (id == 4) {
//    		config.adc = Adc::Name::kAdc0Ad17;
//    	}
//    	else if (id == 5) {
//    		config.adc = Adc::Name::kAdc0Ad18;
//    	}
    	config.resolution = Adc::Config::Resolution::k8Bit;
    	config.speed = Adc::Config::SpeedMode::kExSlow;
    	config.is_continuous_mode = true;
    	config.avg_pass = Adc::Config::AveragePass::k32;
    	config.is_diff_mode = false;
    	return config;
    }


    static AlternateMotor::Config GetMotorConfig(uint8_t id){
    		AlternateMotor::Config config;
    		config.id = id;
    		config.multiplier = 100;
    		return config;
    }
};


#endif /* INC_CONFIG_H_ */
