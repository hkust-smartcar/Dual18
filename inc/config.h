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
#include "libsc/k60/ov7725.h"
#include "libsc/k60/ov7725_configurator.h"
#include "libsc/futaba_s3010.h"
#include "libsc/alternate_motor.h"
#include "libbase/k60/adc.h"
#include "libsc/button.h"
#include "libsc/servo.h"
#include "libsc/dir_encoder.h"

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
using libsc::k60::Ov7725;
using libsc::k60::Ov7725Configurator;
using libsc::FutabaS3010;
using libsc::AlternateMotor;
using libbase::k60::Adc;
using libsc::Button;
using libsc::Servo;
using libsc::DirEncoder;

using std::function;



class Config{
public:

	static Led::Config GetLedConfig(int id){
		Led::Config config;
		config.id = id;

		return config;
	}

    static Joystick::Config GetJoystickConfig(Joystick::Listener
    		 isr){
    	Joystick::Config config;
    	config.id = 0;
    	config.dispatcher = isr;
    	return config;
    }

    static St7735r::Config GetLcdConfig() {
    	St7735r::Config config;
    	config.orientation=0;
    	config.fps = 20;
    	return config;
    }


    static JyMcuBt106::Config GetBluetoothConfig(std::function<bool(const Byte *data, const size_t size)> isr) {
    	JyMcuBt106::Config config;
		config.id=0;
    	config.baud_rate = Uart::Config::BaudRate::k115200;
    	config.rx_isr=isr;
    	return config;
    }

    static Pit::Config GetBluetoothPitConfig(std::function<void(Pit*)> isr){
    	Pit::Config config;
    	config.channel=0;
    	config.count=75000*10;//10ms
    	config.isr=isr;
    	return config;
    }

    static LcdTypewriter::Config GetWriterConfig(Lcd* _lcd){
    	LcdTypewriter::Config config;
    	config.lcd=(libsc::LcdTypewriter::Lcd*)_lcd;
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




    static Ov7725::Config GetCameraConfig(uint16_t w=80,uint16_t h=60){
    Ov7725::Config config;
    	config.id = 0;
    	config.w = w;
    	config.h =h;
    	config.fps = Ov7725Configurator::Config::Fps::kHigh;
//    	config.contrast=0x44;
    	config.contrast=0x64;
//    	config.brightness=0x30;
    	return config;
    }

    static Servo::Config GetServoConfig() {
    	Servo::Config config;
    	config.id = 0;
    	config.period = 3333;
    	config.max_pos_width = 2000;
    	config.min_pos_width = 1000;
    	return config;
    }

    static  AlternateMotor::Config GetMotorConfig(uint8_t id){
    	AlternateMotor::Config config;
    	config.id=id;
    	return config;
    }




    static Button::Config GetbuttonConfig(std::function<void(const uint8_t id)> fun()){
    	Button::Config config;
    	config.id=0;
    	config.listener=fun();
    	return config;
    }

    static Adc::Config GetAdcConfig(Adc::Name adc) {
    Adc::Config config;
    config.adc = adc;
	config.resolution = Adc::Config::Resolution::k8Bit;
	config.speed = Adc::Config::SpeedMode::kExSlow;
	config.is_continuous_mode =true;
	config.avg_pass =Adc::Config::AveragePass::k32;
	return config;
    }

    static DirEncoder::Config GetEncoderConfig(int id){
    	DirEncoder::Config config;
    	config.id = id;
    	return config;
    }
    static AlternateMotor::Config GetMotorConfig(int id){
    		AlternateMotor::Config config;
    		config.id = id;
    		config.multiplier = 100;
    		return config;
    }





};


#endif /* INC_CONFIG_H_ */
