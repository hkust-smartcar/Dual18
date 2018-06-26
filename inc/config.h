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
#include <libsc/futaba_s3010.h>
#include <libsc/alternate_motor.h>
#include <libsc/k60/ov7725.h>
#include <libsc/k60/ov7725_configurator.h>

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
using libsc::FutabaS3010;
using libsc::BatteryMeter;
using libsc::AlternateMotor;
using libsc::k60::Ov7725;
using libsc::k60::Ov7725Configurator;

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
        //TODO: finish it
    	Joystick::Config config;
    	config.id = 0;
    	config.is_active_low = true;
    	config.dispatcher = isr;
//    	config.handlers[0] = isr;
//    	config.handlers[1] = isr;
//    	config.handlers[2] = isr;
//    	config.handlers[3] = isr;
//    	config.listener_triggers[0] = Joystick::Config::Trigger::kBoth;
//    	config.listener_triggers[1] = Joystick::Config::Trigger::kBoth;
//    	config.listener_triggers[2] = Joystick::Config::Trigger::kBoth;
//    	config.listener_triggers[3] = Joystick::Config::Trigger::kBoth;
    	// ADD!
    	return config;
    }

    static St7735r::Config GetLcdConfig() {
        //TODO: finish it
    	St7735r::Config config;
    	config.fps = 20;
    	return config;
    }

    static JyMcuBt106::Config GetBluetoothConfig(std::function<bool(const Byte *data, const size_t size)> isr) {
        //TODO: finish it
    	JyMcuBt106::Config config;
    	config.id = 0;
    	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
    	config.rx_isr = isr;
    	return config;
    }

    static JyMcuBt106::Config GetSlaveBluetoothConfig() {
        //TODO: finish it
    	JyMcuBt106::Config config;
    	config.id = 1;
    	config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
    	return config;
    }

    static Pit::Config GetBluetoothPitConfig(std::function<void(Pit*)> isr){
    	//TODO: finish it
    	Pit::Config pitConfig;
    	pitConfig.channel = 0;
    	pitConfig.count = 75000*10; //job executed once per 10ms
    	pitConfig.isr = isr;
    	return pitConfig;
    }

    static DirEncoder::Config GetEncoderConfig(int m_id){
    	DirEncoder::Config config;
    	config.id = m_id;
    	return config;
    }

    static Adc::Config GetAdcConfig(int id){
    	Adc::Config config;
    	BoardID board;
    	switch (id) {
    			case 4:
    				config.adc = Adc::Name::kAdc0Ad4B;
    				break;
    			case 5:
    				config.adc = Adc::Name::kAdc1Ad4B;
    				break;
    			case 2:
    				config.adc = Adc::Name::kAdc1Ad5B;
    				break;
    			case 3:
    				if(board.isCar1()){
    					config.adc = Adc::Name::kAdc1Ad7B;
    				} else{
    					config.adc = Adc::Name::kAdc1Ad6B;
    				}
    				break;
    			case 0:
    				if(board.isCar1()){
    					config.adc = Adc::Name::kAdc1Ad6B;
    				}else{
    					config.adc = Adc::Name::kAdc1Ad7B;
    				}
    				break;
    			case 1:
    				config.adc = Adc::Name::kAdc0Ad5B;
    				break;
    			case 6:
    				config.adc = Adc::Name::kAdc0Ad6B;
    				break;
    			default:
    				break;
    	}
    	config.resolution = Adc::Config::Resolution::k8Bit;
    	config.speed = Adc::Config::SpeedMode::kExSlow;
    	config.is_continuous_mode =true;
    	config.avg_pass =Adc::Config::AveragePass::k32;
    	return config;
    }

    static FutabaS3010::Config GetServoConfig(){
    		FutabaS3010::Config config;
    		config.id = 0;
    		return config;
    }

    static AlternateMotor::Config GetMotorConfig(int id){
    		AlternateMotor::Config config;
    		config.id = id;
    		config.multiplier = 100;
    		return config;
    }

    static Ov7725::Config getCameraConfig(int width, int height) {
    		Ov7725::Config config;
    		config.id = 0;
    		config.w = width;
    		config.h = height;
    		config.fps = Ov7725Configurator::Config::Fps::kHigh;
    		return config;
    }

    static BatteryMeter::Config GetBatteryMeterConfig(){
    	BatteryMeter::Config config;
    	config.voltage_ratio = 0.365;
    	return config;
    }
};


#endif /* INC_CONFIG_H_ */
