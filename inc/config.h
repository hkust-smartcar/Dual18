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
#include "sprite.h"

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

    static Pit::Config GetBluetoothPitConfig(std::function<void(Pit*)> isr){
    	//TODO: finish it
    	Pit::Config pitConfig;
    	pitConfig.channel = 0;
    	pitConfig.count = 75000*10; //job executed once per 10ms
    	pitConfig.isr = isr;
    	return pitConfig;
    }

    static Sprite::Config GetSpriteConfig(uint16_t fg, uint16_t bg, unsigned int width, unsigned int height, int x, int y, libsc::Lcd* pLcd){
    	Sprite::Config config;
    	config.fg_color = fg;
    	config.bg_color = bg;
    	config.height = height;
    	config.width = width;
    	config.x = x;
   		config.y = y;
   		config.pLcd = pLcd;
   		return config;
    }
};


#endif /* INC_CONFIG_H_ */
