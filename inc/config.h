/*
 * config.h
 *
 *  Created on: Feb 10, 2018
 *      Author: morristseng
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_
#include <functional>
#include <cstdint>
#include "libsc/led.h"
#include "libsc/joystick.h"
#include "libsc/st7735r.h"
#include "libsc/lcd_typewriter.h"
#include "libsc/lcd_console.h"
#include "libsc/battery_meter.h"
#include "libsc/k60/jy_mcu_bt_106.h"
#include "libbase/k60/pit.h"
#include "libbase/k60/uart.h"
#include <libsc/alternate_motor.h>
#include <libbase/k60/adc.h>
#define Width 80
#define Height 60

using libsc::System;
using namespace libsc;
using namespace libbase::k60;
using namespace std;


St7735r::Config getLcdConfig() {
    St7735r::Config config;
    config.orientation = 2;
    return config;
}

LcdTypewriter::Config GetWriterConfig(St7735r *lcd) {
    LcdTypewriter::Config config;
    config.lcd = lcd;
    config.bg_color = 0;
    config.text_color = 0xFFFF;
    config.is_text_wrap = false;
    return config;
}

k60::Ov7725::Config getCameraConfig() {
    k60::Ov7725::Config config;
    config.id = 0;
    config.w = Width;
    config.h = Height;
    config.fps = k60::Ov7725Configurator::Config::Fps::kHigh;
    return config;
}

FutabaS3010::Config getServoConfig() {
    FutabaS3010::Config config;
    config.id = 0;
    return config;
}

AlternateMotor::Config getMotorConfig() {
    AlternateMotor::Config config;
    config.id = 1;
    return config;
}

Adc::Config getMagSensorConfig(Pin::Name pin_name) {
    Adc::Config config;
    config.pin = pin_name;
    config.speed = Adc::Config::SpeedMode::kFast;
    config.is_continuous_mode = true;
    config.avg_pass = Adc::Config::AveragePass::k32;
    return config;
}

Joystick::Config getJoystickConfig(Joystick::Listener isr) {
    Joystick::Config config;
    config.id = 0;
    config.is_active_low = true;
    config.dispatcher = isr;
    return config;
}

Led::Config getLedConfig(int led_id){
	Led::Config config;
	config.id = led_id;
	return config;
}


#endif /* INC_CONFIG_H_ */
